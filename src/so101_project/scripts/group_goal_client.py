#!/usr/bin/env python3
"""Send a MoveIt Planning-Group goal and report what actually happened.

This node is the replacement for the Unreal side of the original blog project
(post 14): an external program picks a group, sends a goal, MoveIt plans and
executes it, and the result is checked against the measured joint states.

Examples
--------
  # plan only (default) - nothing moves
  ros2 run so101_project group_goal_client.py --group arm --named rest

  # plan and execute a joint goal, slowly
  ros2 run so101_project group_goal_client.py --group arm \
      --joints shoulder_pan=0.2,elbow_flex=-0.3 --execute --vel 0.1

  # gripper only
  ros2 run so101_project group_goal_client.py --group gripper --named open --execute

Design notes
------------
* Limits are read from /robot_description, group membership and named poses from
  /robot_description_semantic - nothing about the robot is hard-coded here, so the
  same client works after a model change.
* Planning failure, execution failure, rejection, cancellation and state timeout are
  reported as different outcomes. "Plan succeeded" is never reported as success.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningOptions,
    RobotState,
)
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

ERROR_NAMES = {
    int(v): k
    for k, v in vars(MoveItErrorCodes).items()
    if k.isupper() and isinstance(v, int)
}


def error_name(code: int) -> str:
    return f"{ERROR_NAMES.get(int(code), 'UNKNOWN')}({code})"


class GroupGoalClient(Node):
    def __init__(self):
        super().__init__("group_goal_client")
        self.urdf = None
        self.srdf = None
        self.joint_state = None
        self.create_subscription(String, "/robot_description", self._on_urdf, LATCHED)
        self.create_subscription(
            String, "/robot_description_semantic", self._on_srdf, LATCHED
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.client = ActionClient(self, MoveGroup, "/move_action")

    # ---------------------------------------------------------------- callbacks
    def _on_urdf(self, msg):
        self.urdf = msg.data

    def _on_srdf(self, msg):
        self.srdf = msg.data

    def _on_js(self, msg):
        self.joint_state = msg

    def wait_for(self, attr, timeout, what):
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while getattr(self, attr) is None:
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().error(f"timeout waiting for {what}")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    # ------------------------------------------------------------------- model
    def joint_limits(self):
        """{joint: (lower, upper)} for every movable joint in the URDF."""
        out = {}
        for j in ET.fromstring(self.urdf).findall("joint"):
            if j.get("type") in ("fixed", "floating", None):
                continue
            lim = j.find("limit")
            if lim is not None and lim.get("lower") is not None:
                out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        return out

    def _chain_joints(self, base, tip):
        """Movable joints between base_link and tip_link, walking the URDF up."""
        root = ET.fromstring(self.urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip
        while link != base:
            j = by_child.get(link)
            if j is None:
                raise RuntimeError(f"link '{link}' is not connected to '{base}'")
            if j.get("type") not in ("fixed",):
                chain.append(j.get("name"))
            link = j.find("parent").get("link")
        return list(reversed(chain))

    def group_joints(self, group):
        """Joints of an SRDF group - handles both <joint> lists and <chain>."""
        root = ET.fromstring(self.srdf)
        for g in root.findall("group"):
            if g.get("name") != group:
                continue
            joints = [j.get("name") for j in g.findall("joint")]
            for c in g.findall("chain"):
                joints += self._chain_joints(c.get("base_link"), c.get("tip_link"))
            movable = self.joint_limits()
            return [j for j in joints if j in movable]
        available = [g.get("name") for g in root.findall("group")]
        raise RuntimeError(f"group '{group}' not in SRDF. available: {available}")

    def named_pose(self, group, name):
        for gs in ET.fromstring(self.srdf).findall("group_state"):
            if gs.get("group") == group and gs.get("name") == name:
                return {j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
        names = [
            gs.get("name")
            for gs in ET.fromstring(self.srdf).findall("group_state")
            if gs.get("group") == group
        ]
        raise RuntimeError(f"named pose '{name}' not found for '{group}'. have: {names}")

    # ------------------------------------------------------------------ request
    def build_goal(self, group, targets, args):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.start_state.is_diff = True          # start from the measured state
        req.num_planning_attempts = args.attempts
        req.allowed_planning_time = args.planning_time
        req.max_velocity_scaling_factor = args.vel
        req.max_acceleration_scaling_factor = args.acc

        constraints = Constraints()
        for name, value in targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = args.tolerance
            jc.tolerance_below = args.tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]

        opts = PlanningOptions()
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        opts.plan_only = not args.execute
        goal.planning_options = opts
        return goal

    # ------------------------------------------------------- 충돌 상태 자기진단
    def explain_start_collision(self, group):
        """START_STATE_IN_COLLISION 일 때 '무엇이 무엇과 닿았는지'를 MoveIt 에 직접 묻는다.

        이 에러는 설정 문제가 아니라 현재 자세 문제다. 직접 제어(joint_command.py,
        leader_follow.py)는 충돌 검사를 하지 않으므로 팔이 자기 몸에 닿은 자세로
        멈출 수 있고, 그 자리에서는 MoveIt 이 계획을 시작할 수 없다.
        """
        cli = self.create_client(GetStateValidity, "/check_state_validity")
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("/check_state_validity 가 없어 원인을 더 볼 수 없다")
            return
        req = GetStateValidity.Request()
        rs = RobotState()
        rs.joint_state = self.joint_state
        req.robot_state = rs
        req.group_name = group
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn("/check_state_validity 응답이 없다")
            return

        log = self.get_logger()
        log.error("원인: 지금 자세가 이미 자기충돌 상태다 (설정 문제가 아니다)")
        if not res.contacts:
            log.error("  접촉 목록이 비어 있다. 관절 한계를 살짝 넘은 상태일 수도 있다")
        for c in res.contacts:
            log.error(
                f"  닿은 링크: {c.contact_body_1} <-> {c.contact_body_2}  "
                f"깊이={c.depth * 1000:.3f} mm"
            )
        cur = {n: p for n, p in zip(self.joint_state.name, self.joint_state.position)}
        log.error("  현재 자세: " + ", ".join(f"{k}={v:+.3f}" for k, v in sorted(cur.items())))
        log.error("")
        log.error("  빠져나오는 방법 (충돌 검사를 하지 않는 직접 제어로 먼저 벗어난다):")
        log.error("      ros2 run so101_project joint_command.py --named rest --time 3")
        log.error("  그 다음 다시 계획한다:")
        log.error(f"      ros2 run so101_project group_goal_client.py --group {group} ...")


def parse_joint_arg(text):
    targets = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise SystemExit(f"bad --joints entry '{item}', expected name=value")
        name, value = item.split("=", 1)
        targets[name.strip()] = float(value)
    return targets


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--group", required=True, help="Planning group (arm | gripper)")
    ap.add_argument("--joints", help="name=value pairs in radians, comma separated")
    ap.add_argument("--named", help="SRDF named pose (rest, extended, open, closed ...)")
    ap.add_argument("--execute", action="store_true", help="actually move (default: plan only)")
    ap.add_argument("--vel", type=float, default=0.1, help="velocity scaling 0-1")
    ap.add_argument("--acc", type=float, default=0.1, help="acceleration scaling 0-1")
    ap.add_argument("--tolerance", type=float, default=0.01, help="goal tolerance [rad]")
    ap.add_argument("--attempts", type=int, default=3)
    ap.add_argument("--planning-time", type=float, default=5.0, dest="planning_time")
    ap.add_argument("--timeout", type=float, default=60.0, help="result timeout [s]")
    args = ap.parse_args()

    if bool(args.joints) == bool(args.named):
        raise SystemExit("give exactly one of --joints or --named")
    if not 0.0 < args.vel <= 1.0 or not 0.0 < args.acc <= 1.0:
        raise SystemExit("--vel/--acc must be in (0, 1]")

    rclpy.init()
    node = GroupGoalClient()
    rc = 1
    try:
        if not node.wait_for("urdf", 10.0, "/robot_description"):
            return 1
        if not node.wait_for("srdf", 10.0, "/robot_description_semantic (is move_group up?)"):
            return 1

        limits = node.joint_limits()
        allowed = node.group_joints(args.group)
        node.get_logger().info(f"group '{args.group}' joints: {allowed}")

        targets = node.named_pose(args.group, args.named) if args.named \
            else parse_joint_arg(args.joints)

        # ---- validate before touching the robot
        problems = []
        for name, value in targets.items():
            if name not in allowed:
                problems.append(f"{name}: not in group '{args.group}' ({allowed})")
                continue
            if math.isnan(value) or math.isinf(value):
                problems.append(f"{name}: value is not finite")
                continue
            lo, hi = limits[name]
            if not lo <= value <= hi:
                problems.append(f"{name}: {value:.4f} outside URDF limits [{lo}, {hi}]")
        if problems:
            for p in problems:
                node.get_logger().error(p)
            return 2

        if not node.client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error("/move_action not available - is move_group running?")
            return 1

        mode = "PLAN+EXECUTE" if args.execute else "PLAN ONLY"
        node.get_logger().info(
            f"{mode} group={args.group} targets={ {k: round(v, 4) for k, v in targets.items()} } "
            f"vel={args.vel} acc={args.acc}"
        )

        send = node.client.send_goal_async(node.build_goal(args.group, targets, args))
        rclpy.spin_until_future_complete(node, send, timeout_sec=args.timeout)
        handle = send.result()
        if handle is None or not handle.accepted:
            node.get_logger().error("goal REJECTED by move_group")
            return 3

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
        if result_future.result() is None:
            node.get_logger().error("no result before timeout - cancelling")
            handle.cancel_goal_async()
            return 4

        result = result_future.result().result
        code = int(result.error_code.val)
        planned_points = len(result.planned_trajectory.joint_trajectory.points)
        node.get_logger().info(
            f"error_code={error_name(code)}  planned_points={planned_points}  "
            f"planning_time={result.planning_time:.3f}s"
        )
        if code != MoveItErrorCodes.SUCCESS:
            node.get_logger().error("FAILED (planning or execution) - see error_code above")
            if code == MoveItErrorCodes.START_STATE_IN_COLLISION:
                node.explain_start_collision(args.group)
            return 5
        if not args.execute:
            node.get_logger().info("plan-only succeeded: nothing was executed")
            return 0
        if planned_points == 0:
            # 시작 자세가 이미 목표 자세면 MoveIt 은 빈 궤적을 돌려준다. 성공이지만
            # 화면에서는 아무것도 움직이지 않는다 - 실패로 오해하기 쉬워서 명시한다.
            node.get_logger().info(
                "궤적 점이 0개다: 이미 목표 자세에 있어서 움직일 것이 없다 (정상). "
                "움직이는 것을 보려면 다른 자세를 목표로 준다. 예: --named extended"
            )

        # ---- did the robot actually get there?
        node.joint_state = None
        if not node.wait_for("joint_state", 5.0, "/joint_states after execution"):
            return 6
        for _ in range(10):                      # let a couple of cycles settle
            rclpy.spin_once(node, timeout_sec=0.05)
        actual = dict(zip(node.joint_state.name, node.joint_state.position))
        worst, worst_joint = 0.0, None
        node.get_logger().info("joint         target     actual     error")
        for name, value in targets.items():
            got = actual.get(name, float("nan"))
            err = abs(got - value)
            if not math.isnan(err) and err > worst:
                worst, worst_joint = err, name
            node.get_logger().info(f"{name:13s} {value:+8.4f}  {got:+8.4f}  {err:8.4f}")
        node.get_logger().info(f"worst error {worst:.4f} rad on {worst_joint}")
        rc = 0 if worst <= max(args.tolerance * 5, 0.05) else 7
        if rc:
            node.get_logger().error(
                "action reported SUCCESS but the measured pose is off - check gains, "
                "sign/offset or joint limits before trusting this backend"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
