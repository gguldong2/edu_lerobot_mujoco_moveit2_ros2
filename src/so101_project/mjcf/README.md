# mjcf/ — MuJoCo 물리 모델

**출처: [MuJoCo Menagerie `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101)** (Apache-2.0, `LICENSE` 동봉).
The Robot Studio의 공식 SO-101 MJCF에서 파생된 모델이다. 실험적 URDF→MJCF 변환기를 쓰지 않고
이 모델을 쓰는 이유는 아래 값이 우리 URDF와 **이미 일치**하기 때문이다.

| 확인 항목 | 결과 |
|---|---|
| 관절 이름 6개 | `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` — URDF와 동일 |
| 관절 범위 | URDF 한계와 일치 (wrist_roll 상한만 아래에서 수정) |
| position actuator | 6개 모두 정의됨 (`class="sts3215"`, kp 998.22, kv 2.731, forcerange ±2.94) |
| 고정 베이스 | freejoint 없음 |
| 단위 | `compiler angle="radian"` |
| 씬 | `scene.xml`에 바닥·조명·skybox 포함 (사진 찍기 좋음), `scene_box.xml`은 상자 추가 버전 |
| 요구 버전 | MuJoCo ≥ 3.1.3 (`ros-jazzy-mujoco-vendor`는 3.4.0) |

## 이 프로젝트에서 바꾼 것

1. `so101.xml` — `wrist_roll` joint range 상한 `2.7438473` → **`2.84121`**.
   URDF와 actuator `ctrlrange`는 2.84121인데 joint range만 좁아서, MoveIt이 2.8을 계획하면
   시뮬레이터가 클램프해 추종오차처럼 보인다.
2. `scene.xml` — SRDF `rest` 자세와 같은 `keyframe name="rest"` 추가
   (`qpos="0 -1.57 1.57 0.75 0 0"`, 순서는 모델 관절 순서).

## 쓰는 법

```bash
# 모델만 단독으로 열어 보기 (ROS 없이)
python3 -m mujoco.viewer --mjcf=src/so101_project/mjcf/scene.xml

# ros2_control 백엔드로 사용 (기본 경로라 인자 생략 가능)
ros2 launch so101_project bringup.launch.py hardware_type:=mujoco
```

## 주의

- **MuJoCo에 넣은 장애물은 MoveIt Planning Scene에 자동으로 생기지 않는다.** 충돌 회피를 검증할 때는
  같은 크기·좌표의 box를 양쪽에 등록해야 한다. 먼저 빈 씬으로 관절 제어를 검증한다.
- body 이름(`base`, `shoulder`, ...)은 URDF 링크 이름(`base_link`, `shoulder_link`, ...)과 다르지만
  문제되지 않는다. ros2_control은 **관절 이름**으로 매핑하고, TF는 robot_state_publisher가 URDF로 만든다.
