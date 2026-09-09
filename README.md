# edu_lerobot_mujoco_moveit2_ros2

SO-101 로봇 팔을 **ROS 2 + MoveIt 2 + MuJoCo**로 제어하는 교육용 프로젝트.

[UnrealRobotics: SO-101 연재 22편](https://lightbakery.tistory.com/324)을 따라가되
**Unreal Engine 자리를 MuJoCo(물리) + RViz(계획·상태) + ROS 액션 클라이언트(명령)로 치환**했다.
실행 대상은 launch 인자 하나로 고른다 — `mock`(물리 없음) / `mujoco`(물리 시뮬레이션) / `real`(실물 Feetech).

```
 미션1  leader(데모/슬라이더/실물)  →  leader_follow.py     ┐
 미션2  사람이 각도 지정            →  joint_command.py     ├→ arm_controller / gripper_controller
 미션3  MoveIt 계획                →  RViz / group_goal_client.py ┘   (joint_trajectory_controller)
                                                                          │
                                                       ros2_control 하드웨어 인터페이스
                                                        mock  │  mujoco  │  real
                                                                          │
                                            joint_state_broadcaster → /joint_states → RViz
```

**입구는 셋, 출구는 하나다.** 그래서 `mock`에서 통한 명령이 MuJoCo에서도, 실물에서도
글자 하나 안 바꾸고 동작한다. 이것이 이 저장소의 설계 목표다.

---

## 미션 3개

| 미션 | 블로그 | 하는 일 | 명령 (bringup 후 다른 터미널) |
|---|---|---|---|
| **1. 추종** | 9편 | leader 각도를 초당 30회 스트리밍 → follower가 따라온다 | `ros2 run so101_project leader_follow.py --source demo` |
| **2. 직접 명령** | 10편 | 관절 각도를 시간 궤적으로 보낸다 (MoveIt 미개입) | `ros2 run so101_project joint_command.py --deg shoulder_pan=30 --time 2` |
| **3. MoveIt** | 13편 | 그룹 단위로 계획·충돌검사 후 실행 | `ros2 run so101_project group_goal_client.py --group arm --named extended --execute` |

미션 3은 RViz의 MotionPlanning 패널에서 마우스로도 할 수 있다(Goal State → Plan → Execute).
세 미션의 실행·녹화 절차는 **[docs/05_미션_가이드.md](docs/05_미션_가이드.md)** 가 정본이다.

---

## 요구 환경

| 항목 | 값 | 비고 |
|---|---|---|
| OS | WSL2 **Ubuntu 24.04 (noble)** | 순수 Ubuntu 24.04에서도 동일 |
| ROS | **ROS 2 Jazzy** | `mujoco_ros2_control`·`lerobot-ros`가 Jazzy 기준으로 배포된다 |
| 로봇 | SO-101 (Feetech STS3215) | **없어도 미션 1·2·3 전부 가능** (MuJoCo로 대체) |
| 디스크 | 약 6 GB | ROS desktop + MoveIt + MuJoCo |
| 설치 시간 | 20~40분 | 대부분 `ros-jazzy-desktop` 내려받기 |

물리 모델(MJCF)은 저장소에 포함돼 있어 별도 변환이 필요 없다.

---

## 빠른 시작

```bash
# 0) 클론 (WSL 안에서. Windows 폴더(/mnt/c/...)에 두어도 된다)
git clone https://github.com/gguldong2/edu_lerobot_mujoco_moveit2_ros2
cd edu_lerobot_mujoco_moveit2_ros2

# 1) 설치 (sudo, 한 번, 20~40분)
./scripts/00_install_jazzy.sh

# 2) 워크스페이스 구성 + 빌드 (~/so101_ws)
./scripts/01_setup_workspace.sh

# 3) 모델 점검 (URDF 3개 백엔드 전개 + MJCF 대조)
./scripts/02_export_urdf.sh
./scripts/03_make_mjcf.sh

# 4) 새 터미널마다 자동 준비 (한 번만, 저장소 루트에서)
echo "source $(pwd)/scripts/env.sh" >> ~/.bashrc && exec bash
```

**터미널 1 — 로봇을 올린다(bringup). 이 터미널은 계속 점유된다:**

```bash
./scripts/06_bringup.sh mujoco     # MuJoCo 물리 + MoveIt + RViz
# ./scripts/06_bringup.sh mock     # 물리 없이 배선만 (가장 먼저 해보기 좋다)
# ./scripts/06_bringup.sh real usb_port:=/dev/ttyACM0    # 실물 (집에서)
```

`You can start planning now!` 가 나오면 준비 완료다.

**터미널 2 — 점검하고 미션을 돌린다:**

```bash
./scripts/04_smoke_test.sh                     # 컨트롤러·액션·상태·계획·거부까지 자동 점검

ros2 run so101_project leader_follow.py --source demo                            # 미션 1
ros2 run so101_project joint_command.py --deg shoulder_pan=30 --time 2           # 미션 2
ros2 run so101_project group_goal_client.py --group arm --named extended --execute  # 미션 3
```

`~/.bashrc` 설정을 건너뛰었다면 `ros2 run so101_project X` 대신 `./scripts/08_run.sh X`를 쓴다
(환경 준비를 스크립트가 대신한다).

---

## 무엇을 보면 정상인가

창 두 개와 터미널이 각각 다른 것을 보여준다. **판정은 화면이 아니라 터미널 숫자로 한다.**

| 미션 | MuJoCo 창 (물리) | RViz (모델·계획) | 터미널 (판정) |
|---|---|---|---|
| 1 추종 | 팔이 **계속** 부드럽게 움직인다 | 같은 움직임을 따라간다 | 0.5초마다 `최대추종오차` |
| 2 직접 명령 | 명령한 관절만 `--time` 동안 움직이고 멈춘다 | 같이 움직인다 | `SUCCESSFUL` + 목표/실측 오차 표 |
| 3 MoveIt | **RViz의 Execute를 누른 뒤** 움직인다 | **주황색 목표 + 반투명 경로 재생** | `SUCCESS` + `planned_points` + `worst error` |

- RViz는 세 미션 모두에서 움직인다. RViz가 그리는 것은 계획이 아니라 `/joint_states`(실측)다.
  미션 3에만 있는 것은 **실행 전 미리보기**다.
- 물리는 MuJoCo 창에서만 일어난다. MuJoCo 창에는 로봇에 명령하는 버튼이 없다(물리 뷰어다).

### 오차 숫자 읽기 — 미션 1과 미션 2·3은 다른 것을 잰다

| | 미션 1 `최대추종오차` | 미션 2·3 `최대 오차` |
|---|---|---|
| 언제 | **움직이는 중** 계속 | **멈춘 뒤** 한 번 |
| 무엇 | 방금 보낸 명령 ↔ 지금 각도 | 최종 목표 ↔ 최종 각도 |
| 0이 되나 | **안 된다**(항상 뒤처진다) | **거의 0이어야 한다** |
| 정상 범위 | 0.03~0.15 rad (2~9°) | mock 0.0000 / MuJoCo 0.001 이하 / 실물 0.01~0.05 예상 |

실측 예: 미션 2 MuJoCo `0.0005 rad(0.03°)`, 미션 3 MuJoCo `0.0092 rad`(중력·마찰 때문에 0이 아니다),
미션 1 데모 `0.03~0.10 rad`. 자세한 판정표는 [docs/05](docs/05_미션_가이드.md) 1-2절.

---

## 저장소 구조

```
├─ scripts/                     실행 스크립트 (번호가 실행 순서다)
│  ├─ 00_install_jazzy.sh       설치 (sudo). --gui 로 화면 도구만 설치/확인
│  ├─ 01_setup_workspace.sh     ~/so101_ws 구성 → 업스트림 clone → MoveIt config 패치 → colcon build
│  ├─ 02_export_urdf.sh         3개 백엔드 URDF 전개 + check_urdf + 관절 표
│  ├─ 03_make_mjcf.sh           MJCF ↔ URDF 대조 점검 (--convert 로 실험적 변환)
│  ├─ 04_smoke_test.sh          배선 점검 (rclpy 직접 호출로 컨트롤러·액션·상태·계획·거부 확인)
│  ├─ 05_gui.sh                 viewer | leader | sliders | rqt | foxglove
│  ├─ 06_bringup.sh             **로봇 실행**: mock | mujoco | real
│  ├─ 07_doctor.sh              WSL 통신 진단·복구 (--fix)
│  ├─ 08_run.sh                 우리 노드 실행 래퍼 (환경 준비 자동)
│  ├─ env.sh                    터미널 준비 한 줄 (source 전용)
│  └─ lib_dds.sh                WSL mirrored 환경에서 DDS 우회 (다른 스크립트가 자동 source)
├─ src/so101_project/
│  ├─ urdf/so101.urdf.xacro                   top-level 모델 (robot name = so101_arm)
│  ├─ urdf/so101_ros2_control_backends.xacro  mock | mujoco | real 3분기
│  ├─ config/controllers.yaml                 JTC 2개 + joint_state_broadcaster
│  ├─ config/moveit_controllers.yaml          MoveIt → 컨트롤러 연결
│  ├─ config/joint_mapping.yaml               모델 ↔ 모터 계약서 (실물용 sign/offset)
│  ├─ launch/bringup.launch.py                하나의 인자로 3개 백엔드
│  ├─ scripts/leader_follow.py                미션 1 (추종)
│  ├─ scripts/joint_command.py                미션 2 (직접 명령)
│  ├─ scripts/group_goal_client.py            미션 3 (MoveIt 그룹 목표)
│  └─ mjcf/                                   MuJoCo 물리 모델 (Menagerie SO-101 + 수정 2건)
└─ docs/                        01 계획·의사결정 / 02 실행내역·실물 / 03 사용법 / 04 시나리오 / 05 미션 가이드
```

모델은 `src/`에 두고 빌드는 `~/so101_ws`에서 한다(`01`이 심볼릭 링크를 만든다).
Windows 편집기로 편집하면서 빌드 산출물은 리눅스 파일시스템에 두기 위한 구성이다.

---

## 로봇 사양 (모델에서 실측한 값)

| 관절 | 한계 [rad] | 모터 ID |
|---|---|---|
| `shoulder_pan` | ±1.91986 | 1 |
| `shoulder_lift` | ±1.74533 | 2 |
| `elbow_flex` | ±1.69 | 3 |
| `wrist_flex` | ±1.65806 | 4 |
| `wrist_roll` | −2.74385 … 2.84121 | 5 |
| `gripper` | −0.174533 … 1.74533 | 6 |

- MoveIt Planning Group: **`arm`**(5관절) / **`gripper`**(1관절)
- 저장 자세: arm `zero` `rest` `extended` · gripper `open` `closed`
- 인터페이스: command `[position]`, state `[position, velocity]`
- 말단 프레임 `gripper_frame_link` · 5자유도이므로 임의 자세 IK는 불가(관절 목표를 쓴다)

---

## WSL 사용자 주의: `networkingMode=mirrored`

`.wslconfig`에 `networkingMode=mirrored`가 있으면 **127.0.0.1 통신 자체가 죽어 있을 수 있다**
(TCP·UDP 모두). 그러면 ROS 노드끼리 서로를 못 보고, bringup 로그에
`Waiting for data on 'robot_description' topic`이 반복된다.

```bash
./scripts/07_doctor.sh        # 원인을 지목하고, 필요하면 자동 우회한다
```

`scripts/lib_dds.sh`가 그 경우를 감지해 127.0.0.1을 피하는 Fast DDS 프로필을 만들어 쓰므로
`04·05·06·07·08` 스크립트는 그대로 동작한다. 다만 `ros2 control ...`과 `ros2 daemon ...`은
ros2cli가 자기 RPC 주소를 `127.0.0.1`로 고정해 두어 우회할 수 없다
(`04_smoke_test.sh`가 같은 정보를 서비스 직접 호출로 보여준다).

**근본 해결**: Windows의 `C:\Users\<사용자>\.wslconfig`에서 `networkingMode=mirrored`를 지우고
PowerShell에서 `wsl --shutdown`. 측정 근거는 [docs/04 부록](docs/04_시나리오_실행법.md)에 있다.

---

## 문서

| 파일 | 내용 |
|---|---|
| [docs/05_미션_가이드.md](docs/05_미션_가이드.md) | **여기서 시작.** 미션 3개 실행·녹화, 숫자 읽는 법, 실물 연결 순서 |
| [docs/03_사용법.md](docs/03_사용법.md) | 개념 정리, 스크립트 00~08 사용법, URDF/MJCF가 왜 따로 있나, 문제 해결 |
| [docs/04_시나리오_실행법.md](docs/04_시나리오_실행법.md) | "연결 축"과 "계획 축", 시나리오 A~E, WSL 통신 부록 |
| [docs/01_작업계획서_블로그대응_및_의사결정.md](docs/01_작업계획서_블로그대응_및_의사결정.md) | 블로그 22편 ↔ 이 프로젝트 대응표, **의사결정 13건**(선택지·트레이드오프·근거) |
| [docs/02_실행내역_및_집에서_할일.md](docs/02_실행내역_및_집에서_할일.md) | 검증 내역, 실물 연결 절차, 계층별 트러블슈팅 |

---

## 설계 결정 (요약)

| 결정 | 선택 | 이유 |
|---|---|---|
| ROS 2를 쓸 것인가 | **쓴다** | MoveIt 2는 ROS 2 위에서만 동작한다. MoveIt을 빼면 계획·충돌검사를 직접 구현해야 한다 |
| 시뮬레이터 연결 | `mujoco_ros2_control` 하드웨어 인터페이스 | 컨트롤러 위쪽(그룹·액션·궤적)이 실물과 완전히 같아진다 |
| 궤적 실행 경로 | **표준 JTC** (블로그의 ZMQ 브리지 대신) | 같은 계획이 mock·MuJoCo·실물에서 그대로 실행된다 |
| MoveIt 설정 | 업스트림 `so101_moveit_config` 재사용 + 3곳 수정 | Setup Assistant 재생성보다 재현성이 높다 |
| MJCF 조달 | MuJoCo Menagerie 공식 모델 + 수정 2건 | 손으로 쓰거나 변환기를 돌릴 이유가 없었다 |
| 그리퍼 | 1관절 JTC (`ParallelGripperCommand` 대신) | 팔과 같은 실행 경로를 쓰면 실패 지점이 줄어든다 |

전체 13건과 근거는 [docs/01](docs/01_작업계획서_블로그대응_및_의사결정.md) 2절.

---

## 출처

| 출처 | 쓰인 곳 |
|---|---|
| [UnrealRobotics: SO-101 연재](https://lightbakery.tistory.com/324) | 프로젝트 원본 구성과 미션 정의 |
| [legalaspro/so101-ros-physical-ai](https://github.com/legalaspro/so101-ros-physical-ai) | `so101_description`(URDF·메시), `so101_moveit_config`(SRDF·OMPL·pick_ik), `feetech_ros2_driver` |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) `robotstudio_so101` | `src/so101_project/mjcf/` (**Apache-2.0**, 원본 LICENSE 포함, 수정 2건은 `mjcf/README.md`에 기록) |
| [ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) | MuJoCo ↔ ros2_control 연결 |
| [ycheng517/lerobot-ros](https://github.com/ycheng517/lerobot-ros) | (선택) LeRobot 텔레옵 어댑터 |
| [LeRobot](https://github.com/huggingface/lerobot) | 실물 캘리브레이션·텔레옵, 미션 1의 실물 leader 소스 |

벤더링한 MJCF 모델은 Apache-2.0이며 `src/so101_project/mjcf/LICENSE`에 원본 라이선스를 그대로 두었다.

---

## 검증 상태 (2026-09-10, WSL2 Ubuntu 24.04 + Jazzy)

| 항목 | 결과 |
|---|---|
| `mock` 백엔드 | 컨트롤러 3개 active, MoveIt `You can start planning now!` |
| `mujoco` 백엔드 | `MujocoSystemInterface` 로드, 액추에이터 6개 등록, `/clock` 발행 |
| 미션 1 (추종) | `--source demo` 0.03~0.10 rad, `--source topic` 정착 후 0.003 rad |
| 미션 2 (직접 명령) | 최대 오차 **0.0005 rad**, 한계 초과는 exit 2로 거부 |
| 미션 3 (MoveIt) | 계획+실행 성공, 최대 오차 **0.0092 rad**, 그룹 위반·자기충돌 거부 확인 |
| `04_smoke_test.sh` | 전체 통과 |
| 실물(`real`) · GUI 창 조작 | **미검증** — 실물은 [docs/02](docs/02_실행내역_및_집에서_할일.md) 3절, [docs/05](docs/05_미션_가이드.md) 8절 절차대로 진행 |
