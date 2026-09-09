# shellcheck shell=bash
# 이 파일은 실행하지 않고 source 한다.  (04·05·06·07 스크립트가 자동으로 불러온다)
#
# 하는 일: WSL 의 "mirrored" 네트워킹 모드에서 127.0.0.1 통신이 죽는 문제를 우회한다.
#
# 증상   : 같은 PC 안의 ROS 프로세스끼리 서로를 못 본다.
#          - controller_manager: "Waiting for data on 'robot_description' topic" 무한 반복
#          - ros2 daemon:        TimeoutError: [Errno 110] Connection timed out
# 원인   : .wslconfig 의 networkingMode=mirrored 가 loopback0 장치와 라우팅 테이블 127 을
#          만들어 127.0.0.0/8 트래픽을 Windows 쪽 경로로 보낸다. 이 랩탑에서는 그 경로가
#          패킷을 버려서 TCP·UDP 를 포함한 모든 로컬호스트 통신이 실패한다.
#          (멀티캐스트 자체는 정상이다. DDS 는 발견 후 유니캐스트로 붙기 때문에 막힌다)
# 우회   : Fast DDS 가 127.0.0.1 대신 WSL 내부 주소(10.255.255.254)만 쓰도록
#          프로필 XML 을 만들어 FASTRTPS_DEFAULT_PROFILES_FILE 로 넘긴다.
#          유니캐스트 initial peers 를 직접 나열하므로 멀티캐스트에 의존하지도 않는다.
# 근본 해결: Windows 의 %USERPROFILE%\.wslconfig 에서 networkingMode=mirrored 를 지우고
#          (=기본 NAT) PowerShell 에서 wsl --shutdown. 자세한 내용은 docs/04 부록.

# 127.0.0.1 로 UDP 왕복이 되는지 1.5초 안에 확인한다. 되면 0, 안 되면 1.
so101_loopback_ok() {
  python3 - <<'PY' >/dev/null 2>&1
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 0))
c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
c.sendto(b'x', ('127.0.0.1', s.getsockname()[1]))
s.settimeout(1.5)
s.recvfrom(8)
PY
}

# 주어진 주소로 UDP 왕복이 되는지 확인한다.
so101_addr_ok() {
  python3 - "$1" <<'PY' >/dev/null 2>&1
import socket, sys
a = sys.argv[1]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((a, 0))
c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
c.sendto(b'x', (a, s.getsockname()[1]))
s.settimeout(1.5)
s.recvfrom(8)
PY
}

# 필요할 때만 Fast DDS 프로필을 만들고 export 한다. 이미 정상인 환경에서는 아무것도 안 한다.
so101_dds_setup() {
  # 사용자가 직접 지정한 설정이 있으면 존중한다
  if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then return 0; fi
  case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
    rmw_fastrtps_cpp|"") ;;
    *) return 0 ;;   # Cyclone 등 다른 RMW 는 이 우회가 적용되지 않는다
  esac
  # ROS_LOCALHOST_ONLY 는 127.0.0.1 을 강제하므로 이 환경에서는 독이다
  if [ "${ROS_LOCALHOST_ONLY:-0}" = "1" ]; then
    echo "[dds] ROS_LOCALHOST_ONLY=1 은 이 WSL 에서 통신을 막는다 → 해제했다" >&2
    unset ROS_LOCALHOST_ONLY
  fi

  if so101_loopback_ok; then
    return 0     # 로컬호스트가 정상인 환경 (일반 리눅스, NAT 모드 WSL)
  fi

  local addr=""
  for cand in 10.255.255.254 "$(ip -4 route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9.]+')"; do
    [ -n "$cand" ] || continue
    if so101_addr_ok "$cand"; then addr="$cand"; break; fi
  done
  if [ -z "$addr" ]; then
    echo "[dds] 127.0.0.1 도, 대체 주소도 통신이 안 된다. ./scripts/07_doctor.sh 를 먼저 실행하라." >&2
    return 1
  fi

  local domain="${ROS_DOMAIN_ID:-0}"
  local base=$((7400 + 250 * domain))
  local xml="/tmp/so101_fastdds_${addr}_d${domain}.xml"
  if [ ! -f "$xml" ]; then
    {
      echo '<?xml version="1.0" encoding="UTF-8"?>'
      echo '<dds xmlns="http://www.eprosima.com">'
      echo '  <profiles>'
      echo '    <transport_descriptors>'
      echo '      <transport_descriptor>'
      echo '        <transport_id>so101_udp</transport_id>'
      echo '        <type>UDPv4</type>'
      echo "        <interfaceWhiteList><address>${addr}</address></interfaceWhiteList>"
      echo '      </transport_descriptor>'
      echo '    </transport_descriptors>'
      echo '    <participant profile_name="so101_wsl" is_default_profile="true">'
      echo '      <rtps>'
      echo '        <userTransports><transport_id>so101_udp</transport_id></userTransports>'
      echo '        <useBuiltinTransports>false</useBuiltinTransports>'
      echo '        <builtin>'
      echo '          <avoid_builtin_multicast>true</avoid_builtin_multicast>'
      echo '          <initialPeersList>'
      # participant id 0..39 의 유니캐스트 메타트래픽 포트를 직접 나열한다
      local i port
      for i in $(seq 0 39); do
        port=$((base + 10 + 2 * i))
        echo "            <locator><udpv4><address>${addr}</address><port>${port}</port></udpv4></locator>"
      done
      echo '          </initialPeersList>'
      echo '        </builtin>'
      echo '      </rtps>'
      echo '    </participant>'
      echo '  </profiles>'
      echo '</dds>'
    } > "$xml"
  fi

  export FASTRTPS_DEFAULT_PROFILES_FILE="$xml"
  unset FASTDDS_BUILTIN_TRANSPORTS    # XML 의 userTransports 와 충돌하지 않게
  echo "[dds] 127.0.0.1 이 죽어 있다(WSL mirrored). DDS 를 ${addr} 로 우회한다: $xml" >&2
  so101_daemon_check
  return 0
}

# ros2cli 데몬은 "자기가 실행될 때의 환경"을 그대로 들고 있다. 우회 설정 없이 먼저 떠 있으면
# 그 데몬은 아무 노드도 볼 수 없고, ros2 topic/node/control 조회가 전부 빈 결과를 낸다
# (ros2 control 은 --no-daemon 을 받지 않아서 우회할 수도 없다). 그래서 설정이 다르면 내린다.
# 다음 ros2 명령에서 지금 셸의 환경으로 다시 뜬다.
so101_daemon_check() {
  command -v ros2 >/dev/null 2>&1 || return 0
  local want="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
  local pid
  for pid in $(pgrep -f '_ros2_daemon' 2>/dev/null); do
    local have
    have="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null \
            | sed -n 's/^FASTRTPS_DEFAULT_PROFILES_FILE=//p')"
    if [ "$have" != "$want" ]; then
      echo "[dds] 다른 설정으로 떠 있던 ros2 데몬을 내린다 (조회가 빈 결과를 내는 원인)" >&2
      timeout 15 ros2 daemon stop >/dev/null 2>&1 || true
      return 0
    fi
  done
  return 0
}
