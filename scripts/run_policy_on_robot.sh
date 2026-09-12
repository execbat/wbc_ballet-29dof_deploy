#!/usr/bin/env bash
# One-command setup/check/build/run launcher for the BALLET policy on Unitree G1.
#
# Typical use on the robot:
#   git pull --ff-only
#   ./scripts/run_policy_on_robot.sh --arm
#
# --arm is intentionally explicit. Without it the ROS node starts, but the policy
# remains disabled until /ballet/enable receives std_msgs/msg/Bool {data: true}.

set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CACHE_ROOT="${BALLET_RUNTIME_CACHE:-$HOME/.cache/wbc_ballet_deploy}"
ROS_WS="$CACHE_ROOT/ros_ws"
TOOLS_VENV="$CACHE_ROOT/tools_venv"
POLICY_PATH="${BALLET_POLICY_PATH:-$REPO_ROOT/policy/policy.onnx}"
CONFIG_PATH="$REPO_ROOT/config/ballet_policy.yaml"
ORT_VERSION="${BALLET_ORT_VERSION:-1.23.2}"
UDP_PORT="${BALLET_UDP_PORT:-55001}"
ARM=0
SKIP_APT=0
NO_BUILD=0
CHECK_ONLY=0

TOTAL_STAGES=11
STAGE=0

if [[ -t 1 ]]; then
  BOLD='\033[1m'; GREEN='\033[32m'; YELLOW='\033[33m'; RED='\033[31m'; BLUE='\033[34m'; RESET='\033[0m'
else
  BOLD=''; GREEN=''; YELLOW=''; RED=''; BLUE=''; RESET=''
fi

stage() {
  STAGE=$((STAGE + 1))
  printf '\n%b[%02d/%02d] %s%b\n' "$BOLD$BLUE" "$STAGE" "$TOTAL_STAGES" "$1" "$RESET"
}
info() { printf '%bINFO:%b %s\n' "$GREEN" "$RESET" "$*"; }
warn() { printf '%bWARN:%b %s\n' "$YELLOW" "$RESET" "$*"; }
die() { printf '%bERROR:%b %s\n' "$RED" "$RESET" "$*" >&2; exit 1; }

usage() {
  cat <<'TXT'
Usage: ./scripts/run_policy_on_robot.sh [options]

Options:
  --arm          After all checks, automatically enable /ballet/enable.
                 Requires a valid game_emulator_run_v1.py UDP stream already
                 arriving on port 55001 before the node is started.
  --check-only   Run setup + all preflight checks, but do not launch the node.
  --no-build     Reuse an existing launcher workspace build.
  --skip-apt     Do not attempt apt installation of missing basic build tools.
  -h, --help     Show this help.

Environment overrides:
  BALLET_POLICY_PATH       ONNX path (default: <repo>/policy/policy.onnx)
  BALLET_UDP_PORT          command UDP port (default: 55001)
  BALLET_RUNTIME_CACHE     build/tool cache (default: ~/.cache/wbc_ballet_deploy)
  BALLET_ORT_VERSION       ONNX Runtime version (default: 1.23.2)
  ONNXRUNTIME_ROOT         use an existing ONNX Runtime C/C++ distribution
  UNITREE_ROS2_WORKSPACE   hint to the Unitree ROS2 workspace root
  ROS_DISTRO               hint to the ROS2 distro under /opt/ros
TXT
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm) ARM=1 ;;
    --check-only) CHECK_ONLY=1 ;;
    --no-build) NO_BUILD=1 ;;
    --skip-apt) SKIP_APT=1 ;;
    -h|--help) usage; exit 0 ;;
    *) die "Unknown argument: $1 (use --help)" ;;
  esac
  shift
done

run_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    return 127
  fi
}

install_basic_tools() {
  local -a packages=()
  command -v git >/dev/null 2>&1 || packages+=(git)
  command -v cmake >/dev/null 2>&1 || packages+=(cmake)
  command -v g++ >/dev/null 2>&1 || packages+=(build-essential)
  command -v curl >/dev/null 2>&1 || packages+=(curl)
  command -v tar >/dev/null 2>&1 || packages+=(tar)
  command -v python3 >/dev/null 2>&1 || packages+=(python3)
  command -v colcon >/dev/null 2>&1 || packages+=(python3-colcon-common-extensions)

  # Needed for the isolated Python ONNX checker venv.
  if command -v python3 >/dev/null 2>&1 && ! python3 -m venv --help >/dev/null 2>&1; then
    packages+=(python3-venv)
  fi

  if [[ ${#packages[@]} -eq 0 ]]; then
    info "Basic build tools are already installed."
    return
  fi

  if [[ "$SKIP_APT" -eq 1 ]]; then
    die "Missing tools require packages: ${packages[*]}; --skip-apt was requested."
  fi
  command -v apt-get >/dev/null 2>&1 || die "Missing tools (${packages[*]}) and apt-get is unavailable."
  info "Installing missing packages with apt: ${packages[*]}"
  run_sudo apt-get update || die "Cannot run apt-get update (sudo/root required)."
  run_sudo apt-get install -y "${packages[@]}" || die "apt installation failed."
}

collect_ros_setups() {
  local preferred=""
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    preferred="/opt/ros/$ROS_DISTRO/setup.bash"
    printf '%s\n' "$preferred"
  fi
  local f
  for f in /opt/ros/*/setup.bash; do
    [[ -f "$f" ]] || continue
    [[ "$f" == "$preferred" ]] && continue
    printf '%s\n' "$f"
  done
}

collect_unitree_setups() {
  local -a candidates=()
  local ws="${UNITREE_ROS2_WORKSPACE:-}"
  [[ -n "$ws" && -f "$ws/install/setup.bash" ]] && candidates+=("$ws/install/setup.bash")
  [[ -n "$ws" && -f "$ws/setup.bash" ]] && candidates+=("$ws/setup.bash")

  local f
  for f in \
      "$HOME/unitree_ros2/install/setup.bash" \
      "$HOME/unitree_ros2_ws/install/setup.bash" \
      "$HOME/ros2_ws/install/setup.bash" \
      "$HOME/unitree_ws/install/setup.bash" \
      /opt/unitree_ros2/install/setup.bash \
      /opt/unitree/install/setup.bash; do
    [[ -f "$f" ]] && candidates+=("$f")
  done

  local marker prefix
  while IFS= read -r marker; do
    [[ -n "$marker" ]] || continue
    prefix="${marker%/share/ament_index/resource_index/packages/unitree_hg}"
    [[ -f "$prefix/setup.bash" ]] && candidates+=("$prefix/setup.bash")
  done < <(
    {
      find "$HOME" -maxdepth 7 -type f -path '*/install/share/ament_index/resource_index/packages/unitree_hg' 2>/dev/null || true
      find /opt -maxdepth 7 -type f -path '*/install/share/ament_index/resource_index/packages/unitree_hg' 2>/dev/null || true
    } | head -50
  )

  # Unique, preserving order.
  printf '%s\n' "${candidates[@]}" | awk 'NF && !seen[$0]++'
}

ROS_SETUP=""
UNITREE_SETUP=""
UNITREE_PREFIX=""
DETECTED_ROS_DISTRO=""

detect_ros_and_unitree() {
  local -a ros_setups=()
  local -a unitree_setups=()
  mapfile -t ros_setups < <(collect_ros_setups)
  [[ ${#ros_setups[@]} -gt 0 ]] || die "No ROS2 installation found under /opt/ros. ROS2 must exist on the robot."
  mapfile -t unitree_setups < <(collect_unitree_setups)

  local r u
  # First try each ROS distro by itself; unitree_hg may be installed directly into that prefix.
  for r in "${ros_setups[@]}"; do
    if bash -c 'source "$1" >/dev/null 2>&1 && ros2 pkg prefix unitree_hg >/dev/null 2>&1' _ "$r"; then
      ROS_SETUP="$r"
      UNITREE_SETUP=""
      break
    fi
  done

  # Then try discovered Unitree workspaces on top of each ROS distro.
  if [[ -z "$ROS_SETUP" ]]; then
    for r in "${ros_setups[@]}"; do
      for u in "${unitree_setups[@]}"; do
        if bash -c 'source "$1" >/dev/null 2>&1 && source "$2" >/dev/null 2>&1 && ros2 pkg prefix unitree_hg >/dev/null 2>&1' _ "$r" "$u"; then
          ROS_SETUP="$r"
          UNITREE_SETUP="$u"
          break 2
        fi
      done
    done
  fi

  [[ -n "$ROS_SETUP" ]] || die "Found ROS2, but could not find/source a workspace that provides unitree_hg. Set UNITREE_ROS2_WORKSPACE if it is in a non-standard location."

  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ -n "$UNITREE_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$UNITREE_SETUP"
  fi

  DETECTED_ROS_DISTRO="$(basename "$(dirname "$ROS_SETUP")")"
  UNITREE_PREFIX="$(ros2 pkg prefix unitree_hg 2>/dev/null || true)"
  [[ -n "$UNITREE_PREFIX" ]] || die "unitree_hg disappeared after environment activation."

  info "Detected ROS_DISTRO=$DETECTED_ROS_DISTRO"
  info "ROS setup: $ROS_SETUP"
  if [[ -n "$UNITREE_SETUP" ]]; then
    info "Detected Unitree setup: $UNITREE_SETUP"
    local install_dir
    install_dir="$(dirname "$UNITREE_SETUP")"
    if [[ "$(basename "$install_dir")" == "install" ]]; then
      info "Detected UNITREE_ROS2_WORKSPACE=$(dirname "$install_dir")"
    else
      info "Unitree prefix/setup directory: $install_dir"
    fi
  else
    info "unitree_hg is already visible from the ROS installation. Prefix: $UNITREE_PREFIX"
  fi
}

valid_ort_root() {
  local root="$1"
  [[ -n "$root" ]] || return 1
  [[ -f "$root/include/onnxruntime_cxx_api.h" ]] || return 1
  [[ -n "$(find "$root/lib" "$root/lib64" -maxdepth 1 -type f -name 'libonnxruntime.so*' 2>/dev/null | head -1)" ]] || return 1
}

setup_onnxruntime() {
  local -a candidates=()
  [[ -n "${ONNXRUNTIME_ROOT:-}" ]] && candidates+=("$ONNXRUNTIME_ROOT")
  candidates+=(
    "$REPO_ROOT/thirdparty/onnxruntime-linux-aarch64-$ORT_VERSION"
    "$REPO_ROOT/thirdparty/onnxruntime-linux-x64-$ORT_VERSION"
    "$CACHE_ROOT/onnxruntime-linux-aarch64-$ORT_VERSION"
    "$CACHE_ROOT/onnxruntime-linux-x64-$ORT_VERSION"
    "$HOME/third_party/onnxruntime"
    "$HOME/onnxruntime"
    "/opt/onnxruntime"
  )

  local c
  for c in "${candidates[@]}"; do
    if valid_ort_root "$c"; then
      ONNXRUNTIME_ROOT="$c"
      export ONNXRUNTIME_ROOT
      export LD_LIBRARY_PATH="$ONNXRUNTIME_ROOT/lib:$ONNXRUNTIME_ROOT/lib64:${LD_LIBRARY_PATH:-}"
      info "Using ONNX Runtime: $ONNXRUNTIME_ROOT"
      return
    fi
  done

  local machine asset_arch
  machine="$(uname -m)"
  case "$machine" in
    aarch64|arm64) asset_arch="aarch64" ;;
    x86_64|amd64) asset_arch="x64" ;;
    *) die "Unsupported CPU architecture for automatic ONNX Runtime install: $machine" ;;
  esac

  mkdir -p "$CACHE_ROOT"
  local folder="onnxruntime-linux-${asset_arch}-${ORT_VERSION}"
  local archive="$CACHE_ROOT/${folder}.tgz"
  local url="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${folder}.tgz"
  info "ONNX Runtime C++ not found. Downloading official ${ORT_VERSION} (${asset_arch})..."
  curl -fL --retry 3 --connect-timeout 15 "$url" -o "$archive" || \
    die "Could not download ONNX Runtime. Set ONNXRUNTIME_ROOT to an existing C/C++ distribution and retry."
  tar -xzf "$archive" -C "$CACHE_ROOT"
  ONNXRUNTIME_ROOT="$CACHE_ROOT/$folder"
  valid_ort_root "$ONNXRUNTIME_ROOT" || die "Downloaded ONNX Runtime archive is missing expected headers/library."
  export ONNXRUNTIME_ROOT
  export LD_LIBRARY_PATH="$ONNXRUNTIME_ROOT/lib:$ONNXRUNTIME_ROOT/lib64:${LD_LIBRARY_PATH:-}"
  info "Installed ONNX Runtime at: $ONNXRUNTIME_ROOT"
}

setup_python_checker() {
  mkdir -p "$CACHE_ROOT"
  if [[ ! -x "$TOOLS_VENV/bin/python" ]]; then
    info "Creating isolated checker venv: $TOOLS_VENV"
    rm -rf "$TOOLS_VENV"
    if ! python3 -m venv "$TOOLS_VENV"; then
      if [[ "$SKIP_APT" -eq 0 ]] && command -v apt-get >/dev/null 2>&1; then
        warn "python3 venv support is missing; installing python3-venv and retrying."
        run_sudo apt-get update || die "Cannot install python3-venv (sudo/root required)."
        run_sudo apt-get install -y python3-venv || die "Failed to install python3-venv."
        rm -rf "$TOOLS_VENV"
        python3 -m venv "$TOOLS_VENV" || die "Could not create Python venv after installing python3-venv."
      else
        die "Could not create Python venv. Install python3-venv or rerun without --skip-apt."
      fi
    fi
  fi
  if ! "$TOOLS_VENV/bin/python" -c 'import onnx' >/dev/null 2>&1; then
    info "Installing Python 'onnx' package into checker venv..."
    "$TOOLS_VENV/bin/python" -m pip install --upgrade pip >/dev/null
    "$TOOLS_VENV/bin/python" -m pip install 'onnx>=1.16,<2' || die "Failed to install Python onnx package."
  fi
}

build_package() {
  mkdir -p "$ROS_WS/src"
  ln -sfn "$REPO_ROOT" "$ROS_WS/src/wbc_ballet-29dof_deploy"

  if [[ "$NO_BUILD" -eq 0 ]]; then
    info "Building g1_ballet_onnx_deploy in launcher workspace: $ROS_WS"
    (
      cd "$ROS_WS"
      colcon build \
        --packages-select g1_ballet_onnx_deploy \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    )
  else
    info "Skipping build (--no-build)."
  fi

  [[ -f "$ROS_WS/install/setup.bash" ]] || die "Launcher workspace is not built: $ROS_WS/install/setup.bash missing."
  # shellcheck disable=SC1091
  source "$ROS_WS/install/setup.bash"
  ros2 pkg executables g1_ballet_onnx_deploy | grep -q 'g1_ballet_policy_node' || \
    die "Built package is not exposing g1_ballet_policy_node."
  info "Build/install environment activated."
}

check_lowstate() {
  local deadline=$((SECONDS + 15))
  info "Waiting up to 15 s for /lowstate..."
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -qx '/lowstate'; then
      break
    fi
    sleep 1
  done
  ros2 topic list 2>/dev/null | grep -qx '/lowstate' || \
    die "/lowstate is not visible. Fix Unitree DDS/network/debug environment before running motor control."

  local typ
  typ="$(ros2 topic type /lowstate 2>/dev/null | head -1 || true)"
  [[ "$typ" == "unitree_hg/msg/LowState" ]] || die "/lowstate type is '$typ', expected unitree_hg/msg/LowState."
  info "/lowstate type OK: $typ"

  local sample mode
  sample="$(timeout 6 ros2 topic echo /lowstate unitree_hg/msg/LowState --once 2>/dev/null || true)"
  [[ -n "$sample" ]] || die "Could not receive a /lowstate sample within 6 s."
  mode="$(printf '%s\n' "$sample" | awk '/^[[:space:]]*mode_machine:/ {print $2; exit}')"
  [[ -n "$mode" ]] || die "Could not read mode_machine from /lowstate."
  [[ "$mode" == "5" ]] || die "mode_machine=$mode, expected 5 for G1 29DoF low-level control."
  info "mode_machine=5 confirmed."
}

check_lowcmd_conflict() {
  local details count
  details="$(ros2 topic info /lowcmd 2>/dev/null || true)"
  count="$(printf '%s\n' "$details" | awk -F': ' '/Publisher count:/ {print $2; exit}')"
  count="${count:-0}"
  if [[ "$count" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    printf '%s\n' "$details"
    die "/lowcmd already has $count publisher(s). Stop the competing low-level controller first."
  fi
  info "No competing ROS2 /lowcmd publisher detected."
}

check_udp_port_free() {
  if command -v ss >/dev/null 2>&1 && [[ -n "$(ss -H -lun "sport = :${UDP_PORT}" 2>/dev/null || true)" ]]; then
    die "UDP port $UDP_PORT is already bound. Stop the existing process before launching."
  fi
  info "UDP port $UDP_PORT is free for g1_ballet_policy_node."
}

check_live_gamepad_udp() {
  info "--arm requested: verifying a live game_emulator_run_v1.py stream on UDP :$UDP_PORT..."
  python3 - "$UDP_PORT" <<'PY'
import socket, struct, sys, math
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", port))
s.settimeout(4.0)
try:
    data, addr = s.recvfrom(4096)
except socket.timeout:
    raise SystemExit(f"No UDP packet received on :{port} within 4 s")
finally:
    s.close()
if len(data) != 61 * 4:
    raise SystemExit(f"Bad v1 UDP packet from {addr}: {len(data)} bytes, expected 244")
vals = struct.unpack("<61f", data)
if not all(math.isfinite(x) for x in vals):
    raise SystemExit(f"UDP packet from {addr} contains NaN/Inf")
print(f"Valid BALLET v1 packet from {addr[0]}:{addr[1]}: 61 float32 / 244 bytes")
PY
}

schedule_arm() {
  (
    local i
    for i in $(seq 1 40); do
      if ros2 node list 2>/dev/null | grep -qx '/g1_ballet_onnx'; then
        sleep 0.5
        info "Policy node detected; publishing /ballet/enable=true once."
        ros2 topic pub --once /ballet/enable std_msgs/msg/Bool '{data: true}' >/dev/null
        exit 0
      fi
      sleep 0.25
    done
    printf '%bERROR:%b Policy node did not appear; automatic arm was not sent.\n' "$RED" "$RESET" >&2
  ) &
}

printf '%bUnitree G1 BALLET policy launcher%b\n' "$BOLD" "$RESET"
printf 'Repository: %s\n' "$REPO_ROOT"
printf 'Policy:     %s\n' "$POLICY_PATH"
printf 'Mode:       %s\n' "$([[ "$ARM" -eq 1 ]] && echo 'RUN + ARM' || echo 'RUN DISABLED')"

stage "Repository and policy preflight"
[[ -f "$REPO_ROOT/package.xml" && -f "$REPO_ROOT/src/g1_ballet_policy_node.cpp" ]] || die "Launcher is not inside the expected deploy repository."
[[ -f "$POLICY_PATH" ]] || die "Policy is missing: $POLICY_PATH. Commit policy/policy.onnx to this repository before deploying."
[[ -f "$CONFIG_PATH" ]] || die "Missing config: $CONFIG_PATH"
info "Repository layout and policy file found."

stage "Install/check basic build tools"
install_basic_tools

stage "Detect and activate ROS2 + Unitree ROS2 environment"
detect_ros_and_unitree

stage "Detect/install ONNX Runtime C++"
setup_onnxruntime

stage "Prepare isolated ONNX validation environment"
setup_python_checker

stage "Validate BALLET source I/O contract"
python3 "$REPO_ROOT/scripts/check_contract.py"
python3 "$REPO_ROOT/scripts/check_io_contract.py"

stage "Validate committed ONNX policy ABI"
"$TOOLS_VENV/bin/python" "$REPO_ROOT/scripts/check_onnx.py" "$POLICY_PATH"

stage "Build and activate deployment package"
build_package

stage "Check live G1 /lowstate and 29DoF mode"
check_lowstate

stage "Check /lowcmd ownership and BALLET UDP endpoint"
check_lowcmd_conflict
check_udp_port_free
printf 'Robot IPv4 addresses reachable by the laptop:\n'
ip -br -4 addr show scope global 2>/dev/null || true
if [[ "$ARM" -eq 1 ]]; then
  check_live_gamepad_udp
else
  warn "Node will start DISABLED. To execute the policy, restart with --arm after the v1 game emulator is streaming."
fi

stage "Launch g1_ballet_policy_node"
if [[ "$CHECK_ONLY" -eq 1 ]]; then
  info "CHECK-ONLY complete. Node was not started."
  exit 0
fi

printf '\n%bRuntime contract:%b\n' "$BOLD" "$RESET"
printf '  /lowstate [unitree_hg/msg/LowState] -> observations\n'
printf '  UDP :%s <- game_emulator_run_v1.py\n' "$UDP_PORT"
printf '  actor 186D -> ONNX -> action[29] -> motor_cmd[0..28].q\n'
printf '  /lowcmd [unitree_hg/msg/LowCmd] -> G1 low-level controller\n'
printf '  policy 50 Hz; /lowcmd 500 Hz\n\n'

if [[ "$ARM" -eq 1 ]]; then
  schedule_arm
  warn "ARM MODE: keep the robot supported and emergency stop available. Ctrl+C stops this node."
else
  info "Safe start: policy node is not armed."
fi

exec ros2 run g1_ballet_onnx_deploy g1_ballet_policy_node \
  --ros-args \
  --params-file "$CONFIG_PATH" \
  -p "policy_path:=$POLICY_PATH" \
  -p "udp_port:=$UDP_PORT"
