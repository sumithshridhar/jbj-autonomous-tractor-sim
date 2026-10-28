#!/usr/bin/env bash
# Build and run the sim inside WSL (normal user, not root). From the repo:
#   bash tools/sim.sh build
#   bash tools/sim.sh run <scenario> [tractor] [gui]     e.g. bash tools/sim.sh run cow mf_5m_145 false
#   bash tools/sim.sh all [tractor]                       every scenario, results copied to results/
set -o pipefail   # no -e: ROS setup.bash can return non-zero
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DST="${JBJ_DST:-$HOME/jbj}"   # JBJ_DST + GZ_PARTITION + ROS_DOMAIN_ID let two sims run side by side
export JBJ_ROOT="$DST"
source /opt/ros/lyrical/setup.bash

build() {
  mkdir -p "$DST"
  rsync -a --delete --exclude .git --exclude media --exclude 'blender/out' --exclude results --exclude 'ros2_ws/build' \
    --exclude 'ros2_ws/install' --exclude 'ros2_ws/log' "$SRC/" "$DST/"
  (cd "$DST/ros2_ws" && colcon build --packages-select jbj_sim 2>&1 | tail -3)
}

run() {
  local scen="$1" tid="${2:-}" gui="${3:-false}"
  source "$DST/ros2_ws/install/setup.bash"
  ros2 launch jbj_sim sim.launch.py scenario:="$scen" ${tid:+tractor:="$tid"} gui:="$gui" runs:="${JBJ_RUNS:-$HOME/jbj_runs}" 2>&1 | grep -E "\[jbj\]|scenario_runner|ERROR|Traceback|Error" || true
  local name="${scen}_${tid:-$(python3 -c "import json;print(json.load(open('$DST/specs/tractors.json'))['default'])")}"
  mkdir -p "$SRC/results/$name"
  cp "${JBJ_RUNS:-$HOME/jbj_runs}/$name/"{metrics.json,report.png,log.csv,plan.json} "$SRC/results/$name/" 2>/dev/null || true
}

case "$1" in
  build) build ;;
  run) shift; run "$@" ;;
  all) build; for s in remote_estop remote_lost rtk_loss steering_fault cow person full_field; do run "$s" "${2:-}"; done ;;
  fleet) build; for t in mf_7615 mf_8732; do for s in cow remote_estop; do run "$s" "$t"; done; done ;;
esac
