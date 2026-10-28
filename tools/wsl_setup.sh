#!/usr/bin/env bash
# One-time setup of ROS 2 + Gazebo inside WSL Ubuntu 26.04. Run as root from Windows:
#   wsl -d Ubuntu -u root -e bash tools/wsl_setup.sh check     # add the ROS apt source, show what's available
#   wsl -d Ubuntu -u root -e bash tools/wsl_setup.sh install   # install everything the sim needs
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
. /etc/os-release
CODENAME=${UBUNTU_CODENAME:-$VERSION_CODENAME}
DISTRO=${ROS_DISTRO_WANTED:-lyrical}

PKGS=(
  ros-$DISTRO-ros-base ros-$DISTRO-ros-gz ros-$DISTRO-xacro ros-$DISTRO-robot-state-publisher
  ros-$DISTRO-robot-localization ros-$DISTRO-tf2-ros ros-$DISTRO-tf2-geometry-msgs ros-$DISTRO-rosbag2
  ros-$DISTRO-launch-testing-ament-cmake ros-$DISTRO-ament-cmake-pytest
  python3-colcon-common-extensions python3-rosdep python3-numpy python3-shapely python3-matplotlib python3-yaml
  rsync ffmpeg
)

add_source() {
  apt-get update -qq
  apt-get install -y -qq curl ca-certificates gnupg software-properties-common >/dev/null
  add-apt-repository -y universe >/dev/null
  if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -sSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${CODENAME}_all.deb"
    dpkg -i /tmp/ros2-apt-source.deb
  fi
  apt-get update -qq
}

case "${1:-check}" in
  check)
    add_source
    echo "Ubuntu $VERSION_ID ($CODENAME), ROS distro wanted: $DISTRO"
    for p in "${PKGS[@]}" ros-$DISTRO-fields2cover ros-$DISTRO-desktop; do
      printf '%-44s %s\n' "$p" "$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate/ {print $2}')"
    done
    ;;
  install)
    add_source
    apt-get install -y "${PKGS[@]}"
    if apt-cache policy ros-$DISTRO-fields2cover | grep -q 'Candidate: [0-9]'; then
      apt-get install -y ros-$DISTRO-fields2cover
    fi
    echo "INSTALL DONE"
    ;;
esac
