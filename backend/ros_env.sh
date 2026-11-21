#!/usr/bin/env bash
# Shared by the gateway and every managed ROS subprocess. Source this file.
export JETAUTO_WS="${JETAUTO_WS:-$HOME/ros2_ws}"
export THIRD_PARTY_WS="${THIRD_PARTY_WS:-$HOME/third_party/third_party_ws}"
export ORBBEC_WS="${ORBBEC_WS:-$HOME/third_party/orbbec_ws}"
_cockpit_domain="${ROS_DOMAIN_ID:-3}"
source /opt/ros/humble/setup.bash || return 1
# The vendor .robotrc sources *.zsh and MUST NOT be sourced by Bash.
# .typerc contains the hardware settings and is Bash-compatible.
if [ -f "$JETAUTO_WS/.typerc" ]; then
    source "$JETAUTO_WS/.typerc" >/dev/null || return 1
fi
for _cockpit_ws in "$THIRD_PARTY_WS" "$ORBBEC_WS" "$JETAUTO_WS"; do
    if [ -f "$_cockpit_ws/install/setup.bash" ]; then
        source "$_cockpit_ws/install/setup.bash" || return 1
    fi
done
export ROS_DOMAIN_ID="$_cockpit_domain"
export MACHINE_TYPE="${MACHINE_TYPE:-JetAutoPro}"
export LIDAR_TYPE="${LIDAR_TYPE:-G4}"
export need_compile="${need_compile:-False}"
export MASTER="${MASTER:-/}" HOST="${HOST:-/}"
unset _cockpit_domain _cockpit_ws
