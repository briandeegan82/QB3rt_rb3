# shellcheck shell=bash
# Shared helpers for qb3rt-up / qb3rt-down / qb3rt-status (sourced, not run).

# Command lines that belong to the QB3rt stack: our launch files, anything
# from the /opt/qb3rt overlay (bridge, relays, ORB-SLAM3, RF2O) and the apt
# ROS nodes the launch files start.
QB3RT_PROC_RE='ros2 launch QB3rt|/opt/qb3rt/install/|/opt/ros/jazzy/lib/(rclcpp_components/component_container|rplidar_ros/|robot_localization/|robot_state_publisher/|joint_state_publisher/|tf2_ros/static_transform_publisher|slam_toolbox/|nav2_|joy/|teleop_twist_joy/)'

# Units that own stack processes: the stack service, add-on services started
# by `qb3rt-up --addon`, and the transient units the convention-probe
# playbook starts with systemd-run.
QB3RT_UNIT_GLOBS=('qb3rt-stack@*.service' 'qb3rt-addon@*.service' 'qb3rt-probe-*.service')

qb3rt_active_units() {
    systemctl list-units --plain --no-legend --state=active,activating,deactivating \
        "${QB3RT_UNIT_GLOBS[@]}" 2>/dev/null | awk '{print $1}'
}

# PIDs of stack processes that are NOT inside a systemd-managed stack unit,
# i.e. orphans of a `ros2 launch` started from a (now dead) SSH session.
# The caller itself and its ancestors are excluded: an argument like
# label_file:=/opt/qb3rt/install/... makes their cmdlines match
# QB3RT_PROC_RE, and a forked subshell keeps a copy of that cmdline.
qb3rt_stray_pids() {
    local pid cg self
    declare -A ours=()
    self=$BASHPID
    while [ "$self" -gt 1 ]; do
        ours[$self]=1
        self=$(awk '/^PPid:/{print $2}' "/proc/$self/status" 2>/dev/null)
        [ -n "$self" ] || break
    done
    for pid in $(pgrep -f -- "$QB3RT_PROC_RE"); do
        [ -n "${ours[$pid]:-}" ] && continue
        cg=$(cat "/proc/$pid/cgroup" 2>/dev/null) || continue
        case "$cg" in
            *qb3rt-stack@*|*qb3rt-addon@*|*qb3rt-probe-*) continue ;;
        esac
        echo "$pid"
    done
}

qb3rt_sudo() {
    if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo -n "$@"; fi
}
