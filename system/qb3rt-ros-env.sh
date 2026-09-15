# QB3rt ROS environment — installed to /etc/profile.d/qb3rt-ros-env.sh by
# reference/20_fleet_config.sh. Sourced by login shells on every RB3.
#
# Replaces the old QIRP QB3rt_env.sh. The Ubuntu image has a normal writable
# rootfs, so the old `mount -o remount,rw /usr` and `source qirp-setup.sh`
# lines are gone. ROS now comes from apt (/opt/ros) + the QIRP PPAs, with the
# custom packages layered from the colcon overlay.
#
# Per-unit ROS_DOMAIN_ID lives in /etc/qb3rt/domain_id (written by
# stamp/stamp_unit.sh); everything else here is fleet-wide.

# Base ROS (apt) + Qualcomm qrb_ros_* HW nodes on the ament prefix path.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Custom package overlay (ORB-SLAM3, depthai-ros, wave_rover_controller).
if [ -f /opt/qb3rt/install/setup.bash ]; then
    source /opt/qb3rt/install/setup.bash
fi

# Per-unit DDS domain (defaults to 0 pre-stamp).
if [ -r /etc/qb3rt/domain_id ]; then
    export ROS_DOMAIN_ID="$(cat /etc/qb3rt/domain_id)"
else
    export ROS_DOMAIN_ID=0
fi

# CycloneDDS with the AGV unicast/large-message tuning.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/qb3rt/cyclonedds.xml
