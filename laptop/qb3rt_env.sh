# QB3rt laptop ROS environment - source this in every shell that talks to the
# robot:   source ~/qb3rt_laptop/qb3rt_env.sh
#
# Must match the robot's DDS setup (/root/rover_env.sh on the RB3): same
# domain, same RMW, and a CycloneDDS config that unicast-peers with the robot
# (the AGV WiFi link has multicast disabled).

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/brian/cyclonedds.xml

source /opt/ros/jazzy/setup.bash
