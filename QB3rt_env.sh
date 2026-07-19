# Set HOME variable
export HOME=/opt

# Remount the /usr directory with read-write permissions
mount -o remount,rw /usr

# Set up the runtime environment
source /usr/share/qirp-setup.sh -m

# Set the ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Use CycloneDDS with the AGV network config (deployed from this repo)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/cyclonedds.xml
