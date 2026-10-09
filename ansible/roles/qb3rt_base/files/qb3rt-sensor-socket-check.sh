#!/bin/bash
# Self-heal for qcom sensor_service. If it starts before the SSC remoteproc is
# ready, its SUID discovery returns suid=0 and the daemon stays "running" but
# never creates the client socket — leaving /imu/data dead for the whole boot.
# Restart it when the socket is missing. Run from the qb3rt-sensor-watch timer.
set -u
systemctl is-active --quiet sensor-service.service || exit 0
[ -S /dev/shm/server_socket ] && exit 0
# Grace period: don't flap while a manual/foreground instance is mid-bringup.
sleep 5
[ -S /dev/shm/server_socket ] && exit 0
systemctl restart sensor-service.service
