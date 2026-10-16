#!/bin/bash
# Runs inside the systemd container as root.
set -u
R=/repo
mkdir -p /usr/local/lib/qb3rt /opt/qb3rt/install/share/QB3rt/launch /etc/profile.d
install -m0644 $R/system/bin/qb3rt-common.sh /usr/local/lib/qb3rt/
install -m0755 $R/system/bin/qb3rt-stack-exec $R/system/bin/qb3rt-wait-net /usr/local/lib/qb3rt/
install -m0755 $R/system/bin/qb3rt-up $R/system/bin/qb3rt-down $R/system/bin/qb3rt-status /usr/local/bin/
install -m0644 $R/system/qb3rt-stack@.service /etc/systemd/system/
cp -r $R/scripts /opt/qb3rt/install/share/QB3rt/
cp /repo/tests/systemd/test.launch.py /opt/qb3rt/install/share/QB3rt/launch/
mkdir -p /opt/qb3rt/install/share/ament_index/resource_index/packages && touch /opt/qb3rt/install/share/ament_index/resource_index/packages/QB3rt
mkdir -p /opt/qb3rt/install/share/QB3rt && cp $R/package.xml /opt/qb3rt/install/share/QB3rt/
printf 'source /opt/ros/jazzy/setup.bash\nexport AMENT_PREFIX_PATH=/opt/qb3rt/install:$AMENT_PREFIX_PATH\n' > /etc/profile.d/qb3rt-ros-env.sh
systemctl daemon-reload
chown -R ubuntu /home/ubuntu
echo "== 1. qb3rt-up test"
sudo -u ubuntu qb3rt-up test; sleep 8
sudo -u ubuntu qb3rt-status; echo "status rc=$?"
echo "procs in unit: $(systemctl show -p TasksCurrent --value qb3rt-stack@test)"
echo "== 2. qb3rt-down (clean)"
t0=$(date +%s.%N); sudo -u ubuntu qb3rt-down; t1=$(date +%s.%N)
echo "stop took $(echo "$t1 - $t0" | bc 2>/dev/null || python3 -c "print(round($t1-$t0,2))")s"
journalctl -u qb3rt-stack@test --no-pager | grep -E "Traceback|SIGKILL|SIGTERM|Killing|escalat|exited|Deactivated|Stopped" | tail -8
pgrep -af "/opt/qb3rt|static_transform" || echo "no stack processes left"
echo "== 3. orphan from a dead SSH-style session"
sudo -u ubuntu bash -c 'source /etc/profile.d/qb3rt-ros-env.sh; setsid ros2 launch QB3rt test.launch.py >/tmp/orph.log 2>&1 & echo $! > /tmp/orph.pid'; sleep 8
kill -HUP "$(pgrep -f 'ros2 launch QB3rt test' | head -1)"; sleep 3
echo "after SIGHUP to launch:"; pgrep -af "/opt/qb3rt|static_transform|ros2 launch" | cut -c1-120
sudo -u ubuntu qb3rt-status; echo "status rc=$?"
sudo -u ubuntu qb3rt-down --force; echo "down rc=$?"
pgrep -af "/opt/qb3rt|static_transform|ros2 launch" || echo "no stack processes left"
echo "== 4. qb3rt-up while orphans exist + restart-on-failure"
sudo -u ubuntu bash -c 'source /etc/profile.d/qb3rt-ros-env.sh; setsid ros2 launch QB3rt test.launch.py >/tmp/orph2.log 2>&1 &'; sleep 6
kill -9 "$(pgrep -f 'ros2 launch QB3rt test' | head -1)"; sleep 1
sudo -u ubuntu qb3rt-up test; sleep 6; sudo -u ubuntu qb3rt-status; echo "status rc=$?"
sudo -u ubuntu qb3rt-down; pgrep -af "/opt/qb3rt|static_transform|ros2 launch" || echo "no stack processes left"
