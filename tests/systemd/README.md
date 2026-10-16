# qb3rt-stack@.service check (systemd in Docker, no hardware)

```bash
docker build -t qb3rt-sysd tests/systemd
docker run -d --name qbsd --privileged --cgroupns=host \
  -v /sys/fs/cgroup:/sys/fs/cgroup:rw -v "$PWD":/repo:ro qb3rt-sysd
docker exec qbsd bash /repo/tests/systemd/run.sh; docker rm -f qbsd
```

It installs the unit and helpers, then checks:
`qb3rt-up` / `qb3rt-status` / clean `qb3rt-down`; that a SIGHUP'd `ros2 launch`
(an SSH drop) leaves orphans, which `qb3rt-status` reports and
`qb3rt-down --force` reaps; and that `qb3rt-up` reaps orphans before starting.
Expect `no stack processes left` after each stop.
