FROM public.ecr.aws/docker/library/ros:jazzy-ros-base
RUN apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq systemd systemd-sysv sudo iproute2 procps >/dev/null && \
    (id ubuntu || useradd -m ubuntu) && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-qb3rt && \
    rm -f /lib/systemd/system/multi-user.target.wants/* /etc/systemd/system/*.wants/* 2>/dev/null; true
CMD ["/sbin/init"]
