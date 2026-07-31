FROM ubuntu:22.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=agi
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}
ENV TZ=Asia/Shanghai
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DOMAIN_ID=26
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 与赛事参考 Dockerfile 保持一致，使用 Ubuntu 22.04 + ROS 2 Humble。
RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' \
    -e 's|http://security.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' \
    /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ca-certificates \
    cmake \
    curl \
    gdb \
    git \
    gnupg \
    less \
    locales \
    mesa-utils \
    nano \
    procps \
    python3-pip \
    software-properties-common \
    sudo \
    tmux \
    unzip \
    wget \
    x11-apps \
    && locale-gen en_US.UTF-8 \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    libacl1-dev \
    libgflags-dev \
    liboctomap-dev \
    liborocos-kdl-dev \
    libscrypt-dev \
    python3-argcomplete \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-ros \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    -i https://mirrors.aliyun.com/pypi/simple/ \
    anthropic

RUN groupadd --gid "${USER_GID}" "${USERNAME}" 2>/dev/null || true && \
    useradd --create-home --uid "${USER_UID}" --gid "${USER_GID}" \
      --shell /bin/bash "${USERNAME}" && \
    usermod -aG sudo "${USERNAME}" && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
      > "/etc/sudoers.d/${USERNAME}" && \
    chmod 0440 "/etc/sudoers.d/${USERNAME}" && \
    mkdir -p /agibot/data/var /workspace && \
    chown -R "${USERNAME}:${USERNAME}" /agibot /workspace

USER ${USERNAME}
WORKDIR /workspace

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"' \
    'export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"' \
    'if [ -f /workspace/.runtime/ros/install/setup.bash ]; then source /workspace/.runtime/ros/install/setup.bash; fi' \
    >> "/home/${USERNAME}/.bashrc"

CMD ["/bin/bash"]
