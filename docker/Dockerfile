# syntax=docker/dockerfile:1
# Ubuntu 22.04 + ROS 2 Humble, matching Leosha/README.md:
#   apt: ros-humble-desktop, ros-humble-slam-toolbox, python3-pip, python3-opencv
#   pip into the interpreter behind /usr/bin/python3 (what `ros2` uses)
#   colcon build, then `ros2 launch leo_server leo.launch.py`

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    LEO_ROOT=/opt/leo \
    LEO_CONFIG=/opt/leo/config/robot.yaml

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        locales \
        ca-certificates \
        curl \
        gnupg \
        software-properties-common \
    && locale-gen en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux \
    && ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | head -1 | cut -d'"' -f4)" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y \
        ros-humble-desktop \
        ros-humble-slam-toolbox \
        python3-pip \
        python3-opencv \
        python3-colcon-common-extensions \
        python3-dev \
        build-essential \
        openssh-server \
        zsh \
        git \
        sudo \
        tini \
        command-not-found \
        vim \
        less \
        iproute2 \
        iputils-ping \
        procps \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /var/run/sshd

RUN groupadd -g 1029 leo \
    && useradd -m -u 1028 -g leo -s /bin/zsh leo \
    && echo 'leo ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/leo \
    && chmod 440 /etc/sudoers.d/leo \
    && mkdir -p /home/leo/.cache \
    && chown leo:leo /home/leo/.cache

USER leo
RUN git clone --depth=1 https://github.com/ohmyzsh/ohmyzsh.git /home/leo/.oh-my-zsh \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git /home/leo/.oh-my-zsh/custom/themes/powerlevel10k \
    && git clone --depth=1 https://github.com/z-shell/F-Sy-H.git /home/leo/.oh-my-zsh/custom/plugins/F-Sy-H \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions /home/leo/.oh-my-zsh/custom/plugins/zsh-autosuggestions \
    && git clone --depth=1 https://github.com/zsh-users/zsh-history-substring-search /home/leo/.oh-my-zsh/custom/plugins/zsh-history-substring-search
USER root

COPY docker/constraints.txt /tmp/constraints.txt
# Debian sympy/numpy are distutils installs, so pip cannot uninstall them.
# --ignore-installed writes the new copy under /usr/local. Torch is the CPU wheel:
# the PyPI wheel pulls the CUDA 13 stack. The +cpu build is only on the PyTorch index,
# so later installs keep that index and the frozen pin.
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir --ignore-installed \
        -c /tmp/constraints.txt \
        'torch==2.4.1' 'torchvision==0.19.1' \
        --index-url https://download.pytorch.org/whl/cpu
RUN python3 -m pip freeze | grep -E '^(torch|torchvision)==' >> /tmp/constraints.txt \
    && python3 -m pip install --no-cache-dir --ignore-installed \
        -c /tmp/constraints.txt \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        'numpy>=1.21,<1.25' \
        'msgpack>=1.0' \
        'pyzmq>=22.0' \
        'PyYAML>=5.4' \
        'Flask>=2.2' \
        'scipy>=1.8' \
        'opencv-python-headless>=4.5,<4.11' \
        'pygame>=2.5' \
        'pytest>=7.0,<8' \
        'ultralytics>=8.0' \
        'onnxruntime>=1.16' \
    && python3 -m pip install --no-cache-dir --ignore-installed \
        -c /tmp/constraints.txt \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        'numpy>=1.21,<1.25' \
    && rm -rf /root/.cache/pip

COPY . /opt/leo
RUN python3 -m pip install --no-cache-dir --no-deps \
        -e /opt/leo/src/shared \
        -e /opt/leo/src/client \
        -e /opt/leo/src/server \
    && bash -c 'source /opt/ros/humble/setup.bash && cd /opt/leo/src/server && colcon build --symlink-install' \
    && rm -rf /opt/leo/src/server/log /root/.cache/pip \
    && mkdir -p /opt/leo-seed \
    && cp /opt/leo/config/robot.yaml /opt/leo-seed/robot.yaml

COPY docker/zshrc /home/leo/.zshrc
COPY docker/p10k.zsh /home/leo/.p10k.zsh
COPY docker/sshd.conf /etc/ssh/sshd_config.d/leo.conf
COPY docker/entrypoint.sh /usr/local/bin/leo-entrypoint.sh
COPY docker/start-server.sh /usr/local/bin/leo-start-server.sh
RUN chmod 755 /usr/local/bin/leo-entrypoint.sh /usr/local/bin/leo-start-server.sh \
    && chown leo:leo /home/leo/.zshrc /home/leo/.p10k.zsh \
    && ssh-keygen -A

WORKDIR /opt/leo
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/leo-entrypoint.sh"]
