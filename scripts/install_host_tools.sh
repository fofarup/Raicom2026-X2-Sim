#!/usr/bin/env bash
set -euo pipefail

[[ "$(uname -s)" == "Linux" ]] || {
  echo "此脚本只能在 Ubuntu 22.04 中运行。" >&2
  exit 1
}

source /etc/os-release
[[ "${ID}" == "ubuntu" && "${VERSION_ID}" == "22.04" ]] || {
  echo "需要 Ubuntu 22.04，当前为 ${PRETTY_NAME}。" >&2
  exit 1
}

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "需要 x86_64，当前为 $(uname -m)。" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y ca-certificates curl git git-lfs tmux x11-xserver-utils

if ! command -v docker >/dev/null 2>&1; then
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  . /etc/os-release
  sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${UBUNTU_CODENAME:-$VERSION_CODENAME}
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

  sudo apt-get update
  sudo apt-get install -y \
    docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

sudo systemctl enable --now docker
sudo usermod -aG docker "${USER}"
git lfs install

echo
echo "宿主机工具安装完成。"
echo "重要：请注销 Ubuntu 账户并重新登录，使 docker 用户组权限生效。"
