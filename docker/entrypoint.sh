#!/bin/bash
# Keep the container running: sshd in the foreground supervision loop,
# and the robot server beside it. A stopped server does not stop the container.
set -euo pipefail

LEO_USER=leo
LEO_HOME=/home/leo
SSH_PORT="${SSH_PORT:-2222}"

if [[ ! "$SSH_PORT" =~ ^[0-9]+$ ]]; then
  echo "leo-entrypoint: SSH_PORT must be a number, got ${SSH_PORT}" >&2
  exit 1
fi
sed -i "s/^Port .*/Port ${SSH_PORT}/" /etc/ssh/sshd_config.d/leo.conf

mkdir -p /var/run/sshd /opt/leo/config /opt/leo/data /opt/leo/logs/ros
if [[ ! -f /opt/leo/config/robot.yaml ]]; then
  cp /opt/leo-seed/robot.yaml /opt/leo/config/robot.yaml
fi
chown -R "$LEO_USER:$LEO_USER" /opt/leo/config /opt/leo/data /opt/leo/logs

# The checkout is the host repository. colcon output lives in that tree and is gitignored.
if [[ ! -f /opt/leo/src/server/install/setup.bash ]]; then
  runuser -u "$LEO_USER" -- \
    env HOME="$LEO_HOME" USER="$LEO_USER" LOGNAME="$LEO_USER" \
    bash -lc 'source /opt/ros/humble/setup.bash && cd /opt/leo/src/server && colcon build --symlink-install' \
    >> /opt/leo/logs/server.log 2>&1
fi

install -d -m 700 -o "$LEO_USER" -g "$LEO_USER" "$LEO_HOME/.ssh"
if [[ -d /mnt/host-ssh ]]; then
  if [[ -f /mnt/host-ssh/id_rsa ]]; then
    cp /mnt/host-ssh/id_rsa "$LEO_HOME/.ssh/id_rsa"
  fi
  if [[ -f /mnt/host-ssh/id_rsa.pub ]]; then
    cp /mnt/host-ssh/id_rsa.pub "$LEO_HOME/.ssh/id_rsa.pub"
  fi
  if [[ -f /mnt/host-ssh/authorized_keys ]]; then
    cp /mnt/host-ssh/authorized_keys "$LEO_HOME/.ssh/authorized_keys"
  fi
  if [[ -f "$LEO_HOME/.ssh/id_rsa.pub" ]]; then
    touch "$LEO_HOME/.ssh/authorized_keys"
    if ! grep -qxF "$(cat "$LEO_HOME/.ssh/id_rsa.pub")" "$LEO_HOME/.ssh/authorized_keys"; then
      cat "$LEO_HOME/.ssh/id_rsa.pub" >> "$LEO_HOME/.ssh/authorized_keys"
    fi
  fi
  chown -R "$LEO_USER:$LEO_USER" "$LEO_HOME/.ssh"
  chmod 700 "$LEO_HOME/.ssh"
  [[ -f "$LEO_HOME/.ssh/id_rsa" ]] && chmod 600 "$LEO_HOME/.ssh/id_rsa"
  [[ -f "$LEO_HOME/.ssh/authorized_keys" ]] && chmod 600 "$LEO_HOME/.ssh/authorized_keys"
  [[ -f "$LEO_HOME/.ssh/id_rsa.pub" ]] && chmod 644 "$LEO_HOME/.ssh/id_rsa.pub"
else
  echo "leo-entrypoint: /mnt/host-ssh is not mounted, SSH keys were not copied" >&2
fi

if [[ ! -s "$LEO_HOME/.ssh/authorized_keys" ]]; then
  echo "leo-entrypoint: no authorized_keys; key login will fail until keys are mounted" >&2
fi

if [[ ! -f /etc/ssh/ssh_host_ed25519_key ]]; then
  ssh-keygen -A
fi

# Own process groups so a stop signal reaches sshd, ros2 launch, and the nodes.
set -m

sshd_pid=""
server_pid=""

stop_group() {
  local pid="$1"
  if [[ -z "$pid" ]]; then
    return 0
  fi
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

shutdown() {
  trap '' TERM INT
  stop_group "$server_pid"
  stop_group "$sshd_pid"
  sleep 2
  stop_group "$server_pid"
  wait || true
  exit 0
}
trap shutdown TERM INT

start_sshd() {
  /usr/sbin/sshd -D -e >> /opt/leo/logs/sshd.log 2>&1 &
  sshd_pid=$!
}

start_server() {
  if [[ "${LEO_AUTOSTART:-1}" == "0" ]]; then
    server_pid=""
    return 0
  fi
  runuser -u "$LEO_USER" -- \
    env HOME="$LEO_HOME" USER="$LEO_USER" LOGNAME="$LEO_USER" \
    LEO_CONFIG=/opt/leo/config/robot.yaml \
    ROS_LOG_DIR=/opt/leo/logs/ros \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    LEO_SIM="${LEO_SIM:-}" \
    bash /usr/local/bin/leo-start-server.sh >> /opt/leo/logs/server.log 2>&1 &
  server_pid=$!
}

start_sshd
start_server

set +e
while true; do
  if [[ -z "$sshd_pid" ]] || ! kill -0 "$sshd_pid" 2>/dev/null; then
    echo "leo-entrypoint: sshd is not running, starting it" >> /opt/leo/logs/sshd.log
    start_sshd
  fi
  if [[ "${LEO_AUTOSTART:-1}" != "0" ]]; then
    if [[ -z "$server_pid" ]] || ! kill -0 "$server_pid" 2>/dev/null; then
      echo "leo-entrypoint: server is not running, starting it" >> /opt/leo/logs/server.log
      start_server
    fi
  fi
  sleep 5 &
  wait $! || true
done
