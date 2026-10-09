#!/bin/bash
# The robot PC's resident process, started and stopped by name: `robot/serve.sh start|stop|status`.
# Run on the robot PC (the planning laptop runs it over ssh: tools/robot_pc.sh).  It is what the
# systemd unit (robot/aris-robot.service) does, for a PC where that unit is not installed: the
# three ROS environments, then the venv's `aris-robot --config <repo>/config serve`, detached
# from the terminal, its output in out/operator/launch.log.  One at a time: `start` refuses
# while one runs.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO/out/operator"
PID_FILE="$LOG_DIR/serve.pid"
CONFIG="${ARIS_CONFIG:-$REPO/config}"

running_pid() {
    local pid
    pid="$(cat "$PID_FILE" 2>/dev/null)" || return 1
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && grep -q "aris-robot" "/proc/$pid/cmdline" 2>/dev/null \
        && echo "$pid"
}

case "${1:-status}" in
start)
    if pid="$(running_pid)"; then
        echo "already running (pid $pid)"; exit 0
    fi
    # a runner started some other way (systemd, a terminal) would fight this one for the arms
    other="$(pgrep -f "bin/aris-robot .*serve" | head -1)"
    if [ -n "$other" ]; then
        echo "FAIL: another aris-robot serve runs (pid $other): stop that one first"; exit 1
    fi
    mkdir -p "$LOG_DIR"
    cd "$REPO" || exit 1
    setsid nohup bash -c "
        source /opt/ros/jazzy/setup.bash &&
        source \$HOME/ros2_ws/install/setup.bash &&
        source '$REPO/robot/ros2_ws/install/setup.bash' &&
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONUNBUFFERED=1 &&
        exec '$REPO/.venv/bin/aris-robot' --config '$CONFIG' serve
    " >> "$LOG_DIR/launch.log" 2>&1 < /dev/null &
    echo $! > "$PID_FILE"
    sleep 2
    if pid="$(running_pid)"; then
        echo "started (pid $pid), config $CONFIG; the stacks come up one at a time (a minute for six)"
    else
        echo "FAIL: it did not stay up; the end of $LOG_DIR/launch.log:"; tail -n 15 "$LOG_DIR/launch.log"; exit 1
    fi
    ;;
stop)
    if ! pid="$(running_pid)"; then
        echo "not running"; exit 0
    fi
    kill -TERM "$pid"
    for _ in $(seq 1 70); do                     # serve waits for its long-poll, then stops the stacks
        kill -0 "$pid" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "FAIL: pid $pid does not stop"; exit 1
    fi
    rm -f "$PID_FILE"
    echo "stopped"
    ;;
status)
    if pid="$(running_pid)"; then
        echo "running (pid $pid)"
    else
        echo "not running"
    fi
    ;;
*)
    echo "usage: robot/serve.sh start|stop|status"; exit 2
    ;;
esac
