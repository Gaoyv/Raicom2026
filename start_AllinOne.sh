#!/usr/bin/env bash
# 启动/管理 RAICOM2026 灵犀X2 仿真(sim_mujoco) + 运动控制(mc)
#
# 关键点：start_sim.sh / em_run.sh 自带 pgrep 单实例检测，而容器是 --pid=host，
# 直接把脚本名写进启动命令会被它的 pgrep 误判成“已在运行”。
# 这里改用名字“不含被检测字符串”的中转启动器 + exec，复刻手动方式的进程树，绕开误判。
#
# 用法（必须显式指定子命令，无默认动作）:
#   ./start_AllinOne.sh start    # 创建/复用容器并启动 仿真+mc，自动切 STAND
#   ./start_AllinOne.sh stop     # 停止容器
#   ./start_AllinOne.sh restart  # 停止容器后重新打开容器并启动（最干净的复位）
#   ./start_AllinOne.sh clean    # 删除容器

set -u

CONTAINER=x2_deploy
IMAGE=lingxi-x2-env:v1.0
WS=/home/agi/x2_deploy_workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 中转启动器（容器内路径）。注意名字里故意不含 start_sim.sh / em_run.sh
SIM_HELPER="$WS/.raicom/_sim_launcher.sh"
MC_HELPER="$WS/.raicom/_mc_launcher.sh"


usage() {
  cat >&2 <<USAGE
用法: $(basename "$0") <start|stop|restart|clean>
  start    创建/复用容器并启动 仿真(sim_mujoco)+运动控制(mc)，并自动切 STAND
  stop     停止容器
  restart  停止容器后重新打开容器并启动（清掉残留进程，最干净的复位）
  clean    删除容器
USAGE
}

# 在独立图形窗口里启动容器内的进程。
# bash -ic 走交互式 shell 以 source .bashrc 里的 ROS 环境（与手动操作一致）。
launch() {
  local title="$1" helper="$2"
  if command -v konsole >/dev/null 2>&1; then
    konsole --hold -p tabtitle="$title" -e docker exec -it "$CONTAINER" bash -ic "$helper" &
  elif command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- \
      bash -c "docker exec -it $CONTAINER bash -ic '$helper'; exec bash" &
  else
    echo "未找到图形终端，$title 转后台，日志: $SCRIPT_DIR/${title}.log"
    docker exec -d "$CONTAINER" bash -lc "$helper > $WS/${title}.log 2>&1"
  fi
}

# 创建/复用容器 → 起 仿真+mc → 自动切 STAND。restart 与 start 共用此流程。
do_start() {
  # ---------- 允许容器访问 X ----------
  xhost +local: >/dev/null 2>&1

  # ---------- 确保容器存在且在运行（幂等）----------
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
      echo "容器已在运行，直接复用。"
    else
      echo "容器已存在但未运行，正在启动..."
      docker start "$CONTAINER" >/dev/null
    fi
  else
    echo "容器不存在，正在创建..."
    docker run -it \
      --name="$CONTAINER" \
      --privileged \
      --net=host \
      --ipc=host \
      --pid=host \
      -e DISPLAY="$DISPLAY" \
      -v /dev/input:/dev/input \
      -v /tmp:/tmp \
      -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket:ro \
      -v "$SCRIPT_DIR":"$WS" \
      -d "$IMAGE" >/dev/null
  fi

  if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "错误：容器 $CONTAINER 未能保持运行，请查看 docker logs $CONTAINER" >&2
    exit 1
  fi

  # ---------- 生成中转启动器（每次覆盖，保证内容正确）----------
  # 这两个文件“内容”里有 start_sim.sh/em_run.sh 没关系，pgrep 匹配的是进程命令行而非文件内容。
  # 启动时所有上层包装进程的命令行只出现这两个 _xxx_launcher.sh 名字，不触发单实例检测。
  mkdir -p "$SCRIPT_DIR/.raicom"
  cat > "$SCRIPT_DIR/.raicom/_sim_launcher.sh" <<EOF
#!/usr/bin/env bash
cd $WS/sim_mujoco/bin || exit 1
exec ./start_sim.sh
EOF
  cat > "$SCRIPT_DIR/.raicom/_mc_launcher.sh" <<EOF
#!/usr/bin/env bash
cd $WS/mc/bin || exit 1
exec ./em_run.sh
EOF
  chmod +x "$SCRIPT_DIR/.raicom/"*.sh

  # ---------- 在独立窗口里启动 仿真 与 mc ----------
  echo "启动仿真 (sim_mujoco)..."
  launch "x2-sim" "$SIM_HELPER"

  echo "等待仿真初始化..."
  sleep 6

  echo "启动运动控制 (mc)..."
  launch "x2-mc" "$MC_HELPER"

  # ---------- 等 mc 就绪后自动切 STAND(SD)，让机器人一开始就处于站立平衡 ----------
  # set_mode.py SD 成功返回 0；mc 的 SetMcAction 服务起来前会失败，循环重试。
  echo "等待 mc 就绪并自动切 STAND(SD)..."
  local SD_OK=0
  for i in $(seq 1 20); do
    if docker exec "$CONTAINER" bash -ic "cd $WS/example/py && python3 set_mode.py SD" >/dev/null 2>&1; then
      SD_OK=1
      break
    fi
    sleep 1
  done
  if [ "$SD_OK" = 1 ]; then
    echo "✓ 已切 STAND_DEFAULT：机器人应在出发区站立（若 spawn 时倒地，按一次仿真 Reset 即可立即站起）。"
  else
    echo "⚠ 自动切 SD 未成功（mc 可能尚未就绪）。可手动执行：" \
         "docker exec -it $CONTAINER bash -ic 'cd $WS/example/py && python3 set_mode.py SD'"
  fi

  echo
  echo "完成：仿真与 mc 已分别在独立窗口启动。"
  echo "跑任务1：./run.sh usercode/py/walk_forward_steps.py （记得先在仿真里 Reset）"
  echo "若提示 already running：./$(basename "$0") restart 后重试。"
}


# ---------- 子命令分发（必须显式指定，无默认动作）----------
case "${1:-}" in
  start)
    do_start
    ;;
  stop)
    docker stop "$CONTAINER" 2>/dev/null && echo "已停止 $CONTAINER" || echo "$CONTAINER 未在运行"
    ;;
  restart)
    echo "重启：先停止容器..."
    docker stop "$CONTAINER" 2>/dev/null && echo "已停止 $CONTAINER" || echo "$CONTAINER 未在运行"
    do_start
    ;;
  clean|reset)
    docker rm -f "$CONTAINER" 2>/dev/null && echo "已删除 $CONTAINER" || echo "$CONTAINER 不存在"
    ;;
  "")
    echo "错误：必须显式指定子命令（无默认动作）。" >&2
    usage
    exit 1
    ;;
  *)
    echo "错误：未知子命令 '$1'。" >&2
    usage
    exit 1
    ;;
esac
