#!/usr/bin/env bash
# 在 x2_deploy 容器里运行工作区内的脚本（自动 cd 到脚本所在目录 + source ROS 环境）
#
# 用法（路径相对于本脚本所在的工作区根目录）:
#   ./run.sh example/py/set_mode.py            # = cd example/py && python3 set_mode.py
#   ./run.sh example/py/set_mode.py --foo bar  # 额外参数透传给脚本
#   ./run.sh some/dir/foo.sh                    # .sh 用 bash 执行

set -u

CONTAINER=x2_deploy
WS=/home/agi/x2_deploy_workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ $# -lt 1 ]; then
  echo "用法: $(basename "$0") <相对工作区根目录的脚本路径> [脚本参数...]" >&2
  echo "例:   $(basename "$0") example/py/set_mode.py" >&2
  exit 1
fi

REL="${1#./}"
shift # 取路径并去掉开头的 ./

# 宿主机侧先校验文件存在，提前给清晰报错（而不是容器里一堆 traceback）
if [ ! -f "$SCRIPT_DIR/$REL" ]; then
  echo "错误: 找不到文件 $SCRIPT_DIR/$REL" >&2
  exit 1
fi

# 确保容器在运行
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "容器未运行，正在启动 $CONTAINER ..."
    docker start "$CONTAINER" >/dev/null
  else
    echo "错误: 容器 $CONTAINER 不存在，请先用 start_emulator.sh 创建。" >&2
    exit 1
  fi
fi

DIR="$WS/$(dirname "$REL")"
FILE="$(basename "$REL")"

case "$FILE" in
*.py) RUNNER="python3 " ;;
*.sh) RUNNER="bash " ;;
*) RUNNER="./" ;; # 其它类型直接执行
esac

# 用 printf %q 对路径和透传参数做安全转义，避免空格/特殊字符出问题
ARGS=""
[ $# -gt 0 ] && ARGS="$(printf '%q ' "$@")"
INNER="cd $(printf '%q' "$DIR") && ${RUNNER}$(printf '%q' "$FILE") $ARGS"

# -it 给交互式脚本（如 set_mode.py 选模式）提供 TTY；bash -ic 以 source 容器内 ROS 环境
exec docker exec -it "$CONTAINER" bash -ic "$INNER"
