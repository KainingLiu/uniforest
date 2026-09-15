#!/bin/bash
# Desktop launchers share one lock; main.py retains normal robot cleanup.
set -u

finish() {
    printf '\n任务已结束（退出码 %s）。按回车关闭窗口。\n' "$1"
    read -r _ || true
    exit "$1"
}

selection="${1:-}"
case "$selection" in
    all) label='全任务流程' ;;
    round1) label='round1 第一轮' ;;
    round2) label='round2 第二轮' ;;
    *) printf '无效任务：%s\n' "$selection"; finish 2 ;;
esac

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || finish 1
cd -- "$project_dir" || finish 1
if [[ ! -x .venv/bin/python || ! -f main.py ]]; then
    printf '找不到项目虚拟环境或 main.py：%s\n' "$project_dir"
    finish 1
fi

lock_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$lock_dir/uniforest-desktop-$UID.lock" || finish 1
if ! flock -n 9; then
    printf '已有一个桌面任务正在运行，请回到原来的终端窗口。\n'
    finish 1
fi

printf '正在启动：%s\n运行目录：%s\n按 Ctrl+C 停止任务。\n\n' "$label" "$project_dir"
# Ctrl+C reaches Python too. Keep this shell alive to show its exit status.
trap ':' INT
.venv/bin/python -u main.py --task "$selection"
result=$?
flock -u 9
exec 9>&-
finish "$result"
