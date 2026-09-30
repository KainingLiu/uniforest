#!/bin/bash
# Desktop launchers share one lock; main.py retains normal robot cleanup.
set -u

finish() {
    printf '\n任务已结束（退出码 %s）。按回车关闭窗口。\n' "$1"
    read -r _ || true
    exit "$1"
}

selection="${1:-}"
preview="${2:-}"
if [[ $# -gt 2 || ( -n "$preview" && "$preview" != '--show-plan' ) ]]; then
    printf '用法：%s <任务或策略包> [--show-plan]\n' "$0"
    exit 2
fi
case "$selection" in
    all|classic|PlanA|plana) label='PlanA 原策略' ;;
    PlanB|planb) label='PlanB 采集投放策略' ;;
    round1|set1) label='set1 任务组合' ;;
    round2|set2) label='set2 任务组合' ;;
    collect-build-1|collect-build-2) label="$selection" ;;
    task0|task0-1|task0-2|task0-3|task1-1|task1-2|task1-3|task2-1|task2-2|task3-1|task3-2|task3-3|task4-1|task4-2|task5) label="$selection" ;;
    *) printf '无效任务：%s\n' "$selection"; finish 2 ;;
esac

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || finish 1
cd -- "$project_dir" || finish 1
if [[ ! -x .venv/bin/python || ! -f main.py ]]; then
    printf '找不到项目虚拟环境或 main.py：%s\n' "$project_dir"
    finish 1
fi

case "$selection" in
    PlanA|PlanB|plana|planb|classic|set1|set2|collect-build-1|collect-build-2)
        command=(.venv/bin/python -u main.py --strategy "$selection") ;;
    *) command=(.venv/bin/python -u main.py --task "$selection") ;;
esac
if [[ "$preview" == '--show-plan' ]]; then
    exec "${command[@]}" --show-plan
fi

lock_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"$lock_dir/uniforest-desktop-$UID.lock" || finish 1
if ! flock -n 9; then
    printf '已有一个桌面任务正在运行，请回到原来的终端窗口。\n'
    finish 1
fi

case "$selection" in
    task3-1|task3-2|task3-3|task4-1|task4-2)
        printf '独立 Task3/Task4 要求停在 Task2 结束位置，初始航向 180°。\n'
        printf 'Task3 需准备好搭建方块；Task4 需准备好舱内投放方块。\n'
        printf '请填写之前标定的航向零点（度），不能直接填当前航向；留空取消：'
        IFS= read -r heading || finish 130
        if [[ -z "$heading" ]]; then
            printf '已取消，未连接机器人。\n'
            finish 2
        fi
        command+=("--heading-zero-deg=$heading") ;;
esac

printf '正在启动：%s\n运行目录：%s\n按 Ctrl+C 停止任务。\n\n' "$label" "$project_dir"
# Ctrl+C reaches Python too. Keep this shell alive to show its exit status.
trap ':' INT
"${command[@]}"
result=$?
flock -u 9
exec 9>&-
finish "$result"
