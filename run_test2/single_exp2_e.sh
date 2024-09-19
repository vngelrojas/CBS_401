#!/bin/bash

TYMPath="$1"
outputfile="$2"
weight1="$3"
baseSeed="$4"
tempdir=$(mktemp -d)
echo "$tempdir"

# 定义数组
# map_dirs=(paper_random_32_32)
# ratios=(100)
# times=(10)
map_dirs=(paper_random_32_32 paper_den312d_65_81 paper_empty_32_32 paper_maze_32_32_2 paper_room_64_64_8 paper_warehouse_161_63 paper_orz900d paper_Boston)
ratios=(000 030 060 100)
times=(40 40 40 40 40 40 60 60) # 对应于map_dirs的时间设置

# 存储所有临时文件的顺序和PID
temp_files_order=()
pids=()
idx=0 # 初始化迭代计数器

# 当脚本接收到 SIGINT 信号时，执行 cleanup 函数
trap cleanup SIGINT

cleanup() {
  echo "Caught interrupt signal, cleaning up..."
  # 杀死所有已知的 Python 进程 PID
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null
  done
  # 删除临时目录和文件
  rm -rf "$tempdir"
  # 退出脚本
  exit 1
}



# 生成不同的seed和对应的命令
for i in $(seq 0 3); do
  for ratio in "${ratios[@]}"; do
    seed=$((baseSeed*100 + idx)) # 计算当前的seed
    tempfile="$tempdir/output_${map_dirs[$i]}_${ratio}_${seed}.txt"

    # 运行命令并将输出重定向到临时文件中
    python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir "map_file2/${map_dirs[$i]}_ratio_${ratio}" --time "${times[$i]}" --seed "$seed" --weight1 "$weight1" > "$tempfile" &
    pid=$!
    echo "Started background job with PID: $pid" >&2

    temp_files_order+=("$tempfile")
    pids+=("$pid")
    idx=$((idx + 1)) # 递增迭代计数器
  done
done

# 等待所有后台命令执行完毕
jobs -l
for pid in "${pids[@]}"; do
    wait "$pid" || echo "Job $pid exited with status $?"
done
jobs -l

pids=()
for i in $(seq 4 7); do
  for ratio in "${ratios[@]}"; do
    seed=$((baseSeed*100 + idx)) # 计算当前的seed
    tempfile="$tempdir/output_${map_dirs[$i]}_${ratio}_${seed}.txt"

    # 运行命令并将输出重定向到临时文件中
    python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir "map_file2/${map_dirs[$i]}_ratio_${ratio}" --time "${times[$i]}" --seed "$seed" --weight1 "$weight1" > "$tempfile" &
    pid=$!
    echo "Started background job with PID: $pid" >&2

    temp_files_order+=("$tempfile")
    pids+=("$pid")
    idx=$((idx + 1)) # 递增迭代计数器
  done
done

jobs -l
for pid in "${pids[@]}"; do
    wait "$pid" || echo "Job $pid exited with status $?"
done
jobs -l

# 按照原始命令的顺序合并输出文件
for tempfile in "${temp_files_order[@]}"; do
  cat "$tempfile" >> "$outputfile"
  # rm -f "$tempfile" # 删除已合并的临时文件
done

# 删除临时目录
rm -r "$tempdir"
