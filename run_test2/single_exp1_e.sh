#!/bin/bash

TYMPath=$1
outputfile=$2
weight1=$3

# 创建一个唯一的临时目录
tempdir=$(mktemp -d)

# 使用 & 将每个命令放到后台执行，并为每个命令生成不同的seed
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_random_32_32_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output1" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_den312d_65_81_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output2" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_empty_32_32_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output3" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_maze_32_32_2_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output4" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_room_64_64_8_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output5" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_warehouse_161_63_gp_5 --time 60 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output6" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_orz900d_gp_5 --time 80 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output7" &
python3 python/compareTYM2TA.py --TYMPath "$TYMPath" --map_dir map_file2/paper_Boston_gp_5 --time 80 --seed $RANDOM --weight1 "$weight1" >> "$tempdir/output8" &

# 等待所有后台进程完成
wait

# 汇总所有输出到最终的输出文件
cat "$tempdir"/output* >> "$outputfile"

# 删除临时目录
rm -rf "$tempdir"
