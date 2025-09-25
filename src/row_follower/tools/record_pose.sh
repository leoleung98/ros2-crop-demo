#!/usr/bin/env bash
# Usage: ./record_pose.sh z pitch
# Example: ./record_pose.sh 0.32 0.8

set -euo pipefail

Z="${1:?need z}"
PITCH="${2:?need pitch}"

# 输出目录（按姿态命名）
OUT="$HOME/logs/cam_sweep/z${Z}_p${PITCH}"
mkdir -p "$OUT"

echo "[record] writing CSVs into: $OUT"
echo "[record] duration: 15s"

# 并行录制三个话题（Float32；--csv 保留数值）
timeout 15s bash -c "ros2 topic echo /perception/green_ratio_lane  --csv > '$OUT/green.csv'"  &
timeout 15s bash -c "ros2 topic echo /perception/ground_ratio_lane --csv > '$OUT/ground.csv'" &
timeout 15s bash -c "ros2 topic echo /perception/crop_symmetry     --csv > '$OUT/sym.csv'"    &
wait || true  # 即使某个话题暂时没消息也不报错中断

# 计算均值（跳过CSV首行表头）
# 取每行的“最后一列”，且只累计数值行
mean() {
  awk -F, '
    NR>1 {
      v = $NF
      # 去空白
      gsub(/^[ \t]+|[ \t]+$/, "", v)
      # 过滤非数值（比如 "data" 表头）
      if (v ~ /^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/) { s+=v; n++ }
    }
    END { if (n>0) printf "%.4f\n", s/n; else print "NaN" }
  ' "$1"
}
mean_abs() {
  awk -F, '
    NR>1 {
      v = $NF
      gsub(/^[ \t]+|[ \t]+$/, "", v)
      if (v ~ /^-?[0-9]+(\.[0-9]+)?([eE][-+]?[0-9]+)?$/) {
        if (v<0) v=-v; s+=v; n++
      }
    }
    END { if (n>0) printf "%.4f\n", s/n; else print "NaN" }
  ' "$1"
}


MEAN_GREEN=$(mean "$OUT/green.csv")
MEAN_GROUND=$(mean "$OUT/ground.csv")
MEAN_ABS_SYM=$(mean_abs "$OUT/sym.csv")

# 打印一行可直接粘到 Markdown 表格
printf "\nPaste into table:\n| %s | %s | %s | %s | %s | |\n" "$Z" "$PITCH" "$MEAN_GREEN" "$MEAN_GROUND" "$MEAN_ABS_SYM" | tee "$OUT/summary.txt"

echo "[record] done."
