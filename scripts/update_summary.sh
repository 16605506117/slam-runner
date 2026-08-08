#!/bin/bash
# ============================================================
# 指标自动追加到 results/SUMMARY.md（进 git 的唯一结果文件）
# 用法: update_summary.sh <数据集节名> <完整表格行>
#   节名: 如 "KITTI 07" / "W06" / "N03"（对应 SUMMARY.md 的 ## 小节）
#   表格行: 如 "| 2026-08-08 | FAST-LIO | 0.203 | 0.719 | 0.097 | 575 | note |"
# 行为:
#   找到 "## <节名>" 小节 → 在该节表格（连续 | 开头行）末尾插入新行
#   找不到该节 → 自动新建小节（通用 7 列表头）并写入
# 由 eval_lio_sam.sh / eval_fast_lio.sh 在评估完成后自动调用
# ============================================================
SECTION=$1; ROW=$2
SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SUMMARY="$SELF_DIR/../results/SUMMARY.md"

if [ -z "$SECTION" ] || [ -z "$ROW" ]; then
  echo "用法: update_summary.sh <节名> <表格行>"
  exit 1
fi
if [ ! -f "$SUMMARY" ]; then
  echo "[!] 找不到 $SUMMARY"
  exit 1
fi

LINE=$(grep -n "^## $SECTION" "$SUMMARY" | head -1 | cut -d: -f1)
if [ -z "$LINE" ]; then
  # 无此节 → 追加新节（通用表头）
  {
    echo ""
    echo "## $SECTION"
    echo ""
    echo "| Date | Algorithm | ATE RMSE (m) | ATE 3D (m) | RPE RMSE (m) | Poses | Notes |"
    echo "|---|---|---|---|---|---|---|"
    echo "$ROW"
  } >> "$SUMMARY"
  echo "[+] SUMMARY: 新增节 '$SECTION' 并写入 $ROW"
else
  # 有节 → 在节内表格末尾插入（last = 节内最后一个 | 开头的行；遇下一个 ## 节则重置）
  awk -v sec="$LINE" -v row="$ROW" '
    NR == sec { ins=1 }
    ins && /^## / { ins=0 }
    ins && /^\|/ { last=NR }
    { lines[NR] = $0 }
    END {
      for (i = 1; i <= NR; i++) {
        print lines[i]
        if (i == last) print row
      }
    }' "$SUMMARY" > "$SUMMARY.tmp" && mv "$SUMMARY.tmp" "$SUMMARY"
  echo "[+] SUMMARY: 追加到 '$SECTION' 节 -> $ROW"
fi
