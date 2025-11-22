#!/bin/bash
# 編譯腳本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo "編譯推論程式"
echo "========================================"

cd "$SCRIPT_DIR"

# 檢查 SDK 資料夾
if [ ! -d "sdk" ]; then
    echo "❌ SDK 資料夾不存在: sdk/"
    echo "請確認 SDK 已正確複製到 sdk/ 資料夾"
    exit 1
fi

echo "開始編譯..."
echo ""

# 編譯兩個程式
make -j

echo ""
echo "========================================"
echo "編譯完成！"
echo "========================================"
echo "可執行檔位置："
echo "  - build/inference_batch  (讀取整個 recordings 資料夾)"
echo "  - build/inference_latest  (只讀取最新時間戳資料夾)"
echo ""

