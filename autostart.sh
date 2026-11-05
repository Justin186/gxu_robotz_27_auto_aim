#!/bin/bash
# gxu_robotz_27_auto_aim/autostart.sh
#
# 每台机器跑哪个程序、用哪份配置，由调用方传进来 —— 不要改这个文件，
# 否则每台机器都会因为改同一行而和远程冲突：
#
#   ./autostart.sh configs/infantry3.yaml
#
# 真正的"这台机器跑什么"放在本机的桌面自启项里（不入库）：
#   ~/.config/autostart/sp_vision.desktop
#   Exec=/home/rm/Desktop/gxu_robotz_27_auto_aim/autostart.sh configs/infantry3.yaml

CONFIG=${1:-configs/infantry3.yaml}

# 切到脚本所在目录，不依赖具体的仓库目录名
cd "$(dirname "$0")" || exit

./build/auto_aim_mpc -rec=false "$CONFIG"

# 清理历史
history -c
