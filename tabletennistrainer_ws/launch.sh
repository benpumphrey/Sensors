#!/bin/bash

JETSON_B_IP="192.168.1.20"
JETSON_B_USER="capstone-nano2"
LOCAL_WS="/home/capstone-nano1/TTT-Capstone-Sensors/tabletennistrainer_ws"
REMOTE_WS="/home/capstone-nano2/Desktop/TTT-Capstone-Sensors/tabletennistrainer_ws"
GUI_SCRIPT="$LOCAL_WS/src/gui/marty_gui.py" 

cleanup() {
    echo -e "\n[Master] Shutting down M.A.R.T.Y..."
    pkill -f ttt_bringup
    ssh $JETSON_B_USER@$JETSON_B_IP "pkill -f ttt_bringup"
    echo "[Master] System offline."
    exit
}
trap cleanup SIGINT

echo "--- Starting Project M.A.R.T.Y. Systems ---"

if ! ping -c 1 $JETSON_B_IP &> /dev/null; then
    echo "ERROR: Cannot reach Jetson B at $JETSON_B_IP. Check Ethernet cables!"
    exit 1
fi

echo "[Jetson A] Tuning OV9281 & Launching..."
v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200
source /opt/ros/humble/setup.bash
source "$LOCAL_WS/install/setup.bash"
ros2 launch ttt_bringup jetsonA.launch.py &


echo "[Jetson B] Launching remote nodes..."
ssh -tt $JETSON_B_USER@$JETSON_B_IP << EOF &
    source /opt/ros/humble/setup.bash
    source "$REMOTE_WS/install/setup.bash"
    v4l2-ctl -d /dev/video0 -c exposure=800 -c analogue_gain=1200
    ros2 launch ttt_bringup jetsonB.launch.py
    exit
EOF

echo "[Master] Waiting for nodes to stabilize..."
sleep 4 
echo "[Master] Launching Dashboard..."
export DISPLAY=:0
python3 $GUI_SCRIPT &


wait