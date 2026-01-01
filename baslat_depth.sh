#!/bin/bash

# --- AYARLAR ---
GCS_IP="10.23.3.254"
PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/ros2_drone_ws"
PYTHON_SCRIPT="src/multi_test.py"
SESSION_NAME="drone_sim"
WORLD_NAME="baylands"  # İstersen "baylands" yapabilirsin

# Tmux kontrolü
if ! command -v tmux &> /dev/null; then
    echo "HATA: 'tmux' yüklü değil. Lütfen 'sudo apt install tmux' ile kurun."
    exit 1
fi

echo "---------------------------------------------------"
echo "   SİMÜLASYON v9.1 - HARD RESET & CAMERA FIX       "
echo "   Hedef QGC IP: $GCS_IP                           "
echo "---------------------------------------------------"

# 1. TEMİZLİK (KRİTİK BÖLÜM)
echo "[1/5] Temizlik ve Önbellek Silme..."
pkill -f px4
pkill -f gz
pkill -f ruby
pkill -f MicroXRCEAgent
pkill -f parameter_bridge
pkill -f image_bridge
tmux kill-session -t $SESSION_NAME 2>/dev/null

# --- İŞTE ÇÖZÜM BURADA ---
# Eski instance kayıtlarını siliyoruz ki yeni model (depth) ayarları geçerli olsun.
echo "Eski drone verileri siliniyor (Instance Reset)..."
rm -rf $PX4_DIR/build/px4_sitl_default/instance_*
sleep 2

# 2. OTURUM VE PENCERE DÜZENİ
echo "[2/5] Sanal ekran oluşturuluyor..."
tmux new-session -d -s $SESSION_NAME -x 240 -y 60

# 6 Bölme oluştur
tmux split-window -t $SESSION_NAME:0 -v
tmux split-window -t $SESSION_NAME:0.0 -h
tmux split-window -t $SESSION_NAME:0.1 -h
tmux split-window -t $SESSION_NAME:0.2 -v
tmux split-window -t $SESSION_NAME:0.3 -v
tmux select-layout -t $SESSION_NAME:0 tiled

# 3. TMUX İÇİ KOMUTLAR
echo "[3/5] Simülasyon altyapısı yükleniyor..."

# --- PANE 0: AGENT ---
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- PANE 1: İZLEME EKRANI ---
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source $ROS_WS/install/local_setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROS_WS" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear; echo 'Kamera Köprüsü için ayrı bir terminal açılacak...'" C-m

# --- DRONE FONKSİYONU ---
launch_drone_tmux() {
    local PANE_IDX=$1
    local ID=$2
    local X=$3
    local Y=$4
    local PORT=$5
    local TARGET="$SESSION_NAME:0.$PANE_IDX"
    local EXP_FILE="/tmp/start_drone_${ID}.exp"
    
    local DELAY=0
    if [ "$ID" -gt 0 ]; then DELAY=35; fi 

    if [ "$ID" -eq 0 ]; then
        # MASTER DRONE
        cat > $EXP_FILE <<EOF
set ::env(PX4_GZ_MODEL) "x500_depth"
spawn make px4_sitl
set timeout -1
expect "pxh>"
send "mavlink stop-all\r"
expect "pxh>"
send "mavlink start -x -u 14556 -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    else
        # DİĞER DRONLAR
        cat > $EXP_FILE <<EOF
set ::env(PX4_GZ_MODEL) "x500_depth"
spawn ./build/px4_sitl_default/bin/px4 -i $ID
set timeout -1
expect "pxh>"
send "mavlink stop-all\r"
expect "pxh>"
send "mavlink start -x -u $PORT -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    fi

    tmux send-keys -t $TARGET "sleep $DELAY" C-m
    tmux send-keys -t $TARGET "cd $PX4_DIR" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_WORLD=$WORLD_NAME" C-m
    tmux send-keys -t $TARGET "export PX4_SYS_AUTOSTART=4001" C-m
    # Bash seviyesinde de zorla export ediyoruz
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL=x500_depth" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL_POSE='$X,$Y,0.5,0,0,0'" C-m
    tmux send-keys -t $TARGET "expect $EXP_FILE" C-m
}

# --- DRONLARI BAŞLAT ---
launch_drone_tmux 2 0 0 0 14556
launch_drone_tmux 3 1 3 3 14557
launch_drone_tmux 4 2 -3 3 14558
launch_drone_tmux 5 3 0 -4 14559

# 4. KONTROL TERMİNALİ
echo "[4/5] Kontrol Terminali hazırlanıyor..."
gnome-terminal --tab --title="PYTHON KONTROL MERKEZİ" -- bash -c "
echo 'Simülasyonun ve Kameraların yüklenmesi bekleniyor (70sn)...';
sleep 70;
source /opt/ros/humble/setup.bash;
source $ROS_WS/install/local_setup.bash;
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
cd $ROS_WS;
if [ -f \"$PYTHON_SCRIPT\" ]; then
    python3 $PYTHON_SCRIPT;
else
    echo 'HATA: Dosya bulunamadı!';
fi
read; exec bash"

# 5. ARAYÜZE BAĞLANMA
echo "[5/5] Sistem Hazır! Tmux paneline bağlanılıyor..."
sleep 1
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
