#!/bin/bash


# 1. TEMİZLİK (GÜNCELLENMİŞ)
echo "[1/5] Temizlik yapılıyor..."
# -9 parametresi işlemi beklemeden zorla öldürür (Force Kill)
pkill -9 -f px4
pkill -9 -f gz-sim
pkill -9 -f ignite
pkill -9 -f ruby
pkill -9 -f MicroXRCEAgent
pkill -9 -f multi_test.py

# Sadece bizim oturumu kapatmaya çalış, olmazsa server'ı kapat
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 2
# --- AYARLAR ---
PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/ros2_drone_ws"
PYTHON_SCRIPT="src/multi_test.py"
SESSION_NAME="drone_sim"
BUILD_PATH="./build/px4_sitl_default"

# --- OTOMATİK IP TESPİTİ ---
# 1. Yöntem: Yönlendirme tablosundan ana IP'yi bulmaya çalışır (En sağlıklısı)
GCS_IP=$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\S+')

# 2. Yöntem: Başarısız olursa hostname -I komutunu dener
if [ -z "$GCS_IP" ]; then
    GCS_IP=$(hostname -I | awk '{print $1}')
fi

# 3. Yöntem: Hiç ağ yoksa localhost'a döner
if [ -z "$GCS_IP" ]; then
    echo "UYARI: Ağ bağlantısı bulunamadı, Localhost kullanılıyor."
    GCS_IP="127.0.0.1"
fi

# --- DÜNYA VE MODEL SEÇİMİ ---
WORLD_NAME="baylands"
MODEL_NAME="x500"
SYS_AUTOSTART="4001" # x500 için ID

# Tmux kontrolü
if ! command -v tmux &> /dev/null; then
    echo "HATA: 'tmux' yüklü değil. Lütfen 'sudo apt install tmux' ile kurun."
    exit 1
fi

echo "---------------------------------------------------"
echo "   SİMÜLASYON v9.1 (AUTO IP) - ORTAM: $WORLD_NAME"
echo "   Tespit Edilen QGC IP: $GCS_IP                   "
echo "---------------------------------------------------"

# 1. AGRESİF TEMİZLİK
echo "[1/5] Temizlik yapılıyor..."
pkill -9 -f px4
pkill -9 -f gz-sim
pkill -9 -f ignite
pkill -9 -f ruby
pkill -9 -f MicroXRCEAgent
pkill -9 -f multi_test.py
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 3

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

# --- PANE 1: ROS 2 ORTAMI ---
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source $ROS_WS/install/local_setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROS_WS" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear; echo 'ROS 2 Ortamı Hazır. Python kodu bekleniyor...'" C-m

# --- DRONE FONKSİYONU ---
launch_drone_tmux() {
    local PANE_IDX=$1
    local ID=$2
    local X=$3
    local Y=$4
    local MAV_PORT=$5  # QGC için port
    local TARGET="$SESSION_NAME:0.$PANE_IDX"
    local EXP_FILE="/tmp/start_drone_${ID}.exp"
    
    local DELAY=2
    if [ "$ID" -gt 0 ]; then DELAY=25; fi 

    # --- EXPECT SCRIPT OLUŞTURMA ---
    if [ "$ID" -eq 0 ]; then
        # MASTER DRONE (GUI ile başlar)
        cat > $EXP_FILE <<EOF
spawn make px4_sitl gz_x500
set timeout -1
expect {
    "pxh>" { send "\r" }
    timeout { exit 1 }
}
expect "pxh>"
send "param set NAV_DLL_ACT 0\r"
expect "pxh>"
send "param set COM_RCL_ACT 0\r"
expect "pxh>"
send "param set NAV_RCL_ACT 0\r"
expect "pxh>"
send "mavlink stop-all\r"
expect "pxh>"
# Burada otomatik tespit edilen GCS_IP kullanılıyor
send "mavlink start -x -u 14556 -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    else
        # SLAVE DRONES (Binary olarak başlar)
        cat > $EXP_FILE <<EOF
spawn $BUILD_PATH/bin/px4 -i $ID
set timeout -1
expect {
    "pxh>" { send "\r" }
    timeout { exit 1 }
}
expect "pxh>"
send "param set NAV_DLL_ACT 0\r"
expect "pxh>"
send "param set COM_RCL_ACT 0\r"
expect "pxh>"
send "param set NAV_RCL_ACT 0\r"
expect "pxh>"
send "mavlink stop-all\r"
expect "pxh>"
# Burada otomatik tespit edilen GCS_IP kullanılıyor
send "mavlink start -x -u $MAV_PORT -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    fi

    tmux send-keys -t $TARGET "sleep $DELAY" C-m
    tmux send-keys -t $TARGET "cd $PX4_DIR" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_WORLD=$WORLD_NAME" C-m
    tmux send-keys -t $TARGET "export PX4_SYS_AUTOSTART=$SYS_AUTOSTART" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL=$MODEL_NAME" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL_POSE='$X,$Y,0.5,0,0,0'" C-m
    tmux send-keys -t $TARGET "expect $EXP_FILE" C-m
}

# --- DRONLARI BAŞLAT ---
# launch_drone_tmux <PaneID> <DroneID> <X> <Y> <MavlinkPort>
launch_drone_tmux 2 0 0 0 14556    # Merkez
launch_drone_tmux 3 1 2 2 14557    # Sağ Üst
launch_drone_tmux 4 2 -2 2 14558   # Sol Üst
launch_drone_tmux 5 3 0 -2 14559   # Aşağı

# 4. PYTHON KONTROL PENCERESİ
echo "[4/5] Kontrol Terminali hazırlanıyor..."

gnome-terminal --tab --title="PYTHON KONTROL MERKEZİ" -- bash -c "
echo 'Simülasyon ortamının (Baylands) ve drone parametrelerinin yüklenmesi bekleniyor (50sn)...';
sleep 50;
source /opt/ros/humble/setup.bash;
source $ROS_WS/install/local_setup.bash;
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
cd $ROS_WS;
echo '--------------------------------';
echo 'Dosya aranıyor: $PYTHON_SCRIPT';
if [ -f \"$PYTHON_SCRIPT\" ]; then
    echo 'Dosya bulundu, başlatılıyor...';
    python3 $PYTHON_SCRIPT;
else
    echo 'HATA: $PYTHON_SCRIPT bulunamadı!';
fi
echo '--------------------------------';
read;
exec bash"

# 5. ARAYÜZE BAĞLANMA
echo "[5/5] Sistem Hazır! Tmux paneline bağlanılıyor..."
sleep 1
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
