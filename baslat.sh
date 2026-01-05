#!/bin/bash

# =========================================================
#                 AYARLAR (DÜZENLEMEN GEREKEN YER)
# =========================================================

# 1. HEDEF TELEFON IP ADRESİ
TELEFON_IP="192.168.1.175"

# 2. Klasör Yolları
PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/ros2_drone_ws"
SESSION_NAME="drone_sim"
BUILD_PATH="./build/px4_sitl_default"
WORLD_NAME="baylands"
MODEL_NAME="x500"
SYS_AUTOSTART="4001"

# Simülasyon Hızı
export PX4_SIM_SPEED_FACTOR=1.0

echo "---------------------------------------------------"
echo "   SİMÜLASYON v16.0 (DIRECT IP MODE)"
echo "   HEDEF CİHAZ : $TELEFON_IP"
echo "   HEDEF PORT  : 14550 (QGC Standart)"
echo "---------------------------------------------------"

# Tmux kontrolü
if ! command -v tmux &> /dev/null; then echo "HATA: tmux kurulu değil."; exit 1; fi

# =========================================================
#            1. TEMİZLİK (PORTLARI BOŞALT)
# =========================================================
echo "[1/5] Çakışan portlar ve eski işlemler temizleniyor..."

# Klasik işlem öldürme
pkill -9 -f px4
pkill -9 -f gz-sim
pkill -9 -f ignite
pkill -9 -f MicroXRCEAgent

# ÖNEMLİ: Portları tutan inatçı süreçleri zorla öldür
# Eğer 'fuser' komutu yoksa hata vermemesi için yönlendirme yapıldı
command -v fuser >/dev/null 2>&1 && fuser -k -n udp 14550 > /dev/null 2>&1
command -v fuser >/dev/null 2>&1 && fuser -k -n udp 14556 > /dev/null 2>&1

tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 3

# =========================================================
#            2. OTURUM KURULUMU
# =========================================================
echo "[2/5] Sanal ekran oluşturuluyor..."
tmux new-session -d -s $SESSION_NAME -x 240 -y 60
tmux split-window -t $SESSION_NAME:0 -v
tmux split-window -t $SESSION_NAME:0.0 -h
tmux split-window -t $SESSION_NAME:0.2 -h
tmux split-window -t $SESSION_NAME:0.2 -h
tmux split-window -t $SESSION_NAME:0.2 -h
tmux split-window -t $SESSION_NAME:0.2 -v
tmux split-window -t $SESSION_NAME:0.3 -v
tmux select-layout -t $SESSION_NAME:0 tiled

# =========================================================
#            3. SİMÜLASYON BAŞLATMA
# =========================================================
echo "[3/5] Simülasyon altyapısı yükleniyor..."

# --- PANE 0: AGENT ---
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- PANE 1: ROS 2 ORTAMI ---
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source $ROS_WS/install/local_setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROS_WS; clear; echo 'ROS 2 Hazır.'" C-m

# --- DRONE FONKSİYONU ---
launch_drone_tmux() {
    local PANE_IDX=$1
    local ID=$2
    local X=$3
    local Y=$4
    
    local TARGET="$SESSION_NAME:0.$PANE_IDX"
    local EXP_FILE="/tmp/start_drone_${ID}.exp"
    
    # --- PORT AYARI (ÖNEMLİ) ---
    # "Address in use" hatası almamak için portları 18580'den başlatıyoruz.
    # Drone 0 -> 18580, Drone 1 -> 18581 ...
    local LOCAL_UDP_PORT=$((18580 + ID))
    
    local DELAY=2
    if [ "$ID" -gt 0 ]; then DELAY=15; fi 

    local SPAWN_CMD=""
    if [ "$ID" -eq 0 ]; then
        SPAWN_CMD="spawn make px4_sitl gz_x500"
    else
        SPAWN_CMD="spawn $BUILD_PATH/bin/px4 -i $ID"
    fi

    # Expect script
    cat > $EXP_FILE <<EOF
$SPAWN_CMD
set timeout -1
expect "pxh>" { send "\r" }
expect "pxh>"

# Gereksiz parametreleri kapat (Hata veren COM_RCL_ACT kaldırıldı)
send "param set NAV_DLL_ACT 0\r"; expect "pxh>"
send "param set NAV_RCL_ACT 0\r"; expect "pxh>"
send "param set COM_PREARM_MODE 0\r"; expect "pxh>" 

# --- BAĞLANTI KOMUTU ---
# -t : Hedef IP (Senin Telefonun)
# -o : Hedef Port (Standart 14550)
# -u : Bilgisayar Çıkış Portu (18580+ yaptık ki çakışmasın)
# -r : Veri hızı
send "mavlink start -t $TELEFON_IP -o 14550 -u $LOCAL_UDP_PORT -r 4000000\r"
expect "pxh>"

interact
EOF

    tmux send-keys -t $TARGET "sleep $DELAY" C-m
    tmux send-keys -t $TARGET "cd $PX4_DIR" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_WORLD=$WORLD_NAME" C-m
    tmux send-keys -t $TARGET "export PX4_SYS_AUTOSTART=$SYS_AUTOSTART" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL=$MODEL_NAME" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL_POSE='$X,$Y,1.5,0,0,0'" C-m
    tmux send-keys -t $TARGET "expect $EXP_FILE" C-m
}

# --- DRONLARI BAŞLAT (6 ADET) ---
launch_drone_tmux 2 0   0    0
launch_drone_tmux 3 1  -1.2  2
launch_drone_tmux 4 2  -2.4  4
launch_drone_tmux 5 3  -3.6  7
launch_drone_tmux 6 4  -5    8
launch_drone_tmux 7 5  -6.0  10

# 4. MANUEL KONTROL TERMİNALİ
echo "[4/5] Manuel Kontrol Terminali hazırlanıyor..."
gnome-terminal --tab --title="MANUEL KONTROL" -- bash -c "
echo 'Simülasyon ortamının yüklenmesi için bekleniyor (15sn)...';
sleep 15;
source /opt/ros/humble/setup.bash;
source $ROS_WS/install/local_setup.bash;
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
cd $ROS_WS;
clear;
echo '=================================================';
echo '   SİSTEM HAZIR! ';
echo '   Hedef IP: $TELEFON_IP';
echo '   Hedef Port: 14550';
echo '   Yerel Portlar: 18580-18585 (Çakışma Önleyici)';
echo '=================================================';
exec bash"

# 5. ARAYÜZE BAĞLANMA
echo "[5/5] Sistem Hazır! Tmux paneline bağlanılıyor..."
sleep 1
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
