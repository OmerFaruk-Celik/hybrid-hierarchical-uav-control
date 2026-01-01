#!/bin/bash

# --- AYARLAR ---
GCS_IP="10.196.27.185"
PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/ros2_drone_ws"
PYTHON_SCRIPT="src/multi_test.py"  # Yeni dosya adı
SESSION_NAME="drone_sim"

# Tmux kontrolü
if ! command -v tmux &> /dev/null; then
    echo "HATA: 'tmux' yüklü değil. Lütfen 'sudo apt install tmux' ile kurun."
    exit 1
fi

echo "---------------------------------------------------"
echo "   SİMÜLASYON v3.3 - AYRIK KONTROL TERMİNALİ       "
echo "   Hedef QGC IP: $GCS_IP                           "
echo "---------------------------------------------------"

# 1. TEMİZLİK
echo "[1/5] Temizlik yapılıyor..."
pkill -f px4
pkill -f gz
pkill -f ruby
pkill -f MicroXRCEAgent
tmux kill-session -t $SESSION_NAME 2>/dev/null
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

# 3. TMUX İÇİ KOMUTLAR (Sadece Simülasyon Altyapısı)
echo "[3/5] Simülasyon altyapısı yükleniyor..."

# --- PANE 0: AGENT ---
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- PANE 1: İZLEME EKRANI (ROS Hazır Bekler) ---
# Burası artık python çalıştırmayacak, sadece ortamı hazırlayıp bekleyecek.
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source $ROS_WS/install/local_setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROS_WS" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear; echo 'Sistem İzleme Paneli (Pasif) - Python Kodu ayrı pencerede açılacak...'" C-m

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
    if [ "$ID" -gt 0 ]; then DELAY=15; fi

    if [ "$ID" -eq 0 ]; then
        cat > $EXP_FILE <<EOF
spawn make px4_sitl
set timeout -1
expect "pxh>"
send "mavlink stop-all\r"
expect "pxh>"
send "mavlink start -x -u 14556 -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    else
        cat > $EXP_FILE <<EOF
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
    tmux send-keys -t $TARGET "export PX4_SYS_AUTOSTART=4001" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL=x500" C-m
    tmux send-keys -t $TARGET "export PX4_GZ_MODEL_POSE='$X,$Y,0,0,0,0'" C-m
    tmux send-keys -t $TARGET "expect $EXP_FILE" C-m
}

# --- DRONLARI BAŞLAT ---
launch_drone_tmux 2 0 0 0 14556
launch_drone_tmux 3 1 10 0 14557
launch_drone_tmux 4 2 10 10 14558
launch_drone_tmux 5 3 0 10 14559

# 4. AYRI KONTROL TERMİNALİNİ AÇMA
echo "[4/5] Kontrol Terminali hazırlanıyor..."

# Gnome-terminal'i ayrı başlatıyoruz.
# 45 saniye bekleme koydum ki dronlar tamamen açılmış olsun.
gnome-terminal --tab --title="PYTHON KONTROL MERKEZİ" -- bash -c "
echo 'Simülasyonun tam yüklenmesi bekleniyor (45sn)...';
sleep 45;
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
    echo 'Lütfen dosya yolunu kontrol et: $ROS_WS/$PYTHON_SCRIPT';
fi
echo '--------------------------------';
echo 'Script sonlandı. Çıkış için bir tuşa basın.';
read;
exec bash"

# 5. ARAYÜZE BAĞLANMA
echo "[5/5] Sistem Hazır! Tmux paneline bağlanılıyor..."
sleep 1
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
