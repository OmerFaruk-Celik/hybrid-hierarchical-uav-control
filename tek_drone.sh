#!/bin/bash

# =========================================================
#                 TEK DRONE TEST AYARLARI
# =========================================================
TELEFON_IP="192.168.1.175"
PX4_DIR="$HOME/PX4-Autopilot"
SESSION_NAME="tek_drone_test"
# =========================================================

echo "---------------------------------------------------"
echo "   TEK DRONE BAĞLANTI TESTİ"
echo "   HEDEF IP: $TELEFON_IP"
echo "---------------------------------------------------"

# 1. ESKİLERİ TEMİZLE
echo "[1/4] Temizlik yapılıyor..."
pkill -9 -f px4
pkill -9 -f gz-sim
pkill -9 -f MicroXRCEAgent
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 2

# 2. TMUX OTURUMU AÇ
echo "[2/4] Oturum açılıyor..."
tmux new-session -d -s $SESSION_NAME -x 240 -y 60
tmux split-window -t $SESSION_NAME:0 -h
tmux select-layout -t $SESSION_NAME:0 tiled

# 3. MAVLINK ROUTER/AGENT (Opsiyonel ama iyi olur)
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# 4. DRONE BAŞLATMA VE BAĞLANTI (EXPECT SCRIPT)
# Bu script PX4 açılınca otomatik olarak 'mavlink start' yazar.
echo "[3/4] Drone ve Bağlantı Komutu hazırlanıyor..."

cat > /tmp/tek_drone.exp <<EOF
spawn make px4_sitl gz_x500
set timeout -1

# PX4 Shell (pxh>) gelene kadar bekle
expect "pxh>" { send "\r" }
expect "pxh>"

# Bağlantı komutunu gönder
# -i 2  : Yeni instance
# -t IP : Telefon IP
# -o 14550 : Telefon Portu
# -u 14585 : Bilgisayar Çıkış Portu (Sabit)
send_user "\n>>> TELEFONA BAGLANTI KOMUTU GONDERILIYOR... <<<\n"
send "mavlink start -i 2 -t $TELEFON_IP -o 14550 -u 14585 -r 4000000\r"

expect "pxh>"
interact
EOF

# Tmux sağ pencerede scripti çalıştır
tmux send-keys -t $SESSION_NAME:0.1 "cd $PX4_DIR" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export PX4_GZ_WORLD=baylands" C-m
tmux send-keys -t $SESSION_NAME:0.1 "expect /tmp/tek_drone.exp" C-m

echo "[4/4] Sistem Başlatıldı! Panele bağlanılıyor..."
echo "NOT: Eğer tcpdump akmıyorsa, beyaz ekrana (pxh>) şunu elle yaz:"
echo "mavlink start -i 2 -t $TELEFON_IP -o 14550 -u 14585 -r 4000000"
sleep 2

# Panele bağlan
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
tmux attach-session -t $SESSION_NAME
