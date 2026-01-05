#!/bin/bash

echo "============================================"
echo "   SİMÜLASYON TEMİZLİK OPERASYONU (KILL)    "
echo "============================================"

# 1. PX4 ve Simülatör Çekirdekleri
echo "[-] PX4 ve Gazebo süreçleri öldürülüyor..."
pkill -9 -f px4
pkill -9 -f px4_sitl
pkill -9 -f gz-sim       # Gazebo Harmonic/Garden
pkill -9 -f ignite       # Ignition Gazebo
pkill -9 -f ruby         # Gazebo başlatıcıları genelde Ruby kullanır
pkill -9 -f gzserver     # Gazebo Classic (varsa)
pkill -9 -f gzclient     # Gazebo Classic (varsa)

# 2. Ara Katman ve Köprü Yazılımları
echo "[-] Micro XRCE-DDS Agent ve ROS köprüleri öldürülüyor..."
pkill -9 -f MicroXRCEAgent
pkill -9 -f micrortps_agent

# 3. Python Kontrol Scriptleri
echo "[-] Çalışan Python kontrol kodları öldürülüyor..."
pkill -9 -f oto.py
pkill -9 -f multi_test.py
# Eğer sistemde başka kritik python kodu çalışmıyorsa garanti olsun diye:
# pkill -9 -f python3 

# 4. ROS 2 Arka Plan Servisleri
echo "[-] ROS 2 Daemon durduruluyor..."
ros2 daemon stop > /dev/null 2>&1

# 5. Tmux (Terminal Çoklayıcı)
echo "[-] Tmux oturumları kapatılıyor..."
# Sadece bizim oluşturduğumuz 'drone_sim' oturumunu kapatmak istersen:
# tmux kill-session -t drone_sim 2>/dev/null
# Hepsini kapatmak için (En temiz yöntem):
tmux kill-server > /dev/null 2>&1

# 6. Çöp Dosyaları Temizle (Opsiyonel ama iyidir)
# Bazen paylaşımlı bellek dosyaları kalabilir
rm -rf /tmp/tmux-* 2>/dev/null

echo "============================================"
echo "   TEMİZLİK TAMAMLANDI. YENİDEN BAŞLATABİLİRSİN."
echo "============================================"
