# ROS 2 Multi-Drone Simulation System

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-orange.svg)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)

Çoklu drone simülasyonu için ROS 2, PX4 Autopilot ve Gazebo Harmonic kullanan otomatik başlatma ve kontrol sistemi.

## 📋 İçindekiler

- [ADIM 1: Gerekli Kurulumlar](#adim-1-gerekli-kurulumlar-gazebo-harmonic--araçlar)
- [ADIM 2: ROS 2 Çalışma Alanı ve px4_msgs](#adim-2-ros-2-çalışma-alanı-ve-px4_msgs)
- [ADIM 3: Otomasyon Scripti (baslat.sh)](#adim-3-otomasyon-scripti-baslatsh)
- [ADIM 4: Python Kontrol Kodu (multi_test.py)](#adim-4-python-kontrol-kodu-multi_testpy)
- [ADIM 5: Çalıştırma](#adim-5-çalıştırma)

---

## ADIM 1: Gerekli Kurulumlar (Gazebo Harmonic & Araçlar)

Öncelikle sistemin temizlenmesi ve yeni nesil Gazebo'nun (Harmonic) kurulması gerekti.

### 1. Tmux ve Araçların Kurulumu

```bash
sudo apt update
sudo apt install tmux expect git -y
```

### 2. Gazebo Harmonic Kurulumu (Eski Gazebo Classic yerine)

```bash
sudo apt-get install lsb-release gnupg curl -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic -y
```

### 3. Python Bağımlılıkları

```bash
pip3 install kconfiglib jinja2 jsonschema
```

### 4. PX4 Derlemesi (Eski derlemeyi temizleyip yeni GZ için hazırlık)

```bash
cd ~/PX4-Autopilot
make distclean
# Test için bir kez elle derle (sonra script yapacak)
make px4_sitl gz_x500
```

---

## ADIM 2: ROS 2 Çalışma Alanı ve px4_msgs

Python kodunun çalışması için PX4 mesaj tiplerinin ROS 2 ortamına tanıtılması gerekti.

### 1. Çalışma alanı oluşturma (varsa geç)

```bash
mkdir -p ~/ros2_drone_ws/src
cd ~/ros2_drone_ws/src
```

### 2. px4_msgs paketini indirme

```bash
git clone https://github.com/PX4/px4_msgs.git
```

### 3. Derleme (Build)

```bash
cd ~/ros2_drone_ws
colcon build
```

### 4. Ortamı kaynaklama

```bash
source install/setup.bash
```

---

## ADIM 3: Otomasyon Scripti (baslat.sh)

Bu script şunları yapar:

- **Otomatik IP**: Bilgisayarın ağ IP'sini bulur
- **Agresif Temizlik**: Arkada kalan zombi süreçleri öldürür
- **Güvenlik İptali**: NAV_DLL_ACT 0 vb. ile kumanda/GCS olmadan uçuş izni verir
- **Tmux**: Ekranı böler, Agent'ı ve 4 drone'u (1 Master GUI + 3 Slave Headless) başlatır

**Dosya Konumu**: `~/ros2_drone_ws/baslat.sh`

```bash
#!/bin/bash

# --- AYARLAR ---
PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/ros2_drone_ws"
PYTHON_SCRIPT="src/multi_test.py"
SESSION_NAME="drone_sim"
BUILD_PATH="./build/px4_sitl_default"

# --- OTOMATİK IP TESPİTİ ---
GCS_IP=$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\S+')
if [ -z "$GCS_IP" ]; then GCS_IP=$(hostname -I | awk '{print $1}'); fi
if [ -z "$GCS_IP" ]; then GCS_IP="127.0.0.1"; fi

# --- DÜNYA VE MODEL ---
WORLD_NAME="baylands"
MODEL_NAME="x500"
SYS_AUTOSTART="4001"

# Tmux Kontrol
if ! command -v tmux &> /dev/null; then echo "HATA: tmux kurmalısın."; exit 1; fi

echo "---------------------------------------------------"
echo "   SİMÜLASYON - ORTAM: $WORLD_NAME | IP: $GCS_IP"
echo "---------------------------------------------------"

# 1. TEMİZLİK
pkill -9 -f px4
pkill -9 -f gz-sim
pkill -9 -f ignite
pkill -9 -f ruby
pkill -9 -f MicroXRCEAgent
pkill -9 -f multi_test.py
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 3

# 2. OTURUM AÇMA
tmux new-session -d -s $SESSION_NAME -x 240 -y 60
tmux split-window -t $SESSION_NAME:0 -v
tmux split-window -t $SESSION_NAME:0.0 -h
tmux split-window -t $SESSION_NAME:0.1 -h
tmux split-window -t $SESSION_NAME:0.2 -v
tmux split-window -t $SESSION_NAME:0.3 -v
tmux select-layout -t $SESSION_NAME:0 tiled

# 3. BAŞLATMA
# Pane 0: Agent
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m
# Pane 1: ROS 2 Setup
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source $ROS_WS/install/local_setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $SESSION_NAME:0.1 "cd $ROS_WS; clear; echo 'ROS 2 Hazır.'" C-m

launch_drone_tmux() {
    local PANE_IDX=$1; local ID=$2; local X=$3; local Y=$4; local MAV_PORT=$5
    local TARGET="$SESSION_NAME:0.$PANE_IDX"
    local EXP_FILE="/tmp/start_drone_${ID}.exp"
    local DELAY=2
    if [ "$ID" -gt 0 ]; then DELAY=25; fi 

    if [ "$ID" -eq 0 ]; then
        # MASTER (GUI)
        cat > $EXP_FILE <<EOF
spawn make px4_sitl gz_x500
set timeout -1
expect "pxh>" { send "\r" }
expect "pxh>"
send "param set NAV_DLL_ACT 0\r"; expect "pxh>"
send "param set COM_RCL_ACT 0\r"; expect "pxh>"
send "param set NAV_RCL_ACT 0\r"; expect "pxh>"
send "mavlink stop-all\r"; expect "pxh>"
send "mavlink start -x -u 14556 -r 4000000 -t $GCS_IP -o 14550\r"
interact
EOF
    else
        # SLAVE (HEADLESS)
        cat > $EXP_FILE <<EOF
spawn $BUILD_PATH/bin/px4 -i $ID
set timeout -1
expect "pxh>" { send "\r" }
expect "pxh>"
send "param set NAV_DLL_ACT 0\r"; expect "pxh>"
send "param set COM_RCL_ACT 0\r"; expect "pxh>"
send "param set NAV_RCL_ACT 0\r"; expect "pxh>"
send "mavlink stop-all\r"; expect "pxh>"
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

# Drone'ları Yerleştir
launch_drone_tmux 2 0 0 0 14556
launch_drone_tmux 3 1 2 2 14557
launch_drone_tmux 4 2 -2 2 14558
launch_drone_tmux 5 3 0 -2 14559

# Python Kontrol Terminali
gnome-terminal --tab --title="PYTHON KONTROL" -- bash -c "sleep 50; source /opt/ros/humble/setup.bash; source $ROS_WS/install/local_setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; cd $ROS_WS; python3 $PYTHON_SCRIPT; exec bash"

sleep 1
tmux set-option -t $SESSION_NAME mouse on
tmux attach-session -t $SESSION_NAME
```

**Script'i çalıştırılabilir yapma:**

```bash
chmod +x ~/ros2_drone_ws/baslat.sh
```

---

## ADIM 4: Python Kontrol Kodu (multi_test.py)

Bu kodda kritik olan düzeltme: `time.sleep(3.0)`. Offboard sinyalinin PX4 tarafından kabul edilmesi için sinyal gönderimi başladıktan sonra mod değişiminden önce 3 saniye bekliyoruz.

**Dosya Konumu**: `~/ros2_drone_ws/src/multi_test.py`

```python
#!/usr/bin/env python3
import rclpy
import threading
import sys
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, SensorCombined, VehicleGlobalPosition, VehicleLocalPosition, BatteryStatus

class SafeSwarmController(Node):
    def __init__(self):
        super().__init__('safe_swarm_node')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.active_drones = {}
        self.monitor_sub = None
        self.print_counter = 0
        self.monitoring_info = ""
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_drone_namespace(self, vehicle_id):
        return "" if vehicle_id == 0 else f"/px4_{vehicle_id}"

    def register_drone(self, vehicle_id, start_x, start_y):
        ns = self.get_drone_namespace(vehicle_id)
        prefix = f"{ns}/fmu/in"
        print(f"\n[SİSTEM] Drone {vehicle_id} Bağlanıyor...")

        self.active_drones[vehicle_id] = {
            'pubs': {
                'offboard': self.create_publisher(OffboardControlMode, f'{prefix}/offboard_control_mode', self.qos_profile),
                'traj': self.create_publisher(TrajectorySetpoint, f'{prefix}/trajectory_setpoint', self.qos_profile),
                'cmd': self.create_publisher(VehicleCommand, f'{prefix}/vehicle_command', self.qos_profile)
            },
            'target': [float(start_x), float(start_y), -5.0],
        }

        # KRİTİK NOKTA: Offboard sinyalinin oturması için bekleme
        print(f"[BEKLİYOR] Drone {vehicle_id} sinyal gönderiyor (3sn)...")
        time.sleep(3.0) 

        self.set_mode_offboard(vehicle_id)
        time.sleep(1.0)
        self.arm_vehicle(vehicle_id)
        print(f"[BAŞARILI] Drone {vehicle_id} Havalanıyor!")

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        for v_id, drone_data in self.active_drones.items():
            msg_mode = OffboardControlMode()
            msg_mode.position = True
            msg_mode.velocity = False
            msg_mode.acceleration = False
            msg_mode.timestamp = timestamp
            drone_data['pubs']['offboard'].publish(msg_mode)

            msg_traj = TrajectorySetpoint()
            msg_traj.position = drone_data['target']
            msg_traj.yaw = 0.0
            msg_traj.velocity = [float('nan')] * 3
            msg_traj.acceleration = [float('nan')] * 3
            msg_traj.timestamp = timestamp
            drone_data['pubs']['traj'].publish(msg_traj)

    def set_mode_offboard(self, v_id):
        self.publish_cmd(v_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

    def arm_vehicle(self, v_id):
        self.publish_cmd(v_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def land_vehicle(self, v_id):
        self.publish_cmd(v_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_cmd(self, v_id, command, p1=0.0, p2=0.0):
        if v_id not in self.active_drones: return
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = int(v_id) + 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.active_drones[v_id]['pubs']['cmd'].publish(msg)
    
    def land_all(self):
        for v_id in self.active_drones: self.land_vehicle(v_id)

    def update_target(self, v_id, x, y, z):
        if v_id not in self.active_drones: self.register_drone(v_id, x, y)
        self.active_drones[v_id]['target'] = [float(x), float(y), float(z)]

def main():
    rclpy.init()
    node = SafeSwarmController()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    
    print("SİSTEM HAZIR. Örnek Komut: 0 0 0 -5")
    
    while True:
        try:
            inp = input("\nKomut > ").strip().lower().split()
            if not inp: continue
            if inp[0] in ['q', 'exit']: node.land_all(); break
            if inp[0] == 'land': 
                 if len(inp)>1 and inp[1]=='all': node.land_all()
                 else: node.land_vehicle(int(inp[1]))
                 continue
            
            if len(inp) == 4:
                node.update_target(int(inp[0]), inp[1], inp[2], inp[3])
        except Exception as e: print(e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Not**: Bu kod özetlenmiş versiyondur. Tam versiyon için `src/multi_test.py` dosyasına bakınız.

---

## ADIM 5: Çalıştırma

Her şey hazır olduğunda tek yapman gereken:

```bash
cd ~/ros2_drone_ws
./baslat.sh
```

Açılan Python penceresine `0 0 0 -5` yazarak ilk drone'u kaldırabilirsin.

### Kullanım Örnekleri

**Drone Kontrol Komutları:**
- `0 0 0 -5` - Drone 0'ı (0, 0, -5) konumuna gönder
- `1 2 2 -5` - Drone 1'i (2, 2, -5) konumuna gönder
- `land 0` - Drone 0'a iniş emri ver
- `land all` - Tüm dronlara iniş emri ver
- `q` veya `exit` - Sistemi kapat

**Önemli Notlar:**
- Z ekseni negatif değerler yukarıyı temsil eder (NED koordinat sistemi)
- Offboard moduna geçiş için 3 saniye bekleme süresi kritiktir
- İlk drone (ID: 0) GUI ile başlar, diğerleri headless modda çalışır

---

## 🛠️ Sistem Gereksinimleri

- **OS**: Ubuntu 22.04 LTS (veya Ubuntu 20.04+)
- **ROS 2**: Humble
- **Gazebo**: Harmonic
- **PX4**: SITL (Software In The Loop)
- **Python**: 3.8+
- **RAM**: Minimum 8GB (16GB önerilir)
- **CPU**: Çok çekirdekli işlemci önerilir

## 📁 Proje Yapısı

```
ros2_drone_ws/
├── src/
│   ├── multi_test.py          # Ana kontrol scripti
│   └── px4_msgs/              # PX4 mesaj tanımları
├── baslat.sh                   # Otomasyon scripti
├── install/                    # Derlenmiş paketler
└── README.md                   # Bu dosya
```

## 🐛 Sorun Giderme

### Gazebo başlamıyor
- Gazebo Harmonic'in doğru kurulduğundan emin ol: `gz sim --version`
- PX4 derlemesini temizle ve yeniden derle: `make distclean && make px4_sitl gz_x500`

### Drone'lar bağlanmıyor
- MicroXRCEAgent'ın çalıştığından emin ol (tmux pane 0)
- ROS 2 ortamının kaynaklandığından emin ol
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` export edildiğinden emin ol

### Offboard moduna geçemiyor
- 3 saniye bekleme süresinin yeterli olduğundan emin ol
- PX4 parametrelerinin doğru ayarlandığından emin ol (NAV_DLL_ACT, COM_RCL_ACT, NAV_RCL_ACT)

## 📚 Kaynaklar

- [PX4 Documentation](https://docs.px4.io/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Micro XRCE-DDS](https://micro-xrce-dds.readthedocs.io/)

## 👤 Yazar

**Ömer Faruk Çelik**
- Student ID: 220260138
- Advisor: Prof. Dr. Gülşah Karaduman
- Institution: Computer Engineering Design Project

---

**İyi çalışmalar! 🚁**
