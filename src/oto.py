#!/usr/bin/env python3
"""
OTO.PY - Çoklu Grup Görev Senaryosu (Multitasking Swarm)
Senaryo:
- Grup A (0,1): Nesne Tespiti (Sabit Gözlem)
- Grup B (2,3): Alan Tarama (Grid Pattern)
- Grup C (4,5): Hedef İmha (Dalış Manevrası)
"""

import rclpy
import threading
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

# --- AYARLAR ---
CRUISE_ALTITUDE = -30.0  # Seyir İrtifası (30m)
ATTACK_ALTITUDE = -5.0   # Dalış İrtifası (5m)
SCAN_SIZE = 20.0         # Tarama alanı genişliği

class MultiMissionSwarm(Node):
    def __init__(self):
        super().__init__('multi_mission_swarm')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.active_drones = {}
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_drone_namespace(self, vehicle_id):
        if vehicle_id == 0: return ""
        return f"/px4_{vehicle_id}"

    # --- KURULUM ---
    def setup_drone(self, vehicle_id, start_x, start_y):
        ns = self.get_drone_namespace(vehicle_id)
        prefix = f"{ns}/fmu/in"
        
        self.active_drones[vehicle_id] = {
            'pubs': {
                'offboard': self.create_publisher(OffboardControlMode, f'{prefix}/offboard_control_mode', self.qos_profile),
                'traj': self.create_publisher(TrajectorySetpoint, f'{prefix}/trajectory_setpoint', self.qos_profile),
                'cmd': self.create_publisher(VehicleCommand, f'{prefix}/vehicle_command', self.qos_profile)
            },
            'target': [float(start_x), float(start_y), 0.0],
            'home': [float(start_x), float(start_y)]
        }

    # --- KALKIŞ ---
    def takeoff_drone(self, vehicle_id):
        # Offboard Mode
        self.set_mode_offboard(vehicle_id)
        time.sleep(0.2)
        # Arm
        self.arm_vehicle(vehicle_id)
        # Yüksel
        curr_x = self.active_drones[vehicle_id]['target'][0]
        curr_y = self.active_drones[vehicle_id]['target'][1]
        self.update_target(vehicle_id, curr_x, curr_y, CRUISE_ALTITUDE)

    # --- TEMEL FONKSİYONLAR ---
    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        for v_id, drone_data in self.active_drones.items():
            # Heartbeat
            msg_mode = OffboardControlMode()
            msg_mode.position = True
            msg_mode.velocity = False
            msg_mode.acceleration = False
            msg_mode.timestamp = timestamp
            drone_data['pubs']['offboard'].publish(msg_mode)

            # Setpoint
            msg_traj = TrajectorySetpoint()
            msg_traj.position = drone_data['target']
            msg_traj.yaw = 0.0 # Yaw kontrolü eklenebilir
            msg_traj.velocity = [float('nan')] * 3
            msg_traj.acceleration = [float('nan')] * 3
            msg_traj.timestamp = timestamp
            drone_data['pubs']['traj'].publish(msg_traj)

    def update_target(self, v_id, x, y, z):
        if v_id in self.active_drones:
            self.active_drones[v_id]['target'] = [float(x), float(y), float(z)]

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
        print("\n[İNİŞ] Tüm dronlar inişe geçiyor...")
        for v_id in self.active_drones:
            self.land_vehicle(v_id)

# --- GRUP GÖREV FONKSİYONLARI ---
# Bu fonksiyonlar aynı anda (paralel) çalışacak

def mission_group_detect(node):
    """GRUP A (0, 1): Nesne Tespiti - Sabit Gözlem"""
    ids = [0, 1]
    target_x, target_y = 100.0, 50.0 # Kuzey-Doğu
    print(f"[GRUP A] Gözcüler (0,1) hedefe gidiyor: ({target_x}, {target_y})")
    
    # 1. Hedefe Git
    node.update_target(0, target_x, target_y, CRUISE_ALTITUDE)
    node.update_target(1, target_x + 5, target_y + 5, CRUISE_ALTITUDE) # Formasyonlu
    time.sleep(20) # Ulaşım süresi

    # 2. Gözlem Yap (Simüle: Olduğu yerde bekle)
    print("[GRUP A] Bölgeye ulaşıldı. Gözlem yapılıyor (40sn)...")
    # Burada kamera döndürme vb. eklenebilir. Şimdilik konum koruma.
    time.sleep(40)
    
    print("[GRUP A] Gözlem tamamlandı. Geri dönüş bekleniyor.")

def mission_group_scan(node):
    """GRUP B (2, 3): Alan Tarama - Grid Deseni"""
    ids = [2, 3]
    center_x, center_y = 100.0, -50.0 # Kuzey-Batı
    print(f"[GRUP B] Tarayıcılar (2,3) hedefe gidiyor: ({center_x}, {center_y})")

    # 1. Başlangıç Noktasına Git
    # Drone 2 ve 3, tarama alanının köşelerine gitsin
    start_x = center_x - SCAN_SIZE/2
    start_y = center_y - SCAN_SIZE/2
    
    node.update_target(2, start_x, start_y, CRUISE_ALTITUDE)
    node.update_target(3, start_x - 5, start_y - 5, CRUISE_ALTITUDE) # Wingman
    time.sleep(20)

    # 2. Tarama Başlasın (4 Köşe Gezilecek)
    print("[GRUP B] Tarama başlatıldı (Zikzak Pattern)...")
    
    # Nokta 1 (İleri)
    node.update_target(2, start_x + SCAN_SIZE, start_y, CRUISE_ALTITUDE)
    node.update_target(3, start_x + SCAN_SIZE -5, start_y -5, CRUISE_ALTITUDE)
    time.sleep(10)
    
    # Nokta 2 (Yana Kay)
    node.update_target(2, start_x + SCAN_SIZE, start_y + SCAN_SIZE, CRUISE_ALTITUDE)
    node.update_target(3, start_x + SCAN_SIZE -5, start_y + SCAN_SIZE -5, CRUISE_ALTITUDE)
    time.sleep(10)
    
    # Nokta 3 (Geri Dön)
    node.update_target(2, start_x, start_y + SCAN_SIZE, CRUISE_ALTITUDE)
    node.update_target(3, start_x -5, start_y + SCAN_SIZE -5, CRUISE_ALTITUDE)
    time.sleep(10)

    # Nokta 4 (Başlangıca Dön)
    node.update_target(2, start_x, start_y, CRUISE_ALTITUDE)
    node.update_target(3, start_x -5, start_y -5, CRUISE_ALTITUDE)
    time.sleep(10)
    
    print("[GRUP B] Alan taraması tamamlandı.")

def mission_group_attack(node):
    """GRUP C (4, 5): Hedef İmha - Dalış Manevrası"""
    ids = [4, 5]
    target_x, target_y = 150.0, 0.0 # Uzak Kuzey
    print(f"[GRUP C] Vurucular (4,5) hedefe gidiyor: ({target_x}, {target_y})")
    
    # 1. Hedefe Git
    node.update_target(4, target_x, target_y, CRUISE_ALTITUDE)
    node.update_target(5, target_x, target_y - 10, CRUISE_ALTITUDE)
    time.sleep(25) # Daha uzak olduğu için uzun süre

    # 2. Dalış Manevrası
    print("[GRUP C] HEDEF TESPİT EDİLDİ! DALIŞA GEÇİLİYOR!")
    
    # Hızlı Dalış (5 metreye in)
    node.update_target(4, target_x, target_y, ATTACK_ALTITUDE)
    node.update_target(5, target_x, target_y - 10, ATTACK_ALTITUDE)
    time.sleep(8) 
    
    print("[GRUP C] HEDEF İMHA EDİLDİ! YÜKSELİŞ!")
    
    # Hızlı Yükseliş
    node.update_target(4, target_x, target_y, CRUISE_ALTITUDE)
    node.update_target(5, target_x, target_y - 10, CRUISE_ALTITUDE)
    time.sleep(10)
    
    print("[GRUP C] Operasyon başarılı.")

# --- ANA GÖREV YÖNETİCİSİ ---
def run_mission_manager(node):
    try:
        # 1. KURULUM
        print("\n=== SWARM KOMUTA MERKEZİ BAŞLATILIYOR ===")
        # 6 Drone kaydı (2 sıra halinde kalkış pozisyonları)
        positions = [
            (0, 0), (2, 0),    # Grup A
            (-2, 0), (0, -2),  # Grup B
            (2, -2), (-2, -2)  # Grup C
        ]
        for i, pos in enumerate(positions):
            node.setup_drone(i, pos[0], pos[1])
        
        # 2. TOPLU KALKIŞ
        print("\n>>> FAZ 1: TOPLU KALKIŞ (Hedef: 30m)")
        print("Sinyaller oturtuluyor (5sn)...")
        time.sleep(5)
        
        for i in range(6):
            node.takeoff_drone(i)
            time.sleep(0.5)
        
        print("İrtifaya çıkılıyor (15sn)...")
        time.sleep(15)

        # 3. GÖREV DAĞILIMI (THREADING)
        print("\n>>> FAZ 2: GÖREV DAĞILIMI VE AYRILMA")
        print("Gruplar ayrılıyor ve görev bölgelerine intikal ediyor...")
        
        # Her grup için ayrı bir "Thread" (İş Parçacığı) oluşturuyoruz.
        # Bu sayede Grup B tarama yaparken, Grup C aynı anda dalış yapabilir.
        t1 = threading.Thread(target=mission_group_detect, args=(node,))
        t2 = threading.Thread(target=mission_group_scan, args=(node,))
        t3 = threading.Thread(target=mission_group_attack, args=(node,))
        
        # Görevleri başlat
        t1.start()
        t2.start()
        t3.start()
        
        # Tüm görevlerin bitmesini bekle
        t1.join()
        t2.join()
        t3.join()
        
        print("\n>>> TÜM GÖREVLER TAMAMLANDI. EKİPLER BİRLEŞİYOR.")

        # 4. EVE DÖNÜŞ
        print("\n>>> FAZ 3: EVE DÖNÜŞ (RTL)")
        # Her drone kendi "home" koordinatına dönsün
        for v_id, drone in node.active_drones.items():
            home = drone['home']
            node.update_target(v_id, home[0], home[1], CRUISE_ALTITUDE)
        
        print("Üsse dönülüyor (30sn)...")
        time.sleep(30)

        # 5. İNİŞ
        print("\n>>> FAZ 4: İNİŞ")
        node.land_all()
        time.sleep(10)
        print("[SON] Görev başarıyla tamamlandı.")

    except KeyboardInterrupt:
        print("\n[İPTAL] ACİL İNİŞ YAPILIYOR!")
        node.land_all()

def main():
    rclpy.init()
    node = MultiMissionSwarm()
    
    # ROS 2 Spin Thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Görev Yöneticisini Çalıştır
    run_mission_manager(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
