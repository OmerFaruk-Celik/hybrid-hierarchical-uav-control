#!/usr/bin/env python3
import rclpy
import threading
import sys
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- PX4 MESAJ TİPLERİ ---
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import SensorCombined        # Jiroskop/İvmeölçer
from px4_msgs.msg import VehicleGlobalPosition # GÜNCELLEME: GPS için en garantisi bu
from px4_msgs.msg import VehicleLocalPosition  # Konum ve Hız
from px4_msgs.msg import BatteryStatus         # Batarya

class SafeSwarmController(Node):
    def __init__(self):
        super().__init__('safe_swarm_node')

        # PX4 QoS Ayarları (Veri alamazsan burası kritiktir)
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.active_drones = {}
        
        # Telemetri Değişkenleri
        self.monitor_sub = None       
        self.print_counter = 0        
        self.monitoring_info = ""     

        # 10 Hz Timer (Offboard kontrol sinyali için)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_drone_namespace(self, vehicle_id):
        if vehicle_id == 0: return ""
        return f"/px4_{vehicle_id}"

    # --- KONTROL KISMI ---
    def register_drone(self, vehicle_id, start_x, start_y):
        ns = self.get_drone_namespace(vehicle_id)
        prefix = f"{ns}/fmu/in"
        
        print(f"\n[SİSTEM] Drone {vehicle_id} Bağlanıyor... (Başlangıç: {start_x}, {start_y})")

        offboard_pub = self.create_publisher(OffboardControlMode, f'{prefix}/offboard_control_mode', self.qos_profile)
        traj_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/trajectory_setpoint', self.qos_profile)
        cmd_pub = self.create_publisher(VehicleCommand, f'{prefix}/vehicle_command', self.qos_profile)

        self.active_drones[vehicle_id] = {
            'pubs': {'offboard': offboard_pub, 'traj': traj_pub, 'cmd': cmd_pub},
            'target': [float(start_x), float(start_y), -5.0], 
            'state': 'INIT'
        }

        time.sleep(0.5)
        self.set_mode_offboard(vehicle_id)
        time.sleep(0.5)
        self.arm_vehicle(vehicle_id)
        print(f"[BAŞARILI] Drone {vehicle_id} Motorları Çalıştırdı ve Yükseliyor!")

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
            msg_traj.velocity = [float('nan'), float('nan'), float('nan')]
            msg_traj.acceleration = [float('nan'), float('nan'), float('nan')]
            msg_traj.timestamp = timestamp
            drone_data['pubs']['traj'].publish(msg_traj)

    # --- TELEMETRİ İZLEME KISMI (DÜZELTİLDİ) ---
    def start_monitoring(self, drone_id, data_type):
        if self.monitor_sub:
            self.destroy_subscription(self.monitor_sub)
            self.monitor_sub = None
        
        self.print_counter = 0
        ns = self.get_drone_namespace(drone_id)
        prefix = f"{ns}/fmu/out"
        
        topic_name = ""
        callback_func = None
        msg_type = None

        # 1. Jiroskop (Topic listende _v1 yoktu, düz kullanıyoruz)
        if data_type == "jiroskop":
            topic_name = f"{prefix}/sensor_combined"
            msg_type = SensorCombined
            callback_func = self.cb_gyro
        
        # 2. GPS (Listende vehicle_global_position var, bunu kullanacağız)
        elif data_type == "gps":
            topic_name = f"{prefix}/vehicle_global_position" 
            msg_type = VehicleGlobalPosition
            callback_func = self.cb_gps
            
        # 3. Konum (Listende _v1 VARDI, o yüzden _v1 ekledik)
        elif data_type == "konum":
            topic_name = f"{prefix}/vehicle_local_position_v1" 
            msg_type = VehicleLocalPosition
            callback_func = self.cb_pos
            
        # 4. Hız (Konum ile aynı mesaj)
        elif data_type == "hiz":
            topic_name = f"{prefix}/vehicle_local_position_v1"
            msg_type = VehicleLocalPosition
            callback_func = self.cb_vel
            
        # 5. Batarya (Listende _v1 VARDI)
        elif data_type == "batarya":
            topic_name = f"{prefix}/battery_status_v1"
            msg_type = BatteryStatus
            callback_func = self.cb_bat
            
        else:
            print(f"HATA: '{data_type}' geçersiz. (jiroskop, gps, konum, hiz, batarya)")
            return

        print(f"\033[94m>>> Drone {drone_id} [{data_type.upper()}] verisi ({topic_name}) izleniyor...\033[0m")
        # Abone ol
        self.monitor_sub = self.create_subscription(msg_type, topic_name, callback_func, self.qos_profile)
        self.monitoring_info = f"Drone {drone_id} - {data_type.upper()}"

    # --- Callback Fonksiyonları ---
    def cb_gyro(self, msg):
        self.print_counter += 1
        if self.print_counter % 20 == 0:
            print(f"\033[92m[{self.monitoring_info}] Gyro X:{msg.gyro_rad[0]:.2f} Y:{msg.gyro_rad[1]:.2f} Z:{msg.gyro_rad[2]:.2f}\033[0m")

    def cb_gps(self, msg):
        self.print_counter += 1
        if self.print_counter % 5 == 0:
            # VehicleGlobalPosition direk derece verir, 1e7 bölmeye gerek YOKTUR (SensorGps'ten farkı budur)
            print(f"\033[93m[{self.monitoring_info}] Lat:{msg.lat:.6f} Lon:{msg.lon:.6f} Alt:{msg.alt:.2f}m\033[0m")

    def cb_pos(self, msg):
        self.print_counter += 1
        if self.print_counter % 10 == 0:
            print(f"\033[96m[{self.monitoring_info}] X:{msg.x:.2f}m Y:{msg.y:.2f}m Z:{msg.z:.2f}m\033[0m")

    def cb_vel(self, msg):
        self.print_counter += 1
        if self.print_counter % 10 == 0:
            total_v = (msg.vx**2 + msg.vy**2)**0.5
            print(f"\033[95m[{self.monitoring_info}] Vx:{msg.vx:.2f} Vy:{msg.vy:.2f} Vz:{msg.vz:.2f} Yatay:{total_v:.2f}m/s\033[0m")

    def cb_bat(self, msg):
        self.print_counter += 1
        if self.print_counter % 5 == 0:
            print(f"\033[91m[{self.monitoring_info}] Voltaj:{msg.voltage_v:.2f}V Kalan:%{msg.remaining*100:.1f}\033[0m")

    def stop_monitoring(self):
        if self.monitor_sub:
            self.destroy_subscription(self.monitor_sub)
            self.monitor_sub = None
            print("--- İzleme Durduruldu ---")

    # --- HELPER FONKSİYONLAR ---
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

    def update_target(self, v_id, x, y, z):
        if v_id not in self.active_drones:
            self.register_drone(v_id, x, y)
        self.active_drones[v_id]['target'] = [float(x), float(y), float(z)]
        print(f"--> Drone {v_id} Yeni Rota: {x}, {y}, {z}")

    def land_all(self):
        for v_id in self.active_drones:
            self.land_vehicle(v_id)
        print("Tüm dronlara iniş emri verildi.")

def main():
    rclpy.init()
    node = SafeSwarmController()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("=========================================")
    print("   SWARM KONTROL VE TELEMETRİ v4.2       ")
    print("=========================================")
    print("1. KONTROL KOMUTLARI (4 Parametre):")
    print("   [id] [x] [y] [z]   -> Örn: 1 0 0 -5")
    print("\n2. İZLEME KOMUTLARI (2 Parametre):")
    print("   [id] [veri]        -> Örn: 0 gps")
    print("   Veri Tipleri: jiroskop, gps, konum, hiz, batarya")
    print("   Durdurmak için: dur")
    print("\n3. DİĞER:")
    print("   land all / exit / q")
    print("=========================================")

    while True:
        try:
            raw_input = input("\nKomut > ").strip().lower()
            inp = raw_input.split()
            if not inp: continue
            
            if inp[0] in ['exit', 'q']:
                node.land_all(); time.sleep(1); break
            
            if inp[0] == 'land':
                if len(inp) > 1 and inp[1] == 'all': node.land_all()
                elif len(inp) > 1: node.land_vehicle(int(inp[1]))
                continue

            if inp[0] == 'dur':
                node.stop_monitoring()
                continue

            if len(inp) == 4:
                try:
                    v_id = int(inp[0])
                    x, y, z = inp[1], inp[2], inp[3]
                    node.update_target(v_id, x, y, z)
                except ValueError:
                    print("Hata: Koordinatlar sayı olmalı.")

            elif len(inp) == 2:
                try:
                    v_id = int(inp[0])
                    data_type = inp[1]
                    if not data_type.replace('.', '', 1).isdigit(): 
                        node.start_monitoring(v_id, data_type)
                    else:
                        print("Hata: Eksik parametre.")
                except ValueError:
                    print("Hata: ID sayı olmalı.")
            else:
                print("Hata: Yanlış format.")

        except Exception as e:
            print(f"Sistem Hatası: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
