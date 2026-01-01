import rclpy
import threading
import sys
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class SafeSwarmController(Node):
    def __init__(self):
        super().__init__('safe_swarm_node')

        # PX4 için en kritik ayar: BEST_EFFORT
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.active_drones = {}
        # 100ms'de bir sinyal gönder (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_drone_namespace(self, vehicle_id):
        # Dron ID'sine göre namespace belirle
        if vehicle_id == 0: return ""
        return f"/px4_{vehicle_id}"

    def register_drone(self, vehicle_id, start_x, start_y):
        ns = self.get_drone_namespace(vehicle_id)
        print(f"\n[SİSTEM] Drone {vehicle_id} Bağlanıyor... (Başlangıç: {start_x}, {start_y})")

        # Publisher'lar
        offboard_pub = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', self.qos_profile)
        traj_pub = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', self.qos_profile)
        cmd_pub = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', self.qos_profile)

        # Dronu sözlüğe kaydet
        # ÖNEMLİ: İlk hedefi, dronun DOĞDUĞU X ve Y koordinatlarıdır.
        # Böylece havalanırken sağa sola fırlamaz, sadece yukarı çıkar.
        self.active_drones[vehicle_id] = {
            'pubs': {'offboard': offboard_pub, 'traj': traj_pub, 'cmd': cmd_pub},
            'target': [float(start_x), float(start_y), -5.0], # Olduğu yerde 5m kalk
            'state': 'INIT'
        }

        # Hazırlık süresi ver
        time.sleep(0.5)
        self.set_mode_offboard(vehicle_id)
        time.sleep(0.5)
        self.arm_vehicle(vehicle_id)
        
        print(f"[BAŞARILI] Drone {vehicle_id} Motorları Çalıştırdı ve Yükseliyor!")

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        for v_id, drone_data in self.active_drones.items():
            # 1. Offboard Sinyali
            msg_mode = OffboardControlMode()
            msg_mode.position = True
            msg_mode.velocity = False
            msg_mode.acceleration = False
            msg_mode.timestamp = timestamp
            drone_data['pubs']['offboard'].publish(msg_mode)

            # 2. Konum Sinyali
            msg_traj = TrajectorySetpoint()
            msg_traj.position = drone_data['target']
            
            # --- KRİTİK DÜZELTME: YAW ARTIK 0.0 (KUZEY) ---
            # NaN yaparsan takla atar!
            msg_traj.yaw = 0.0 
            
            # Hız ve İvme kontrolünü drona bırakıyoruz (NaN)
            msg_traj.velocity = [float('nan'), float('nan'), float('nan')]
            msg_traj.acceleration = [float('nan'), float('nan'), float('nan')]
            msg_traj.timestamp = timestamp
            
            drone_data['pubs']['traj'].publish(msg_traj)

    # --- Komut Helperları ---
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
        # Eğer drone listede yoksa, varsayılan olarak (0,0)'da olduğunu varsayarak başlatır.
        # (Manuel başlatmalarda X,Y'yi doğru girmek kullanıcının sorumluluğundadır)
        if v_id not in self.active_drones:
            # Eğer ilk defa komut veriliyorsa, kullanıcıdan gelen X,Y ile başlatırız
            # Böylece ani hareket yapmaz.
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
    print("   GÜVENLİ SWARM KONTROL PANELİ v3.0     ")
    print("=========================================")
    print("Mantık: Drone ID ve Hedef Koordinat gir.")
    print("NOT: Drone'u ilk çağırdığında olduğu konuma yakın değer gir!")
    print("\nKomut Formatı: [id] [x] [y] [z]")
    print("Örnek: 1 0 0 -5   (1. Drone 5m yüksel)")
    print("Örnek: 2 2 0 -5   (2. Drone 2m ileride yüksel)")
    print("Çıkış: land all / exit")
    print("=========================================")

    while True:
        try:
            inp = input("\nKomut > ").lower().split()
            if not inp: continue
            
            if inp[0] in ['exit', 'q']:
                node.land_all(); time.sleep(1); break
            
            if inp[0] == 'land':
                if len(inp) > 1 and inp[1] == 'all': node.land_all()
                elif len(inp) > 1: node.land_vehicle(int(inp[1]))
                continue

            if len(inp) == 4:
                node.update_target(int(inp[0]), inp[1], inp[2], inp[3])
            else:
                print("Hata: 4 değer girilmeli (id x y z)")

        except Exception as e:
            print(f"Hata: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
