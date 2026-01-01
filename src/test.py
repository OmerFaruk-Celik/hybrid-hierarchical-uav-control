import rclpy
import threading
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class DroneInputControl(Node):
    def __init__(self):
        super().__init__('drone_input_control')

        # --- QoS Ayarları ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publisher'lar ---
        self.offboard_ctrl_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # --- Başlangıç Hedef Konumu (X=0, Y=0, Z=-5m) ---
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0 # Havada başlasın diye
        
        # --- Zamanlayıcı (Heartbeat - 10Hz) ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_set = False

    def timer_callback(self):
        # 1. Offboard Sinyali (Sürekli gitmeli)
        msg_mode = OffboardControlMode()
        msg_mode.position = True
        msg_mode.velocity = False
        msg_mode.acceleration = False
        msg_mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_mode_pub.publish(msg_mode)

        # 2. Konum Sinyali (Kullanıcının girdiği son değerleri sürekli basar)
        msg_traj = TrajectorySetpoint()
        msg_traj.position = [self.target_x, self.target_y, self.target_z]
        msg_traj.yaw = 0.0 # Kuzeye baksın
        msg_traj.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg_traj)

    # --- Komut Fonksiyonları ---
    def set_mode_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

    def arm_vehicle(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    
    def land_vehicle(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    def update_target(self, x, y, z):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)
        print(f"--> Hedef Güncellendi: X={x}, Y={y}, Z={z}")

def main():
    rclpy.init()
    drone_node = DroneInputControl()

    # ROS döngüsünü ayrı bir Thread'e atıyoruz ki input() burayı kilitlemesin
    spin_thread = threading.Thread(target=rclpy.spin, args=(drone_node,), daemon=True)
    spin_thread.start()

    print("--- DRONE KONTROL MERKEZİ ---")
    print("Dron otomatik olarak 5 metreye kalkacak.")
    print("Sonra komutları bekleyecek.")
    print("-----------------------------")
    
    # Otomatik Kalkış Rutini
    import time
    time.sleep(1)
    drone_node.set_mode_offboard()
    print("1. Offboard Moduna Geçildi...")
    time.sleep(1)
    drone_node.arm_vehicle()
    print("2. Motorlar Çalıştırıldı (Kalkış Başlıyor)...")
    
    print("\n--- KOMUT SİSTEMİ ---")
    print("Format: x y z (Örnek: 5 5 -10)")
    print("Uyarı: Yukarı çıkmak için Z EKSİ (-) olmalı!")
    print("İnmek için 'land' veya 'exit' yaz.")
    
    while True:
        try:
            user_input = input("\nYeni Koordinat (x y z): ")
            
            if user_input.lower() in ['exit', 'quit', 'q', 'land']:
                drone_node.land_vehicle()
                print("İniş yapılıyor... Program kapatılıyor.")
                break
            
            # Girdiyi parçala (Örn: "5 10 -5" -> 5.0, 10.0, -5.0)
            coords = user_input.split()
            
            if len(coords) == 3:
                drone_node.update_target(coords[0], coords[1], coords[2])
            else:
                print("HATA: Lütfen aralarında boşluk olan 3 sayı girin! (Örn: 2 2 -5)")
                
        except ValueError:
            print("HATA: Sadece sayı girin!")
        except Exception as e:
            print(f"Beklenmedik hata: {e}")

    drone_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
