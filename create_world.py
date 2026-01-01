import random
import os

# --- AYARLAR ---
WORLD_NAME = "pro_orman"
TREE_COUNT = 60      # Ağaç Sayısı
ROCK_COUNT = 20      # Kaya Sayısı
HOUSE_COUNT = 12     # Ev Sayısı
AREA_SIZE = 70       # Harita Genişliği
ROAD_WIDTH = 8       # Yol Genişliği

# --- SDF BAŞLANGICI ---
sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>

    <scene>
      <ambient>0.8 0.8 0.8</ambient>
      <background>0.6 0.7 0.9</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- ZEMİN -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
          <material>
            <ambient>0.2 0.35 0.15 1</ambient>
            <diffuse>0.2 0.35 0.15 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- YOL 1 (Kuzey-Güney) -->
    <model name="road_ns">
      <static>true</static>
      <pose>0 0 0.01 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>{ROAD_WIDTH} 150 0.02</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <collision name="col">
          <geometry><box><size>{ROAD_WIDTH} 150 0.02</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- YOL 2 (Doğu-Batı) -->
    <model name="road_ew">
      <static>true</static>
      <pose>0 0 0.01 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><box><size>150 {ROAD_WIDTH} 0.02</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <collision name="col">
          <geometry><box><size>150 {ROAD_WIDTH} 0.02</size></box></geometry>
        </collision>
      </link>
    </model>
'''

# --- YARDIMCI FONKSİYON ---
def is_on_road(x, y):
    limit = (ROAD_WIDTH / 2) + 2
    if abs(x) < limit or abs(y) < limit:
        return True
    return False

# --- EVLERİ OLUŞTUR ---
for i in range(HOUSE_COUNT):
    while True:
        x = random.uniform(-AREA_SIZE, AREA_SIZE)
        y = random.uniform(-AREA_SIZE, AREA_SIZE)
        if not is_on_road(x, y): 
            if abs(x) < 20 or abs(y) < 20: 
                break
    
    w = random.uniform(4, 8)
    d = random.uniform(4, 8)
    h = random.uniform(3, 5)
    
    colors = ["0.6 0.3 0.2 1", "0.8 0.8 0.8 1", "0.4 0.4 0.5 1"]
    color = random.choice(colors)

    sdf_content += f'''
    <model name="house_{i}">
      <static>true</static>
      <pose>{x} {y} {h/2} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{w} {d} {h}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>{w} {d} {h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>
    '''

# --- AĞAÇLARI OLUŞTUR ---
count = 0
attempts = 0
while count < TREE_COUNT and attempts < 1000:
    attempts += 1
    x = random.uniform(-AREA_SIZE, AREA_SIZE)
    y = random.uniform(-AREA_SIZE, AREA_SIZE)
    
    if is_on_road(x, y): continue
    
    scale = random.uniform(0.8, 1.8)
    sdf_content += f'''
    <include>
      <name>pine_tree_{count}</name>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree</uri>
      <pose>{x} {y} 0 0 0 0</pose>
      <scale>{scale} {scale} {scale}</scale>
    </include>
    '''
    count += 1

# --- KAYALARI OLUŞTUR ---
count = 0
attempts = 0
while count < ROCK_COUNT and attempts < 1000:
    attempts += 1
    x = random.uniform(-AREA_SIZE, AREA_SIZE)
    y = random.uniform(-AREA_SIZE, AREA_SIZE)
    
    if is_on_road(x, y): continue
    
    s_x = random.uniform(1, 3)
    s_y = random.uniform(1, 3)
    s_z = random.uniform(0.5, 1.5)
    
    sdf_content += f'''
    <model name="rock_{count}">
      <static>true</static>
      <pose>{x} {y} {s_z/2} 0 0 {random.uniform(0,3)}</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{s_x} {s_y} {s_z}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>{s_x} {s_y} {s_z}</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>
    '''
    count += 1  # <--- İŞTE UNUTULAN PARÇA BUYDU! ARTIK EKLENDİ.

sdf_content += '</world></sdf>'

# Dosyayı kaydet
path = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds/pro_orman.sdf")
with open(path, "w") as f:
    f.write(sdf_content)

print(f"DÜZELTİLMİŞ DÜNYA OLUŞTURULDU: {path}")
