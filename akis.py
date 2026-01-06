import matplotlib.pyplot as plt
import matplotlib.patches as patches

def draw_box(ax, x, y, w, h, text, color='#e6f2ff', edge='#0052cc'):
    # Kutucuk
    rect = patches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1", 
                                  linewidth=2, edgecolor=edge, facecolor=color)
    ax.add_patch(rect)
    # Metin
    ax.text(x + w/2, y + h/2, text, ha='center', va='center', fontsize=10, fontweight='bold', color='#333333')
    return (x + w/2, y) # Alt orta nokta (bağlantı için)

def draw_arrow(ax, x1, y1, x2, y2):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="->", lw=2, color='#444444'))

fig, ax = plt.subplots(figsize=(10, 12))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis('off')

# --- 1. KULLANICI KATMANI (EN ÜST) ---
# QGC ve Terminal
qgc_center = draw_box(ax, 10, 85, 35, 8, "Kullanıcı Arayüzü\n(QGroundControl)", color='#ffe6e6', edge='#cc0000')
term_center = draw_box(ax, 55, 85, 35, 8, "Otomasyon Betiği\n(baslat.sh)", color='#e6ffe6', edge='#006600')

# --- 2. ROS 2 KONTROL KATMANI (ORTA - BÜYÜK KUTU) ---
# Dış Çerçeve (ROS 2 Node)
main_rect = patches.Rectangle((5, 45), 90, 30, linewidth=2, edgecolor='#0052cc', facecolor='#f0f8ff', linestyle='--')
ax.add_patch(main_rect)
ax.text(10, 72, "ROS 2: Görev Orkestratörü (oto.py)", fontsize=11, fontweight='bold', color='#0052cc')

# İç Bileşenler (CRC Yapıları)
strat_center = draw_box(ax, 35, 62, 30, 6, "Stratejik Planlayıcı\n(Global Rota)", color='#ffffff', edge='#0052cc')
tact_center = draw_box(ax, 15, 50, 30, 6, "Taktiksel Planlayıcı\n(Grid/Kaçınma)", color='#ffffff', edge='#0052cc')
refl_center = draw_box(ax, 55, 50, 30, 6, "IHA Agent Kontrolcüsü\n(Ajan Nesneleri)", color='#ffffff', edge='#0052cc')

# --- 3. ARA KATMAN (MIDDLEWARE) ---
mw_center = draw_box(ax, 30, 30, 40, 6, "Middleware Köprüsü\n(Micro-XRCE-DDS Agent)", color='#fff2cc', edge='#d6b656')

# --- 4. SİMÜLASYON KATMANI (EN ALT) ---
px4_center = draw_box(ax, 15, 10, 30, 10, "Uçuş Kontrolcüsü\n(PX4 Autopilot)", color='#e0e0e0', edge='#666666')
gz_center = draw_box(ax, 55, 10, 30, 10, "Fizik Motoru\n(Gazebo Garden)", color='#e0e0e0', edge='#666666')

# --- BAĞLANTILAR (OKLAR) ---
# Kullanıcı -> ROS 2
draw_arrow(ax, 27.5, 85, 50, 75) # QGC'den aşağı (Telemetri görselleştirme)
draw_arrow(ax, 72.5, 85, 50, 75) # Script'ten aşağı (Başlatma)

# ROS 2 İçi
draw_arrow(ax, 50, 62, 30, 56) # Stratejik -> Taktiksel
draw_arrow(ax, 50, 62, 70, 56) # Stratejik -> Ajan

# ROS 2 -> Middleware
draw_arrow(ax, 50, 45, 50, 36) # ROS'tan Middleware'e

# Middleware -> PX4
draw_arrow(ax, 50, 30, 30, 20) # MAVLink

# PX4 <-> Gazebo
ax.annotate("", xy=(30, 15), xytext=(55, 15), arrowprops=dict(arrowstyle="<->", lw=2, color='#444444'))
ax.text(42.5, 16, "Sensor/\nMotor", ha='center', fontsize=8)

# QGC -> PX4 (Direkt UDP Bağlantısı)
# Eğri ok çizimi (Manuel)
style = "Simple, tail_width=0.5, head_width=4, head_length=8"
kw = dict(arrowstyle=style, color="#cc0000", alpha=0.3)
patch = patches.FancyArrowPatch((27.5, 85), (30, 20), connectionstyle="arc3,rad=-0.3", **kw)
ax.add_patch(patch)
ax.text(15, 40, "UDP Broadcast\n(MAVLink)", color='#cc0000', alpha=0.5, fontsize=8)

plt.title("Hibrit ve Hiyerarşik Sürü İHA Kontrol Sistemi Mimarisi", fontsize=14, pad=20)
plt.savefig("sistem_mimarisi.png", dpi=300, bbox_inches='tight')
plt.show()
