import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Grafik Ayarları
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_facecolor('#f0f0f0') # Hafif gri arka plan
plt.grid(True, linestyle='--', alpha=0.6)

# --- GRUP A: GÖZLEM (Sabit Nokta / Loiter) ---
# Drone 0 ve 1
target_a = [100, 200]
ax.plot(target_a[0], target_a[1], marker='o', markersize=15, color='blue', label='Grup A: Gözlem (Loiter)')
# Daire (Loiter alanı temsili)
circle = patches.Circle((target_a[0], target_a[1]), radius=15, edgecolor='blue', facecolor='none', linestyle=':')
ax.add_patch(circle)
ax.text(target_a[0], target_a[1]-30, "GRUP A\n(Sabit Gözlem)", ha='center', color='blue', fontweight='bold')

# --- GRUP B: TARAMA (Grid / Boustrophedon) ---
# Drone 2 ve 3
start_b = [200, 50]
width = 60
height = 80
# Zikzak Yolu Koordinatları
path_x = [200, 260, 260, 200, 200, 260]
path_y = [50, 50, 75, 75, 100, 100]

ax.plot(path_x, path_y, color='green', linewidth=2, linestyle='-', label='Grup B: Tarama (Grid)')
# Drone Simgeleri
ax.plot(200, 50, marker='^', markersize=10, color='green') # Başlangıç
ax.plot(260, 100, marker='>', markersize=10, color='green') # Bitiş
ax.text(230, 40, "GRUP B\n(Alan Tarama)", ha='center', color='green', fontweight='bold')

# --- GRUP C: SALDIRI (Direct Path / Dive) ---
# Drone 4 ve 5
start_c = [50, 50]
target_c = [50, 250]

# Ok çizimi
ax.arrow(start_c[0], start_c[1], 0, 180, head_width=10, head_length=15, fc='red', ec='red', label='Grup C: Saldırı')
ax.text(60, 150, "GRUP C\n(Hedef İmha)", ha='left', color='red', fontweight='bold')
ax.text(50, 260, "HEDEF\n(Dalış Noktası)", ha='center', color='darkred', fontsize=9)

# --- BAŞLANGIÇ NOKTASI (HOME) ---
ax.plot(0, 0, marker='s', markersize=12, color='black', label='Home (Kalkış)')
ax.text(0, -20, "HOME", ha='center', fontweight='bold')

# Yolların Home'dan çıkışı (Temsili)
ax.plot([0, 100], [0, 200], 'b--', alpha=0.3)
ax.plot([0, 200], [0, 50], 'g--', alpha=0.3)
ax.plot([0, 50], [0, 50], 'r--', alpha=0.3)

# Eksen ve Başlıklar
ax.set_title("Otonom Sürü Görev Dağılımı ve Yörünge Planlaması", fontsize=14, pad=20)
ax.set_xlabel("Doğu (Y) - Metre")
ax.set_ylabel("Kuzey (X) - Metre")
ax.legend(loc='lower right')
plt.axis('equal')

# Kaydet ve Göster
plt.savefig("gorev_algoritma_semasi.png", dpi=300, bbox_inches='tight')
plt.show()
