import time
import math
import board
import busio
import adafruit_bno055

# BNO055 Setup
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c)

# Parameter robot
r = 0.05          # Jari-jari roda (meter)
Lx = 0.15         # Jarak dari pusat ke roda kiri-kanan (meter)
Ly = 0.15         # Jarak dari pusat ke roda depan-belakang (meter)
L = Lx + Ly       # Kombinasi jarak untuk inverse kinematics
cpr = 1000        # Count per revolution encoder

# Inisialisasi posisi robot
x = 0.0
y = 0.0
theta = 0.0

# Simulasi pembacaan encoder
def read_encoder():
    # Ganti dengan data real-time dari sensor encoder
    return [0, 0, 0, 0]  # [FL, FR, RL, RR] dalam tick

# Konversi tick menjadi jarak (m)
def tick_to_meter(ticks):
    return (2 * math.pi * r) * (ticks / cpr)

# Loop utama
last_ticks = [0, 0, 0, 0]
last_time = time.time()

while True:
    current_ticks = read_encoder()
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    delta_ticks = [curr - last for curr, last in zip(current_ticks, last_ticks)]
    delta_s = [tick_to_meter(t) for t in delta_ticks]

    # Urutan: [v1, v2, v3, v4] = [FL, FR, RL, RR]
    v1, v2, v3, v4 = delta_s

    # Inverse Kinematics (dalam local frame)
    Vx = (v1 + v2 + v3 + v4) / 4
    Vy = (-v1 + v2 + v3 - v4) / 4
    Wz = (-v1 + v2 - v3 + v4) / (4 * L)

    # Ambil orientasi yaw dari IMU (radian)
    yaw_deg = bno.euler[0] or 0.0
    yaw = math.radians(yaw_deg)

    # Update posisi global
    dx = Vx * math.cos(yaw) - Vy * math.sin(yaw)
    dy = Vx * math.sin(yaw) + Vy * math.cos(yaw)
    dtheta = Wz

    x += dx
    y += dy
    theta = yaw  # override dari IMU

    print(f"Posisi: x={x:.2f} m, y={y:.2f} m, θ={math.degrees(theta):.2f}°")

    last_ticks = current_ticks
    time.sleep(0.05)
