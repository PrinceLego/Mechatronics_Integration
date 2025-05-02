import numpy as np
import math

def Triangulation_control(RSSI1, RSSI2, RSSI3):
    # 檢查是否有任一 RSSI 是 None
    if RSSI1 is None or RSSI2 is None or RSSI3 is None:
        print("⚠️ RSSI 資料不完整，停止移動")
        return 0, 0, 0

    def trilateration(P1, P2, P3, d1, d2, d3):
        x1, y1 = P1
        x2, y2 = P2
        x3, y3 = P3

        A = np.array([
            [2 * (x2 - x1), 2 * (y2 - y1)],
            [2 * (x3 - x1), 2 * (y3 - y1)]
        ])

        B = np.array([
            d1**2 - d2**2 + x2**2 - x1**2 + y2**2 - y1**2,
            d1**2 - d3**2 + x3**2 - x1**2 + y3**2 - y1**2
        ])

        pos = np.linalg.inv(A).dot(B)
        x, y = pos

        distance = np.sqrt(x**2 + y**2)

        # 轉換角度：車頭為 0 度、順時針為正
        angle_rad = math.atan2(y, x)
        angle_deg = (90 - math.degrees(angle_rad)) % 360

        return x, y, distance, angle_deg

    def compute_angular_velocity(target_angle_deg, current_heading_deg=0.0, k=0.01):
        angle_error = target_angle_deg - current_heading_deg
        angle_error = (angle_error + 180) % 360 - 180
        omega = k * angle_error
        return omega, angle_error

    def compute_linear_velocity(distance, angle_error_deg, angle_tolerance_deg=5, max_speed=1.0, k_v=0.5):
        if abs(angle_error_deg) < angle_tolerance_deg:
            velocity = min(k_v * distance, max_speed)
        else:
            velocity = 0.0
        return velocity

    angle_tolerance = 5
    p1 = (0, 0)
    p2 = (3, 0)
    p3 = (0, 4)

    # 三角定位
    x, y, distance, angle = trilateration(p1, p2, p3, RSSI1, RSSI2, RSSI3)

    # 控制計算
    omega, angle_error = compute_angular_velocity(angle)
    velocity = compute_linear_velocity(distance, angle_error, angle_tolerance)

    # === 狀態輸出 ===
    print(f"目標座標: ({x:.2f}, {y:.2f})")
    print(f"距離: {distance:.2f} m")
    print(f"角度（相對車頭順時針）: {angle:.2f}°")
    print(f"角度誤差: {angle_error:.2f}°")
    print(f"角速度 ω: {omega:.4f} rad/s")
    print(f"前進速度 V: {velocity:.2f} m/s")

    return velocity, 0, omega


# === 測試 ===
print(Triangulation_control(5.0, 4.0, 3.0))     # 正常情況
print(Triangulation_control(None, 4.0, 3.0))  # RSSI 缺失
