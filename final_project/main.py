from bleak import BleakClient
import pygame
import asyncio
import math
import struct
import requests
import numpy as np
import cv2
import time
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
from bleak import BleakScanner

# BLE 設定
device_address = "6BBBBE46-E627-92F7-7299-8DE80626783E"  # 設備 UUID
characteristic_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 特徵 UUID
ESP32_URL = "http://192.168.0.38:81/stream"

target_devices = {
    "290D0DE6-BFC1-D0B7-A8B4-845862F094CE": None,  # 設備1
    "6BBBBE46-E627-92F7-7299-8DE80626783E": None   # 設備2
}
last_detected_time = {addr: None for addr in target_devices}  # 記錄每個設備最後更新 RSSI 的時間
RSSI_TIMEOUT = 3.0  # 若超過此秒數未更新 RSSI，則視為 None


turning_radius=60
wheel_radius=55
# 初始化
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("未偵測到 PS5 搖桿")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# 搖桿
AXIS_NAMES = {
    0: "Left_X",   # 左搖桿 X
    1: "Left_Y",   # 左搖桿 Y
    2: "Right_X",  # 右搖桿 X
    3: "Right_Y",  # 右搖桿 Y
    4: "L2_Trigger",     # L2 
    5: "R2_Trigger"      # R2 
}
BUTTON_NAMES = {
    0: "Cross",      # 叉叉
    1: "Circle",     # 圈圈
    2: "Square",     # 方形
    3: "Triangle",   # 三角形
    4: "Select",      # Select
    5: "PS",         # PS
    6: "Options",    # Options
    7: "L3",         # L3
    8: "R3",         # R3
    9: "L1",         # L1
    10: "R1",        # R1
    11: "Up",        # 上
    12: "Down",      # 下
    13: "Left",      # 左
    14: "Right",     # 右
    15: "Touchpad",  # 觸控板按鍵
    16: "Button1",  # 新按鍵3
    17: "Button2",  # 新按鍵4
}

# 轉換函數
def get_frame():
    try:
        
        response = requests.get(ESP32_URL, stream=True, timeout=5)
        if response.status_code == 200:
            # 獲取
            bytes_data = b""
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                
                a = bytes_data.find(b'\xff\xd8')  # JPEG 起始標誌
                b = bytes_data.find(b'\xff\xd9')  # JPEG 結束標誌
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]  # 獲得 JPEG 
                    bytes_data = bytes_data[b+2:]  # 清理已處理的部分
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    return frame
        else:
            print(f"Failed to fetch image, HTTP status code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request error: {e}")
        return None
    
def get_speed(v, x, y, vmin, vmax, o, omax):
    def polar_to_cartesian(r, theta):
        return r * math.cos(math.radians(theta)), r * math.sin(math.radians(theta))

    omega = o * omax if abs(o) > 0.5 else 0
    if v > 0:
        speed = v * (vmax - vmin)
        angle = (90 - math.degrees(math.atan2(-y, x))) % 360
        vx, vy = polar_to_cartesian(speed, angle)
    else:
        speed, vx, vy, angle = 0, 0, 0, 0
    return speed, angle, vx, vy, omega

def mecanum_v(vx, vy, omega,R,r):
    omega=np.deg2rad(omega)
    return (vx - vy - omega*R)/r, (vx + vy + omega*R)/r, (vx + vy - omega*R)/r, (vx - vy + omega*R)/r

def l298n(Vfl, Vfr, Vrl, Vrr):
    def to_bytes(value):
        # 限制數值範圍在 0 到 255 
        return min(max(0, value), 255)

    def motor_control(V):
        if V == 0:
            return [0, 0, 0]
        direction = [1, 0] if V > 0 else [0, 1]
        speed = int(abs(V) / 14.15 * 255)  # 根據 V 計算速度
        return direction + [to_bytes(speed)]
    return motor_control(Vfl) + motor_control(Vfr) + motor_control(Vrl) + motor_control(Vrr)

def apply_perspective_transform_and_draw_grid_on_image(image_path,pitch=-45,h=0.09,HFOV = 70.42,VFOV = 43.3):
    
    
    """
    輸入
    image_path:影像來源
    pitch:相機傾斜角度
    h:相機高度(m)
    HFOV:相機水平視角角度
    VFOV:相機垂直視角角度

    輸出
    透視變換後影像
    """

    yaw=0
    roll=0

    frame = image_path
    if frame is None:
        print("Error: Could not read image.")
        return

    interval = 100
    height, width = frame.shape[:2]

    # K矩陣
    def calculate_camera_intrinsics(W, H, HFOV, VFOV):
        f_x = W / (2 * np.tan(np.deg2rad(HFOV) / 2))
        f_y = H / (2 * np.tan(np.deg2rad(VFOV) / 2))
        K = np.array([
            [f_x, 0, W / 2],
            [0, f_y, H / 2],
            [0, 0, 1]
        ])
        return K

    # 旋轉矩陣
    def rotation_matrix(yaw, pitch, roll):
        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])

        R_roll = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        return R_yaw @ R_pitch @ R_roll


    def pixel_to_world(u, v, K, R, h):
        uv1 = np.array([u, height - v, 1.0])
        x_n = np.linalg.inv(K) @ uv1
        X_c = R @ x_n
        X_w = h * X_c[0] / X_c[2]
        Y_w = h * X_c[1] / X_c[2]
        return X_w, Y_w

    def output_pixel_num(dst_pts):
        aspect_ratio_img = width / height
        aspect_ratio_pts = (np.max(dst_pts[:, 0]) - np.min(dst_pts[:, 0])) / (np.max(dst_pts[:, 1]) - np.min(dst_pts[:, 1]))

        if aspect_ratio_img < aspect_ratio_pts:
            amp = width / (np.max(dst_pts[:, 0]) - np.min(dst_pts[:, 0]))
        else:
            amp = height / (np.max(dst_pts[:, 1]) - np.min(dst_pts[:, 1]))

        X_w_test = amp * (dst_pts[:, 0] - np.min(dst_pts[:, 0]))
        Y_w_test = amp * (np.max(dst_pts[:, 1]) - dst_pts[:, 1])

        return np.array(list(zip(X_w_test, Y_w_test)), dtype=np.float32)

    def mesh_point_draw(po, resulttt, num=2):
        cv2.circle(resulttt, po, 1, (0, 0, 255), -1)
        world_coord = pixel_to_world(po[0], po[1], K, R, h)
        cv2.putText(resulttt, str(tuple(round(coord, num) for coord in world_coord)), (po[0] + 5, po[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

    def save_frame(name, show_name, photo):
        current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        cv2.imwrite(f"/Users/prince_lego/Desktop/AAAA/{name}/{name}_{current_time}.jpg", photo)
        cv2.imshow(show_name, photo)

    K = calculate_camera_intrinsics(width, height, HFOV, VFOV)
    R = rotation_matrix(np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll))

    src_pts = np.array([[0, 0],             # 左上
                        [width, 0],         # 右上
                        [0, height],        # 左下
                        [width, height]],   # 右下
                    dtype=np.float32)

    dst_pts = np.array([pixel_to_world(0, 0, K, R, h),              # 左上
                        pixel_to_world(width, 0, K, R, h),          # 右上
                        pixel_to_world(0, height, K, R, h),         # 左下
                        pixel_to_world(width, height, K, R, h)],    # 右下
                    dtype=np.float32)

    dst_pts = output_pixel_num(dst_pts)

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    result = cv2.warpPerspective(frame, M, (width, height))
    result_mesh = result.copy()
    result_gps = result.copy()

    focus_point = (int(width / 2), int(height / 2))

    def world_to_pixel(X_w, Y_w, K, R, h):
        #轉換世界座標到相機坐標系
        X_c = np.array([X_w, Y_w, h])  #假設物體在地面，高度為 h

        #計算相機坐標到歪斜相機視角的變換
        X_n = np.linalg.inv(R) @ X_c

        #投影到 2D 平面
        uv1 = K @ X_n
        u = uv1[0] / uv1[2]  #正規化 x
        v = height - (uv1[1] / uv1[2])  #正規化 y，調整成 OpenCV 影像座標

        return int(u), int(v)  #返回整數像素座標

    # 繪製網格與交點
    for x in range(focus_point[0], width, interval):
        cv2.line(result_mesh, (x, 0), (x, height), (0, 0, 0), 1) 
        cv2.line(result_gps, (x, 0), (x, height), (0, 0, 0), 1)  
    for x in range(focus_point[0], 0, -interval):
        cv2.line(result_mesh, (x, 0), (x, height), (0, 0, 0), 1)  
        cv2.line(result_gps, (x, 0), (x, height), (0, 0, 0), 1)  
    for y in range(focus_point[1], height, interval):
        cv2.line(result_mesh, (0, y), (width, y), (0, 0, 0), 1)
        cv2.line(result_gps, (0, y), (width, y), (0, 0, 0), 1) 
    for y in range(focus_point[1], 0, -interval):
        cv2.line(result_mesh, (0, y), (width, y), (0, 0, 0), 1) 
        cv2.line(result_gps, (0, y), (width, y), (0, 0, 0), 1)

    # 標記交點
    for x in range(focus_point[0], width, interval):
        for y in range(focus_point[1], height, interval):
            mesh_point_draw((x, y), result_mesh)
        for y in range(focus_point[1], 0, -interval):
            mesh_point_draw((x, y), result_mesh)

    for x in range(focus_point[0], 0, -interval):
        for y in range(focus_point[1], height, interval):
            mesh_point_draw((x, y), result_mesh)
        for y in range(focus_point[1], 0, -interval):
            mesh_point_draw((x, y), result_mesh)

    u, v = world_to_pixel(0, 0, K, R, h)

    # 在影像上標示該點
    cv2.circle(result_mesh, (u, v), 5, (0, 255, 0), -1)  # 綠色點
    cv2.putText(result_mesh, "(0,0)", (u + 10, v - 10), 
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return result

# 監控影像可在此關閉
def detect_lane_angle_and_offset(image, y_heights):

    """

    影像處理參數可視情況調整

    輸入
    image:影像來源
    y_heights:偵測標線位置

    輸出
    角度差、距離差
    """

    height, width = image.shape[:2]
    center_x = width // 2  

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                  # 轉換為灰階
    blur = cv2.GaussianBlur(gray, (7, 7), 0)                        # 高斯模糊
    _, binary = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)    # 二值化
    edges = cv2.Canny(binary, 90, 200)                              # Canny 邊緣檢測

    # 顯示前處理影像

    box_width = 70
    box_height = 10
    n = 20  

    boxes = []
    image_before_adjustment = image.copy()  
    image_before_adjustment1 = image.copy()

    for i in range(n):
        y = height - (i + 1) * box_height
        x = center_x - box_width // 2
        boxes.append((x, y))
        cv2.rectangle(image_before_adjustment, (x, y), (x + box_width, y + box_height), (0, 255, 0), 2) 

    cv2.imshow("Original Image", image)
    cv2.imshow("Blurred Image", blur)
    cv2.imshow("Gray Image", gray)
    cv2.imshow("Binary Image", binary)
    #cv2.imshow("Boxes Before Adjustment", image_before_adjustment)  # 顯示未移動的框框

    center_points = []

    for idx, box in enumerate(boxes):
        x, y = box
        roi = edges[y:y + box_height, x:x + box_width]
        white_pixels = np.where(roi == 255)
        
        if len(white_pixels[0]) > 0:
            avg_x = int(np.mean(white_pixels[1]))  # 計算白色像素的平均 x 座標
            new_x = avg_x + x - box_width // 2
            center_points.append((new_x + box_width // 2, y + box_height // 2))

            cv2.rectangle(image_before_adjustment1, (new_x, y), (new_x + box_width, y + box_height), (0, 255, 0), 2)  # 綠色框框

    # 顯示調整後的框框圖
    cv2.imshow("Boxes After Adjustment", image_before_adjustment1)  

    # 擬合曲線
    if len(center_points) > 1:
        center_points = np.array(center_points)
        if len(center_points) > 2:
            curve = np.poly1d(np.polyfit(center_points[:, 1], center_points[:, 0], 2))
            y_vals = np.linspace(0, height, 100)
            x_vals = curve(y_vals)
            for i in range(1, len(x_vals)):
                cv2.line(image, (int(x_vals[i-1]), int(y_vals[i-1])), (int(x_vals[i]), int(y_vals[i])), (0, 255, 255), 2)

    # 計算角度差與偏移量
    angle_differences = []
    offset = []
    new_y_heights = [height - y for y in y_heights]
    cv2.line(image, (center_x, 0), (center_x, height), (0, 0, 255), 2)
    for y_target in new_y_heights:
        if len(center_points) > 2:
            offset.append(curve(y_target) - center_x)
            yellow_slope = (curve(y_target - 1) - curve(y_target + 1)) / 3 
            yellow_angle = np.arctan(yellow_slope) * 180 / np.pi            
            angle_differences.append(yellow_angle)

    # 距離差與角度差與邊緣偵測 用於監控 
    cv2.imshow("Canny Edges", edges)
    cv2.imshow("Distance Difference and Angle Difference", image)  

    return image, angle_differences, offset

def calculate_mecanum_wheel_speeds(angle_input, position_input):

    """
    輸入
    angle_input:各位置角度差
    y_heights:偵各位置標線距離差

    輸出
    車輛前進速度、橫移速度、角速度

    隸屬函數尚需依照實際車速調整
    """
        
    #定義輸入變數
    angle = ctrl.Antecedent(np.arange(-30, 30.1, 0.1), 'angle')
    position = ctrl.Antecedent(np.arange(-50, 50, 0.1), 'position')

    # 定義輸出變數
    Vx = ctrl.Consequent(np.arange(0, 200, 0.1), 'Vx')
    Vy = ctrl.Consequent(np.arange(-50, 50, 0.1), 'Vy')
    omega = ctrl.Consequent(np.arange(-20, 20, 0.1), 'omega')

    # 隸屬函數定義
    angle.automf(5, names=['BL', 'SL', 'Z', 'SR', 'BR'])
    position.automf(5, names=['BL', 'SL', 'Z', 'SR', 'BR'])

    Vx['S'] = fuzz.trapmf(Vx.universe, [0, 0, 50, 100])
    Vx['M'] = fuzz.trimf(Vx.universe, [50, 100, 150])
    Vx['F'] = fuzz.trapmf(Vx.universe, [100, 150, 200, 200])

    Vy['LL'] = fuzz.trapmf(Vy.universe, [-50, -50, -40, -20])
    Vy['L'] = fuzz.trimf(Vy.universe, [-30, -15, 0])
    Vy['Z'] = fuzz.trimf(Vy.universe, [-15, 0, 15])
    Vy['R'] = fuzz.trimf(Vy.universe, [0, 15, 30])
    Vy['RR'] = fuzz.trapmf(Vy.universe, [20, 40, 50, 50])

    omega['CCW2'] = fuzz.trapmf(omega.universe, [-20, -20, -15, -10])
    omega['CCW'] = fuzz.trimf(omega.universe, [-15, -10, 0])
    omega['Z'] = fuzz.trimf(omega.universe, [-10, 0, 10])
    omega['CW'] = fuzz.trimf(omega.universe, [0, 10, 15])
    omega['CW2'] = fuzz.trapmf(omega.universe, [10, 15, 20, 20])


    #定義位置控制規則
    rules = [
        ctrl.Rule((angle['BL'] & position['BL']), (Vx['S'], Vy['RR'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['BL']), (Vx['S'], Vy['RR'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['BL']), (Vx['M'], Vy['RR'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['BL']), (Vx['S'], Vy['RR'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['BL']), (Vx['S'], Vy['RR'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['SL']), (Vx['S'], Vy['R'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['SL']), (Vx['M'], Vy['R'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['SL']), (Vx['F'], Vy['R'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['SL']), (Vx['M'], Vy['R'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['SL']), (Vx['S'], Vy['R'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['Z']), (Vx['S'], Vy['Z'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['Z']), (Vx['F'], Vy['Z'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['Z']), (Vx['F'], Vy['Z'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['Z']), (Vx['F'], Vy['Z'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['Z']), (Vx['S'], Vy['Z'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['SR']), (Vx['S'], Vy['L'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['SR']), (Vx['M'], Vy['L'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['SR']), (Vx['F'], Vy['L'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['SR']), (Vx['M'], Vy['L'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['SR']), (Vx['S'], Vy['L'], omega['CCW2'])),
                
        ctrl.Rule((angle['BL'] & position['BR']), (Vx['S'], Vy['LL'], omega['CW2'])),
        ctrl.Rule((angle['SL'] & position['BR']), (Vx['S'], Vy['LL'], omega['CW'])),
        ctrl.Rule((angle['Z'] & position['BR']), (Vx['M'], Vy['LL'], omega['Z'])),
        ctrl.Rule((angle['SR'] & position['BR']), (Vx['S'], Vy['LL'], omega['CCW'])),
        ctrl.Rule((angle['BR'] & position['BR']), (Vx['S'], Vy['LL'], omega['CCW2'])),
    ]


    #建立模糊控制系統
    control_system = ctrl.ControlSystem(rules)
    simulator = ctrl.ControlSystemSimulation(control_system)
    
    Vx_results, Vy_results, omega_results = [], [], []
    
    #設定輸入並計算
    for angle_input, position_input in zip(angle_input, position_input):
        simulator.input['angle'] = angle_input
        simulator.input['position'] = position_input
        simulator.compute()
        Vx_results.append(simulator.output['Vx'])
        Vy_results.append(simulator.output['Vy'])
        omega_results.append(simulator.output['omega'])

    #使用模糊集合的 α-截集 (Alpha Cut)
    #只取 排名前 80% 的值來避免極端值影響

    Vx_sorted = sorted(Vx_results)
    Vy_sorted = sorted(Vy_results)
    omega_sorted = sorted(omega_results)

    Vx_out = np.mean(Vx_sorted[:int(len(Vx_sorted) * 0.8)]) if Vx_sorted else 0
    Vy_out = np.mean(Vy_sorted[:int(len(Vy_sorted) * 0.8)]) if Vy_sorted else 0
    omega_out = np.mean(omega_sorted[-int(len(omega_sorted) * 0.8):]) if omega_sorted else 0

    return Vx_out,Vy_out,omega_out

async def scan_once():
    """掃描 BLE 設備，並更新 RSSI"""
    try:
        devices = await BleakScanner.discover(timeout=0.3)
        rssi_data = {}
        for device in devices:
            if device.address in target_devices:
                rssi_data[device.address] = device.rssi
        return rssi_data
    except Exception as e:
        print(f"掃描錯誤: {e}")
        return {}

async def rssi_return():
    global target_devices, last_detected_time
    current_time = time.time()
    scanned_rssi = await scan_once()

    for address in target_devices:
        if address in scanned_rssi:
            target_devices[address] = scanned_rssi[address]
            last_detected_time[address] = current_time  # 更新時間戳記
        else:
            # 若長時間未收到新數據，則設為 None
            if last_detected_time[address] and (current_time - last_detected_time[address] > RSSI_TIMEOUT):
                target_devices[address] = None  # 設為 None，代表設備可能離線

    # 顯示結果
    rssi_1 = target_devices["290D0DE6-BFC1-D0B7-A8B4-845862F094CE"]
    rssi_2 = target_devices["6BBBBE46-E627-92F7-7299-8DE80626783E"]
    print(f"RSSI 1: {rssi_1 if rssi_1 is not None else 'None'} dBm | RSSI 2: {rssi_2 if rssi_2 is not None else 'None'} dBm")

    await asyncio.sleep(0.2)  # 掃描間隔

async def send_motor_commands():

    async with BleakClient(device_address) as client:
        if not client.is_connected:
            print("無法連接到設備")
            return
        print(f"已成功連接到設備 {device_address}")
        await asyncio.sleep(2)


        running = True
        mode1 = False
        mode2 = False
        mode3 = False
        mode4 = False
        mode0 = True
        choose_mode = False
        while running:

            pygame.event.pump()
            axis_states = {AXIS_NAMES[i]: round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())}
            button_states = {BUTTON_NAMES[i]: joystick.get_button(i) for i in range(joystick.get_numbuttons())}
            
            # 讀取搖桿數值
            R2_Trigger, Right_X, Right_Y, Left_X = axis_states.get("R2_Trigger", 0), axis_states.get("Right_X", 0), axis_states.get("Right_Y", 0), axis_states.get("Left_X", 0)
            Select = button_states.get("Select", 0)
            Cross = button_states.get("Cross", 0)
            Circle= button_states.get("Circle", 0)

            Up= button_states.get("Up", 0)
            Down= button_states.get("Down", 0)
            Left= button_states.get("Left", 0)
            Right= button_states.get("Right", 0)

            if  mode0 == True and not choose_mode:
                data = [0,0,0 ,0,0,0 ,0,0,0 ,0,0,0]
                packed_data = struct.pack("<12B", *data)  # 確保是小端序的 uint8_t
                await client.write_gatt_char(characteristic_uuid, packed_data)
                print(f"發送: {list(data)}")
                await asyncio.sleep(0.2)


            if  Cross == 1 and not choose_mode and not mode1 and not mode2 and not mode3 and not mode4 and mode0:
                print("檢測到Cross 請選擇模式")
                print()
                print("上：搖桿控制模式")
                print("左：影像循線模式")
                print("右：藍牙自動跟隨模式")
                print("下：模式四(無功能)")
                print()
                choose_mode=True
                

            if  mode1 == True:
                # 計算速度
                ss, direction, vx, vy, omega = get_speed(R2_Trigger, Right_X, Right_Y, 0, 10, Left_X, 3)
                Vfl, Vfr, Vrl, Vrr = mecanum_v(vx, vy, omega,turning_radius,wheel_radius)
                data = bytes(l298n(Vfl, Vfr, Vrl, Vrr))
                packed_data = struct.pack("<12B", *data)  # 確保是小端序的 uint8_t
                await client.write_gatt_char(characteristic_uuid, packed_data)
                print(f"發送: {list(data)}")
                await asyncio.sleep(0.2)

            if  mode2 == True:

                frame = get_frame()
                if frame is not None:
                    result=apply_perspective_transform_and_draw_grid_on_image(frame, -15, 0.1,70.42,43.3)
                    processed_image, angle_differences, offset = detect_lane_angle_and_offset(result,[150,300])
                    Vx_out,Vy_out,omega_out = calculate_mecanum_wheel_speeds(angle_differences, offset)
                    cv2.imshow("ESP32-CAM Stream", frame)
                    Vfl, Vfr, Vrl, Vrr = mecanum_v(vx, vy, omega,turning_radius,wheel_radius)
                    data = bytes(l298n(Vfl, Vfr, Vrl, Vrr))
                    packed_data = struct.pack("<12B", *data)  # 確保是小端序的 uint8_t
                    await client.write_gatt_char(characteristic_uuid, packed_data)
                    print(f"發送: {list(data)}")

                await asyncio.sleep(0.2)

            if  mode3 == True:
                await rssi_return()
                await asyncio.sleep(0.2)

            if  mode4 == True:
                print("模式四")
                await asyncio.sleep(0.2)

            if Up == 1 and choose_mode :
                print("切換到搖桿控制模式")
                mode1 = True
                mode0 = False
                choose_mode = False

            if Left == 1 and choose_mode :
                print("切換影像循線模式")
                mode2 = True
                mode0 == False
                choose_mode = False

            if Right == 1 and choose_mode :
                print("切換到模式三")
                mode3 = True
                mode0 = False
                choose_mode = False

            if Down == 1 and choose_mode :
                print("切換到模式四")
                mode4 = True
                mode0 = False
                choose_mode = False

            if Circle == 1:
                print("檢測到 Circle 按鍵被按下，退出模式")
                mode0 = True
                mode1 = False
                mode2 = False
                mode3 = False
                mode4 = False
                cv2.destroyAllWindows()

            if Select == 1:
                print("檢測到 Select 按鍵被按下，程式將退出")
                running = False
            

loop = asyncio.get_event_loop()
loop.run_until_complete(send_motor_commands())
