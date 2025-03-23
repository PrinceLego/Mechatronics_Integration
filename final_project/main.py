import cv2
import numpy as np
import time
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


# 隸屬函數尚需依照實際車速調整
# 影像處理參數需視現場情況調整


def main(images_path):

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

        """
        #顯示並儲存影像
        save_frame('a', 'Original Image', frame)
        save_frame('b', 'Perspective Transformed Image', result)
        save_frame('c', 'Perspective Transformed Mesh', result_mesh)
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """

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
        n = 50  

        boxes = []
        image_before_adjustment = image.copy()  
        image_before_adjustment1 = image.copy()

        for i in range(n):
            y = height - (i + 1) * box_height
            x = center_x - box_width // 2
            boxes.append((x, y))
            cv2.rectangle(image_before_adjustment, (x, y), (x + box_width, y + box_height), (0, 255, 0), 2) 

        """
        cv2.imshow("Original Image", image)
        cv2.imshow("Blurred Image", blur)
        cv2.imshow("Gray Image", gray)
        cv2.imshow("Binary Image", binary)
        cv2.imshow("Boxes Before Adjustment", image_before_adjustment)  # 顯示未移動的框框
        """

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
        #cv2.imshow("Boxes After Adjustment", image_before_adjustment1)  

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
        angle = ctrl.Antecedent(np.arange(-30, 31, 1), 'angle')         # 角度偏差 (-30° ~ 30°)
        position = ctrl.Antecedent(np.arange(-10, 11, 1), 'position')   # 位置偏移 (-10 ~ 10)

        #定義輸出變數
        Vx = ctrl.Consequent(np.arange(0, 101, 1), 'Vx')
        Vy = ctrl.Consequent(np.arange(-50, 50, 1), 'Vy')
        omega = ctrl.Consequent(np.arange(-50, 51, 1), 'omega')

        #定義隸屬函數
        angle['BL'] = fuzz.trimf(angle.universe, [-30, -30, -15])
        angle['SL'] = fuzz.trimf(angle.universe, [-20, -10, 0])
        angle['Z'] = fuzz.trimf(angle.universe, [-5, 0, 5])
        angle['SR'] = fuzz.trimf(angle.universe, [0, 10, 20])
        angle['BR'] = fuzz.trimf(angle.universe, [15, 30, 30])

        position['BL'] = fuzz.trimf(position.universe, [-10, -10, -5])
        position['SL'] = fuzz.trimf(position.universe, [-7, -3, 0])
        position['Z'] = fuzz.trimf(position.universe, [-3, 0, 3])
        position['SR'] = fuzz.trimf(position.universe, [0, 3, 7])
        position['BR'] = fuzz.trimf(position.universe, [5, 10, 10])

        Vx['S'] = fuzz.trimf(Vx.universe, [0, 25, 50])                  #慢速
        Vx['M'] = fuzz.trimf(Vx.universe, [25, 50, 75])                 #中速
        Vx['F'] = fuzz.trimf(Vx.universe, [50, 75, 100])                #快速

        Vy['LL'] = fuzz.trimf(Vy.universe, [-50, -50, -25])             #最左
        Vy['L'] = fuzz.trimf(Vy.universe, [-40, -25, -10])              #左
        Vy['Z'] = fuzz.trimf(Vy.universe, [-15, 0, 15])                 #中間
        Vy['R'] = fuzz.trimf(Vy.universe, [10, 25, 40])                 #右
        Vy['RR'] = fuzz.trimf(Vy.universe, [25, 50, 50])                #最右


        omega['CCW2'] = fuzz.trimf(omega.universe, [-50, -50, -35])     #強烈逆時針
        omega['CCW'] = fuzz.trimf(omega.universe, [-40, -25, -10])      #輕微逆時針
        omega['Z'] = fuzz.trimf(omega.universe, [-15, 0, 15])           #中間
        omega['CW'] = fuzz.trimf(omega.universe, [10, 25, 40])          #輕微順時針
        omega['CW2'] = fuzz.trimf(omega.universe, [35, 50, 50])         #強烈順時針

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

        #計算麥克納姆輪速度
        """
        V_FL = Vx_out - Vy_out - omega_out
        V_FR = Vx_out + Vy_out + omega_out
        V_RL = Vx_out + Vy_out - omega_out
        V_RR = Vx_out - Vy_out + omega_out
        """
        
        """
        # 畫出角度的隸屬函數
        plt.figure()
        plt.plot(angle.universe, angle['BL'].mf, label='BL')
        plt.plot(angle.universe, angle['SL'].mf, label='SL')
        plt.plot(angle.universe, angle['Z'].mf, label='Z')
        plt.plot(angle.universe, angle['SR'].mf, label='SR')
        plt.plot(angle.universe, angle['BR'].mf, label='BR')
        plt.title('Angle Membership Functions')
        plt.legend()
        plt.show()

        # 畫出位置的隸屬函數
        plt.figure()
        plt.plot(position.universe, position['BL'].mf, label='BL')
        plt.plot(position.universe, position['SL'].mf, label='SL')
        plt.plot(position.universe, position['Z'].mf, label='Z')
        plt.plot(position.universe, position['SR'].mf, label='SR')
        plt.plot(position.universe, position['BR'].mf, label='BR')
        plt.title('Position Membership Functions')
        plt.legend()
        plt.show()

        # 畫出 Vx 的隸屬函數
        plt.figure()
        plt.plot(Vx.universe, Vx['S'].mf, label='S')
        plt.plot(Vx.universe, Vx['M'].mf, label='M')
        plt.plot(Vx.universe, Vx['F'].mf, label='F')
        plt.title('Vx Membership Functions')
        plt.legend()
        plt.show()

        # 畫出 Vy 的隸屬函數
        plt.figure()
        plt.plot(Vy.universe, Vy['L'].mf, label='L')
        plt.plot(Vy.universe, Vy['Z'].mf, label='Z')
        plt.plot(Vy.universe, Vy['R'].mf, label='R')
        plt.title('Vy Membership Functions')
        plt.legend()
        plt.show()

        # 畫出 Omega 的隸屬函數
        plt.figure()
        plt.plot(omega.universe, omega['CCW'].mf, label='CCW')
        plt.plot(omega.universe, omega['Z'].mf, label='Z')
        plt.plot(omega.universe, omega['CW'].mf, label='CW')
        plt.title('Omega Membership Functions')
        plt.legend()
        plt.show()
        """
    

        # 10. 返回結果
        return Vx_out,Vy_out,omega_out

    start_time = time.time()

    result=apply_perspective_transform_and_draw_grid_on_image(images_path, -15, 0.1,70.42,43.3)
    processed_image, angle_differences, offset = detect_lane_angle_and_offset(result,[150,300])
    Vx_out,Vy_out,omega_out = calculate_mecanum_wheel_speeds(angle_differences, offset)
    
    """
    print("Angle Differences:", angle_differences)
    print("Offset:", offset)
    print(f"Vx: {Vx_out}, Vy: {Vy_out}, Omega: {omega_out}")
    
    cv2.imshow("Processed Image", processed_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    """

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"運算時間: {execution_time:.4f} 秒")


# 開啟默認的攝像頭
cap = cv2.VideoCapture(0)

# 檢查是否成功開啟攝像頭
if not cap.isOpened():
    print("無法開啟")
    exit()

while True:
    # 讀取即時影像
    ret, frame = cap.read()
    
    # 檢查是否成功讀取影像
    if not ret:
        print("無法讀取影像")
        break
    
    main(frame)
    
    # 按q退出程式
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 釋放攝像頭並關閉所有視窗
cap.release()
cv2.destroyAllWindows()


