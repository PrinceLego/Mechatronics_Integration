import cv2
import numpy as np
import time

def apply_perspective_transform_and_draw_grid_on_image(image_path,pitch=-45,h=0.9):

    yaw=0
    roll=0
    # 读取图像
    frame = cv2.imread(image_path)
    if frame is None:
        print("Error: Could not read image.")
        return

    interval = 100
    HFOV = 70.42
    VFOV = 43.3

    # 获取图像的宽度和高度
    height, width = frame.shape[:2]

    # K矩阵
    def calculate_camera_intrinsics(W, H, HFOV, VFOV):
        f_x = W / (2 * np.tan(np.deg2rad(HFOV) / 2))
        f_y = H / (2 * np.tan(np.deg2rad(VFOV) / 2))
        K = np.array([
            [f_x, 0, W / 2],
            [0, f_y, H / 2],
            [0, 0, 1]
        ])
        return K

    # 旋转矩阵
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
        cv2.circle(resulttt, po, 1, (0, 0, 255), -1)  # 红色网格交点
        world_coord = pixel_to_world(po[0], po[1], K, R, h)
        cv2.putText(resulttt, str(tuple(round(coord, num) for coord in world_coord)), (po[0] + 5, po[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)  # 黑色文字

    def save_frame(name, show_name, photo):
        current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        cv2.imwrite(f"/Users/prince_lego/Desktop/AAAA/{name}/{name}_{current_time}.jpg", photo)
        cv2.imshow(show_name, photo)

    K = calculate_camera_intrinsics(width, height, HFOV, VFOV)
    R = rotation_matrix(np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll))

    src_pts = np.array([[0, 0],      # 左上
                        [width, 0],  # 右上
                        [0, height],  # 左下
                        [width, height]],  # 右下
                       dtype=np.float32)

    dst_pts = np.array([pixel_to_world(0, 0, K, R, h),   # 左上
                        pixel_to_world(width, 0, K, R, h),   # 右上
                        pixel_to_world(0, height, K, R, h),   # 左下
                        pixel_to_world(width, height, K, R, h)],  # 右下
                       dtype=np.float32)

    dst_pts = output_pixel_num(dst_pts)

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    result = cv2.warpPerspective(frame, M, (width, height))
    result_mesh = result.copy()
    result_gps = result.copy()

    focus_point = (int(width / 2), int(height / 2))

    def world_to_pixel(X_w, Y_w, K, R, h):
        # 轉換世界座標到相機坐標系
        X_c = np.array([X_w, Y_w, h])  # 假設物體在地面，高度為 h

        # 計算相機坐標到歪斜相機視角的變換
        X_n = np.linalg.inv(R) @ X_c

        # 投影到 2D 平面
        uv1 = K @ X_n
        u = uv1[0] / uv1[2]  # 正規化 x
        v = height - (uv1[1] / uv1[2])  # 正規化 y，調整成 OpenCV 影像座標

        return int(u), int(v)  # 返回整數像素座標





    # 绘制网格线和交点
    for x in range(focus_point[0], width, interval):
        cv2.line(result_mesh, (x, 0), (x, height), (0, 0, 0), 1)  # 垂直线
        cv2.line(result_gps, (x, 0), (x, height), (0, 0, 0), 1)  # 垂直线
    for x in range(focus_point[0], 0, -interval):
        cv2.line(result_mesh, (x, 0), (x, height), (0, 0, 0), 1)  # 垂直线
        cv2.line(result_gps, (x, 0), (x, height), (0, 0, 0), 1)  # 垂直线
    for y in range(focus_point[1], height, interval):
        cv2.line(result_mesh, (0, y), (width, y), (0, 0, 0), 1)  # 水平线
        cv2.line(result_gps, (0, y), (width, y), (0, 0, 0), 1)  # 垂直线
    for y in range(focus_point[1], 0, -interval):
        cv2.line(result_mesh, (0, y), (width, y), (0, 0, 0), 1)  # 水平线
        cv2.line(result_gps, (0, y), (width, y), (0, 0, 0), 1)  # 垂直线

    # 标记所有网格交点
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



    save_frame('a', 'Original Image', frame)
    save_frame('b', 'Perspective Transformed Image', result)
    save_frame('c', 'Perspective Transformed Mesh', result_mesh)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return result

# 用法：将路径传入图像文件路径
result=apply_perspective_transform_and_draw_grid_on_image("/Users/prince_lego/Desktop/1234.jpg", -15, 2)
