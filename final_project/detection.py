import cv2
import numpy as np

def detect_lane_angle_and_offset(image, y_heights):
    height, width = image.shape[:2]
    center_x = width // 2  

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                  # 轉換為灰階
    blur = cv2.GaussianBlur(gray, (7, 7), 0)                        # 高斯模糊
    _, binary = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)    # 二值化
    edges = cv2.Canny(binary, 90, 200)                              # Canny 邊緣檢測

    # 顯示前處理影像
    cv2.imshow("Original Image", image)
    cv2.imshow("Blurred Image", blur)
    cv2.imshow("Gray Image", gray)
    cv2.imshow("Binary Image", binary)
    cv2.imshow("Canny Edges", edges)
    
    box_width = 70
    box_height = 10
    n = 15  

    boxes = []
    image_before_adjustment = image.copy()  # 複製原圖以顯示未調整的框框
    image_before_adjustment1 = image.copy()

    for i in range(n):
        y = height - (i + 1) * box_height
        x = center_x - box_width // 2
        boxes.append((x, y))
        cv2.rectangle(image_before_adjustment, (x, y), (x + box_width, y + box_height), (0, 255, 0), 2)  # 藍色框框



    cv2.imshow("Boxes Before Adjustment", image_before_adjustment)  # 顯示未移動的框框圖

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

    cv2.imshow("Boxes After Adjustment", image_before_adjustment1)  # 顯示調整後的框框圖

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
    #cv2.imshow("Boxes After Adjustment", image)  # 顯示調整後的框框圖
    for y_target in new_y_heights:
        if len(center_points) > 2:
            offset.append(curve(y_target) - center_x)
            yellow_slope = (curve(y_target - 1) - curve(y_target + 1)) / 3 
            yellow_angle = np.arctan(yellow_slope) * 180 / np.pi            
            angle_differences.append(yellow_angle)
        
    return image, angle_differences, offset

# 呼叫函式
image = cv2.imread("/Users/prince_lego/Desktop/1234.jpg")

calculate_position = [50, 100, 150]
processed_image, angle_differences, offset = detect_lane_angle_and_offset(image, calculate_position)

cv2.imshow("Processed Image", processed_image)
print("Angle Differences:", angle_differences)
print("Offset:", offset)

cv2.waitKey(0)
cv2.destroyAllWindows()
