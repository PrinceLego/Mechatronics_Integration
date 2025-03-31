import cv2
import requests
import numpy as np

ESP32_URL = "http://192.168.0.38:81/stream"

def get_frame():
    try:
        # 開始從流中讀取數據
        response = requests.get(ESP32_URL, stream=True, timeout=5)
        if response.status_code == 200:
            # 從流中獲取一個幀
            bytes_data = b""
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                # 如果我們成功獲得完整的 JPEG 影像
                a = bytes_data.find(b'\xff\xd8')  # JPEG 起始標誌
                b = bytes_data.find(b'\xff\xd9')  # JPEG 結束標誌
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]  # 獲得 JPEG 數據
                    bytes_data = bytes_data[b+2:]  # 清理已處理的部分
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    return frame
        else:
            print(f"Failed to fetch image, HTTP status code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request error: {e}")
        return None

while True:
    frame = get_frame()
    if frame is not None:
        cv2.imshow("ESP32-CAM Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
