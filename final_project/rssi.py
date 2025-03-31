import asyncio
import time
from bleak import BleakScanner

# 目標設備的地址
target_devices = {
    "290D0DE6-BFC1-D0B7-A8B4-845862F094CE": None,  # 設備1
    "6BBBBE46-E627-92F7-7299-8DE80626783E": None   # 設備2
}
last_detected_time = {addr: None for addr in target_devices}  # 記錄每個設備最後更新 RSSI 的時間
RSSI_TIMEOUT = 3.0  # 若超過此秒數未更新 RSSI，則視為 None

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
    while True:
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

asyncio.run(rssi_return())
