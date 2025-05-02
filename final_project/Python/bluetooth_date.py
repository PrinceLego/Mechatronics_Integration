from bleak import BleakClient
import asyncio

device_address = "6BBBBE46-E627-92F7-7299-8DE80626783E"  # 設備的 UUID
characteristic_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 特徵 UUID

async def send_motor_commands():
    async with BleakClient(device_address) as client:
        if client.is_connected:
            print(f"已成功連接到設備 {device_address}")
            
            while True:
                # L298N 控制數據 (IN1, IN2, IN3, IN4, ENA, ENB)
                motor_data = bytes([1, 0, 1, 0, 200, 200])  
                await client.write_gatt_char(characteristic_uuid, motor_data)
                print(f"發送數據: {list(motor_data)}")
                await asyncio.sleep(1)  # 每秒發送一次
                motor_data = bytes([0, 1, 0, 1, 200, 200])  
                await client.write_gatt_char(characteristic_uuid, motor_data)
                print(f"發送數據: {list(motor_data)}")
                await asyncio.sleep(1) 
        else:
            print("無法連接到設備")

loop = asyncio.get_event_loop()
loop.run_until_complete(send_motor_commands())
