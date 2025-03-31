import asyncio
from bleak import BleakScanner

async def get_rssi():
    devices = await BleakScanner.discover()
    print(f"Found {len(devices)} devices.")
    for device in devices:
        print(f"Device: {device.name}, Address: {device.address}, RSSI: {device.rssi} dBm")

asyncio.run(get_rssi())
