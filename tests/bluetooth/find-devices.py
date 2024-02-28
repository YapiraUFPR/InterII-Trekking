import asyncio
from bleak import BleakClient, BleakScanner

# address = "15:98:D8:29:B9:64:"
MODEL_NBR_UUID = "2A24"

devices = []

async def find_devices():
    global devices 
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)

async def connect_to_devices():
    global devices
    for address in devices:
        try:
            async with BleakClient(address) as client:
                model_number = await client.read_gatt_char(MODEL_NBR_UUID)
                print("Model Number: {0}".format("".join(map(chr, model_number))))
        except Exception as e:
            print(e)
            pass

loop = asyncio.get_event_loop()
loop.run_until_complete(find_devices())

# loop.run_until_complete(connect_to_devices())