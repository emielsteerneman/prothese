import os
import asyncio
from bleak import BleakScanner, BleakClient
import time
import collections
import aiofiles

# These must match the UUIDs in your Arduino code
SERVICE_UUID        = "12345678-1234-5678-1234-56789abcdef0"
ARDUINO_TO_PC_UUID  = "12345678-1234-5678-1234-56789abcdef1"
PC_TO_ARDUINO_UUID  = "12345678-1234-5678-1234-56789abcdef2"

# Replace this with the MAC address (or BLE address) of your Nano 33 BLE once you discover it
# Example Linux format might look like: "XX:XX:XX:XX:XX:XX"
NANO_BLE_ADDRESS = "A1:BB:0C:69:74:13"

QUEUE_TIME = collections.deque(maxlen=200)
QUEUE_IMU = collections.deque(maxlen=200)

QUEUE_WRITE = asyncio.Queue()

async def writer_thread(filename):
    print(f"[writer_thread] Writing to {filename}")
    async with aiofiles.open(filename, 'w') as f:
        while True:
            line = await QUEUE_WRITE.get()  # Wait for data
            
            if line is None:
                print("[writer_thread] Stopping")
                # A None signals we want to stop
                QUEUE_WRITE.task_done()
                break

            # Write the line to file, plus a newline
            await f.write(line + "\n")
            # Optionally flush if you need near-real-time writes
            # await f.flush()

            # Mark this item as processed
            QUEUE_WRITE.task_done()

def notification_handler(sender, data):
    """ Called by BLE when a notification arrives. """
    
    # print("Length of data:", len(data))
    # f1, f2, f3 = struct.unpack('<fff', data)
    # print("Received floats:", f1, f2, f3)
    # return

    text = data.decode('utf-8', errors='ignore')
    print(f"[Arduino] {len(data)} {text}")
    

    now = time.time()
    # count, *imu_data = map(float, text.split(" "))

    global QUEUE_TIME, QUEUE_IMU
    QUEUE_TIME.append(now)
    QUEUE_IMU.append(1)

    if len(QUEUE_TIME) == QUEUE_TIME.maxlen:
        time_diff = QUEUE_TIME[-1] - QUEUE_TIME[0]
        imu_diff = QUEUE_IMU[-1] - QUEUE_IMU[0]
        imu_rate = imu_diff / time_diff
        msg_rate = len(QUEUE_TIME) / time_diff
        print(f"[  Stats] IMU rate: {imu_rate:.2f} | MSG rate: {msg_rate:.2f}")

    return
    loop = asyncio.get_running_loop()
    loop.call_soon_threadsafe(QUEUE_WRITE.put_nowait, f"{now} {text}")

async def user_input_loop(client):
    """Continuously prompt user for input and send to Arduino."""
    while True:
        user_text = await asyncio.to_thread(input, "")
        
        # Stop if user types "stop"
        if user_text.lower() == "quit": break

        # Skip empty lines
        if user_text == "": continue

        # Send text to Arduino
        await client.write_gatt_char(PC_TO_ARDUINO_UUID, user_text.encode())
        
        print(f"[Python ] {user_text}")

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=2.0)
    address_to_connect = None
    for d in devices:
        if d.name != "Arduino" and d.name != "Prothese": continue
        address_to_connect = d.address
    
    if address_to_connect is None:
        print("No Arduino device found.")
        return


    # Boot up the writer thread
    os.makedirs("logs", exist_ok=True)
    filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")
    writer_task = asyncio.create_task(writer_thread(filename_log))

    # Connect to the BLE device
    print(f"Connecting to {NANO_BLE_ADDRESS}...")
    async with BleakClient(address_to_connect) as client:

        # Subscribe to notifications
        print(f"Subscribing to characteristic {ARDUINO_TO_PC_UUID}...")
        await client.start_notify(ARDUINO_TO_PC_UUID, notification_handler)

        # Run user input in a separate task
        input_task = asyncio.create_task(user_input_loop(client))

        await input_task

        # Optionally stop notifications before disconnecting
        await client.stop_notify(ARDUINO_TO_PC_UUID)
    
    print("Disconnected.")

    # Stop the writer thread
    await QUEUE_WRITE.put(None)
    await writer_task

if __name__ == "__main__":
    asyncio.run(main())