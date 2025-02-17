# libraries
import os
import asyncio
from bleak import BleakScanner, BleakClient
import time
import collections
import aiofiles

# These must match the UUIDs in the Arduino code
SERVICE_UUID        = "12345678-1234-5678-1234-56789abcdef0"
ARDUINO_TO_PC_UUID  = "12345678-1234-5678-1234-56789abcdef1"
PC_TO_ARDUINO_UUID  = "12345678-1234-5678-1234-56789abcdef2"

# Create buffers, debugging purposes
QUEUE_TIME = collections.deque(maxlen=200)

# Create collection
QUEUE_WRITE = asyncio.Queue()

async def writer_thread(filename):
    print(f"[writer_thread] Writing to {filename}")
    async with aiofiles.open(filename, 'w') as f:
        while True:
            # Wait for data to enter the queue
            line = await QUEUE_WRITE.get() 
            
            # If we placed a None in the queue, it means we want to stop
            if line is None:
                print("[writer_thread] Stopping")
                QUEUE_WRITE.task_done()
                break

            # Write the line to file, plus a newline
            await f.write(line + "\n")

            # Mark this item as processed
            QUEUE_WRITE.task_done()
    # The file is automatically flushed when closed

def notification_handler(sender, data):
    ### Called by BLE when a notification arrives

    text = data.decode('utf-8', errors='ignore')
    
    if text.startswith("TEXTINIT"):
        return
    
    if text.startswith("TEXT"):
        print(f"[Arduino ->  Python] {text[4:]}                      ")
        return

    now = time.time()
    global QUEUE_TIME
    QUEUE_TIME.append(now)

    msg_rate = 0.
    if len(QUEUE_TIME) == QUEUE_TIME.maxlen:
        time_diff = QUEUE_TIME[-1] - QUEUE_TIME[0]
        msg_rate = len(QUEUE_TIME) / time_diff

    print(f"\r[Arduino ->  Python] {msg_rate:5.1f}Hz | {text}        ", end="")

    # Add message to queue so that the writer task can write it to the log file
    loop = asyncio.get_running_loop()
    loop.call_soon_threadsafe(QUEUE_WRITE.put_nowait, f"{now} {text}")

async def user_input_loop(client):
    ### Continuously prompt user for input and send to Arduino

    while True:
        # Wait for user to type something
        user_text = await asyncio.to_thread(input, "")
        # Quit if user types "quit"
        if user_text.lower() == "quit": break
        # Skip empty lines
        if user_text == "": continue
        # Send text to Arduino
        await client.write_gatt_char(PC_TO_ARDUINO_UUID, user_text.encode())
        
        print(f"\n[Python  -> Arduino] {user_text}")

async def main():
    print("[main] Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=2.0)
    address_to_connect = None
    for d in devices:
        if d.name != "Arduino" and d.name != "Prothese": continue
        address_to_connect = d.address
    
    if address_to_connect is None:
        print("[main] No Arduino device found.")
        return

    # Boot up the writer thread
    os.makedirs("logs", exist_ok=True)
    filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")
    # writer_task = asyncio.create_task(writer_thread(filename_log))

    # Connect to the BLE device
    print(f"[main] Connecting to {address_to_connect}...")
    async with BleakClient(address_to_connect) as client:

        # Subscribe to notifications
        print(f"[main] Subscribing to characteristic {ARDUINO_TO_PC_UUID}...")
        # await asyncio.sleep(3)  # Give Arduino some time
        await client.start_notify(ARDUINO_TO_PC_UUID, notification_handler)
        print("[main] Subscribed to notifications")

        await client.write_gatt_char(PC_TO_ARDUINO_UUID, "START".encode())

        # Run user input in a separate task
        input_task = asyncio.create_task(user_input_loop(client))

        # Wait until the user writes 'quit'
        await input_task

        # Stop notifications before disconnecting
        await client.stop_notify(ARDUINO_TO_PC_UUID)
    
    print("[main] Disconnected the PC from the Arduino")

    # Stop the writer thread
    await QUEUE_WRITE.put(None)
    # await writer_task

    print("[main] All done!")

if __name__ == "__main__":
    asyncio.run(main())