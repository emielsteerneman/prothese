# libraries
import os
import asyncio
from bleak import BleakScanner, BleakClient
import time
import collections
import aiofiles
import matplotlib.pyplot as plt
import numpy as np

# These must match the UUIDs in the Arduino code
SERVICE_UUID        = "12345678-1234-5678-1234-56789abcdef0"
ARDUINO_TO_PC_UUID  = "12345678-1234-5678-1234-56789abcdef1"
PC_TO_ARDUINO_UUID  = "12345678-1234-5678-1234-56789abcdef2"

# Create buffers, debugging purposes
QUEUE_TIME = collections.deque(maxlen=200)  # For message timing
SPEED_DATA = collections.deque(maxlen=50)  # Last 100 speed values for plotting
TIME_DATA = collections.deque(maxlen=50)   # Time axis
start_time = time.time()

# Create collection
QUEUE_WRITE = asyncio.Queue()

async def writer_thread(filename):
    print(f"[writer_thread] Writing to {filename}")
    async with aiofiles.open(filename, 'w') as f:
        while True:
            line = await QUEUE_WRITE.get()
            if line is None:
                print("[writer_thread] Stopping")
                QUEUE_WRITE.task_done()
                break
            await f.write(line + "\n")
            QUEUE_WRITE.task_done()

def notification_handler(sender, data):
    text = data.decode('utf-8', errors='ignore')

    # Debugging raw messages
    print(f"\n[Arduino -> Python] {text.strip()}")  

    try:
        if "TIME:" in text and "SPEED:" in text:
            parts = text.split("|")
            time_part = [p for p in parts if "TIME:" in p]
            speed_part = [p for p in parts if "SPEED:" in p]

            if time_part and speed_part:
                timestamp = int(time_part[0].split("TIME:")[1].strip())  # Extract timestamp
                speed_value = float(speed_part[0].split("SPEED:")[1].strip())  # Extract speed

                TIME_DATA.append(timestamp / 1000.0)  # Convert from ms to seconds
                SPEED_DATA.append(speed_value)

                # Log the values
            QUEUE_WRITE.put_nowait(f"{timestamp},{speed_value}")
    except ValueError:
        print(f"⚠️ Could not parse: {text}")

    msg_rate = len(QUEUE_TIME) / (QUEUE_TIME[-1] - QUEUE_TIME[0]) if len(QUEUE_TIME) > 1 else 0
    print(f"\r[Arduino -> Python] {msg_rate:5.1f}Hz | Speed: {SPEED_DATA[-1] if SPEED_DATA else 0}", end="")

async def user_input_loop(client):
    while True:
        user_text = await asyncio.to_thread(input, "")
        if user_text.lower() == "quit":
            break
        if user_text == "":
            continue
        await client.write_gatt_char(PC_TO_ARDUINO_UUID, user_text.encode())
        print(f"\n[Python  -> Arduino] {user_text}")

async def ble_task():
    print("[main] Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=2.0)
    address_to_connect = next((d.address for d in devices if d.name in ["Arduino", "Prothese"]), None)

    if address_to_connect is None:
        print("[main] No Arduino device found.")
        return


    # os.makedirs("logs", exist_ok=True)
    # filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")


    print(f"[main] Connecting to {address_to_connect}...")
    async with BleakClient(address_to_connect) as client:
        print(f"[main] Subscribing to characteristic {ARDUINO_TO_PC_UUID}...")
        await client.start_notify(ARDUINO_TO_PC_UUID, notification_handler)
        print("[main] Subscribed to notifications")
        await client.write_gatt_char(PC_TO_ARDUINO_UUID, "START".encode())

        input_task = asyncio.create_task(user_input_loop(client))
        await input_task
        await client.stop_notify(ARDUINO_TO_PC_UUID)
    
    print("[main] Disconnected the PC from the Arduino")
    await QUEUE_WRITE.put(None)
    print("[main] All done!")

# --- Matplotlib Real-Time Plotting ---
plt.ion()  # Turn on interactive mode

fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-', label="Motor Speed")

def init_plot():
    """ Initialize plot axes """
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 500)  # Adjust based on expected motor speed
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (steps/sec)")
    ax.set_title("Motor Speed via BLE")
    ax.legend()

def update_plot():
    if len(TIME_DATA) > 2:
        # ax.set_xlim(max(0, TIME_DATA[0]), TIME_DATA[-1] + 1)
        line.set_data(list(range(len(SPEED_DATA))), list(SPEED_DATA))
        fig.canvas.draw()
        fig.canvas.flush_events()
        # plt.pause(0.005)  # Reduce pause to update faster


async def plot_task():
    """ Runs Matplotlib event loop separately on Windows """
    init_plot()
    while True:
        update_plot()
        plt.pause(0.005)
        await asyncio.sleep(0.005)  # Update every 100ms

# async def matplotlib_loop():
#     """ Separate function to keep Matplotlib GUI running """
#     while True:
#         plt.pause(0.005)  # Ensures Matplotlib's event loop runs
#         await asyncio.sleep(0.005)

async def main():
    """ Run BLE communication, plotting, and logging concurrently """
    os.makedirs("logs", exist_ok=True)
    filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")

    # Start the writer thread
    writer = asyncio.create_task(writer_thread(filename_log))

    # Start BLE communication and plotting
    ble = asyncio.create_task(ble_task())
    plot = asyncio.create_task(plot_task())

    # Wait for BLE & plot tasks to finish
    await asyncio.gather(ble, plot)

    # Ensure writer_thread finishes properly
    await writer

# Run the main event loop
asyncio.run(main())



#Thread is configured for Windows GUI but callbacks are not working.