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

    print(f"[Arduino ->  Python] {msg_rate:5.1f}Hz | {text}        ")

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
    writer_task = asyncio.create_task(writer_thread(filename_log))

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

# # libraries
# import os
# import asyncio
# from bleak import BleakScanner, BleakClient
# import time
# import collections
# import aiofiles
# import matplotlib.pyplot as plt
# import numpy as np

# # These must match the UUIDs in the Arduino code
# SERVICE_UUID        = "12345678-1234-5678-1234-56789abcdef0"
# ARDUINO_TO_PC_UUID  = "12345678-1234-5678-1234-56789abcdef1"
# PC_TO_ARDUINO_UUID  = "12345678-1234-5678-1234-56789abcdef2"

# # Create buffers, debugging purposes
# QUEUE_TIME = collections.deque(maxlen=200)  # For message timing
# SPEED_DATA = collections.deque(maxlen=50)  # Last 100 speed values for plotting
# TIME_DATA = collections.deque(maxlen=50)   # Time axis
# start_time = time.time()

# # Create collection
# QUEUE_WRITE = asyncio.Queue()

# async def writer_thread(filename):
#     print(f"[writer_thread] Writing to {filename}")
#     async with aiofiles.open(filename, 'w') as f:
#         while True:
#             line = await QUEUE_WRITE.get()
#             if line is None:
#                 print("[writer_thread] Stopping")
#                 QUEUE_WRITE.task_done()
#                 break
#             await f.write(line + "\n")
#             QUEUE_WRITE.task_done()

# def notification_handler(sender, data):
#     text = data.decode('utf-8', errors='ignore')

#     # Debugging raw messages
#     print(f"\n[Arduino -> Python] {text.strip()}")  

#     try:
#         if "TIME:" in text and "SPEED:" in text:
#             parts = text.split("|")
#             time_part = [p for p in parts if "TIME:" in p]
#             speed_part = [p for p in parts if "SPEED:" in p]

#             if time_part and speed_part:
#                 timestamp = int(time_part[0].split("TIME:")[1].strip())  # Extract timestamp
#                 speed_value = float(speed_part[0].split("SPEED:")[1].strip())  # Extract speed

#                 TIME_DATA.append(timestamp / 1000.0)  # Convert from ms to seconds
#                 SPEED_DATA.append(speed_value)

#                 # Log the values
#             QUEUE_WRITE.put_nowait(f"{timestamp},{speed_value}")
#     except ValueError:
#         print(f"⚠️ Could not parse: {text}")

#     msg_rate = len(QUEUE_TIME) / (QUEUE_TIME[-1] - QUEUE_TIME[0]) if len(QUEUE_TIME) > 1 else 0
#     print(f"\r[Arduino -> Python] {msg_rate:5.1f}Hz | Speed: {SPEED_DATA[-1] if SPEED_DATA else 0}", end="")

# async def user_input_loop(client):
#     while True:
#         user_text = await asyncio.to_thread(input, "")
#         if user_text.lower() == "quit":
#             break
#         if user_text == "":
#             continue
#         await client.write_gatt_char(PC_TO_ARDUINO_UUID, user_text.encode())
#         print(f"\n[Python  -> Arduino] {user_text}")

# async def ble_task():
#     print("[main] Scanning for BLE devices...")
#     devices = await BleakScanner.discover(timeout=2.0)
#     address_to_connect = next((d.address for d in devices if d.name in ["Arduino", "Prothese"]), None)

#     if address_to_connect is None:
#         print("[main] No Arduino device found.")
#         return


#     # os.makedirs("logs", exist_ok=True)
#     # filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")


#     print(f"[main] Connecting to {address_to_connect}...")
#     async with BleakClient(address_to_connect) as client:
#         print(f"[main] Subscribing to characteristic {ARDUINO_TO_PC_UUID}...")
#         await client.start_notify(ARDUINO_TO_PC_UUID, notification_handler)
#         print("[main] Subscribed to notifications")
#         await client.write_gatt_char(PC_TO_ARDUINO_UUID, "START".encode())

#         input_task = asyncio.create_task(user_input_loop(client))
#         await input_task
#         await client.stop_notify(ARDUINO_TO_PC_UUID)
    
#     print("[main] Disconnected the PC from the Arduino")
#     await QUEUE_WRITE.put(None)
#     print("[main] All done!")

# # --- Matplotlib Real-Time Plotting ---
# plt.ion()  # Turn on interactive mode

# fig, ax = plt.subplots()
# line, = ax.plot([], [], 'b-', label="Motor Speed")

# def init_plot():
#     """ Initialize plot axes """
#     ax.set_xlim(0, 10)
#     ax.set_ylim(0, 500)  # Adjust based on expected motor speed
#     ax.set_xlabel("Time (s)")
#     ax.set_ylabel("Speed (steps/sec)")
#     ax.set_title("Motor Speed via BLE")
#     ax.legend()

# def update_plot():
#     if len(TIME_DATA) > 2:
#         # ax.set_xlim(max(0, TIME_DATA[0]), TIME_DATA[-1] + 1)
#         line.set_data(list(range(len(SPEED_DATA))), list(SPEED_DATA))
#         fig.canvas.draw()
#         fig.canvas.flush_events()
#         # plt.pause(0.005)  # Reduce pause to update faster


# async def plot_task():
#     """ Runs Matplotlib event loop separately on Windows """
#     init_plot()
#     while True:
#         update_plot()
#         plt.pause(0.005)
#         await asyncio.sleep(0.005)  # Update every 100ms

# # async def matplotlib_loop():
# #     """ Separate function to keep Matplotlib GUI running """
# #     while True:
# #         plt.pause(0.005)  # Ensures Matplotlib's event loop runs
# #         await asyncio.sleep(0.005)

# async def main():
#     """ Run BLE communication, plotting, and logging concurrently """
#     os.makedirs("logs", exist_ok=True)
#     filename_log = os.path.join("logs", f"log_{time.strftime('%Y%m%d_%H%M%S')}.txt")

#     # Start the writer thread
#     writer = asyncio.create_task(writer_thread(filename_log))

#     # Start BLE communication and plotting
#     ble = asyncio.create_task(ble_task())
#     plot = asyncio.create_task(plot_task())

#     # Wait for BLE & plot tasks to finish
#     await asyncio.gather(ble, plot)

#     # Ensure writer_thread finishes properly
#     await writer

# # Run the main event loop
# asyncio.run(main())



#Thread is configured for Windows GUI but callbacks are not working.