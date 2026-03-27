
import tkinter as tk
from tkinter import filedialog, ttk

import asyncio

try:
    import bleak
    from bleak import BleakClient

except ImportError:
    bleak = None
bluetooth_address = ""



async def search_ble_devices():
    if bleak is not None:
        discovered_devices = []
        try:
            devices = await bleak.discover()
            for device in devices:
                device_info = {
                    "address": device.address,
                    "name": device.name if device.name else "Unknown"

                }
                discovered_devices.append(device_info)
                print(f"Discovered BLE Device: {device_info}")
        except Exception as e:
            print(f"Error scanning BLE devices: {e}")
    else:
        print("The 'bleak' library is not available. Install it using 'pip install bleak'.")
        # If bleak is not available, simulate the device discovery
        discovered_devices = [
            {"address": "00:11:22:33:44:55", "name": "Device1"},
            {"address": "11:22:33:44:55:66", "name": None},
        ]

    return discovered_devices


def on_search_ble_clicked():
    discovered_devices = asyncio.run(search_ble_devices())

    # Clear previous text and items in the Text widget and Comboboxes
    text_output.delete(1.0, tk.END)

    # Create separate lists to hold values for each Combobox
    drones = []
    global deviceNames
    global deviceAddresses
    deviceNames = []
    deviceAddresses = []
    # Display the discovered devices in the Text widget and add them to the Comboboxes
    for device_info in discovered_devices:
        text_output.insert(tk.END, f"Address: {device_info['address']}, Name: {device_info['name']}\n")
        deviceAddresses.append(device_info['address'])
        deviceNames.append(device_info['name'])
        # Add device address to the Power Meter Combobox
        drones.append(device_info['name'])

    # Update the Comboboxes with the new values
    drones['values'] = drones

    # Clear the selection in the Comboboxes
    drones.current('')

def get_address_from_name(device_name):
    for i in range(len(deviceNames)):
        if deviceNames[i] == device_name:
            return deviceAddresses[i]
    return "Unknown"


async def connect_to_ble_device(ble_address):
    # 'async with' ensures the connection is closed automatically
    async with BleakClient(ble_address) as client:
        print(f"Connected: {client.is_connected}")
        if client.is_connected:
            # You can now read/write characteristics, e.g.:
            # data = await client.read_gatt_char(CHARACTERISTIC_UUID)
            # await client.write_gatt_char(CHARACTERISTIC_UUID, bytearray(b'\x01'))
            pass


# Function to set the chosen power meter
def SetDrone():
    drone = dronelabelbox.get()
    print(f"Chosen Drone: {drone}")
    bluetooth_address = get_address_from_name(drone)
    print(f"Bluetooth Address: {bluetooth_address}")
    asyncio.run(connect_to_ble_device(bluetooth_address))

async def Start_Mission():
    
    print("Starting mission with drone:", drone)
    async with BleakClient(bluetooth_address) as client:
        print(f"Connected: {client.is_connected}")
        if client.is_connected:
            # You can now read/write characteristics, e.g.:
            # data = await client.read_gatt_char(CHARACTERISTIC_UUID)
            await client.write_gatt_char("Arm/Disarm", bytearray(b'\x01'))
            pass


# Create the main window
root = tk.Tk()
root.title("Drone Control")

# Define global variables

# Create a notebook for tabs
notebook = ttk.Notebook(root)
notebook.pack(fill="both", expand=True)

# Existing FIT file processing tab
Mission = ttk.Frame(notebook)
notebook.add(Mission, text="Mission Status")

# FIT file processing buttons
StartButton = tk.Button(Mission, text="Select FIT Files", command=Start_Mission)
StartButton.pack()

# BLE sensor discovery tab
ble_tab = ttk.Frame(notebook)
notebook.add(ble_tab, text="BLE Sensors")

# BLE sensor search button
search_ble_button = tk.Button(ble_tab, text="Search BLE Sensors", command=on_search_ble_clicked)
search_ble_button.pack()

# Text widget to display discovered devices
text_output = tk.Text(ble_tab, height=10, width=50)
text_output.pack()

# Combobox to select the power meter
dronelabelbox = tk.Label(ble_tab, text="Select Drone:")
dronelabelbox.pack()

# Run the GUI
root.mainloop()