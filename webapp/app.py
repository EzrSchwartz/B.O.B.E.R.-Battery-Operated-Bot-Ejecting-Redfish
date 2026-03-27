
import tkinter as tk
from tkinter import filedialog, ttk

import asyncio

import subprocess, sys

def _bleak_available():
    try:
        result = subprocess.run(
            [sys.executable, "-c", "import bleak"],
            capture_output=True, timeout=5
        )
        print(f"[DEBUG] bleak probe returncode: {result.returncode}")
        return result.returncode == 0
    except Exception as e:
        print(f"[DEBUG] bleak probe exception: {e}")
        return False

print("[DEBUG] checking bleak...")
_available = _bleak_available()
print(f"[DEBUG] bleak available: {_available}")

if _available:
    print("[DEBUG] importing bleak...")
    try:
        import bleak
        from bleak import BleakClient, BleakScanner
        print("[DEBUG] bleak imported OK")
    except ImportError:
        bleak = None
else:
    bleak = None

print(f"[DEBUG] bleak = {bleak}")

bluetooth_address = ""
deviceNames = []
deviceAddresses = []


async def search_ble_devices():
    if bleak is not None:
        discovered_devices = []
        try:
            devices = await BleakScanner.discover()
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
        discovered_devices = [
            {"address": "00:11:22:33:44:55", "name": "Device1"},
            {"address": "11:22:33:44:55:66", "name": "Device2"},
        ]

    return discovered_devices


def on_search_ble_clicked():
    discovered_devices = asyncio.run(search_ble_devices())

    text_output.delete(1.0, tk.END)

    global deviceNames
    global deviceAddresses
    deviceNames = []
    deviceAddresses = []
    drone_names = []

    for device_info in discovered_devices:
        text_output.insert(tk.END, f"Address: {device_info['address']}, Name: {device_info['name']}\n")
        deviceAddresses.append(device_info['address'])
        deviceNames.append(device_info['name'])
        drone_names.append(device_info['name'])

    drone_combobox['values'] = drone_names
    if drone_names:
        drone_combobox.current(0)


def get_address_from_name(device_name):
    for i in range(len(deviceNames)):
        if deviceNames[i] == device_name:
            return deviceAddresses[i]
    return "Unknown"


async def connect_to_ble_device(ble_address):
    async with BleakClient(ble_address) as client:
        print(f"Connected: {client.is_connected}")


def SetDrone():
    global bluetooth_address
    drone = drone_combobox.get()
    print(f"Chosen Drone: {drone}")
    bluetooth_address = get_address_from_name(drone)
    print(f"Bluetooth Address: {bluetooth_address}")
    asyncio.run(connect_to_ble_device(bluetooth_address))


def Start_Mission():
    drone = drone_combobox.get()
    print("Starting mission with drone:", drone)

    async def _run():
        async with BleakClient(bluetooth_address) as client:
            print(f"Connected: {client.is_connected}")
            if client.is_connected:
                await client.write_gatt_char("Arm/Disarm", bytearray(b'\x01'))

    asyncio.run(_run())


# Create the main window
print("[DEBUG] creating Tk window...")
root = tk.Tk()
print("[DEBUG] Tk window created")
root.title("Drone Control")

# Create a notebook for tabs
notebook = ttk.Notebook(root)
notebook.pack(fill="both", expand=True)

# Mission tab
mission_tab = ttk.Frame(notebook)
notebook.add(mission_tab, text="Mission Status")

StartButton = tk.Button(mission_tab, text="Start Mission", command=Start_Mission)
StartButton.pack()

# BLE sensor discovery tab
ble_tab = ttk.Frame(notebook)
notebook.add(ble_tab, text="BLE Sensors")

search_ble_button = tk.Button(ble_tab, text="Search BLE Sensors", command=on_search_ble_clicked)
search_ble_button.pack()

text_output = tk.Text(ble_tab, height=10, width=50)
text_output.pack()

drone_label = tk.Label(ble_tab, text="Select Drone:")
drone_label.pack()

drone_combobox = ttk.Combobox(ble_tab, state="readonly")
drone_combobox.pack()

set_drone_button = tk.Button(ble_tab, text="Set Drone", command=SetDrone)
set_drone_button.pack()

# Run the GUI
root.mainloop()
