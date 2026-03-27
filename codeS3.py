import board
import busio
import wifi
import socketpool
import json
import time
import espcamera
import _thread
import struct
import digitalio
import adafruit_ble
from adafruit_ble.advertising import Advertisement
from adafruit_ble.services.nordic import UARTService
import _bleio

# ============================================
# CONFIGURATION
# ============================================

# WiFi credentials
bluetooth_ssid = "FishingDrone"
# # ============================================
# # SBUS PROTOCOL CLASS
# # ============================================

def bluetooth(name = bluetooth_ssid):
    while _bleio.connected == False:
        print("Advertising Bluetooth...")
        ble = adafruit_ble.BLE()
        ble.name = name
        uart_service = UARTService()
        advertisement = Advertisement()
        advertisement.short_name = name
        advertisement.connectable = True
        advertisement.add_service(uart_service)
        ble.start_advertising(advertisement)
        advertisement.add_to_service(ble,uuid="Arm/Disarm",properties=2, read_perm=1, write_perm=0, maxlength = 1, fixed_length = True, initial_value = b'\x00', user_description="Arm/Disarm Command")
        while not ble.connected:
            pass
        print("Bluetooth connected!")

class SBUS:
    """SBUS protocol for RC control"""
    
    def __init__(self, tx_pin):
        self.uart = busio.UART(
            tx=tx_pin,
            rx=None,
            baudrate=100000,
            bits=8,
            parity=busio.UART.Parity.EVEN,
            stop=2,
            timeout=0
        )
        self.channels = [992] * 16
    
    def set(self, channel, value):
        """Set channel (0-15) to value (172-1811)"""
        if 0 <= channel < 16:
            self.channels[channel] = max(172, min(1811, value))
    
    def send(self):
        """Send SBUS packet"""
        # Pack 16 channels (11 bits each)
        bits = 0
        for i, ch in enumerate(self.channels):
            bits |= (ch & 0x7FF) << (i * 11)
        
        # Build packet
        packet = bytearray(25)
        packet[0] = 0x0F  # Header
        
        for i in range(22):
            packet[i + 1] = (bits >> (i * 8)) & 0xFF
        
        packet[23] = 0x00  # Flags
        packet[24] = 0x00  # Footer
        
        self.uart.write(bytes(packet))

def send_sbus(sbus, channel_values):
    """Helper to set channels and send SBUS packet"""
    for ch, val in channel_values.items():
        sbus.set(ch, val)
    sbus.send()

while True:
    bluetooth()
    if _bleio.connected:
        sbus = SBUS(tx_pin=board.TX)
        while _bleio.connected:
            if _bleio.characteristic_exists("Arm/Disarm"):
                command = _bleio.read_characteristic("Arm/Disarm")
                if command == b'\x01':  # Arm command
                    print("Arming drone...")
                    send_sbus(sbus, {8: 1811})  # Example: Arm the drone (channel 0 to 1811)
                elif command == b'\x00':  # Disarm command
                    print("Disarming drone...")
                    send_sbus(sbus, {8: 992})  # Example: Disarm the drone (channel 0 to 992)
            time.sleep(0.1)  # Adjust as needed for responsiveness

