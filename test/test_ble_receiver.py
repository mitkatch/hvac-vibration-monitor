#!/usr/bin/env python3
"""
Test BLE receiver for HVAC Vibration Monitor
Receives burst data from nRF52840 sensor node
"""

import asyncio
import struct
from bleak import BleakClient, BleakScanner

# UUIDs (must match your nRF52840 ble.c code)
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
BURST_DATA_UUID = "12345678-1234-5678-1234-56789abcdef1"
ENV_DATA_UUID = "12345678-1234-5678-1234-56789abcdef2"

# Device name to search for
DEVICE_NAME = "HVAC-Vibe"

# Buffer for receiving burst data
burst_buffer = bytearray()
expected_bytes = 3072  # 512 samples × 6 bytes


def notification_handler(sender, data):
    """Handle incoming BLE notifications (called automatically)"""
    global burst_buffer
    
    print(f"📦 Received packet: {len(data)} bytes")
    burst_buffer.extend(data)
    
    print(f"   Progress: {len(burst_buffer)}/{expected_bytes} bytes ({len(burst_buffer)*100//expected_bytes}%)")
    
    # Check if we have complete burst
    if len(burst_buffer) >= expected_bytes:
        print("\n✅ Complete burst received!\n")
        process_burst(burst_buffer)
        burst_buffer = bytearray()  # Reset for next burst


def env_notification_handler(sender, data):
    """Handle incoming environment data notifications"""
    if len(data) < 6:
        print(f"⚠️  Env packet too short: {len(data)} bytes")
        return

    temp_raw, hum_raw, press_raw = struct.unpack('>hhH', data[:6])

    temp  = temp_raw  / 100.0   # °C
    hum   = hum_raw   / 100.0   # %RH
    press = press_raw            # hPa

    print("\n🌡️  Environment Data")
    print(f"   Temperature: {temp:.2f} °C")
    print(f"   Humidity:    {hum:.2f} %RH")
    print(f"   Pressure:    {press} hPa\n")


def process_burst(data):
    """Process received burst data"""
    print("=" * 60)
    print("PROCESSING BURST DATA")
    print("=" * 60)
    
    # Parse samples (each sample: int16 x, int16 y, int16 z)
    num_samples = len(data) // 6
    
    print(f"\nTotal samples: {num_samples}")
    
    samples = []
    for i in range(num_samples):
        offset = i * 6
        # Little-endian signed 16-bit integers
        x, y, z = struct.unpack('<hhh', data[offset:offset+6])
        samples.append((x, y, z))
    
    # Print first 5 samples
    print("\n--- First 5 Samples ---")
    for i in range(min(5, len(samples))):
        x, y, z = samples[i]
        x_mg = x * 4  # Convert to milli-g (4 mg/LSB)
        y_mg = y * 4
        z_mg = z * 4
        print(f"  [{i:3d}] X={x_mg:6d} mg, Y={y_mg:6d} mg, Z={z_mg:6d} mg")
    
    if len(samples) > 10:
        print("  ...")
        print("\n--- Last 5 Samples ---")
        for i in range(len(samples) - 5, len(samples)):
            x, y, z = samples[i]
            x_mg = x * 4
            y_mg = y * 4
            z_mg = z * 4
            print(f"  [{i:3d}] X={x_mg:6d} mg, Y={y_mg:6d} mg, Z={z_mg:6d} mg")
    
    # Calculate RMS
    print("\n--- RMS Analysis ---")
    sum_x2 = sum(x*x for x, _, _ in samples)
    sum_y2 = sum(y*y for _, y, _ in samples)
    sum_z2 = sum(z*z for _, _, z in samples)
    
    rms_x = int((sum_x2 / len(samples)) ** 0.5) * 4
    rms_y = int((sum_y2 / len(samples)) ** 0.5) * 4
    rms_z = int((sum_z2 / len(samples)) ** 0.5) * 4
    
    print(f"X-axis RMS: {rms_x:5d} mg")
    print(f"Y-axis RMS: {rms_y:5d} mg")
    print(f"Z-axis RMS: {rms_z:5d} mg")
    
    # Calculate peak-to-peak
    print("\n--- Peak-to-Peak Analysis ---")
    x_vals = [x for x, _, _ in samples]
    y_vals = [y for _, y, _ in samples]
    z_vals = [z for _, _, z in samples]
    
    x_p2p = (max(x_vals) - min(x_vals)) * 4
    y_p2p = (max(y_vals) - min(y_vals)) * 4
    z_p2p = (max(z_vals) - min(z_vals)) * 4
    
    print(f"X-axis: {x_p2p} mg")
    print(f"Y-axis: {y_p2p} mg")
    print(f"Z-axis: {z_p2p} mg")
    
    print("\n" + "=" * 60)
    print("BURST PROCESSING COMPLETE")
    print("=" * 60 + "\n")


async def read_metadata(client):
    """Read burst metadata characteristic"""
    try:
        data = await client.read_gatt_char(METADATA_UUID)
        
        # Parse metadata struct
        timestamp, rate, count, sensor_id = struct.unpack('<IHHB', data[:9])
        
        print("\n" + "=" * 60)
        print("BURST METADATA")
        print("=" * 60)
        print(f"Timestamp:    {timestamp} ms")
        print(f"Sample rate:  {rate} Hz")
        print(f"Sample count: {count}")
        print(f"Sensor ID:    {sensor_id}")
        print("=" * 60 + "\n")
        
    except Exception as e:
        print(f"⚠️  Could not read metadata: {e}")


async def main():
    """Main BLE connection loop"""
    print("\n" + "=" * 60)
    print("HVAC VIBRATION MONITOR - BLE RECEIVER")
    print("=" * 60 + "\n")
    
    print(f"🔍 Scanning for '{DEVICE_NAME}'...")
    print("   (Make sure your nRF52840 is powered on and advertising)\n")
    
    # Scan for device (10 second timeout)
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    
    if device is None:
        print(f"❌ ERROR: Could not find device '{DEVICE_NAME}'")
        print("\nTroubleshooting:")
        print("  1. Check nRF52840 is powered on")
        print("  2. Check serial output shows: 'BLE: Advertising started'")
        print("  3. Try moving laptop closer to device")
        print("  4. Check Bluetooth is enabled on your laptop")
        return
    
    print(f"✅ Found device!")
    print(f"   Name:    {device.name}")
    print(f"   Address: {device.address}\n")
    
    print("🔗 Connecting...")
    
    # Connect to device
    async with BleakClient(device) as client:
        print(f"✅ Connected!\n")
        
        # Subscribe to burst data notifications
        print("📡 Subscribing to burst data notifications...")
        await client.start_notify(BURST_DATA_UUID, notification_handler)
        print("✅ Subscribed!\n")

        print("📡 Subscribing to environment notifications...")
        await client.start_notify(ENV_DATA_UUID, env_notification_handler)
        print("✅ Subscribed!\n")
        
        print("⏳ Waiting for burst data...")
        print("   (Device sends bursts every 10 seconds)\n")
        print("   Press Ctrl+C to disconnect\n")
        
        # Keep connection alive and wait for data
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n\n🛑 Disconnecting...")
            await client.stop_notify(BURST_DATA_UUID)
            await client.stop_notify(ENV_DATA_UUID)
            print("✅ Disconnected cleanly\n")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 Goodbye!\n")