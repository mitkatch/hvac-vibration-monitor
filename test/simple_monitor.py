"""
Simple BLE Monitor - Use AFTER pairing via iPhone/nRF Connect

This script assumes the device is already paired.
Just connect and monitor notifications.

Usage:
1. First pair device using iPhone + nRF Connect
2. Then run this script on Windows to monitor data
"""

import asyncio
from bleak import BleakClient, BleakScanner
import struct
import time

DEVICE_NAME = "HVAC-Vibe"
BURST_DATA_UUID = "12345678-1234-5678-1234-56789abcdef2"

# Data counters
packet_count = 0
sample_count = 0
start_time = None

def notification_handler(sender, data):
    """Handle incoming vibration data notifications"""
    global packet_count, sample_count, start_time
    
    if start_time is None:
        start_time = time.time()
    
    packet_count += 1
    
    # Each sample is 6 bytes: int16 x, y, z
    num_samples = len(data) // 6
    sample_count += num_samples
    
    print(f"📦 Packet {packet_count}: {len(data)} bytes, {num_samples} samples")
    
    # Show first sample
    if len(data) >= 6:
        x, y, z = struct.unpack('<hhh', data[0:6])
        print(f"   First sample: X={x:6d} Y={y:6d} Z={z:6d}")
    
    # Statistics every 10 packets
    if packet_count % 10 == 0:
        elapsed = time.time() - start_time
        rate = sample_count / elapsed if elapsed > 0 else 0
        print(f"\n📊 Stats: {sample_count} samples in {elapsed:.1f}s = {rate:.1f} samples/sec\n")

async def main():
    print("🔍 Scanning for HVAC-Vibe...")
    
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    
    if not device:
        print("❌ Device not found")
        print("\n💡 Make sure:")
        print("   1. Device is powered on")
        print("   2. Device is advertising")
        print("   3. Device is NOT connected to iPhone")
        return
    
    print(f"✅ Found: {device.name} ({device.address})")
    print(f"🔗 Connecting...")
    
    try:
        async with BleakClient(device, timeout=20.0) as client:
            print(f"✅ Connected!")
            
            # Check if already paired
            try:
                # Try to read a protected characteristic
                services = await client.get_services()
                print(f"📋 Found {len(services)} services")
            except Exception as e:
                print(f"⚠️  Service discovery issue: {e}")
                print("\n💡 If you see pairing errors:")
                print("   1. Pair device using iPhone + nRF Connect FIRST")
                print("   2. Then disconnect from iPhone")
                print("   3. Then run this script")
                return
            
            print(f"📡 Subscribing to burst data notifications...")
            
            try:
                await client.start_notify(BURST_DATA_UUID, notification_handler)
                print(f"✅ Subscribed! Waiting for data...")
                print(f"   (Device sends data every 10 seconds)\n")
                
                # Wait for data (5 minutes)
                await asyncio.sleep(300)
                
            except Exception as e:
                print(f"❌ Subscription failed: {e}")
                print("\n💡 Common causes:")
                print("   1. Device not paired - pair with iPhone first")
                print("   2. Device already connected elsewhere")
                print("   3. Bluetooth permissions not granted")
                return
            
    except Exception as e:
        print(f"❌ Connection error: {e}")
        print("\n💡 Troubleshooting:")
        print("   1. Make sure device is paired (use iPhone)")
        print("   2. Restart Bluetooth on Windows")
        print("   3. Try 'Forget device' then re-pair")

if __name__ == "__main__":
    print("=" * 60)
    print("  HVAC-Vibe Monitor (Windows)")
    print("  Assumes device already paired via iPhone")
    print("=" * 60)
    print()
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n👋 Stopped by user")
        if packet_count > 0:
            print(f"📊 Final: {packet_count} packets, {sample_count} samples")
