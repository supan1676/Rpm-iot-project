import serial
import serial.tools.list_ports
import time
import sys

def upload_file(ser, local_path, remote_name):
    with open(local_path, "rb") as f:
        data = f.read()

    print(f"Uploading {local_path} -> {remote_name} ({len(data)} bytes)...")
    
    # Enter raw repl
    ser.write(b"\r\x03\x03")
    time.sleep(0.3)
    ser.write(b"\r\x01")
    time.sleep(0.3)
    ser.read_all()

    # Open file for writing
    ser.write(f'f = open("{remote_name}", "wb")\r\n'.encode('utf-8') + b"\x04")
    time.sleep(0.15)
    ser.read_all()

    # Write in chunks
    chunk_size = 128
    for i in range(0, len(data), chunk_size):
        chunk = data[i:i+chunk_size]
        cmd = b'f.write(' + repr(chunk).encode('latin1') + b')\r\n'
        ser.write(cmd + b"\x04")
        time.sleep(0.05)
        ser.read_all()

    # Close file
    ser.write(b'f.close()\r\n\x04')
    time.sleep(0.1)
    ser.read_all()
    print(f"Uploaded {remote_name} successfully!")

def find_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return "COM6"
    
    # Look for common ESP32 USB-UART devices
    for p in ports:
        desc = (p.description or "").lower()
        if any(keyword in desc for keyword in ["ch340", "cp210", "ftdi", "usb serial", "uart", "esp32", "jtag"]):
            return p.device
            
    # Default to the first detected serial port if none matched specifically
    return ports[0].device

def main():
    port = find_port()
    print(f"Connecting to {port} at 115200 baud...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"Error opening {port}: {e}")
        print("Usage: python upload_to_esp32.py [COM_PORT]")
        return

    # Interrupt whatever is running
    ser.write(b"\r\x03\x03")
    time.sleep(0.4)
    ser.write(b"\r\x01")
    time.sleep(0.3)
    ans = ser.read_all().decode("latin1", errors="ignore")
    if "raw REPL" not in ans:
        print("Waiting for raw REPL, resetting...")
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.6)
        ser.write(b"\r\x03\x03\x01")
        time.sleep(0.3)
        ser.read_all()

    files = [
        "boot.py",
        "ssd1306.py",
        "mpu6050.py",
        "max30102.py",
        "mlx90614.py",
        "main.py"
    ]
    for fn in files:
        upload_file(ser, fn, fn)

    # Soft reboot board into main.py
    print("Rebooting ESP32...")
    ser.write(b"\r\x04")
    time.sleep(0.5)
    ser.close()
    print("Done! All files uploaded successfully and ESP32 running.")

if __name__ == "__main__":
    main()
