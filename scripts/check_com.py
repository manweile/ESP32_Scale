import sys
import serial

port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
try:
    s = serial.Serial(port, 115200, timeout=1)
    s.close()
    print(f'Opened and closed {port} successfully')
except Exception as e:
    print(f'Error opening {port}: {repr(e)}')
    sys.exit(1)
