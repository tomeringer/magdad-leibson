import serial

PORT = "COM6"     # change if needed
BAUD = 115200

START = 0xAA
END   = 0x55

ser = serial.Serial(PORT, BAUD, timeout=1)

def read_packet():
    """Read one [AA][data][55] packet safely."""
    
    # 1. Wait for START_BYTE
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == START:
            break

    # 2. Read data byte (the 0/1)
    data = ser.read(1)
    if not data:
        return None
    value = data[0]

    # 3. Verify END_BYTE
    e = ser.read(1)
    if not e or e[0] != END:
        return None

    return value


print("Listening on", PORT, "...")

while True:
    result = read_packet()
    if result is None:
        print("Waiting...")
        continue

    # Print nicely
    if result == 1:
        print("Sensor: HIGH (1)")
    elif result == 0:
        print("Sensor: LOW  (0)")
    else:
        print("Invalid value:", result)