import serial

PORT = "COM6"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

START = 0xAA
END   = 0x55

def read_packet():
    # wait for start byte
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == START:
            break

    # read data byte
    data = ser.read(1)
    if not data:
        return None
    data_byte = data[0]

    # read end byte
    end = ser.read(1)
    if not end or end[0] != END:
        return None

    # extract the 4 bits
    s1 = (data_byte >> 0) & 1
    s2 = (data_byte >> 1) & 1
    s3 = (data_byte >> 2) & 1
    s4 = (data_byte >> 3) & 1

    return [s1, s2, s3, s4]


while True:
    sensors = read_packet()
    if sensors:
        print("Sensors:", sensors)