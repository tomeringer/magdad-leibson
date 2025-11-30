import serial

START = 0xAA
END   = 0x55

def read_packet(ser):
    """Reads one valid packet of the form: START, DATA, END"""
    # Wait for START byte
    while True:
        b = ser.read(1)
        if not b:
            return None  # timeout or no data
        if b[0] == START:
            break

    # Read DATA byte
    data = ser.read(1)
    if not data:
        return None
    data = data[0]

    # Read END byte
    end = ser.read(1)
    if not end or end[0] != END:
        return None  # corrupted packet

    return data


def parse_bits(byte_val):
    """Returns a dict of the 4 sensor states from a packed byte"""
    return {
        "sensor1": (byte_val >> 0) & 1,
        "sensor2": (byte_val >> 1) & 1,
        "sensor3": (byte_val >> 2) & 1,
        "sensor4": (byte_val >> 3) & 1,
    }


def main():
    ser = serial.Serial("COM8", 115200, timeout=1)

    while True:
        packet = read_packet(ser)
        if packet is None:
            continue  # skip bad or incomplete packets

        states = parse_bits(packet)
        print(states)


if __name__ == "__main__":
    main()
