import serial
import numpy as np

START = 0xAA
END   = 0x55


def read_packet(ser):
    """Reads one valid packet of the form: START, CSV_STRING, END."""
    
    # Wait for START
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == START:
            break

    # Read until END
    data_bytes = b""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == END:
            break
        data_bytes += b

    return data_bytes.decode("utf-8")


def parse_data(data_str):
    """
    Expects CSV string: R1,R2,R3,R4,Roll,Pitch,Yaw
    Returns numpy array length 7, or None if bad data.
    """
    pass


def start_baseline(ser, n_samples=100):
    """
    Compute initial baseline mean & variance from first readings.
    """
    readings = []

    print("Collecting baseline...")

    while len(readings) < n_samples:
        packet = read_packet(ser)
        if packet is None:
            continue

        data = parse_data(packet)
        if data is None:
            continue

        readings.append(data)

    readings = np.array(readings)
    baseline_mean = np.mean(readings, axis=0)
    baseline_var  = np.var(readings, axis=0)

    print("Baseline mean:", baseline_mean)
    print("Baseline var: ", baseline_var)

    return baseline_mean, baseline_var


def update_and_compute(baseline_mean, baseline_var, new_data,
                       alpha=0.01, K=2.5):
    """
    - computes std
    - computes threshold
    - computes finger states
    - updates baseline using neutral samples
    """

    std = np.sqrt(baseline_var)
    threshold = baseline_mean + K * std

    # finger states (bent)
    states = (new_data[:4] > threshold[:4]).astype(int)

    # determine neutral samples (close to baseline)
    neutral_mask = np.abs(new_data - baseline_mean) < (K * std)

    # update baseline for neutral values
    for i in range(len(new_data)):
        if neutral_mask[i]:
            baseline_mean[i] = (1 - alpha) * baseline_mean[i] + alpha * new_data[i]
            diff = new_data[i] - baseline_mean[i]
            baseline_var[i]  = (1 - alpha) * baseline_var[i] + alpha * (diff * diff)

    return baseline_mean, baseline_var, states


def main():
    ser = serial.Serial("COM8", 115200, timeout=1)

    baseline_mean, baseline_var = start_baseline(ser)

    while True:
        packet = read_packet(ser)
        if packet is None:
            continue

        data = parse_data(packet)
        if data is None:
            continue

        baseline_mean, baseline_var, states = update_and_compute(
            baseline_mean, baseline_var, data
        )

        print("R:", data[:4], "States:", states)


if __name__ == "__main__":
    main()
