import serial, csv, time

port = "/dev/tty.usbmodem101"  # <-- UPDATE THIS
baud = 115200

ser = serial.Serial(port, baud)
print("Logging... press CTRL+C to stop")

with open("imu_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["AccX_raw","AccY_raw","AccZ_raw",
                     "AccX_f","AccY_f","AccZ_f",
                     "MagX_raw","MagY_raw","MagZ_raw",
                     "MagX_f","MagY_f","MagZ_f",
                     "Yaw"])

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("DATA"):
                fields = line.split(",")[1:]
                writer.writerow(fields)
    except KeyboardInterrupt:
        print("Saved imu_log.csv")
