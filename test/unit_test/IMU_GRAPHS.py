import pandas as pd
import matplotlib.pyplot as plt

# ---- Load CSV ----
df = pd.read_csv("data.csv")

# ---- GRAPH 1: Accelerometer Raw vs Filtered ----
plt.figure(figsize=(10,5))

plt.plot(df.index, df["ax"], label="AX Raw", alpha=0.6)
plt.plot(df.index, df["ax_filt"], label="AX Filtered", linewidth=2)

# plt.plot(df.index, df["ay"], label="AY Raw", alpha=0.6)
# plt.plot(df.index, df["ay_filt"], label="AY Filtered", linewidth=2)

# plt.plot(df.index, df["az"], label="AZ Raw", alpha=0.6)
# plt.plot(df.index, df["az_filt"], label="AZ Filtered", linewidth=2)

plt.title("Accelerometer Raw vs Filtered")
plt.xlabel("Sample Index")
plt.ylabel("Acceleration")
plt.legend()
plt.grid(True)
plt.show()


# ---- GRAPH 2: Magnetometer Raw vs Filtered ----
plt.figure(figsize=(10,5))

# plt.plot(df.index, df["mx"], label="MX Raw", alpha=0.6)
# plt.plot(df.index, df["mx_filt"], label="MX Filtered", linewidth=2)

plt.plot(df.index, df["my"], label="MY Raw", alpha=0.6)
plt.plot(df.index, df["my_filt"], label="MY Filtered", linewidth=2)

# plt.plot(df.index, df["mz"], label="MZ Raw", alpha=0.6)
# plt.plot(df.index, df["mz_filt"], label="MZ Filtered", linewidth=2)

plt.title("Magnetometer Raw vs Filtered")
plt.xlabel("Sample Index")
plt.ylabel("Magnetic Field")
plt.legend()
plt.grid(True)
plt.show()


# ---- GRAPH 3: Yaw vs Time ----
plt.figure(figsize=(10,5))
plt.plot(df.index, df["yaw"], label="Yaw", color="purple")
plt.title("Yaw vs Time")
plt.xlabel("Sample Index")
plt.ylabel("Yaw (Degrees)")
plt.grid(True)
plt.legend()
plt.show()
