import polars as pl
import matplotlib.pyplot as plt

df = pl.read_parquet("actual_speed.parquet").to_pandas()

plt.figure()
plt.plot(df["t"], df["actual_speed"])
plt.xlabel("time [s]")
plt.ylabel("actual_speed [m/s]")
plt.title("Actual speed from Odometry")
plt.show()
