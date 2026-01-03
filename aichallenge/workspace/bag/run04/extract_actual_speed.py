import sqlite3
from pathlib import Path

import polars as pl
from rosbags.typesys import Stores, get_typestore

# -------- config --------
DB = Path("run04_0.db3")
TOPIC_NAME = "/localization/kinematic_state"
MSGTYPE = "nav_msgs/msg/Odometry"
OUT = Path("actual_speed.parquet")  # CSVでもOK。まずparquet推奨（速い）
# ------------------------

typestore = get_typestore(Stores.ROS2_HUMBLE)

def decode_speed(data: bytes) -> float:
    # rosbags の typestore から CDR をデコード
    msg = typestore.deserialize_cdr(data, MSGTYPE)
    return float(msg.twist.twist.linear.x)

conn = sqlite3.connect(DB)

# timestamp(ns) と data(BLOB) だけ取る（速い）
df = pl.read_database(
    f"""
    SELECT m.timestamp AS t_ns, m.data AS data
    FROM messages m
    JOIN topics t ON t.id = m.topic_id
    WHERE t.name = '{TOPIC_NAME}'
    ORDER BY m.timestamp
    """,
    connection=conn,
)

conn.close()

# data(BLOB) -> actual_speed
df = df.with_columns(
    pl.col("data").map_elements(decode_speed, return_dtype=pl.Float64).alias("actual_speed")
).drop("data")

# 秒にして、t=0開始に
df = df.with_columns(
    (pl.col("t_ns") * 1e-9).alias("t")
).with_columns(
    (pl.col("t") - pl.col("t").first()).alias("t")
).drop("t_ns")

print(df.head(5))
print("rows:", df.height)

df.write_parquet(OUT)
print("saved:", OUT)
