import sqlite3
import polars as pl
from pathlib import Path

db_path = Path("run04_0.db3")  # 今いる run04/ の中にある前提
conn = sqlite3.connect(db_path)

topics = pl.read_database("SELECT id, name, type FROM topics", connection=conn)
print(topics)

conn.close()