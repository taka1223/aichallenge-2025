import pandas as pd
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from pathlib import Path

# ==== パス設定 ====
csv_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage_2.csv")
osm_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm")

# ==== raceline 読み込み ====
print(f"Loading raceline: {csv_path}")
df = pd.read_csv(csv_path)

print(f"points: {len(df)}")
print("raceline x range:", df["x"].min(), "→", df["x"].max())
print("raceline y range:", df["y"].min(), "→", df["y"].max())

# ==== lanelet2 map 読み込み ====
print(f"Loading lanelet2 map: {osm_path}")
tree = ET.parse(osm_path)
root = tree.getroot()

def tag(elem):  # OSM は namespace がつくので末尾だけ見る
    return elem.tag.split('}')[-1]

# node(id -> (x, y)) を辞書に入れる
nodes = {}
for n in root:
    if tag(n) == "node":
        node_id = int(n.attrib["id"])
        x = float(n.attrib.get("x", n.attrib.get("lon", 0.0)))
        y = float(n.attrib.get("y", n.attrib.get("lat", 0.0)))
        nodes[node_id] = (x, y)

print("lanelet nodes:", len(nodes))

# way ごとに座標列を作成
ways = []
for w in root:
    if tag(w) != "way":
        continue
    node_ids = []
    for nd in w:
        if tag(nd) == "nd":
            node_ids.append(int(nd.attrib["ref"]))
    if len(node_ids) >= 2:
        ways.append(node_ids)

print("lanelet ways:", len(ways))

# ==== 描画 ====
plt.figure(figsize=(10, 10))

# まず lanelet2 の線を薄い線で描画
for node_ids in ways:
    xs = [nodes[nid][0] for nid in node_ids if nid in nodes]
    ys = [nodes[nid][1] for nid in node_ids if nid in nodes]
    if len(xs) >= 2:
        plt.plot(xs, ys, linewidth=0.5)  # コース全体

# その上に raceline を太い線で描画
plt.plot(df["x"], df["y"], "-", linewidth=2)

# ★ raceline の範囲に自動ズームする（ここがポイント）
margin = 20.0  # 余白[m]
xmin, xmax = df["x"].min(), df["x"].max()
ymin, ymax = df["y"].min(), df["y"].max()
plt.xlim(xmin - margin, xmax + margin)
plt.ylim(ymin - margin, ymax + margin)

plt.xlabel("x")
plt.ylabel("y")
plt.title("Raceline on Lanelet2 Map (zoomed to raceline)")
plt.axis("equal")
plt.grid(True)
plt.tight_layout()
plt.show()
