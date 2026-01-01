import polars as pl
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from pathlib import Path

# ==== パス ====
csv_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage_2.csv")
osm_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm")

# ==== raceline ====
df = pl.read_csv(csv_path).with_row_index("idx")
x = df["x"].to_numpy()
y = df["y"].to_numpy()

# ==== lanelet2 ====
tree = ET.parse(osm_path)
root = tree.getroot()

nodes = {}
for n in root.findall(".//{*}node"):
    nid = n.attrib.get("id")
    if nid is None:
        continue
    tags = {t.attrib.get("k"): t.attrib.get("v") for t in n.findall("{*}tag")}
    if "local_x" in tags and "local_y" in tags:
        nodes[int(nid)] = (float(tags["local_x"]), float(tags["local_y"]))

ways = []
for w in root.findall(".//{*}way"):
    nds = [int(nd.attrib["ref"]) for nd in w.findall("{*}nd")]
    if len(nds) >= 2:
        ways.append(nds)

# ==== 描画 ====
fig, ax = plt.subplots(figsize=(10,10))

for nds in ways:
    xs, ys = zip(*[nodes[n] for n in nds if n in nodes])
    ax.plot(xs, ys, linewidth=0.5, alpha=0.5)

ax.plot(x, y, "-o", markersize=3, linewidth=2)

# racelineでズーム固定
margin = 10
ax.set_xlim(x.min()-margin, x.max()+margin)
ax.set_ylim(y.min()-margin, y.max()+margin)
ax.set_aspect("equal", adjustable="box")
ax.set_autoscale_on(False)
ax.grid(True)
ax.set_title("Click near the red-circled point")

# ==== クリックイベント ====
def on_click(event):
    if event.xdata is None or event.ydata is None:
        return

    # 最近傍点
    d2 = (x - event.xdata)**2 + (y - event.ydata)**2
    i = int(np.argmin(d2))

    print(f"\nClicked at x={event.xdata:.3f}, y={event.ydata:.3f}")
    print(f"Nearest CSV row: {i}")
    print(f"  raceline x={x[i]:.3f}, y={y[i]:.3f}")

    # 強調表示
    ax.plot(x[i], y[i], "ro", markersize=10)
    ax.text(x[i], y[i], f"  {i}", color="red", fontsize=10)
    fig.canvas.draw()

cid = fig.canvas.mpl_connect("button_press_event", on_click)

plt.show()
