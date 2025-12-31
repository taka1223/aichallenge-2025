import polars as pl
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib as mpl
mpl.rcParams["keymap.save"] = []  

# ========= パス =========
csv_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage_2_manual_edit.csv")
osm_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm")

assert csv_path.exists(), f"CSV not found: {csv_path}"
assert osm_path.exists(), f"OSM not found: {osm_path}"

# ========= 調整 =========
zoom_margin = 12.0
wall_bbox_margin = 30.0

# 「点を掴める半径」[m]（大きいほど掴みやすい）
pick_radius_m = 1.5

# 壁距離の目標（表示用）
d_target = 2.2

# ========= Raceline読み込み =========
df = pl.read_csv(csv_path).with_row_index("idx")
x = df["x"].to_numpy()
y = df["y"].to_numpy()
n = len(x)

x_mod = x.copy()
y_mod = y.copy()

print(f"Loaded raceline points: n={n}")
print("x range:", x.min(), "->", x.max())
print("y range:", y.min(), "->", y.max())

# ========= Lanelet2 OSM読み込み =========
tree = ET.parse(osm_path)
root = tree.getroot()

# node id -> (x,y) （local_x/local_y優先）
nodes = {}
for node in root.findall(".//{*}node"):
    nid = node.attrib.get("id")
    if nid is None:
        continue
    tags = {t.attrib.get("k"): t.attrib.get("v") for t in node.findall("{*}tag")}
    if "local_x" in tags and "local_y" in tags:
        nodes[int(nid)] = (float(tags["local_x"]), float(tags["local_y"]))
    elif "x" in tags and "y" in tags:
        nodes[int(nid)] = (float(tags["x"]), float(tags["y"]))
    else:
        lat = node.attrib.get("lat")
        lon = node.attrib.get("lon")
        if lat is not None and lon is not None:
            nodes[int(nid)] = (float(lon), float(lat))

def way_tags(way_elem):
    return {t.attrib.get("k"): t.attrib.get("v") for t in way_elem.findall("{*}tag")}

def is_wall_candidate(tags: dict) -> bool:
    barrier = (tags.get("barrier") or "").lower()
    typ = (tags.get("type") or "").lower()
    subtype = (tags.get("subtype") or "").lower()
    line_type = (tags.get("line_type") or "").lower()
    marking = (tags.get("road_marking") or "").lower()

    if barrier in {"wall", "fence", "guard_rail", "kerb", "curb"}:
        return True
    if typ in {"curbstone", "kerbstone", "wall", "fence", "guard_rail", "road_border"}:
        return True
    if subtype in {"curbstone", "kerbstone", "wall", "fence", "guard_rail"}:
        return True
    if line_type in {"solid", "solid_line", "line_solid", "line_thick"}:
        return True
    if marking in {"curb", "kerb"}:
        return True
    return False

all_ways = []
wall_ways = []

for way in root.findall(".//{*}way"):
    nds = [int(nd.attrib["ref"]) for nd in way.findall("{*}nd") if "ref" in nd.attrib]
    coords = [nodes.get(nid) for nid in nds]
    coords = [c for c in coords if c is not None]
    if len(coords) < 2:
        continue
    xs = np.array([c[0] for c in coords], dtype=float)
    ys = np.array([c[1] for c in coords], dtype=float)
    tags = way_tags(way)
    all_ways.append((xs, ys, tags))
    if is_wall_candidate(tags):
        wall_ways.append((xs, ys, tags))

print(f"OSM ways: total={len(all_ways)}, wall_candidates={len(wall_ways)}")

# ========= 近傍だけ表示（高速化） =========
xmin, xmax = x.min(), x.max()
ymin, ymax = y.min(), y.max()
bbox = (xmin - wall_bbox_margin, xmax + wall_bbox_margin, ymin - wall_bbox_margin, ymax + wall_bbox_margin)

def intersects_bbox(xs, ys, bbox):
    x0, x1, y0, y1 = bbox
    return (xs.max() >= x0) and (xs.min() <= x1) and (ys.max() >= y0) and (ys.min() <= y1)

wall_ways_near = [(xs, ys, tags) for (xs, ys, tags) in wall_ways if intersects_bbox(xs, ys, bbox)]
all_ways_near  = [(xs, ys, tags) for (xs, ys, tags) in all_ways  if intersects_bbox(xs, ys, bbox)]
print(f"Near bbox: all={len(all_ways_near)}, wall_candidates={len(wall_ways_near)}")

# ========= 距離計算用：線分配列 =========
def point_to_segments_min_dist(px, py, seg_x1, seg_y1, seg_x2, seg_y2):
    vx = seg_x2 - seg_x1
    vy = seg_y2 - seg_y1
    wx = px - seg_x1
    wy = py - seg_y1
    vv = vx*vx + vy*vy
    vv = np.where(vv == 0, 1e-12, vv)
    t = (wx*vx + wy*vy) / vv
    t = np.clip(t, 0.0, 1.0)
    projx = seg_x1 + t*vx
    projy = seg_y1 + t*vy
    dx = px - projx
    dy = py - projy
    return np.sqrt(dx*dx + dy*dy)

def build_segments(ways_list):
    segs = []
    for xs, ys, _ in ways_list:
        if len(xs) < 2:
            continue
        segs.append((xs[:-1], ys[:-1], xs[1:], ys[1:]))
    if not segs:
        return None
    seg_x1 = np.concatenate([s[0] for s in segs])
    seg_y1 = np.concatenate([s[1] for s in segs])
    seg_x2 = np.concatenate([s[2] for s in segs])
    seg_y2 = np.concatenate([s[3] for s in segs])
    return seg_x1, seg_y1, seg_x2, seg_y2

# 距離の対象はまず壁候補。なければ全ways
target_ways = wall_ways_near if len(wall_ways_near) > 0 else all_ways_near
segments = build_segments(target_ways)
assert segments is not None, "No segments found. OSM parsing/coordinate system might be wrong."
seg_x1, seg_y1, seg_x2, seg_y2 = segments

def wall_dist(px, py):
    return point_to_segments_min_dist(px, py, seg_x1, seg_y1, seg_x2, seg_y2).min()

# ========= 便利：最近傍 idx =========
def nearest_idx(px, py, xx, yy):
    d2 = (xx - px)**2 + (yy - py)**2
    return int(np.argmin(d2))

# ========= 描画 =========
fig, ax = plt.subplots(figsize=(10, 10))

for xs, ys, _ in all_ways_near:
    ax.plot(xs, ys, linewidth=0.4, alpha=0.35)
for xs, ys, _ in wall_ways_near:
    ax.plot(xs, ys, linewidth=1.0, alpha=0.75)

# BEFORE
ax.plot(x, y, "--", linewidth=2.0, alpha=0.5, label="raceline BEFORE")

# AFTER (編集対象)
line_after, = ax.plot(x_mod, y_mod, "-", linewidth=2.5, label="raceline AFTER (editable)")

# 選択点の表示（赤い丸）
sel_scatter = ax.scatter([], [], s=220, marker="o", facecolors="none", edgecolors="red", linewidths=2, zorder=20)
sel_text = ax.text(0, 0, "", color="red", fontsize=11, weight="bold", zorder=21)

# zoom
ax.set_xlim(xmin - zoom_margin, xmax + zoom_margin)
ax.set_ylim(ymin - zoom_margin, ymax + zoom_margin)
ax.set_aspect("equal", adjustable="box")
ax.set_autoscale_on(False)
ax.grid(True)
ax.set_title("LeftClick: info. Drag: move point. Press 'w' to save CSV. Press 'r' to reset.")
ax.legend(loc="best")
fig.tight_layout()

# ========= インタラクション状態 =========
drag = {"active": False, "idx": None}

def update_selection(i):
    d = wall_dist(x_mod[i], y_mod[i])
    sel_scatter.set_offsets([[x_mod[i], y_mod[i]]])
    sel_text.set_position((x_mod[i], y_mod[i]))
    sel_text.set_text(f"  idx={i}\n  d={d:.2f} m\n  target={d_target:.1f} m")
    fig.canvas.draw_idle()
    print(f"[SELECT] idx={i} x={x_mod[i]:.3f} y={y_mod[i]:.3f} wall_dist={d:.3f} m")

def on_press(event):
    if event.xdata is None or event.ydata is None:
        return
    i = nearest_idx(event.xdata, event.ydata, x_mod, y_mod)
    dist = np.hypot(x_mod[i] - event.xdata, y_mod[i] - event.ydata)
    if dist > pick_radius_m:
        return
    drag["active"] = True
    drag["idx"] = i
    update_selection(i)

def on_motion(event):
    if not drag["active"]:
        return
    if event.xdata is None or event.ydata is None:
        return
    i = drag["idx"]
    x_mod[i] = event.xdata
    y_mod[i] = event.ydata
    line_after.set_data(x_mod, y_mod)
    update_selection(i)

def on_release(event):
    if not drag["active"]:
        return
    i = drag["idx"]
    drag["active"] = False
    drag["idx"] = None
    # 最終位置を表示
    d = wall_dist(x_mod[i], y_mod[i])
    print(f"[DRAG END] idx={i} x={x_mod[i]:.3f} y={y_mod[i]:.3f} wall_dist={d:.3f} m")

def on_click(event):
    # クリック（ドラッグ開始もこれを通るので、activeのときは無視）
    if drag["active"]:
        return
    if event.xdata is None or event.ydata is None:
        return
    i = nearest_idx(event.xdata, event.ydata, x_mod, y_mod)
    update_selection(i)

def on_key(event):
    k = (event.key or "").lower()

    # 'w' でCSV保存（sはmatplotlibに取られがち）
    if k == "w":
        out_path = csv_path.with_name(csv_path.stem + "_manual_edit.csv")
        df_out = df.select([c for c in df.columns if c != "idx"]).with_columns([
            pl.Series(name="x", values=x_mod.tolist()),
            pl.Series(name="y", values=y_mod.tolist()),
        ])
        df_out.write_csv(out_path)
        print(f"[CSV SAVE OK] {out_path}")

    elif k == "r":
        x_mod[:] = x
        y_mod[:] = y
        line_after.set_data(x_mod, y_mod)
        sel_scatter.set_offsets(np.empty((0, 2)))
        sel_text.set_text("")
        fig.canvas.draw_idle()
        print("[RESET] back to original (BEFORE)")


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("key_press_event", on_key)


# ========= 保存（確実版） =========
out_path = csv_path.with_name(csv_path.stem + "_manual_edit.csv")

df_out = df.select([c for c in df.columns if c != "idx"]).with_columns([
    pl.Series(name="x", values=x_mod.tolist()),
    pl.Series(name="y", values=y_mod.tolist()),
])

df_out.write_csv(out_path)
print(f"[SAVE OK] {out_path}")

plt.show()
