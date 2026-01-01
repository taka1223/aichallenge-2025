import polars as pl
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from pathlib import Path

# ========= パス =========
csv_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage_2.csv")
osm_path = Path("/home/ishikawa/workspace/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm")

assert csv_path.exists(), f"CSV not found: {csv_path}"
assert osm_path.exists(), f"OSM not found: {osm_path}"

# ========= 調整パラメータ =========
i_center = 60            # ←壁が近い場所（あなたの例）
half_win = 15            # 影響範囲（±） ※12→15推奨
offset_m = 1.5           # ★追加：アウト側へ逃がす量[m]（0.6→0.9推奨）
zoom_margin = 12.0       # 描画の余白
wall_bbox_margin = 30.0  # 壁候補を絞るbbox余白（m）
# ==================================

# ========= Raceline読み込み =========
df = pl.read_csv(csv_path).with_row_index("idx")
x = df["x"].to_numpy()
y = df["y"].to_numpy()

n = len(x)
i0 = max(1, i_center - half_win)
i1 = min(n - 2, i_center + half_win)

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

# wayを座標列として保持
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

# ========= 壁候補をraceline近傍だけに絞る =========
xmin, xmax = x.min(), x.max()
ymin, ymax = y.min(), y.max()
bbox = (xmin - wall_bbox_margin, xmax + wall_bbox_margin, ymin - wall_bbox_margin, ymax + wall_bbox_margin)

def intersects_bbox(xs, ys, bbox):
    x0, x1, y0, y1 = bbox
    return (xs.max() >= x0) and (xs.min() <= x1) and (ys.max() >= y0) and (ys.min() <= y1)

wall_ways_near = [(xs, ys, tags) for (xs, ys, tags) in wall_ways if intersects_bbox(xs, ys, bbox)]
all_ways_near  = [(xs, ys, tags) for (xs, ys, tags) in all_ways  if intersects_bbox(xs, ys, bbox)]
print(f"Near bbox: all={len(all_ways_near)}, wall_candidates={len(wall_ways_near)}")

# ========= 距離計算：点→線分群 =========
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

target_ways = wall_ways_near if len(wall_ways_near) > 0 else all_ways_near
segments = build_segments(target_ways)
assert segments is not None, "No segments found. OSM parsing/coordinate system might be wrong."

seg_x1, seg_y1, seg_x2, seg_y2 = segments

def min_dist_for_range(xx, yy, a0, a1):
    d = []
    for i in range(a0, a1 + 1):
        d.append(point_to_segments_min_dist(xx[i], yy[i], seg_x1, seg_y1, seg_x2, seg_y2).min())
    d = np.array(d)
    imin_local = int(np.argmin(d))
    imin = a0 + imin_local
    return d, imin, d[imin_local]

# ========= まず修正前の距離 =========
d_before, imin_b, dmin_b = min_dist_for_range(x, y, i0, i1)
print(f"\n[BEFORE] nearest within [{i0}, {i1}]: idx={imin_b}, dist={dmin_b:.3f} m")

# ========= ★局所オフセット（アウト側へ） =========
def smooth_weight(t):
    # 0..1..0 のなだらか重み
    return 0.5 - 0.5*np.cos(2*np.pi*t)

x_mod = x.copy()
y_mod = y.copy()

# 「向き判定用」の微小ステップ（大きすぎると判定が不安定）
probe = 0.10  # m

for i in range(i0, i1 + 1):
    # 接線（前後差分）
    tx = x[i+1] - x[i-1]
    ty = y[i+1] - y[i-1]
    tnorm = np.hypot(tx, ty)
    if tnorm == 0:
        continue
    tx /= tnorm
    ty /= tnorm

    # 左法線（左に90度）
    nxL, nyL = -ty, tx
    # 右法線
    nxR, nyR = -nxL, -nyL

    # 現在の壁距離
    d0 = point_to_segments_min_dist(x[i], y[i], seg_x1, seg_y1, seg_x2, seg_y2).min()

    # 左へ probe 動かした場合の壁距離
    dL = point_to_segments_min_dist(x[i] + nxL*probe, y[i] + nyL*probe,
                                    seg_x1, seg_y1, seg_x2, seg_y2).min()

    # 右へ probe 動かした場合の壁距離
    dR = point_to_segments_min_dist(x[i] + nxR*probe, y[i] + nyR*probe,
                                    seg_x1, seg_y1, seg_x2, seg_y2).min()

    # 壁からより離れる方向を採用
    if dL >= dR:
        nx_away, ny_away = nxL, nyL
    else:
        nx_away, ny_away = nxR, nyR

    # 重み（中心で最大）
    t = (i - i0) / (i1 - i0 + 1e-9)
    w = smooth_weight(t)

    x_mod[i] += nx_away * offset_m * w
    y_mod[i] += ny_away * offset_m * w

# ========= 修正後の距離 =========
d_after, imin_a, dmin_a = min_dist_for_range(x_mod, y_mod, i0, i1)
print(f"[AFTER ] nearest within [{i0}, {i1}]: idx={imin_a}, dist={dmin_a:.3f} m")
print(f"Suggested goal: >= 2.2 m (now: {dmin_a:.3f} m)")

# ========= 保存 =========
df_out = df.with_columns([
    pl.Series("x", x_mod),
    pl.Series("y", y_mod),
])
out_path = csv_path.with_name(csv_path.stem + f"_safe_i{i_center}_off{offset_m:.2f}.csv")
df_out.drop("idx").write_csv(out_path)  # idx列は不要なら落とす
print("Saved:", out_path)

# ========= 描画（壁＋修正前後raceline＋危険点） =========
fig, ax = plt.subplots(figsize=(10, 10))

for xs, ys, _ in all_ways_near:
    ax.plot(xs, ys, linewidth=0.4, alpha=0.35)

for xs, ys, _ in wall_ways_near:
    ax.plot(xs, ys, linewidth=1.0, alpha=0.75)

# before/after
ax.plot(x, y, "--", linewidth=2.0, alpha=0.6, label="raceline BEFORE")
ax.plot(x_mod, y_mod, "-", linewidth=2.5, label="raceline AFTER")

# 編集範囲を強調
ax.scatter(x[i0:i1+1], y[i0:i1+1], s=18, alpha=0.25, label="edited range (before)")
ax.scatter(x_mod[i0:i1+1], y_mod[i0:i1+1], s=18, alpha=0.25, label="edited range (after)")

# 最短点マーキング
ax.scatter([x[imin_b]], [y[imin_b]], s=140, marker="x", label=f"min BEFORE idx={imin_b}, d={dmin_b:.2f}m")
ax.scatter([x_mod[imin_a]], [y_mod[imin_a]], s=140, marker="x", label=f"min AFTER idx={imin_a}, d={dmin_a:.2f}m")

# zoom
margin = zoom_margin
ax.set_xlim(xmin - margin, xmax + margin)
ax.set_ylim(ymin - margin, ymax + margin)
ax.set_aspect("equal", adjustable="box")
ax.set_autoscale_on(False)
ax.grid(True)
ax.set_title("Raceline + Lanelet2 (walls) + distance check + local offset")
ax.legend(loc="best")
fig.tight_layout()
plt.show()
