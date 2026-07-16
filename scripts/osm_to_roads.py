"""Convert OpenStreetMap (Overpass) data into a .roads map (format v2).

Builds a real town: the drivable road network (with per-road type, speed limit,
lane count, and one-way direction from OSM tags), traffic signals where the town
actually has them, Entry/Exit nodes where roads cross the town line, and
Destination POIs (homes, shops, workplaces, schools, leisure, parking) at real
building locations. Every POI is connected to the road by a dirt single-lane
two-way driveway — the same connector the in-app Destination tool builds for
homes. Dirt shared-lane connectors rank below every road class, so the stop-sign
auto-assignment always derives a minor-road stop at the driveway foot: the
driveway stops, the through road flows free.

Geometry rules the simulation depends on (mirrors RoadGraph conventions):
  - Bezier handle length is exactly chord/3 (inside the loader's keep band).
  - No road edge shorter than ~8 m: short slivers from boundary clipping or
    dense OSM geometry are collapsed by merging their end nodes.
  - Driveway cut points keep >= 12 m from segment ends and >= 10 m apart;
    nearby POIs share one foot node.

Inputs are three Overpass API JSON responses (see the queries below).

Usage:
  python scripts/osm_to_roads.py --roads roads.json --pois pois.json \
      --boundary boundary.json --out town.roads \
      [--home-cap 1500] [--center LAT,LON] [--zoom 0.35] [--time 7.25]

With --water, real hydrography is painted into the map's water layer (format v3):
OSM natural=water polygons (ponds, lakes, wide-river banks) become circle fills —
a quadtree cover whose circle radii follow the distance to the shoreline — and
linear waterways (rivers, streams, brooks, canals, drains) become smooth
stream-segment chains, width from the width/est_width tag or a per-class default.
Waterway lines already covered by a water polygon are skipped (no double paint).

Overpass queries (bbox = S,W,N,E covering the town):
  roads:     way["highway"~"^(motorway|trunk|primary|secondary|tertiary|
             unclassified|residential|living_street|motorway_link|trunk_link|
             primary_link|secondary_link|tertiary_link)$"](bbox);
             node["highway"="traffic_signals"](bbox);   (._;>;); out body;
  pois:      way["building"](bbox); node/way with shop/amenity/office/craft/
             tourism/leisure tags (bbox);                out center;
  boundary:  relation["boundary"="administrative"]["admin_level"="8"]
             ["name"="<Town>"](bbox);                    out geom;
  water:     way["natural"="water"](bbox); relation["natural"="water"](bbox);
             way["waterway"~"^(river|stream|canal|drain|brook)$"](bbox); out geom;
"""

import argparse
import json
import math
import struct
from collections import Counter, defaultdict

# --- .roads enums (must match C# definitions) --------------------------------

POI_NONE, POI_HOME, POI_WORK, POI_SHOP, POI_LEISURE, POI_SCHOOL, POI_PARKING, POI_ENTRYEXIT = range(8)
POI_NAMES = ["None", "Home", "Work", "Shop", "Leisure", "School", "Parking", "EntryExit"]

RT_RESIDENTIAL, RT_ARTERIAL, RT_HIGHWAY, RT_DIRT = 0, 1, 2, 3

FLAG_TRAFFIC_LIGHT = 1
FLAG_STOP_SIGN = 2
FLAG_MANUAL_SIGNAL = 16
FLAG_DESTINATION = 32
FLAG_ACTUATED = 128

EDGEFLAG_SHARED_LANE = 1

MPH = 0.44704  # m/s per mph

# --- Tuning -------------------------------------------------------------------

DP_TOLERANCE = 1.5        # Douglas-Peucker simplification tolerance (m)
MIN_POINT_SPACING = 14.0  # minimum spacing of chain waypoints (m)
MIN_EDGE_LEN = 8.0        # road edges shorter than this are collapsed (m)
CUT_END_SETBACK = 12.0    # driveway cut clearance from segment ends (m)
CUT_CLUSTER_GAP = 12.0    # cuts closer than this share one foot node (m)
ENDPOINT_ATTACH_LEN = 26.0  # segments shorter than this attach at an endpoint (m)
MAX_DRIVEWAY = 130.0      # POIs farther than this from a road are skipped (m)

# --- OSM highway classification ----------------------------------------------

# highway value -> (RoadType, default speed m/s). "unclassified" town connector
# roads rank as Arterial so junctions with residential streets become minor-road
# stops (side street stops, connector flows) rather than all-way stops.
HIGHWAY_CLASS = {
    "motorway":       (RT_HIGHWAY, 65 * MPH),
    "trunk":          (RT_HIGHWAY, 50 * MPH),
    "motorway_link":  (RT_HIGHWAY, 35 * MPH),
    "trunk_link":     (RT_HIGHWAY, 35 * MPH),
    "primary":        (RT_ARTERIAL, 40 * MPH),
    "primary_link":   (RT_ARTERIAL, 30 * MPH),
    "secondary":      (RT_ARTERIAL, 35 * MPH),
    "secondary_link": (RT_ARTERIAL, 30 * MPH),
    "tertiary":       (RT_ARTERIAL, 30 * MPH),
    "tertiary_link":  (RT_ARTERIAL, 25 * MPH),
    "unclassified":   (RT_ARTERIAL, 30 * MPH),
    "residential":    (RT_RESIDENTIAL, 25 * MPH),
    "living_street":  (RT_RESIDENTIAL, 15 * MPH),
}

UNPAVED_SURFACES = {
    "unpaved", "gravel", "dirt", "ground", "earth", "grass", "sand",
    "compacted", "fine_gravel", "pebblestone", "mud", "rock", "woodchips",
}

# --- POI classification --------------------------------------------------------

HOME_BUILDINGS = {
    "house", "detached", "residential", "apartments", "semidetached_house",
    "terrace", "bungalow", "static_caravan", "mobile_home", "farm", "duplex",
    "dormitory", "semi", "cabin",
}
SCHOOL_AMENITIES = {"school", "kindergarten", "college", "university"}
SHOP_AMENITIES = {
    "restaurant", "cafe", "fast_food", "bar", "pub", "bank", "pharmacy",
    "fuel", "ice_cream", "post_office", "veterinary", "clinic", "doctors",
    "dentist", "car_wash", "marketplace",
}
LEISURE_KINDS = {
    "park", "pitch", "playground", "sports_centre", "golf_course", "stadium",
    "recreation_ground", "dog_park", "fitness_centre", "track", "marina",
}
LEISURE_AMENITIES = {
    "place_of_worship", "community_centre", "library", "theatre", "cinema",
    "arts_centre",
}
LEISURE_TOURISM = {"museum", "attraction", "gallery", "zoo"}
WORK_AMENITIES = {"townhall", "police", "fire_station", "courthouse", "post_depot"}
WORK_BUILDINGS = {"industrial", "warehouse", "office", "manufacture", "works"}
SHOP_BUILDINGS = {"retail", "commercial", "supermarket", "kiosk"}

# Dedupe priority (lower = kept preferentially when two POIs overlap).
POI_PRIORITY = {POI_SCHOOL: 0, POI_SHOP: 1, POI_LEISURE: 2, POI_WORK: 3, POI_PARKING: 4, POI_HOME: 5}


def classify_poi(tags):
    """Map OSM tags to a POIType, or None when the feature is not a destination."""
    building = tags.get("building", "")
    amenity = tags.get("amenity", "")
    shop = tags.get("shop", "")
    leisure = tags.get("leisure", "")
    tourism = tags.get("tourism", "")

    if amenity in SCHOOL_AMENITIES or building == "school":
        return POI_SCHOOL
    if shop or amenity in SHOP_AMENITIES or building in SHOP_BUILDINGS:
        return POI_SHOP
    if leisure in LEISURE_KINDS or amenity in LEISURE_AMENITIES or tourism in LEISURE_TOURISM:
        return POI_LEISURE
    if tags.get("office") or tags.get("craft") or amenity in WORK_AMENITIES or building in WORK_BUILDINGS:
        return POI_WORK
    if amenity == "parking" and tags.get("name"):
        return POI_PARKING
    if building in HOME_BUILDINGS:
        return POI_HOME
    if building == "yes" and "addr:housenumber" in tags:
        return POI_HOME
    return None


# --- Geometry helpers -----------------------------------------------------------

def meters_per_degree(lat_deg):
    """Accurate meters per degree of latitude/longitude at the given latitude."""
    phi = math.radians(lat_deg)
    m_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return m_lat, m_lon


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bezier_point(p0, c1, c2, p3, t):
    u = 1.0 - t
    a = u * u * u
    b = 3 * u * u * t
    c = 3 * u * t * t
    d = t * t * t
    return (a * p0[0] + b * c1[0] + c * c2[0] + d * p3[0],
            a * p0[1] + b * c1[1] + c * c2[1] + d * p3[1])


def bezier_tangent(p0, c1, c2, p3, t):
    u = 1.0 - t
    dx = 3 * u * u * (c1[0] - p0[0]) + 6 * u * t * (c2[0] - c1[0]) + 3 * t * t * (p3[0] - c2[0])
    dy = 3 * u * u * (c1[1] - p0[1]) + 6 * u * t * (c2[1] - c1[1]) + 3 * t * t * (p3[1] - c2[1])
    n = math.hypot(dx, dy)
    if n < 1e-9:
        n = 1.0
    return (dx / n, dy / n)


def bezier_length(p0, c1, c2, p3, steps=32):
    total = 0.0
    prev = p0
    for i in range(1, steps + 1):
        cur = bezier_point(p0, c1, c2, p3, i / steps)
        total += dist(prev, cur)
        prev = cur
    return total


def douglas_peucker(pts, tolerance):
    """Iterative Douglas-Peucker; returns indices of kept points (endpoints always)."""
    n = len(pts)
    if n <= 2:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[n - 1] = True
    stack = [(0, n - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 - i0 < 2:
            continue
        ax, ay = pts[i0]
        bx, by = pts[i1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        best_d, best_i = -1.0, -1
        for i in range(i0 + 1, i1):
            px, py = pts[i]
            if seg_len_sq < 1e-12:
                d = math.hypot(px - ax, py - ay)
            else:
                t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
                t = max(0.0, min(1.0, t))
                d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d > best_d:
                best_d, best_i = d, i
        if best_d > tolerance:
            keep[best_i] = True
            stack.append((i0, best_i))
            stack.append((best_i, i1))
    return [i for i in range(n) if keep[i]]


def point_in_ring(p, ring):
    """Even-odd point-in-polygon test. ring is a list of (x, y), not closed."""
    x, y = p
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y):
            x_cross = xi + (y - yi) / (yj - yi) * (xj - xi)
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def segment_ring_crossings(p, q, ring):
    """Parametric positions t in (0,1) along p->q where the segment crosses the ring."""
    ts = []
    px, py = p
    dx, dy = q[0] - px, q[1] - py
    lo_x, hi_x = min(px, q[0]), max(px, q[0])
    lo_y, hi_y = min(py, q[1]), max(py, q[1])
    n = len(ring)
    j = n - 1
    for i in range(n):
        ax, ay = ring[j]
        bx, by = ring[i]
        j = i
        if max(ax, bx) < lo_x or min(ax, bx) > hi_x or max(ay, by) < lo_y or min(ay, by) > hi_y:
            continue
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue
        t = ((ax - px) * ey - (ay - py) * ex) / denom
        s = ((ax - px) * dy - (ay - py) * dx) / denom
        if 1e-9 < t < 1 - 1e-9 and -1e-9 <= s <= 1 + 1e-9:
            ts.append(t)
    return sorted(ts)


# --- Boundary -------------------------------------------------------------------

def load_boundary(path, project):
    """Stitch the admin relation's outer ways into rings; return the largest, projected."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    rels = [e for e in data["elements"] if e["type"] == "relation"]
    if not rels:
        raise SystemExit("boundary file contains no relation")
    rel = rels[0]

    def key(pt):
        return (round(pt["lat"], 7), round(pt["lon"], 7))

    ways = [m["geometry"] for m in rel["members"]
            if m.get("role") == "outer" and m["type"] == "way" and "geometry" in m]
    rings = []
    remaining = list(range(len(ways)))
    while remaining:
        chain = list(ways[remaining.pop(0)])
        extended = True
        while extended and key(chain[0]) != key(chain[-1]):
            extended = False
            for ri, wi in enumerate(remaining):
                w = ways[wi]
                if key(w[0]) == key(chain[-1]):
                    chain.extend(w[1:])
                elif key(w[-1]) == key(chain[-1]):
                    chain.extend(reversed(w[:-1]))
                elif key(w[-1]) == key(chain[0]):
                    chain[:0] = w[:-1]
                elif key(w[0]) == key(chain[0]):
                    chain[:0] = reversed(w[1:])
                else:
                    continue
                remaining.pop(ri)
                extended = True
                break
        if key(chain[0]) == key(chain[-1]) and len(chain) >= 4:
            rings.append(chain[:-1])

    if not rings:
        raise SystemExit("could not stitch a closed boundary ring")

    def area(ring):
        pts = [project(p["lat"], p["lon"]) for p in ring]
        s = 0.0
        for i in range(len(pts)):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % len(pts)]
            s += x0 * y1 - x1 * y0
        return abs(s) * 0.5

    best = max(rings, key=area)
    return [project(p["lat"], p["lon"]) for p in best]


def boundary_centroid_latlon(path):
    """Rough lat/lon centroid of the relation's outer-way points (projection origin)."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    rels = [e for e in data["elements"] if e["type"] == "relation"]
    if not rels:
        raise SystemExit("boundary file contains no relation")
    lats, lons = [], []
    for m in rels[0]["members"]:
        if m.get("role") == "outer" and m["type"] == "way" and "geometry" in m:
            for p in m["geometry"]:
                lats.append(p["lat"])
                lons.append(p["lon"])
    return sum(lats) / len(lats), sum(lons) / len(lons)


# --- Road network build -----------------------------------------------------------

def parse_speed(tags, default):
    ms = tags.get("maxspeed", "")
    if ms.endswith("mph"):
        try:
            return float(ms[:-3].strip()) * MPH
        except ValueError:
            return default
    try:
        return float(ms) / 3.6  # bare number = km/h
    except ValueError:
        return default


def parse_lanes(tags, oneway):
    try:
        lanes = int(tags.get("lanes", ""))
    except ValueError:
        return 1, 1
    if oneway != 0:
        return max(1, min(3, lanes)), max(1, min(3, lanes))
    per_dir = max(1, min(3, round(lanes / 2)))
    return per_dir, per_dir


def build_road_segments(roads_path, ring, project):
    """OSM ways -> graph nodes + undirected segment records (directed pair info kept)."""
    with open(roads_path, encoding="utf-8") as f:
        data = json.load(f)

    osm_nodes = {}
    signal_positions = []
    ways = []
    for e in data["elements"]:
        if e["type"] == "node":
            osm_nodes[e["id"]] = project(e["lat"], e["lon"])
            if e.get("tags", {}).get("highway") == "traffic_signals":
                signal_positions.append(project(e["lat"], e["lon"]))
        elif e["type"] == "way":
            ways.append(e)
    ways.sort(key=lambda w: w["id"])

    inside_cache = {}

    def inside(ref, p):
        v = inside_cache.get(ref)
        if v is None:
            v = point_in_ring(p, ring)
            inside_cache[ref] = v
        return v

    synth_counter = [0]
    boundary_refs = set()

    def clip_way(refs):
        """Clip a way's polyline to the boundary ring; yield runs of (ref, point)."""
        pts = [(r, osm_nodes[r]) for r in refs if r in osm_nodes]
        runs, cur = [], []
        for (ra, pa), (rb, pb) in zip(pts, pts[1:]):
            if pa == pb:
                continue
            in_a = inside(ra, pa)
            in_b = inside(rb, pb)
            crossings = []
            if in_a != in_b or (not in_a and not in_b):
                crossings = segment_ring_crossings(pa, pb, ring)
            sub = [(ra, pa, in_a)]
            for t in crossings:
                cp = (pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t)
                synth_counter[0] -= 1
                sub.append((synth_counter[0], cp, None))
            sub.append((rb, pb, in_b))
            for k in range(len(sub) - 1):
                u_ref, u_pt, _ = sub[k]
                v_ref, v_pt, _ = sub[k + 1]
                mid = ((u_pt[0] + v_pt[0]) / 2, (u_pt[1] + v_pt[1]) / 2)
                if point_in_ring(mid, ring):
                    if not cur:
                        cur = [(u_ref, u_pt)]
                        if u_ref < 0:
                            boundary_refs.add(u_ref)
                    cur.append((v_ref, v_pt))
                    if v_ref < 0:
                        boundary_refs.add(v_ref)
                else:
                    if cur:
                        runs.append(cur)
                        cur = []
        if cur:
            runs.append(cur)
        return runs

    # Clip all ways, collect runs with per-way road properties.
    all_runs = []
    for w in ways:
        tags = w.get("tags", {})
        hw = tags.get("highway", "")
        if hw not in HIGHWAY_CLASS or tags.get("area") == "yes":
            continue
        rtype, def_speed = HIGHWAY_CLASS[hw]
        speed = parse_speed(tags, def_speed)
        ow = tags.get("oneway", "")
        if tags.get("junction") in ("roundabout", "circular"):
            oneway = 1
        elif ow in ("yes", "true", "1"):
            oneway = 1
        elif ow in ("-1", "reverse"):
            oneway = -1
        else:
            oneway = 0
        if tags.get("surface", "") in UNPAVED_SURFACES:
            rtype = RT_DIRT
            speed = min(speed, 20 * MPH)
        lanes_f, lanes_b = parse_lanes(tags, oneway)
        for run in clip_way(w["nodes"]):
            if len(run) >= 2:
                all_runs.append((run, rtype, speed, lanes_f, lanes_b, oneway))

    # Junctions: refs used more than once across/within runs, plus run endpoints.
    usage = Counter()
    for run, *_ in all_runs:
        for ref, _ in run:
            usage[ref] += 1

    nodes = []          # [x, y, flags, poi]
    node_of_ref = {}

    def graph_node(ref, pt):
        idx = node_of_ref.get(ref)
        if idx is None:
            idx = len(nodes)
            nodes.append([pt[0], pt[1], 0, 0])
            node_of_ref[ref] = idx
        return idx

    segs = []

    def emit_chain(chain_pts, rtype, speed, lanes_f, lanes_b, oneway):
        """chain_pts: [(ref, (x, y))] between two break nodes. Simplify + smooth."""
        pts = [p for _, p in chain_pts]
        keep = douglas_peucker(pts, tolerance=DP_TOLERANCE)
        # Enforce minimum spacing between kept points (endpoints always kept).
        spaced = [keep[0]]
        for ki in keep[1:-1]:
            if dist(pts[spaced[-1]], pts[ki]) >= MIN_POINT_SPACING:
                spaced.append(ki)
        while len(spaced) > 1 and dist(pts[spaced[-1]], pts[keep[-1]]) < MIN_POINT_SPACING / 2:
            spaced.pop()
        spaced.append(keep[-1])
        pl = [pts[i] for i in spaced]
        if len(pl) < 2 or (len(pl) == 2 and dist(pl[0], pl[1]) < 0.8):
            return
        # Loop chains (same break node at both ends) need >= 3 points to be sane.
        if chain_pts[0][0] == chain_pts[-1][0] and len(pl) < 3:
            return

        # Node indices: shared graph nodes at ends, fresh waypoint nodes inside.
        idxs = [graph_node(chain_pts[0][0], pl[0])]
        for p in pl[1:-1]:
            idxs.append(len(nodes))
            nodes.append([p[0], p[1], 0, 0])
        idxs.append(graph_node(chain_pts[-1][0], pl[-1]))

        # Unit tangents: Catmull-Rom style at interior points, segment dirs at ends.
        tangents = []
        for i in range(len(pl)):
            if i == 0:
                d = (pl[1][0] - pl[0][0], pl[1][1] - pl[0][1])
            elif i == len(pl) - 1:
                d = (pl[-1][0] - pl[-2][0], pl[-1][1] - pl[-2][1])
            else:
                d = (pl[i + 1][0] - pl[i - 1][0], pl[i + 1][1] - pl[i - 1][1])
            n = math.hypot(d[0], d[1])
            if n < 1e-9:
                d, n = (1.0, 0.0), 1.0
            tangents.append((d[0] / n, d[1] / n))

        for i in range(len(pl) - 1):
            a, b = pl[i], pl[i + 1]
            chord = dist(a, b)
            if chord < 0.8 or idxs[i] == idxs[i + 1]:
                continue
            h = chord / 3.0
            cp1 = (a[0] + tangents[i][0] * h, a[1] + tangents[i][1] * h)
            cp2 = (b[0] - tangents[i + 1][0] * h, b[1] - tangents[i + 1][1] * h)
            segs.append({
                "a": idxs[i], "b": idxs[i + 1],
                "cp1": cp1, "cp2": cp2,
                "len": bezier_length(a, cp1, cp2, b),
                "rtype": rtype, "speed": speed,
                "lanes_f": lanes_f, "lanes_b": lanes_b,
                "dir": oneway, "shared": False, "conn": False, "dead": False,
            })

    for run, rtype, speed, lanes_f, lanes_b, oneway in all_runs:
        chain = [run[0]]
        for ref, pt in run[1:]:
            chain.append((ref, pt))
            if usage[ref] >= 2 and len(chain) >= 2:
                emit_chain(chain, rtype, speed, lanes_f, lanes_b, oneway)
                chain = [(ref, pt)]
        if len(chain) >= 2:
            emit_chain(chain, rtype, speed, lanes_f, lanes_b, oneway)

    entry_nodes = {node_of_ref[r] for r in boundary_refs if r in node_of_ref}
    return nodes, segs, entry_nodes, signal_positions


def keep_largest_component(nodes, segs, entry_nodes):
    """Keep only the largest weakly-connected component (by road length)."""
    parent = list(range(len(nodes)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for s in segs:
        ra, rb = find(s["a"]), find(s["b"])
        if ra != rb:
            parent[ra] = rb

    comp_len = defaultdict(float)
    for s in segs:
        comp_len[find(s["a"])] += s["len"]
    if not comp_len:
        raise SystemExit("no road segments inside the boundary")
    best = max(comp_len, key=lambda c: comp_len[c])

    keep_seg = [s for s in segs if find(s["a"]) == best]
    used = set()
    for s in keep_seg:
        used.add(s["a"])
        used.add(s["b"])
    remap = {}
    new_nodes = []
    for old in sorted(used):
        remap[old] = len(new_nodes)
        new_nodes.append(nodes[old])
    for s in keep_seg:
        s["a"] = remap[s["a"]]
        s["b"] = remap[s["b"]]
    new_entries = {remap[n] for n in entry_nodes if n in remap}
    dropped = len(segs) - len(keep_seg)
    return new_nodes, keep_seg, new_entries, dropped


def rebuild_seg_geometry(nodes, s):
    """Re-derive a segment's handles (direction kept, length chord/3) after an
    endpoint moved, and recompute its arc length. Kills the segment when the
    endpoints collapse onto each other."""
    pa = (nodes[s["a"]][0], nodes[s["a"]][1])
    pb = (nodes[s["b"]][0], nodes[s["b"]][1])
    chord = dist(pa, pb)
    if chord < 0.5:
        s["dead"] = True
        return
    for key, anchor, other in (("cp1", pa, pb), ("cp2", pb, pa)):
        hx, hy = s[key][0] - anchor[0], s[key][1] - anchor[1]
        hl = math.hypot(hx, hy)
        if hl < 0.05 * chord:
            dx, dy = (other[0] - anchor[0]) / chord, (other[1] - anchor[1]) / chord
        else:
            dx, dy = hx / hl, hy / hl
        s[key] = (anchor[0] + dx * chord / 3.0, anchor[1] + dy * chord / 3.0)
    s["len"] = bezier_length(pa, s["cp1"], s["cp2"], pb)


def collapse_short_edges(nodes, segs, pinned, min_len=MIN_EDGE_LEN):
    """Merge the end nodes of every road segment shorter than min_len (boundary-clip
    slivers, dense OSM geometry). Pinned nodes (entry/exit) keep their position and
    always survive a merge; two pinned ends are never merged. Neighbor segments have
    their handles rebuilt around the moved node. Duplicate parallel segments left by
    a merge are dropped."""
    merged = 0
    changed = True
    while changed:
        changed = False
        for s in segs:
            if s["dead"] or s["conn"] or s["len"] >= min_len:
                continue
            a, b = s["a"], s["b"]
            if a == b:
                s["dead"] = True
                changed = True
                continue
            if a in pinned and b in pinned:
                continue
            keep, gone = (a, b) if (a in pinned or (b not in pinned and a < b)) else (b, a)
            if keep not in pinned:
                nodes[keep][0] = (nodes[a][0] + nodes[b][0]) / 2.0
                nodes[keep][1] = (nodes[a][1] + nodes[b][1]) / 2.0
            s["dead"] = True
            merged += 1
            changed = True
            for t in segs:
                if t["dead"]:
                    continue
                moved = False
                if t["a"] == gone:
                    t["a"] = keep
                    moved = True
                if t["b"] == gone:
                    t["b"] = keep
                    moved = True
                if t["a"] == t["b"]:
                    t["dead"] = True
                    continue
                if moved or t["a"] == keep or t["b"] == keep:
                    rebuild_seg_geometry(nodes, t)
        # Drop duplicate parallel segments between the same node pair.
        seen = {}
        for t in segs:
            if t["dead"] or t["conn"]:
                continue
            key = (min(t["a"], t["b"]), max(t["a"], t["b"]))
            if key in seen:
                t["dead"] = True
                changed = True
            else:
                seen[key] = True
    return merged


# --- POI placement ---------------------------------------------------------------

def load_pois(path, ring, project, home_cap):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    pois = []
    for e in data["elements"]:
        tags = e.get("tags")
        if not tags:
            continue
        kind = classify_poi(tags)
        if kind is None:
            continue
        if e["type"] == "node":
            lat, lon = e["lat"], e["lon"]
        elif e["type"] == "way" and "center" in e:
            lat, lon = e["center"]["lat"], e["center"]["lon"]
        else:
            continue
        p = project(lat, lon)
        if not point_in_ring(p, ring):
            continue
        pois.append({"pos": p, "kind": kind, "id": e["id"], "name": tags.get("name", "")})

    # Dedupe: overlapping features keep the highest-priority (then lowest-id) POI.
    pois.sort(key=lambda q: (POI_PRIORITY[q["kind"]], q["id"]))
    cell = 10.0
    grid = defaultdict(list)
    kept = []
    for q in pois:
        cx, cy = int(q["pos"][0] // cell), int(q["pos"][1] // cell)
        clash = False
        for gx in range(cx - 2, cx + 3):
            for gy in range(cy - 2, cy + 3):
                for other in grid[(gx, gy)]:
                    limit = 8.0 if other["kind"] == q["kind"] else 15.0
                    if dist(other["pos"], q["pos"]) < limit:
                        clash = True
                        break
                if clash:
                    break
            if clash:
                break
        if not clash:
            kept.append(q)
            grid[(cx, cy)].append(q)

    homes = [q for q in kept if q["kind"] == POI_HOME]
    others = [q for q in kept if q["kind"] != POI_HOME]

    # Spatially stratified downsample of homes (round-robin over 200 m grid cells).
    if len(homes) > home_cap:
        cells = defaultdict(list)
        for q in homes:
            cells[(int(q["pos"][0] // 200), int(q["pos"][1] // 200))].append(q)
        for c in cells.values():
            c.sort(key=lambda q: q["id"])
        order = sorted(cells.keys())
        sampled = []
        rnd = 0
        while len(sampled) < home_cap:
            took = False
            for c in order:
                lst = cells[c]
                if rnd < len(lst):
                    sampled.append(lst[rnd])
                    took = True
                    if len(sampled) >= home_cap:
                        break
            if not took:
                break
            rnd += 1
        homes = sampled

    return others + homes, len(kept)


class SegIndex:
    """Spatial hash of segment curve samples for nearest-segment queries."""

    def __init__(self, segs, cell=40.0):
        self.segs = segs
        self.cell = cell
        self.grid = defaultdict(set)
        self.samples = {}
        for i, s in enumerate(segs):
            if s["dead"] or s["conn"] or s["rtype"] == RT_HIGHWAY:
                continue
            self.add_seg(i)

    def add_seg(self, i):
        s = self.segs[i]
        n = max(6, int(s["len"] / 6))
        p0, p3 = s["_p0"], s["_p3"]
        pts = [bezier_point(p0, s["cp1"], s["cp2"], p3, k / n) for k in range(n + 1)]
        self.samples[i] = pts
        for p in pts:
            self.grid[(int(p[0] // self.cell), int(p[1] // self.cell))].add(i)

    def nearest(self, p, max_dist=MAX_DRIVEWAY):
        cx, cy = int(p[0] // self.cell), int(p[1] // self.cell)
        best = (None, max_dist, 0.0)  # (seg idx, dist, arclen)
        for radius in (1, 2, 4):
            cand = set()
            for gx in range(cx - radius, cx + radius + 1):
                for gy in range(cy - radius, cy + radius + 1):
                    cand |= self.grid.get((gx, gy), set())
            for i in cand:
                if self.segs[i]["dead"]:
                    continue
                d, arc = self.project(i, p)
                if d < best[1]:
                    best = (i, d, arc)
            if best[0] is not None and best[1] <= radius * self.cell:
                break
        return best

    def project(self, i, p):
        pts = self.samples[i]
        best_d, best_k = float("inf"), 0
        for k, sp in enumerate(pts):
            d = dist(sp, p)
            if d < best_d:
                best_d, best_k = d, k
        # Refine on the two polyline segments adjacent to the best sample.
        arcs = [0.0]
        for k in range(1, len(pts)):
            arcs.append(arcs[-1] + dist(pts[k - 1], pts[k]))
        best = (best_d, arcs[best_k])
        for k in (best_k - 1, best_k):
            if k < 0 or k + 1 >= len(pts):
                continue
            ax, ay = pts[k]
            bx, by = pts[k + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
            proj = (ax + t * dx, ay + t * dy)
            d = dist(proj, p)
            if d < best[0]:
                best = (d, arcs[k] + t * math.sqrt(l2))
        return best


def connect_pois(nodes, segs, pois, entry_nodes):
    """Split roads and attach each POI with a dirt single-lane two-way driveway
    (the same connector the in-app Destination tool builds for homes — it ranks
    below every road class, so the through road always flows free past the foot)."""
    for s in segs:
        s["_p0"] = (nodes[s["a"]][0], nodes[s["a"]][1])
        s["_p3"] = (nodes[s["b"]][0], nodes[s["b"]][1])

    index = SegIndex(segs)
    cuts = defaultdict(list)  # seg idx -> [(arclen, poi)]
    skipped = 0
    for q in pois:
        i, d, arc = index.nearest(q["pos"])
        if i is None:
            skipped += 1
            continue
        cuts[i].append((arc, q))

    placed = Counter()
    for i, lst in sorted(cuts.items()):
        s = segs[i]
        L = s["len"]
        lst.sort(key=lambda e: (e[0], e[1]["id"]))

        # Cluster cut positions into shared foot nodes.
        groups = []
        for arc, q in lst:
            if groups and arc - groups[-1][0][-1] <= CUT_CLUSTER_GAP:
                groups[-1][0].append(arc)
                groups[-1][1].append(q)
            else:
                groups.append(([arc], [q]))

        entries = []  # (clamped arclen or None-for-endpoint, node idx or None, pois)
        for arcs, qs in groups:
            arc = sum(arcs) / len(arcs)
            if L < ENDPOINT_ATTACH_LEN:
                # Too short to split with setbacks — attach at the nearer endpoint,
                # unless it is flagged (existing POI) or an entry/exit boundary node.
                end = s["a"] if arc < L / 2 else s["b"]
                if nodes[end][2] != 0 or end in entry_nodes:
                    end = s["b"] if end == s["a"] else s["a"]  # try the other end
                if nodes[end][2] != 0 or end in entry_nodes:
                    skipped += len(qs)
                    continue
                entries.append((None, end, qs))
            else:
                entries.append((max(CUT_END_SETBACK, min(L - CUT_END_SETBACK, arc)), None, qs))

        # Merge clamped positions that collided after clamping.
        cut_list = []
        for arc, end, qs in entries:
            if arc is not None and cut_list and cut_list[-1][1] is None \
                    and arc - cut_list[-1][0] < CUT_CLUSTER_GAP - 2.0:
                cut_list[-1][2].extend(qs)
            else:
                cut_list.append([arc if arc is not None else -1.0, end, list(qs)])

        # Split the segment once at all cut arclens (single pass, ascending).
        split_arcs = [c[0] for c in cut_list if c[1] is None]
        foot_nodes = {}
        if split_arcs:
            p0, c1, c2, p3 = s["_p0"], s["cp1"], s["cp2"], s["_p3"]
            n_samp = 64
            arc_table = [0.0]
            prev = p0
            for k in range(1, n_samp + 1):
                cur = bezier_point(p0, c1, c2, p3, k / n_samp)
                arc_table.append(arc_table[-1] + dist(prev, cur))
                prev = cur
            total = arc_table[-1]

            def param_at(target):
                tgt = target / L * total
                lo, hi = 0, n_samp
                while lo < hi:
                    mid = (lo + hi) // 2
                    if arc_table[mid] < tgt:
                        lo = mid + 1
                    else:
                        hi = mid
                k = max(1, lo)
                span = arc_table[k] - arc_table[k - 1]
                frac = 0.0 if span < 1e-9 else (tgt - arc_table[k - 1]) / span
                return (k - 1 + frac) / n_samp

            ts = [param_at(a) for a in split_arcs]
            chain_nodes = [s["a"]]
            for a, t in zip(split_arcs, ts):
                p = bezier_point(p0, c1, c2, p3, t)
                idx = len(nodes)
                nodes.append([p[0], p[1], 0, 0])
                chain_nodes.append(idx)
                foot_nodes[a] = (idx, t)
            chain_nodes.append(s["b"])

            t_list = [0.0] + ts + [1.0]
            s["dead"] = True
            for k in range(len(chain_nodes) - 1):
                t0, t1 = t_list[k], t_list[k + 1]
                a_pt = bezier_point(p0, c1, c2, p3, t0)
                b_pt = bezier_point(p0, c1, c2, p3, t1)
                chord = dist(a_pt, b_pt)
                if chord < 0.5:
                    continue
                ta = bezier_tangent(p0, c1, c2, p3, t0)
                tb = bezier_tangent(p0, c1, c2, p3, t1)
                h = chord / 3.0
                sub_cp1 = (a_pt[0] + ta[0] * h, a_pt[1] + ta[1] * h)
                sub_cp2 = (b_pt[0] - tb[0] * h, b_pt[1] - tb[1] * h)
                segs.append({
                    "a": chain_nodes[k], "b": chain_nodes[k + 1],
                    "cp1": sub_cp1, "cp2": sub_cp2,
                    "len": bezier_length(a_pt, sub_cp1, sub_cp2, b_pt),
                    "rtype": s["rtype"], "speed": s["speed"],
                    "lanes_f": s["lanes_f"], "lanes_b": s["lanes_b"],
                    "dir": s["dir"], "shared": s["shared"], "conn": False, "dead": False,
                    "_p0": a_pt, "_p3": b_pt,
                })

        # Attach driveways.
        for arc, end, qs in cut_list:
            if end is not None:
                foot_idx = end
                foot_pt = (nodes[end][0], nodes[end][1])
                tangent = None
            else:
                foot_idx, t = foot_nodes[arc]
                foot_pt = (nodes[foot_idx][0], nodes[foot_idx][1])
                tangent = bezier_tangent(s["_p0"], s["cp1"], s["cp2"], s["_p3"], t)
            for q in qs:
                pos = q["pos"]
                if dist(pos, foot_pt) < 8.0:
                    if tangent is None:
                        tangent = (1.0, 0.0)
                    normal = (-tangent[1], tangent[0])
                    side = (pos[0] - foot_pt[0]) * normal[0] + (pos[1] - foot_pt[1]) * normal[1]
                    sign = 1.0 if side >= 0 else -1.0
                    pos = (foot_pt[0] + normal[0] * sign * 10.0, foot_pt[1] + normal[1] * sign * 10.0)
                poi_idx = len(nodes)
                nodes.append([pos[0], pos[1], FLAG_DESTINATION, q["kind"]])
                chord = dist(foot_pt, pos)
                third = ((pos[0] - foot_pt[0]) / 3.0, (pos[1] - foot_pt[1]) / 3.0)
                segs.append({
                    "a": foot_idx, "b": poi_idx,
                    "cp1": (foot_pt[0] + third[0], foot_pt[1] + third[1]),
                    "cp2": (foot_pt[0] + 2 * third[0], foot_pt[1] + 2 * third[1]),
                    "len": chord,
                    "rtype": RT_DIRT,
                    "speed": 15 * MPH,
                    "lanes_f": 1, "lanes_b": 1, "dir": 0,
                    "shared": True, "conn": True, "dead": False,
                    "_p0": foot_pt, "_p3": pos,
                })
                placed[q["kind"]] += 1

    return placed, skipped


# --- Water layer -------------------------------------------------------------------

# waterway value -> default painted width (m) when no width/est_width tag parses.
WATERWAY_WIDTHS = {
    "river": 25.0,
    "canal": 8.0,
    "stream": 3.5,
    "brook": 3.5,
    "drain": 2.5,
}
WATER_CIRCLE_MAX_R = 40.0   # largest fill circle
WATER_CIRCLE_MIN_R = 2.2    # slivers thinner than this are left to the shore band
WATER_ROOT_CELL = 48.0      # quadtree root cell (0.71*48 = 34 m needed radius < max)
WATER_SEG_SPACING = 12.0    # min waypoint spacing along stream chains (m)


def parse_width_tag(tags):
    for key in ("width", "est_width"):
        v = tags.get(key, "").replace("m", "").strip()
        if not v:
            continue
        try:
            w = float(v)
            if w > 0:
                return w
        except ValueError:
            pass
    return None


def point_seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


class WaterPolygon:
    """A water body: outer ring + island holes, with a segment hash for fast
    distance-to-shoreline queries (drives the fill-circle radii)."""

    CELL = 32.0

    def __init__(self, outer, inners):
        self.outer = outer
        self.inners = [i for i in inners if len(i) >= 3]
        xs = [p[0] for p in outer]
        ys = [p[1] for p in outer]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        self._segs = defaultdict(list)
        for ring in [outer] + self.inners:
            n = len(ring)
            for i in range(n):
                self._insert(ring[i], ring[(i + 1) % n])

    def _insert(self, a, b):
        cs = self.CELL
        x0, x1 = sorted((a[0], b[0]))
        y0, y1 = sorted((a[1], b[1]))
        for cx in range(int(x0 // cs), int(x1 // cs) + 1):
            for cy in range(int(y0 // cs), int(y1 // cs) + 1):
                self._segs[(cx, cy)].append((a, b))

    def boundary_dist(self, p, cap):
        """Distance from p to the nearest shoreline segment, capped at `cap`."""
        cs = self.CELL
        cx, cy = int(p[0] // cs), int(p[1] // cs)
        best = cap
        max_ring = int(cap // cs) + 2
        for radius in range(max_ring + 1):
            for gx in range(cx - radius, cx + radius + 1):
                for gy in range(cy - radius, cy + radius + 1):
                    if max(abs(gx - cx), abs(gy - cy)) != radius:
                        continue
                    for a, b in self._segs.get((gx, gy), ()):
                        d = point_seg_dist(p, a, b)
                        if d < best:
                            best = d
            # No segment in a farther ring can beat what we already have.
            if best <= max(0, radius - 1) * cs:
                break
        return best

    def contains(self, p):
        if not (self.bbox[0] <= p[0] <= self.bbox[2] and self.bbox[1] <= p[1] <= self.bbox[3]):
            return False
        if not point_in_ring(p, self.outer):
            return False
        return all(not point_in_ring(p, inner) for inner in self.inners)


def stitch_ways_to_rings(point_lists):
    """Stitches open way point-lists into closed rings (projected meters; endpoint
    match tolerance 0.01 m). Returns the list of closed rings (unclosed leftovers
    are dropped)."""

    def key(p):
        return (round(p[0], 2), round(p[1], 2))

    rings = []
    remaining = [list(pl) for pl in point_lists if len(pl) >= 2]
    while remaining:
        chain = remaining.pop(0)
        extended = True
        while extended and key(chain[0]) != key(chain[-1]):
            extended = False
            for i, w in enumerate(remaining):
                if key(w[0]) == key(chain[-1]):
                    chain.extend(w[1:])
                elif key(w[-1]) == key(chain[-1]):
                    chain.extend(reversed(w[:-1]))
                elif key(w[-1]) == key(chain[0]):
                    chain[:0] = w[:-1]
                elif key(w[0]) == key(chain[0]):
                    chain[:0] = list(reversed(w[1:]))
                else:
                    continue
                remaining.pop(i)
                extended = True
                break
        if key(chain[0]) == key(chain[-1]) and len(chain) >= 4:
            rings.append(chain[:-1])
    return rings


def fill_polygon_circles(poly, town_ring, circles):
    """Covers the polygon's water area with circles: a quadtree descent that emits a
    big circle wherever a whole cell is deep inside the water (radius >= the cell's
    half-diagonal, so coverage has no gaps) and refines toward the shoreline."""

    def cover(cx, cy, s):
        half_diag = s * 0.7072
        r_need = max(half_diag, WATER_CIRCLE_MIN_R)
        d = poly.boundary_dist((cx, cy), cap=r_need + s)
        if d >= r_need:
            # Deep on one side of the shoreline: fully water or fully land.
            if poly.contains((cx, cy)) and point_in_ring((cx, cy), town_ring):
                circles.append((cx, cy, min(max(d * 0.98, r_need), WATER_CIRCLE_MAX_R)))
            return
        if s <= 8.0:
            if d >= WATER_CIRCLE_MIN_R and poly.contains((cx, cy)) \
                    and point_in_ring((cx, cy), town_ring):
                circles.append((cx, cy, min(d, s)))
            return
        q = s / 4.0
        h = s / 2.0
        cover(cx - q, cy - q, h)
        cover(cx + q, cy - q, h)
        cover(cx - q, cy + q, h)
        cover(cx + q, cy + q, h)

    minx, miny, maxx, maxy = poly.bbox
    root = WATER_ROOT_CELL
    gx0, gx1 = int(minx // root), int(maxx // root)
    gy0, gy1 = int(miny // root), int(maxy // root)
    for gx in range(gx0, gx1 + 1):
        for gy in range(gy0, gy1 + 1):
            cover((gx + 0.5) * root, (gy + 0.5) * root, root)


def clip_polyline(pts, ring):
    """Clips a raw polyline to the inside of a ring; returns runs of points with
    exact interpolated boundary endpoints (the ref-less analog of the road clipper)."""
    runs, cur = [], []
    for a, b in zip(pts, pts[1:]):
        if a == b:
            continue
        crossings = segment_ring_crossings(a, b, ring)
        sub = [a]
        for t in crossings:
            sub.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        sub.append(b)
        for u, v in zip(sub, sub[1:]):
            mid = ((u[0] + v[0]) / 2, (u[1] + v[1]) / 2)
            if point_in_ring(mid, ring):
                if not cur:
                    cur = [u]
                cur.append(v)
            else:
                if cur:
                    runs.append(cur)
                    cur = []
    if cur:
        runs.append(cur)
    return runs


def resample_polyline(pts, max_step):
    """Inserts interpolated points so no sub-segment exceeds max_step meters."""
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        d = dist(a, b)
        n = max(1, int(math.ceil(d / max_step)))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out


def split_out_covered(pts, polys):
    """Splits a run into pieces whose sub-segments are NOT inside any water polygon
    (a river centerline over its own riverbank polygon would double-paint)."""
    pieces, cur = [], []
    for a, b in zip(pts, pts[1:]):
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        covered = any(p.contains(mid) for p in polys)
        if not covered:
            if not cur:
                cur = [a]
            cur.append(b)
        else:
            if cur:
                pieces.append(cur)
                cur = []
    if cur:
        pieces.append(cur)
    return pieces


def polyline_to_water_segments(pl, width, segments):
    """DP-simplify + smooth a stream run and emit cubic water segments with the
    same tangent/chord-thirds conventions as the road chains."""
    keep = douglas_peucker(pl, tolerance=1.5)
    spaced = [keep[0]]
    for ki in keep[1:-1]:
        if dist(pl[spaced[-1]], pl[ki]) >= WATER_SEG_SPACING:
            spaced.append(ki)
    while len(spaced) > 1 and dist(pl[spaced[-1]], pl[keep[-1]]) < WATER_SEG_SPACING / 2:
        spaced.pop()
    spaced.append(keep[-1])
    sm = [pl[i] for i in spaced]
    if len(sm) < 2:
        return

    tangents = []
    for i in range(len(sm)):
        if i == 0:
            d = (sm[1][0] - sm[0][0], sm[1][1] - sm[0][1])
        elif i == len(sm) - 1:
            d = (sm[-1][0] - sm[-2][0], sm[-1][1] - sm[-2][1])
        else:
            d = (sm[i + 1][0] - sm[i - 1][0], sm[i + 1][1] - sm[i - 1][1])
        n = math.hypot(d[0], d[1])
        if n < 1e-9:
            d, n = (1.0, 0.0), 1.0
        tangents.append((d[0] / n, d[1] / n))

    for i in range(len(sm) - 1):
        a, b = sm[i], sm[i + 1]
        chord = dist(a, b)
        if chord < 1.0:
            continue
        h = chord / 3.0
        segments.append({
            "p0": a,
            "c1": (a[0] + tangents[i][0] * h, a[1] + tangents[i][1] * h),
            "c2": (b[0] - tangents[i + 1][0] * h, b[1] - tangents[i + 1][1] * h),
            "p3": b,
            "w": width,
        })


def load_water(path, ring, project):
    """Parses the water Overpass download into fill circles + stream segments,
    clipped to the town ring."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)

    poly_ways = []      # (points, id) closed natural=water ways
    lines = []          # (points, width, id) linear waterways
    relations = []
    for e in data["elements"]:
        tags = e.get("tags", {})
        if e["type"] == "way" and "geometry" in e:
            pts = [project(g["lat"], g["lon"]) for g in e["geometry"]]
            if tags.get("natural") == "water":
                if len(pts) >= 4:
                    poly_ways.append((pts, e["id"]))
            elif tags.get("waterway") in WATERWAY_WIDTHS:
                width = parse_width_tag(tags) or WATERWAY_WIDTHS[tags["waterway"]]
                lines.append((pts, width, e["id"]))
        elif e["type"] == "relation" and tags.get("natural") == "water":
            relations.append(e)

    polys = []
    for pts, _ in sorted(poly_ways, key=lambda q: q[1]):
        outer = pts[:-1] if pts[0] == pts[-1] else pts
        if len(outer) >= 3:
            polys.append(WaterPolygon(outer, []))

    for rel in sorted(relations, key=lambda r: r["id"]):
        outers, inners = [], []
        for role, bucket in (("outer", outers), ("inner", inners)):
            member_pts = [
                [project(g["lat"], g["lon"]) for g in m["geometry"]]
                for m in rel.get("members", [])
                if m.get("role") == role and m.get("type") == "way" and "geometry" in m
            ]
            bucket.extend(stitch_ways_to_rings(member_pts))
        for outer in outers:
            mine = [i for i in inners if point_in_ring(i[0], outer)]
            polys.append(WaterPolygon(outer, mine))

    circles = []
    for poly in polys:
        fill_polygon_circles(poly, ring, circles)

    segments = []
    for pts, width, _ in sorted(lines, key=lambda q: q[2]):
        for run in clip_polyline(pts, ring):
            run = resample_polyline(run, 25.0)
            for piece in split_out_covered(run, polys):
                if sum(dist(a, b) for a, b in zip(piece, piece[1:])) >= 8.0:
                    polyline_to_water_segments(piece, width, segments)

    return circles, segments, len(polys), len(lines)


# --- Signals, entry/exit, output ---------------------------------------------------

def emit_directed_edges(segs):
    edges = []
    for s in segs:
        if s["dead"]:
            continue
        flags = EDGEFLAG_SHARED_LANE if s["shared"] else 0
        if s["dir"] >= 0:
            edges.append((s["a"], s["b"], s["len"], s["speed"], s["lanes_f"],
                          s["rtype"], flags, s["cp1"], s["cp2"]))
        if s["dir"] <= 0:
            edges.append((s["b"], s["a"], s["len"], s["speed"], s["lanes_b"],
                          s["rtype"], flags, s["cp2"], s["cp1"]))
    return edges


def apply_entry_exit(nodes, edges, entry_nodes):
    out_deg = Counter()
    for e in edges:
        out_deg[e[0]] += 1
    count = 0
    for n in entry_nodes:
        if nodes[n][2] == 0 and out_deg[n] <= 2:
            nodes[n][2] = FLAG_DESTINATION
            nodes[n][3] = POI_ENTRYEXIT
            count += 1
    return count


def apply_signals(nodes, edges, signal_positions):
    in_deg = Counter()
    for e in edges:
        in_deg[e[1]] += 1
    junctions = [n for n in in_deg if in_deg[n] >= 3 and nodes[n][2] == 0]
    count = 0
    lit = set()
    for sp in signal_positions:
        best, best_d = None, 40.0
        for n in junctions:
            d = dist((nodes[n][0], nodes[n][1]), sp)
            if d < best_d:
                best, best_d = n, d
        if best is not None and best not in lit:
            nodes[best][2] |= FLAG_TRAFFIC_LIGHT | FLAG_MANUAL_SIGNAL | FLAG_ACTUATED
            lit.add(best)
            count += 1
    return count


def write_roads(path, nodes, edges, camera, time_of_day, water_circles, water_segments):
    with open(path, "wb") as f:
        f.write(b"ROAD")
        f.write(struct.pack("<HBf", 3, 0, time_of_day))
        f.write(struct.pack("<i", len(nodes)))
        for x, y, flags, poi in nodes:
            f.write(struct.pack("<ffBB", x, y, flags, poi))
        f.write(struct.pack("<i", len(edges)))
        for a, b, length, speed, lanes, rtype, eflags, cp1, cp2 in edges:
            f.write(struct.pack("<iiffBBBffff", a, b, length, speed, lanes,
                                rtype, eflags, cp1[0], cp1[1], cp2[0], cp2[1]))
        f.write(struct.pack("<i", 0))            # lane restrictions
        f.write(struct.pack("<i", 0))            # stop-sign exemptions
        f.write(struct.pack("<i", 0))            # yield exemptions
        f.write(struct.pack("<i", 0))            # phase rotations
        f.write(struct.pack("<fff", *camera))    # center x, center y, zoom
        # Section 8 — Water (format v3; no vehicles were written, so it follows Camera).
        f.write(struct.pack("<i", len(water_circles)))
        for x, y, r in water_circles:
            f.write(struct.pack("<fff", x, y, r))
        f.write(struct.pack("<i", len(water_segments)))
        for s in water_segments:
            f.write(struct.pack("<9f", s["p0"][0], s["p0"][1], s["c1"][0], s["c1"][1],
                                s["c2"][0], s["c2"][1], s["p3"][0], s["p3"][1], s["w"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--roads", required=True)
    ap.add_argument("--pois", required=True)
    ap.add_argument("--boundary", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--water", default=None,
                    help="Overpass water JSON (natural=water + waterway lines, out geom); "
                         "omit for a map with an empty water layer")
    ap.add_argument("--home-cap", type=int, default=1500)
    ap.add_argument("--center", default=None, help="initial camera LAT,LON (default: town centroid)")
    ap.add_argument("--zoom", type=float, default=0.35)
    ap.add_argument("--time", type=float, default=7.25, help="time of day in hours")
    args = ap.parse_args()

    lat0, lon0 = boundary_centroid_latlon(args.boundary)
    m_lat, m_lon = meters_per_degree(lat0)

    def project(lat, lon):
        return ((lon - lon0) * m_lon, (lat0 - lat) * m_lat)  # north = up (-y)

    ring = load_boundary(args.boundary, project)
    print(f"boundary: {len(ring)} vertices, origin {lat0:.5f},{lon0:.5f}")

    nodes, segs, entry_nodes, signal_positions = build_road_segments(args.roads, ring, project)
    print(f"road graph: {len(nodes)} nodes, {len(segs)} segments (pre-component-filter)")

    nodes, segs, entry_nodes, dropped = keep_largest_component(nodes, segs, entry_nodes)
    print(f"largest component: {len(nodes)} nodes, {len(segs)} segments ({dropped} segments dropped)")

    merged = collapse_short_edges(nodes, segs, entry_nodes)
    print(f"short-edge collapse: {merged} segments merged away")

    pois, total_classified = load_pois(args.pois, ring, project, args.home_cap)
    print(f"POIs: {total_classified} classified in town, {len(pois)} after home cap {args.home_cap}")

    placed, skipped = connect_pois(nodes, segs, pois, entry_nodes)
    placed_str = ", ".join(f"{POI_NAMES[k]}={v}" for k, v in sorted(placed.items()))
    print(f"POIs placed: {placed_str}; skipped {skipped}")

    edges = emit_directed_edges(segs)
    n_entry = apply_entry_exit(nodes, edges, entry_nodes)
    n_signals = apply_signals(nodes, edges, signal_positions)
    print(f"entry/exit nodes: {n_entry}, traffic signals: {n_signals}")

    water_circles, water_segments = [], []
    if args.water:
        water_circles, water_segments, n_polys, n_lines = load_water(args.water, ring, project)
        print(f"water: {n_polys} polygons -> {len(water_circles)} fill circles; "
              f"{n_lines} waterways -> {len(water_segments)} stream segments")

    # Sanity checks (mirror the app's structural limits).
    assert len(edges) <= 65535, f"too many directed edges ({len(edges)}) for ushort adjacency"
    out_deg = Counter(e[0] for e in edges)
    assert not out_deg or out_deg.most_common(1)[0][1] <= 255, "node out-degree exceeds byte"
    short_roads = 0
    for e in edges:
        assert e[0] != e[1], "self-loop edge"
        assert e[2] > 0.4, f"degenerate edge length {e[2]}"
        assert 0 <= e[0] < len(nodes) and 0 <= e[1] < len(nodes)
    for s in segs:
        if not s["dead"] and not s["conn"] and s["len"] < MIN_EDGE_LEN - 0.5:
            short_roads += 1
    if short_roads:
        print(f"warning: {short_roads} road segments below {MIN_EDGE_LEN}m survived collapse")

    if args.center:
        clat, clon = (float(v) for v in args.center.split(","))
        cx, cy = project(clat, clon)
    else:
        cx, cy = 0.0, 0.0
    camera = (-args.zoom * cx, -args.zoom * cy, args.zoom)

    write_roads(args.out, nodes, edges, camera, args.time, water_circles, water_segments)

    road_km = sum(s["len"] for s in segs if not s["dead"] and not s["conn"]) / 1000
    lens = sorted(s["len"] for s in segs if not s["dead"] and not s["conn"])
    print(f"written: {args.out} (format v3)")
    print(f"  {len(nodes)} nodes, {len(edges)} directed edges, {road_km:.1f} km of road")
    print(f"  shortest road segment: {lens[0]:.1f}m, median {lens[len(lens) // 2]:.1f}m")
    print(f"  water: {len(water_circles)} circles, {len(water_segments)} stream segments")
    print(f"  time of day {args.time:.2f}h, camera zoom {args.zoom}")


if __name__ == "__main__":
    main()
