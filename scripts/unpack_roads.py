"""Unpack a .roads binary file into human-readable text.

Usage:  python scripts/unpack_roads.py <file.roads> [output.txt]
If output.txt is omitted, prints to stdout.
"""

import struct
import sys
import math
from pathlib import Path

# --- Enum look-ups (must match C# definitions) ---

NODE_FLAGS = {
    0: "None",
    1: "TrafficLight",
    2: "StopSign",
    4: "Yield",
    8: "Spawn(legacy)",  # retired; the app masks this bit out on load
    16: "ManualSignal",
    32: "Destination",
    128: "ActuatedSignal",  # traffic light runs actuated (demand-responsive) control
}

POI_TYPES = ["None", "Home", "Work", "Shop", "Leisure", "School", "Parking", "EntryExit"]

ROAD_TYPES = ["Residential", "Arterial", "Highway", "Dirt"]

DRIVER_ARCHETYPES = ["Commuter", "SundayDriver", "LeadFoot", "NervousNellie", "Trucker"]

RESIDENT_ACTIVITY = ["Dormant", "Driving", "OffMap", "MovingIn"]

EDGE_FLAGS = {
    0: "None",
    1: "SharedLane",  # single-lane two-way
    2: "Bridge",      # elevated; passes over crossings, connects only at end nodes
}


def decode_flags(value: int, flag_map: dict) -> str:
    if value == 0:
        return flag_map.get(0, "None")
    parts = []
    for bit, name in flag_map.items():
        if bit == 0:
            continue
        if value & bit:
            parts.append(name)
    return " | ".join(parts) if parts else f"0x{value:02X}"


def enum_name(value: int, names: list) -> str:
    if 0 <= value < len(names):
        return names[value]
    return f"Unknown({value})"


def read_fmt(f, fmt):
    size = struct.calcsize(fmt)
    data = f.read(size)
    if len(data) < size:
        raise EOFError(f"Expected {size} bytes, got {len(data)}")
    return struct.unpack(fmt, data)


def speed_to_mph(mps: float) -> float:
    return mps * 2.23694


def unpack(filepath: str) -> str:
    lines = []

    def w(s=""):
        lines.append(s)

    with open(filepath, "rb") as f:
        # --- Header ---
        magic = f.read(4)
        if magic != b"ROAD":
            raise ValueError(f"Bad magic: {magic!r} (expected b'ROAD')")

        (version,) = read_fmt(f, "<H")
        (flags_byte,) = read_fmt(f, "<B")
        has_vehicles = bool(flags_byte & 1)
        (time_of_day,) = read_fmt(f, "<f")

        hours = int(time_of_day)
        minutes = int((time_of_day - hours) * 60)

        w(f"=== ROADS FILE: {Path(filepath).name} ===")
        w(f"Format version : {version}")
        w(f"Flags          : 0x{flags_byte:02X} (vehicles={'yes' if has_vehicles else 'no'})")
        w(f"Time of day    : {time_of_day:.2f} ({hours:02d}:{minutes:02d})")
        w()

        # --- Section 1: Nodes ---
        (node_count,) = read_fmt(f, "<i")
        w(f"=== NODES ({node_count}) ===")

        nodes = []
        for i in range(node_count):
            x, y = read_fmt(f, "<ff")
            (nf,) = read_fmt(f, "<B")
            (poi,) = read_fmt(f, "<B")
            nodes.append((x, y, nf, poi))
            flag_str = decode_flags(nf, NODE_FLAGS)
            poi_str = enum_name(poi, POI_TYPES)
            poi_part = f"  poi={poi_str}" if poi != 0 else ""
            w(f"  [{i:3d}] ({x:10.2f}, {y:10.2f})  flags={flag_str}{poi_part}")
        w()

        # --- Section 2: Edges ---
        (edge_count,) = read_fmt(f, "<i")
        w(f"=== EDGES ({edge_count}) ===")

        edges = []
        for i in range(edge_count):
            from_n, to_n = read_fmt(f, "<ii")
            length, speed = read_fmt(f, "<ff")
            (lanes,) = read_fmt(f, "<B")
            (rtype,) = read_fmt(f, "<B")
            # v4 widened edge Flags to ushort (EdgeFlags is a ushort enum); pre-v4 = 1 byte.
            (eflags,) = read_fmt(f, "<H" if version >= 4 else "<B")
            cp1x, cp1y, cp2x, cp2y = read_fmt(f, "<ffff")
            edges.append((from_n, to_n, length, speed, lanes, rtype, eflags,
                          cp1x, cp1y, cp2x, cp2y))

            rtype_str = enum_name(rtype, ROAD_TYPES)
            eflag_str = decode_flags(eflags, EDGE_FLAGS)
            mph = speed_to_mph(speed)
            w(f"  [{i:3d}] node {from_n} -> {to_n}  "
              f"len={length:.1f}m  speed={speed:.1f}m/s ({mph:.0f}mph)  "
              f"lanes={lanes}  type={rtype_str}  flags={eflag_str}")
            w(f"         cp1=({cp1x:.1f}, {cp1y:.1f})  cp2=({cp2x:.1f}, {cp2y:.1f})")
        w()

        # --- Section 3: Lane Restrictions ---
        (restrict_count,) = read_fmt(f, "<i")
        w(f"=== LANE RESTRICTIONS ({restrict_count}) ===")
        for _ in range(restrict_count):
            (in_edge,) = read_fmt(f, "<i")
            (in_lane,) = read_fmt(f, "<B")
            (pair_count,) = read_fmt(f, "<i")
            pairs = []
            for __ in range(pair_count):
                (out_edge,) = read_fmt(f, "<i")
                (out_lane,) = read_fmt(f, "<B")
                pairs.append((out_edge, out_lane))
            pair_str = ", ".join(f"edge {oe} lane {ol}" for oe, ol in pairs)
            w(f"  edge {in_edge} lane {in_lane} -> [{pair_str}]")
        w()

        # --- Section 4: Traffic Control Overrides ---
        w("=== TRAFFIC CONTROL OVERRIDES ===")
        (stop_count,) = read_fmt(f, "<i")
        stop_exempt = [read_fmt(f, "<i")[0] for _ in range(stop_count)]
        w(f"  Stop-sign exempt edges ({stop_count}): {stop_exempt}")

        (yield_count,) = read_fmt(f, "<i")
        yield_exempt = [read_fmt(f, "<i")[0] for _ in range(yield_count)]
        w(f"  Yield exempt edges ({yield_count}): {yield_exempt}")

        (phase_count,) = read_fmt(f, "<i")
        w(f"  Phase rotations ({phase_count}):")
        for _ in range(phase_count):
            (node,) = read_fmt(f, "<i")
            (rot,) = read_fmt(f, "<B")
            w(f"    node {node} -> rotation {rot}")
        w()

        # --- Section 5: Camera ---
        cx, cy, zoom = read_fmt(f, "<fff")
        w("=== CAMERA ===")
        w(f"  Center: ({cx:.1f}, {cy:.1f})  Zoom: {zoom:.3f}")
        w()

        # --- Section 6: Vehicles ---
        if has_vehicles:
            (veh_count,) = read_fmt(f, "<i")
            w(f"=== VEHICLES ({veh_count}) ===")
            for i in range(veh_count):
                px, py, heading, speed, steering = read_fmt(f, "<fffff")
                (cur_edge,) = read_fmt(f, "<i")
                (edge_prog,) = read_fmt(f, "<f")
                (cur_lane,) = read_fmt(f, "<B")
                (tgt_lane,) = read_fmt(f, "<B")
                (lc_prog,) = read_fmt(f, "<f")
                (cur_arc,) = read_fmt(f, "<i")
                (arc_prog,) = read_fmt(f, "<f")
                (dest_node,) = read_fmt(f, "<i")
                (path_idx,) = read_fmt(f, "<i")
                (path_len,) = read_fmt(f, "<i")
                path = []
                if path_len > 0:
                    path = list(read_fmt(f, f"<{path_len}i"))
                aggr, sbias, react, steer_s, brake_c, lc_bias, patience = read_fmt(f, "<fffffff")
                (pref_veh,) = read_fmt(f, "<B")
                (archetype,) = read_fmt(f, "<B")
                cr, cg, cb = read_fmt(f, "<BBB")

                heading_deg = math.degrees(heading)
                w(f"  [{i:3d}] pos=({px:.1f}, {py:.1f}) heading={heading_deg:.1f}deg "
                  f"speed={speed:.1f}m/s edge={cur_edge} prog={edge_prog:.2f}")
                w(f"         lane={cur_lane} tgt_lane={tgt_lane} dest={dest_node} "
                  f"color=({cr},{cg},{cb})")
                if path:
                    w(f"         path[{path_idx}]: {path}")
        else:
            w("=== VEHICLES: none saved ===")
        w()

        # --- Section 7: Population (v2+, present only when vehicles were saved) ---
        if version >= 2 and has_vehicles:
            (res_count,) = read_fmt(f, "<i")
            w(f"=== POPULATION ({res_count} residents) ===")
            for i in range(res_count):
                rid, home, work = read_fmt(f, "<iii")
                (archetype,) = read_fmt(f, "<B")
                aggr, sbias, react, steer, brake, lane, patience = read_fmt(f, "<fffffff")
                (pref_veh,) = read_fmt(f, "<B")
                cr, cg, cb = read_fmt(f, "<BBB")
                (sched_len,) = read_fmt(f, "<i")
                schedule = []
                for _ in range(sched_len):
                    (dep,) = read_fmt(f, "<f")
                    (dest,) = read_fmt(f, "<B")
                    schedule.append((dep, dest))
                (sched_idx,) = read_fmt(f, "<i")
                (activity,) = read_fmt(f, "<B")
                (cur_poi,) = read_fmt(f, "<i")
                (veh_idx,) = read_fmt(f, "<i")

                arch_str = enum_name(archetype, DRIVER_ARCHETYPES)
                act_str = enum_name(activity, RESIDENT_ACTIVITY)
                w(f"  [{i:3d}] id={rid} home={home} work={work}  {arch_str} {act_str}  "
                  f"sched[{sched_idx}/{sched_len}] veh={veh_idx} poi={cur_poi}")
                if schedule:
                    sched_str = ", ".join(
                        f"{d:.2f}h->{enum_name(dst, POI_TYPES)}" for d, dst in schedule)
                    w(f"         schedule: {sched_str}")
            w()

        # --- Section 8: Water (v3+) ---
        water_circles = 0
        water_segments = 0
        if version >= 3:
            (water_circles,) = read_fmt(f, "<i")
            w(f"=== WATER: {water_circles} circles ===")
            for i in range(water_circles):
                x, y, r = read_fmt(f, "<fff")
                if i < 10:
                    w(f"  [{i:4d}] ({x:9.1f}, {y:9.1f})  r={r:.1f}m")
            if water_circles > 10:
                w(f"  ... {water_circles - 10} more")
            (water_segments,) = read_fmt(f, "<i")
            w(f"=== WATER: {water_segments} stream segments ===")
            for i in range(water_segments):
                p0x, p0y, c1x, c1y, c2x, c2y, p3x, p3y, width = read_fmt(f, "<9f")
                if i < 10:
                    w(f"  [{i:4d}] ({p0x:8.1f},{p0y:8.1f}) -> ({p3x:8.1f},{p3y:8.1f})  w={width:.1f}m")
            if water_segments > 10:
                w(f"  ... {water_segments - 10} more")
            w()

        # --- Section 9: World Settings (v5+) ---
        if version >= 5:
            (through_enabled,) = read_fmt(f, "<?")
            (through_mult,) = read_fmt(f, "<f")
            (base_per_min,) = read_fmt(f, "<f")
            (rush_hour,) = read_fmt(f, "<?")
            w("=== WORLD SETTINGS ===")
            w(f"  Through traffic  : {'on' if through_enabled else 'off'}  multiplier=x{through_mult:.2f}")
            w(f"  Base traffic     : {base_per_min:.1f} cars/min (population-independent)")
            w(f"  Rush-hour curve  : {'on' if rush_hour else 'off'}")
            w()

        # --- Summary ---
        w("=== SUMMARY ===")
        # Count distinct undirected road segments (pair forward/reverse edges)
        edge_pairs = set()
        for i, e in enumerate(edges):
            fn, tn = e[0], e[1]
            key = (min(fn, tn), max(fn, tn))
            edge_pairs.add(key)
        w(f"  {node_count} nodes, {edge_count} edges ({len(edge_pairs)} road segments)")
        if version >= 3:
            w(f"  Water: {water_circles} circles, {water_segments} stream segments")

        # Count intersection types
        flag_counts = {}
        for _, _, nf, _ in nodes:
            for bit, name in NODE_FLAGS.items():
                if bit == 0:
                    continue
                if nf & bit:
                    flag_counts[name] = flag_counts.get(name, 0) + 1
        if flag_counts:
            w(f"  Node types: {flag_counts}")

        # Count road types
        rtype_counts = {}
        for e in edges:
            rt = enum_name(e[5], ROAD_TYPES)
            rtype_counts[rt] = rtype_counts.get(rt, 0) + 1
        if rtype_counts:
            w(f"  Road types: {rtype_counts}")

        # Connectivity: edges per node
        out_deg = {}
        in_deg = {}
        for e in edges:
            out_deg[e[0]] = out_deg.get(e[0], 0) + 1
            in_deg[e[1]] = in_deg.get(e[1], 0) + 1
        max_out = max(out_deg.values()) if out_deg else 0
        intersection_nodes = [n for n in range(node_count)
                              if out_deg.get(n, 0) + in_deg.get(n, 0) > 4]
        w(f"  Max out-degree: {max_out}")
        w(f"  Intersection nodes (degree > 4): {len(intersection_nodes)}")

    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        print(__doc__.strip())
        sys.exit(1)

    filepath = sys.argv[1]
    result = unpack(filepath)

    if len(sys.argv) >= 3:
        out_path = sys.argv[2]
        Path(out_path).write_text(result, encoding="utf-8")
        print(f"Written to {out_path}")
    else:
        print(result)


if __name__ == "__main__":
    main()
