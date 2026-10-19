#!/usr/bin/env python3
"""
Export laser scans + poses from a rosbag2 recording so the occupancy map can be
rebuilt offline (ICTE4001 Case Study 2 - Member C).

For every scan kept (default 2 Hz) it writes the LASER pose under three estimates:
  slam : map->odom (SLAM Toolbox) ∘ odom->base_link ∘ base_link->laser
  odom : odom->base_link ∘ base_link->laser               (no SLAM)
  gt   : ground truth (Gazebo) ∘ base_link->laser
Outputs to the ICTE4001_SLAM folder:
  <bag>_scanposes.csv, <bag>_ranges.f32 (float32, n_scans x n_beams), <bag>_scanmeta.json
Usage:  python3 bag_export.py ~/bags/e2_loop_on
"""
import sys, os, math, json, bisect
from array import array

OUT_DIR = "/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
KEEP_DT = 0.5


def yaw_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def st(s):
    return s.sec + s.nanosec * 1e-9


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def nearest(lst, keys, t, max_dt):
    i = bisect.bisect_left(keys, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(lst) and abs(lst[j][0] - t) <= max_dt:
            if best is None or abs(lst[j][0] - t) < abs(best[0] - t):
                best = lst[j]
    return best


def open_reader(uri):
    import rosbag2_py
    last = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("cdr", "cdr"))
            return r
        except Exception as e:
            last = e
    raise last


def main():
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag = os.path.expanduser(sys.argv[1])
    name = os.path.basename(bag.rstrip("/"))
    r = open_reader(bag)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {}
    mo, ob, gt, scans = [], [], [], []
    static = {}   # child -> (parent, (x, y, yaw))
    meta = None
    last_scan = -1e9
    print(f"Reading {bag} ...")
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in ("/tf", "/tf_static", "/scan", "/sim_ground_truth_pose"):
            continue
        if topic == "/scan":
            # cheap pre-check of rate is not possible without deserialising; just deserialise
            pass
        if topic not in cls:
            cls[topic] = get_message(types[topic])
        msg = deserialize_message(data, cls[topic])
        if topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                p = tr.header.frame_id.lstrip("/"); c = tr.child_frame_id.lstrip("/")
                pose = (tr.transform.translation.x, tr.transform.translation.y, yaw_q(tr.transform.rotation))
                if topic == "/tf_static":
                    static[c] = (p, pose)
                elif p == "map" and c == "odom":
                    mo.append((st(tr.header.stamp),) + pose)
                elif p == "odom" and c == "base_link":
                    ob.append((st(tr.header.stamp),) + pose)
        elif topic == "/sim_ground_truth_pose":
            ps = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            gt.append((st(msg.header.stamp), ps.position.x, ps.position.y, yaw_q(ps.orientation)))
        else:
            t = st(msg.header.stamp)
            if t - last_scan < KEEP_DT:
                continue
            last_scan = t
            if meta is None:
                meta = {"frame": msg.header.frame_id.lstrip("/"), "angle_min": msg.angle_min,
                        "angle_increment": msg.angle_increment, "range_min": msg.range_min,
                        "range_max": msg.range_max, "n_beams": len(msg.ranges)}
            if len(msg.ranges) == meta["n_beams"]:
                scans.append((t, array("f", msg.ranges)))
    for l in (mo, ob, gt):
        l.sort()
    print(f"  scans kept: {len(scans)}  map->odom: {len(mo)}  odom->base: {len(ob)}  gt: {len(gt)}")

    # static chain base_link -> laser frame
    chain, f = [], meta["frame"]
    while f != "base_link":
        if f not in static:
            print(f"ERROR: no static transform chain from base_link to {meta['frame']} (stuck at {f})"); sys.exit(1)
        p, pose = static[f]
        chain.append(pose); f = p
    b2l = (0.0, 0.0, 0.0)
    for pose in reversed(chain):
        b2l = compose(b2l, pose)
    meta["base_to_laser"] = b2l
    print(f"  base_link -> laser: x={b2l[0]:.3f} y={b2l[1]:.3f} yaw={math.degrees(b2l[2]):.1f} deg")

    k_mo = [x[0] for x in mo]; k_ob = [x[0] for x in ob]; k_gt = [x[0] for x in gt]
    os.makedirs(OUT_DIR, exist_ok=True)
    rows, ranges = [], array("f")
    for t, rg in scans:
        o = nearest(ob, k_ob, t, 0.1); m = nearest(mo, k_mo, t, 0.5); g = nearest(gt, k_gt, t, 0.1)
        if o is None or m is None or g is None:
            continue
        sl = compose(compose(m[1:], o[1:]), b2l); ol = compose(o[1:], b2l); gl = compose(g[1:], b2l)
        rows.append((t,) + sl + ol + gl)
        ranges.extend(rg)
    with open(os.path.join(OUT_DIR, f"{name}_scanposes.csv"), "w") as fh:
        fh.write("t,slam_x,slam_y,slam_yaw,odom_x,odom_y,odom_yaw,gt_x,gt_y,gt_yaw\n")
        for row in rows:
            fh.write(",".join(f"{v:.5f}" for v in row) + "\n")
    with open(os.path.join(OUT_DIR, f"{name}_ranges.f32"), "wb") as fh:
        ranges.tofile(fh)
    meta["n_scans"] = len(rows)
    with open(os.path.join(OUT_DIR, f"{name}_scanmeta.json"), "w") as fh:
        json.dump(meta, fh, indent=1)
    print(f"Saved {len(rows)} scans -> {name}_scanposes.csv / _ranges.f32 / _scanmeta.json")


if __name__ == "__main__":
    main()
