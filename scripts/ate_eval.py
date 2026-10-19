#!/usr/bin/env python3
"""
ATE (Absolute Trajectory Error) for SLAM Toolbox vs Gazebo ground truth,
computed from a rosbag2 recording.  ICTE4001 Case Study 2 - Member C.

Estimated pose  = map->odom (SLAM Toolbox) composed with odom->base_link (odometry), from /tf
Odometry pose   = odom->base_link alone (what the robot would believe WITHOUT SLAM)
Ground truth    = /sim_ground_truth_pose (Gazebo)

Usage (ROS sourced):
    python3 ate_eval.py ~/bags/e4_run3
Outputs (in the ICTE4001_SLAM Windows folder):
    ate_<bag>_traj.csv   matched trajectories (for plotting)
    ate_<bag>_summary.txt
"""
import sys, os, math, csv, bisect

OUT_DIR = "/mnt/c/Users/Har Zi Ao/Documents/ICTE4001_SLAM"
SAMPLE_DT = 0.1          # evaluate every 0.1 s of sim time
MAX_DT_ODOM = 0.10       # max time gap allowed when matching odom->base_link
MAX_DT_MAP = 0.50        # max time gap allowed when matching map->odom


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def open_reader(uri):
    import rosbag2_py
    last = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("cdr", "cdr"))
            return r
        except Exception as e:  # try next storage plugin
            last = e
    raise last


def read_bag(uri):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = open_reader(uri)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {}
    map_odom, odom_base, gt, odom_topic = [], [], [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in ("/tf", "/sim_ground_truth_pose", "/odom"):
            continue
        if topic not in cls:
            cls[topic] = get_message(types[topic])
        msg = deserialize_message(data, cls[topic])
        if topic == "/tf":
            for tr in msg.transforms:
                p = tr.header.frame_id.lstrip("/")
                c = tr.child_frame_id.lstrip("/")
                rec = (stamp(tr.header.stamp), tr.transform.translation.x,
                       tr.transform.translation.y, yaw_from_q(tr.transform.rotation))
                if p == "map" and c == "odom":
                    map_odom.append(rec)
                elif p == "odom" and c in ("base_link", "base_footprint"):
                    odom_base.append(rec)
        else:
            pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
            rec = (stamp(msg.header.stamp), pose.position.x, pose.position.y,
                   yaw_from_q(pose.orientation))
            (gt if topic == "/sim_ground_truth_pose" else odom_topic).append(rec)
    if not odom_base:            # fall back to /odom topic
        odom_base = odom_topic
    for l in (map_odom, odom_base, gt):
        l.sort()
    return map_odom, odom_base, gt


def nearest(lst, keys, t, max_dt):
    i = bisect.bisect_left(keys, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(lst) and abs(lst[j][0] - t) <= max_dt:
            if best is None or abs(lst[j][0] - t) < abs(best[0] - t):
                best = lst[j]
    return best


def compose(a, b):
    """a ∘ b for 2D poses (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ath + bth)


def align_se2(est, ref):
    """Closed-form least-squares SE(2) alignment (Umeyama, no scale). Returns aligned est."""
    n = len(est)
    ex = sum(p[0] for p in est) / n; ey = sum(p[1] for p in est) / n
    rx = sum(p[0] for p in ref) / n; ry = sum(p[1] for p in ref) / n
    sd = sc = 0.0
    for (a, b), (c, d) in zip(est, ref):
        a -= ex; b -= ey; c -= rx; d -= ry
        sd += a * c + b * d
        sc += a * d - b * c
    th = math.atan2(sc, sd)
    C, S = math.cos(th), math.sin(th)
    tx = rx - (C * ex - S * ey); ty = ry - (S * ex + C * ey)
    return [(C * a - S * b + tx, S * a + C * b + ty) for a, b in est], th, (tx, ty)


def stats(est, ref):
    e = sorted(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(est, ref))
    n = len(e)
    return {"rmse": math.sqrt(sum(x * x for x in e) / n), "mean": sum(e) / n,
            "median": e[n // 2], "max": e[-1], "n": n}


def main():
    bag = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/bags/e4_run3")
    name = os.path.basename(bag.rstrip("/"))
    print(f"Reading {bag} ...")
    map_odom, odom_base, gt = read_bag(bag)
    print(f"  map->odom: {len(map_odom)}   odom->base_link: {len(odom_base)}   ground truth: {len(gt)}")
    if not (map_odom and odom_base and gt):
        print("ERROR: a required stream is missing from the bag."); sys.exit(1)

    k_mo = [r[0] for r in map_odom]; k_ob = [r[0] for r in odom_base]
    t0 = max(map_odom[0][0], odom_base[0][0], gt[0][0])
    rows, last_t = [], -1e9
    for g in gt:
        t = g[0]
        if t < t0 or t - last_t < SAMPLE_DT:
            continue
        ob = nearest(odom_base, k_ob, t, MAX_DT_ODOM)
        mo = nearest(map_odom, k_mo, t, MAX_DT_MAP)
        if ob is None or mo is None:
            continue
        est = compose(mo[1:], ob[1:])
        rows.append((t, g[1], g[2], est[0], est[1], ob[1], ob[2]))
        last_t = t
    if len(rows) < 10:
        print("ERROR: too few matched samples."); sys.exit(1)

    ref = [(r[1], r[2]) for r in rows]
    slam = [(r[3], r[4]) for r in rows]
    odo = [(r[5], r[6]) for r in rows]
    slam_al, th_s, _ = align_se2(slam, ref)
    odo_al, th_o, _ = align_se2(odo, ref)
    s_raw, s_al = stats(slam, ref), stats(slam_al, ref)
    o_raw, o_al = stats(odo, ref), stats(odo_al, ref)
    length = sum(math.hypot(ref[i][0] - ref[i - 1][0], ref[i][1] - ref[i - 1][1]) for i in range(1, len(ref)))
    dur = rows[-1][0] - rows[0][0]

    lines = [
        f"ATE evaluation - bag: {name}",
        f"Samples: {len(rows)} (every {SAMPLE_DT} s sim)   duration: {dur:.1f} s   ground-truth path length: {length:.2f} m",
        "",
        "                     RMSE     mean   median     max   [m]",
        f"SLAM (no align)    {s_raw['rmse']:.3f}   {s_raw['mean']:.3f}   {s_raw['median']:.3f}   {s_raw['max']:.3f}",
        f"SLAM (SE2 align)   {s_al['rmse']:.3f}   {s_al['mean']:.3f}   {s_al['median']:.3f}   {s_al['max']:.3f}   (align rot {math.degrees(th_s):+.2f} deg)",
        f"Odom (no align)    {o_raw['rmse']:.3f}   {o_raw['mean']:.3f}   {o_raw['median']:.3f}   {o_raw['max']:.3f}",
        f"Odom (SE2 align)   {o_al['rmse']:.3f}   {o_al['mean']:.3f}   {o_al['median']:.3f}   {o_al['max']:.3f}   (align rot {math.degrees(th_o):+.2f} deg)",
    ]
    print("\n" + "\n".join(lines))

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, f"ate_{name}_traj.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sim", "gt_x", "gt_y", "slam_x", "slam_y", "odom_x", "odom_y", "slam_aligned_x", "slam_aligned_y"])
        for r, a in zip(rows, slam_al):
            w.writerow([f"{r[0]:.3f}"] + [f"{v:.4f}" for v in r[1:]] + [f"{a[0]:.4f}", f"{a[1]:.4f}"])
    with open(os.path.join(OUT_DIR, f"ate_{name}_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nSaved ate_{name}_traj.csv and ate_{name}_summary.txt to the ICTE4001_SLAM folder.")


if __name__ == "__main__":
    main()
