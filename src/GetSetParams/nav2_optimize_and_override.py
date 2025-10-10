#!/usr/bin/env python3
import argparse, random, yaml, math, sys

def get_nested(d, path):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur

def set_nested(d, path, value):
    cur = d
    for p in path[:-1]:
        cur = cur.setdefault(p, {})
    cur[path[-1]] = value

def parse_key(dotted):
    # expects node.dotted.param.path (e.g., controller_server.FollowPath.max_vel_x)
    if "." not in dotted:
        raise ValueError(f"Bad key '{dotted}', needs node.dotted.path")
    node, rest = dotted.split(".", 1)
    return node, rest.split(".")

def clamp(v, lo, hi):
    if lo is not None: v = max(v, lo)
    if hi is not None: v = min(v, hi)
    return v

def main():
    ap = argparse.ArgumentParser(
        description="Load Nav2 YAML, auto-propose new values for given keys, write new YAML.")
    ap.add_argument("--in",  dest="in_path",  required=True, help="Input base YAML (e.g., nav2_params.yaml)")
    ap.add_argument("--out", dest="out_path", required=True, help="Output YAML with proposed values")
    ap.add_argument("--keys", nargs="+", required=True,
                    help="Keys to optimize, e.g. controller_server.FollowPath.max_vel_x controller_server.FollowPath.acc_lim_x ...")
    ap.add_argument("--j", type=float, required=True, help="KPI score J from last run (float)")

    # Strategy: perturb around current value (default) OR full-random in a range
    ap.add_argument("--strategy", choices=["perturb", "random"], default="perturb",
                    help="How to propose new values")
    ap.add_argument("--perturb-frac", type=float, default=0.15,
                    help="Max fractional change for 'perturb' (e.g., 0.15 = ±15%%)")
    ap.add_argument("--float-range", nargs=2, type=float, metavar=("MIN", "MAX"),
                    help="For 'random' strategy on floats")
    ap.add_argument("--int-range",   nargs=2, type=int,   metavar=("MIN", "MAX"),
                    help="For 'random' strategy on ints")
    ap.add_argument("--lower", type=float, default=None, help="Clamp lower bound for floats (applies to all keys)")
    ap.add_argument("--upper", type=float, default=None, help="Clamp upper bound for floats (applies to all keys)")
    ap.add_argument("--seed",  type=int, default=None, help="Random seed for reproducibility")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    with open(args.in_path, "r") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        print("ERROR: input YAML top-level must be a mapping.", file=sys.stderr)
        sys.exit(2)

    print(f"[INFO] J(last) = {args.j}")

    # Optional: scale perturb step a bit by J (smaller J → smaller step)
    # scale in [0.5, 1.0] of perturb-frac
    if args.strategy == "perturb":
        scale = 0.5 + 0.5 / (1.0 + max(args.j, 0.0))
        step = args.perturb_frac * scale
    else:
        step = None  # not used

    for dotted in args.keys:
        node, subpath = parse_key(dotted)
        node_map = doc.setdefault(node, {})
        ros = node_map.setdefault("ros__parameters", {})
        cur_val = get_nested(ros, subpath)
        before = cur_val

        if cur_val is None:
            print(f"[WARN] {dotted} not found in YAML, creating it with 0.0")
            cur_val = 0.0

        # Propose
        if isinstance(cur_val, bool):
            # flip with small probability if perturb; random picks True/False uniform
            if args.strategy == "perturb":
                new_val = cur_val if random.random() > 0.2 else (not cur_val)
            else:
                new_val = bool(random.getrandbits(1))
        elif isinstance(cur_val, int):
            if args.strategy == "perturb":
                delta = cur_val * random.uniform(-step, step)
                new_val = int(round(cur_val + delta))
            else:
                if not args.int_range:
                    print(f"ERROR: --int-range required for random int key '{dotted}'", file=sys.stderr)
                    sys.exit(2)
                lo, hi = args.int_range
                new_val = random.randint(lo, hi)
        elif isinstance(cur_val, float) or isinstance(cur_val, int):  # treat ints as floats if needed
            cur = float(cur_val)
            if args.strategy == "perturb":
                delta = cur * random.uniform(-step, step)
                new_val = cur + delta
                new_val = clamp(new_val, args.lower, args.upper)
            else:
                if not args.float_range:
                    print(f"ERROR: --float-range required for random float key '{dotted}'", file=sys.stderr)
                    sys.exit(2)
                lo, hi = args.float_range
                new_val = random.uniform(lo, hi)
                new_val = clamp(new_val, args.lower, args.upper)
        else:
            # strings/arrays: leave as-is
            print(f"[INFO] {dotted} is non-numeric (type {type(cur_val).__name__}); leaving unchanged.")
            new_val = cur_val

        set_nested(ros, subpath, new_val)
        print(f"{dotted}: {before} -> {new_val}")

    with open(args.out_path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"[OK] wrote: {args.out_path}")

if __name__ == "__main__":
    main()
