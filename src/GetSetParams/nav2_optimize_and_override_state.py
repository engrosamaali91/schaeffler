#!/usr/bin/env python3
"""
Load Nav2 YAML, use a minimal optimizer to propose new values for given keys, and write a new YAML.

Optimizer logic (simple, dependency-free):
- Keeps a tiny state (JSON): best_J, step sizes per key, last values.
- If J improved: shrink step (exploit). If J got worse: grow step (explore) and flip the perturbation direction.
- Proposes bounded values (global bounds or per-key from a bounds YAML).

Usage (one iteration):
  python3 nav2_optimize_and_override.py \
    --in nav2_params.yaml \
    --out nav2_trial03.yaml \
    --keys controller_server.FollowPath.max_vel_x controller_server.FollowPath.acc_lim_x \
    --j 1.23 \
    --state optimizer_state.json \
    --lower 0.0 --upper 2.0 \
    --seed 42

Optional per-key bounds file (YAML), e.g.:
  bounds.yaml:
    controller_server.FollowPath.max_vel_x: [0.05, 1.5]
    controller_server.FollowPath.acc_lim_x: [0.5, 4.0]

Then run with: --bounds-yaml bounds.yaml
"""
import argparse, json, random, yaml, os, sys
from typing import Dict, Tuple, Optional, List, Any

# ---------- basic nested dict helpers ----------
def get_nested(d: dict, path: List[str]) -> Any:
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur

def set_nested(d: dict, path: List[str], value: Any) -> None:
    cur = d
    for p in path[:-1]:
        cur = cur.setdefault(p, {})
    cur[path[-1]] = value

def parse_key(dotted: str) -> Tuple[str, List[str]]:
    if "." not in dotted:
        raise ValueError(f"Bad key '{dotted}', needs 'node.param.path' form")
    node, rest = dotted.split(".", 1)
    return node, rest.split(".")

def clamp(v: float, lo: Optional[float], hi: Optional[float]) -> float:
    if lo is not None: v = max(v, lo)
    if hi is not None: v = min(v, hi)
    return v

# ---------- tiny optimizer with persistent state ----------
DEFAULT_STEP_FRAC = 0.15   # start with ±15% steps
GROW   = 1.25              # grow step when J got worse
SHRINK = 0.7               # shrink step when J improved

def load_state(path: str) -> dict:
    if path and os.path.exists(path):
        with open(path, "r") as f: return json.load(f)
    return {"best_J": None, "best_vals": {}, "step_frac": {}, "last_vals": {}, "last_J": None, "sign": {}}

def save_state(path: str, state: dict) -> None:
    if path:
        with open(path, "w") as f: json.dump(state, f, indent=2)

def per_key_bounds(dotted_key: str, bounds_yaml: Optional[dict], global_lo: Optional[float], global_hi: Optional[float]) -> Tuple[Optional[float], Optional[float]]:
    if bounds_yaml and dotted_key in bounds_yaml:
        lo, hi = bounds_yaml[dotted_key]
        return float(lo), float(hi)
    return global_lo, global_hi

def propose_value_for_key(
    dotted_key: str,
    cur_val: Any,
    J: float,
    state: dict,
    global_lo: Optional[float],
    global_hi: Optional[float],
    bounds_yaml: Optional[dict],
) -> Any:
    """
    Adaptive perturbation around current numeric value.
    - step_frac per key increases if J worse, decreases if J improved.
    - sign per key flips when J worse (try other side).
    - bounded by per-key or global bounds.
    Non-numerics are returned unchanged.
    """
    # Leave non-numerics unchanged
    if isinstance(cur_val, (list, dict, str)) or cur_val is None:
        return cur_val

    # Decide numeric type
    is_bool = isinstance(cur_val, bool)
    is_int  = isinstance(cur_val, int) and not is_bool
    is_float = isinstance(cur_val, float) or (is_int)

    if is_bool:
        # flip rarely if improved; else maybe flip
        improved = (state.get("best_J") is None) or (J < state["best_J"])
        return cur_val if improved else (not cur_val)

    if not is_float:
        return cur_val

    cur_float = float(cur_val)
    key = dotted_key

    # step_frac per key
    step_frac = state["step_frac"].get(key, DEFAULT_STEP_FRAC)

    # Did we improve compared to last_J?
    last_J = state.get("last_J")
    improved = (last_J is None) or (J < last_J)

    # Adapt step
    step_frac = step_frac * (SHRINK if improved else GROW)
    step_frac = max(0.01, min(step_frac, 0.5))  # keep sane bounds: 1%..50%
    state["step_frac"][key] = step_frac

    # Direction memory: flip if worse, keep if improved
    sign = state["sign"].get(key, 1.0)
    sign = sign if improved else -sign
    state["sign"][key] = sign

    # Draw a small random in [-1,1], bias by sign
    r = random.uniform(0.0, 1.0)  # 0..1
    direction = sign * (0.5 + r/2.0)  # 0.5..1.0 scaled, keeps inertia

    delta = cur_float * step_frac * direction
    proposal = cur_float + delta

    # Bounds
    lo, hi = per_key_bounds(key, bounds_yaml, global_lo, global_hi)
    proposal = clamp(proposal, lo, hi)

    # If int in YAML, keep it int
    if isinstance(cur_val, int) and not isinstance(cur_val, bool):
        proposal = int(round(proposal))

    return proposal

# ---------- CLI & main ----------
def main():
    ap = argparse.ArgumentParser(description="Minimal optimizer: propose new values for keys and write a new Nav2 YAML.")
    ap.add_argument("--in",  dest="in_path",  required=True, help="Input base YAML (e.g., nav2_params.yaml)")
    ap.add_argument("--out", dest="out_path", required=True, help="Output YAML with proposed values")
    ap.add_argument("--keys", nargs="+", required=True,
                    help="Keys to optimize, e.g. controller_server.FollowPath.max_vel_x controller_server.FollowPath.acc_lim_x ...")
    ap.add_argument("--j", type=float, required=True, help="KPI score J from last run (float)")
    ap.add_argument("--state", type=str, default="optimizer_state.json", help="Path to persistent optimizer state JSON")
    ap.add_argument("--bounds-yaml", type=str, default="", help="Optional per-key bounds YAML (key: [lo, hi])")
    ap.add_argument("--lower", type=float, default=None, help="Global lower bound for floats (used if no per-key bound)")
    ap.add_argument("--upper", type=float, default=None, help="Global upper bound for floats (used if no per-key bound)")
    ap.add_argument("--seed",  type=int, default=None, help="Random seed for reproducibility")
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    with open(args.in_path, "r") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        print("ERROR: input YAML top-level must be a mapping.", file=sys.stderr)
        sys.exit(2)

    bounds_yaml = None
    if args.bounds_yaml:
        with open(args.bounds_yaml, "r") as bf:
            bounds_yaml = yaml.safe_load(bf) or {}

    # Load optimizer state
    state = load_state(args.state)
    print(f"[INFO] J(last) = {args.j}   best_J={state.get('best_J')}")

    # Propose new values
    for dotted in args.keys:
        node, subpath = parse_key(dotted)
        node_map = doc.setdefault(node, {})
        ros = node_map.setdefault("ros__parameters", {})
        cur_val = get_nested(ros, subpath)

        if cur_val is None:
            print(f"[WARN] {dotted} not found; initializing to 0.0")
            cur_val = 0.0

        new_val = propose_value_for_key(
            dotted_key=dotted,
            cur_val=cur_val,
            J=args.j,
            state=state,
            global_lo=args.lower,
            global_hi=args.upper,
            bounds_yaml=bounds_yaml,
        )

        set_nested(ros, subpath, new_val)
        print(f"{dotted}: {cur_val} -> {new_val}")

        # Track last values
        state["last_vals"][dotted] = new_val

    # Update state with J
    state["last_J"] = args.j
    if state["best_J"] is None or args.j < state["best_J"]:
        state["best_J"] = args.j
        # snapshot best values for the tuned keys
        for dotted in args.keys:
            node, subpath = parse_key(dotted)
            val = get_nested(doc.setdefault(node, {}).setdefault("ros__parameters", {}), subpath)
            state["best_vals"][dotted] = val

    # Write outputs
    with open(args.out_path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    save_state(args.state, state)
    print(f"[OK] wrote: {args.out_path}")
    print(f"[STATE] saved: {args.state}")

if __name__ == "__main__":
    main()
