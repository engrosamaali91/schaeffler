#!/usr/bin/env python3
import argparse, yaml, sys

def ensure_path(d, path_parts):
    cur = d
    for p in path_parts:
        cur = cur.setdefault(p, {})
    return cur

def set_dotted(doc, node_key, dotted, value):
    # node_key like "controller_server"; inside it we set ros__parameters dotted path
    node = doc.setdefault(node_key, {})
    ros = node.setdefault("ros__parameters", {})
    parts = dotted.split(".")
    cur = ros
    for p in parts[:-1]:
        cur = cur.setdefault(p, {})
    cur[parts[-1]] = value

def parse_set_list(items):
    changes = {}
    for s in items:
        if "=" not in s:
            raise ValueError(f"Bad --set '{s}', expected node.key=val (e.g., controller_server.FollowPath.max_vel_x=0.3)")
        left, val = s.split("=", 1)
        # left: node.key1.key2...
        if "." not in left:
            raise ValueError(f"--set needs node and key (e.g., controller_server.FollowPath.max_vel_x=0.3)")
        node, dotted = left.split(".", 1)
        # cast value: try int, float, bool, else string
        v = val.strip()
        if v.lower() in ("true","false"):
            v = (v.lower() == "true")
        else:
            try:
                if "." in v: v = float(v)
                else: v = int(v)
            except ValueError:
                pass
        changes.setdefault(node, {})[dotted] = v
    return changes

def main():
    ap = argparse.ArgumentParser(description="Override selected parameters in a Nav2 YAML.")
    ap.add_argument("--in", dest="in_path", required=True, help="Input full Nav2 YAML")
    ap.add_argument("--out", dest="out_path", required=True, help="Output YAML (overrides applied)")
    ap.add_argument("--set", nargs="+", required=True,
                    help="Overrides as node.dotted.key=value (e.g., controller_server.FollowPath.max_vel_x=0.3)")
    args = ap.parse_args()

    with open(args.in_path, "r") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        print("Input YAML top-level must be a mapping.", file=sys.stderr); sys.exit(2)

    changes = parse_set_list(args._get_kwargs()[-1][1]) if False else parse_set_list(args.set)
    for node_key, kv in changes.items():
        for dotted, v in kv.items():
            set_dotted(doc, node_key, dotted, v)
            print(f"set {node_key}.{dotted} = {v}")

    with open(args.out_path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    print(f"[OK] wrote: {args.out_path}")

if __name__ == "__main__":
    main()
