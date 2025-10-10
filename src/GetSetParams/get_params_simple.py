#!/usr/bin/env python3
import argparse, yaml
from typing import Dict, Any, List
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import ParameterType

def build_tree(flat: Dict[str, Any]) -> Dict[str, Any]:
    root: Dict[str, Any] = {}
    for dotted, value in flat.items():
        cur = root
        parts = dotted.split(".")
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        cur[parts[-1]] = value
    return root

class Dumper(Node):
    def __init__(self, node_name: str):
        super().__init__("nav2_dump_all")
        self.node_name = node_name
        self.cli_list = self.create_client(ListParameters, f"{node_name}/list_parameters")
        self.cli_get  = self.create_client(GetParameters,  f"{node_name}/get_parameters")
        for cli, n in [(self.cli_list,"list_parameters"), (self.cli_get,"get_parameters")]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {node_name}/{n} ...")

    def list_all(self) -> List[str]:
        req = ListParameters.Request()
        req.depth = 0
        req.prefixes = []
        fut = self.cli_list.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        names = list(res.result.names) if res else []
        names.sort()
        return names

    def get_values(self, names: List[str]) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        # chunk to be safe
        for i in range(0, len(names), 100):
            grp = names[i:i+100]
            req = GetParameters.Request(); req.names = grp
            fut = self.cli_get.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            for n, v in zip(grp, res.values):
                if   v.type == ParameterType.PARAMETER_BOOL: out[n] = bool(v.bool_value)
                elif v.type == ParameterType.PARAMETER_INTEGER: out[n] = int(v.integer_value)
                elif v.type == ParameterType.PARAMETER_DOUBLE: out[n] = float(v.double_value)
                elif v.type == ParameterType.PARAMETER_STRING: out[n] = str(v.string_value)
                elif v.type == ParameterType.PARAMETER_BYTE_ARRAY: out[n] = list(v.byte_array_value)
                elif v.type == ParameterType.PARAMETER_BOOL_ARRAY: out[n] = list(v.bool_array_value)
                elif v.type == ParameterType.PARAMETER_INTEGER_ARRAY: out[n] = [int(x) for x in v.integer_array_value]
                elif v.type == ParameterType.PARAMETER_DOUBLE_ARRAY: out[n] = [float(x) for x in v.double_array_value]
                elif v.type == ParameterType.PARAMETER_STRING_ARRAY: out[n] = list(v.string_array_value)
                else: out[n] = None
        return out

def main():
    ap = argparse.ArgumentParser(description="Dump multiple Nav2 nodes to a single launch-ready YAML.")
    ap.add_argument("--nodes", nargs="+", required=True,
                    help="Node names as seen in `ros2 node list`, e.g. /controller_server /planner_server /amcl ...")
    ap.add_argument("--out", required=True, help="Output YAML path (full Nav2 params)")
    args = ap.parse_args()

    rclpy.init()
    big_doc = {}
    for node_name in args.nodes:
        d = Dumper(node_name)
        names = d.list_all()
        vals  = d.get_values(names)
        rclpy.spin_once(d, timeout_sec=0.0)
        d.destroy_node()

        # YAML node key: usually without leading slash for nav2 files
        yaml_node_key = node_name[1:] if node_name.startswith("/") else node_name
        tree = build_tree(vals)
        big_doc[yaml_node_key] = {"ros__parameters": tree}

    rclpy.shutdown()
    with open(args.out, "w") as f:
        yaml.safe_dump(big_doc, f, sort_keys=False)
    print(f"[OK] wrote: {args.out}")

if __name__ == "__main__":
    main()
