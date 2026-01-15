#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class TunerNode(Node):
    def __init__(self):
        super().__init__("integrated_tuner_gui")

        # 클라이언트 1: MPC
        self.cli_mpc = self.create_client(
            SetParameters, "/mpc_path_tracker/set_parameters"
        )

        self.get_logger().info(
            "Integrated Tuner Node Started. Connecting to services..."
        )

    def send_parameter(self, target_node, name, value, param_type):
        """
        target_node: 'mpc' or 'frenet'
        name: Parameter name
        value: New value
        param_type: int, float, or bool
        """
        req = SetParameters.Request()
        param = Parameter()
        param.name = name

        if param_type == int:
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        elif param_type == bool:
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = bool(value)
        else:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)

        req.parameters = [param]

        if self.cli_mpc.service_is_ready():
            future = self.cli_mpc.call_async(req)
            future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"Parameter update failed: {e}")


class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("BISA Integrated Tuner & Control")
        self.root.geometry("600x800")

        # [핵심] YAML 파일 로드
        self.mpc_params = {}
        self.frenet_params = {}
        self.load_yaml_params()

        # 탭 생성 (Notebook)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=10)

        # 탭 1: MPC Controller
        self.frame_mpc = ttk.Frame(self.notebook)
        self.notebook.add(self.frame_mpc, text="MPC Controller")
        self.setup_mpc_tab()

    def load_yaml_params(self):
        """패키지 내의 config/mpc_params.yaml 파일을 읽어옵니다."""
        try:
            package_name = "bisa"  # 패키지 이름
            share_dir = get_package_share_directory(package_name)
            yaml_path = os.path.join(share_dir, "config", "mpc_params.yaml")

            print(f">>> Loading YAML from: {yaml_path}")

            if os.path.exists(yaml_path):
                with open(yaml_path, "r") as f:
                    data = yaml.safe_load(f)

                    # MPC 파라미터 파싱
                    if "mpc_path_tracker" in data:
                        self.mpc_params = data["mpc_path_tracker"].get(
                            "ros__parameters", {}
                        )
            else:
                print(f"!!! Warning: YAML file not found at {yaml_path}")

        except Exception as e:
            print(f"!!! Error loading YAML: {e}")

    def apply_yaml_init(self, configs, yaml_data):
        """GUI 설정 리스트의 init 값을 YAML 데이터로 덮어씌웁니다."""
        for cfg in configs:
            param_name = cfg["name"]
            if param_name in yaml_data:
                # YAML에서 값 가져오기
                yaml_val = yaml_data[param_name]
                print(f"   - Overriding {param_name}: {cfg['init']} -> {yaml_val}")
                cfg["init"] = yaml_val
        return configs

    def setup_mpc_tab(self):
        mpc_configs = [
            {
                "name": "horizon",
                "min": 5,
                "max": 100,
                "init": 50,
                "res": 1,
                "type": int,
            },
            {
                "name": "dt",
                "min": 0.01,
                "max": 0.2,
                "init": 0.02,
                "res": 0.005,
                "type": float,
            },
            {
                "name": "lookahead_base",
                "min": 0.1,
                "max": 5.0,
                "init": 0.3,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "lookahead_per_speed",
                "min": 0.0,
                "max": 2.0,
                "init": 0.2,
                "res": 0.05,
                "type": float,
            },
            {
                "name": "Q_pos",
                "min": 0.0,
                "max": 100.0,
                "init": 20.0,
                "res": 0.5,
                "type": float,
            },
            {
                "name": "Q_heading",
                "min": 0.0,
                "max": 100.0,
                "init": 28.0,
                "res": 0.5,
                "type": float,
            },
            {
                "name": "R_v",
                "min": 0.0,
                "max": 10.0,
                "init": 0.3,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "R_w",
                "min": 0.0,
                "max": 10.0,
                "init": 0.3,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "max_velocity",
                "min": 0.0,
                "max": 10.0,
                "init": 3.0,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "max_accel",
                "min": 0.0,
                "max": 10.0,
                "init": 2.0,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "max_angular_vel",
                "min": 0.0,
                "max": 5.0,
                "init": 2.0,
                "res": 0.1,
                "type": float,
            },
        ]
        # YAML 값 적용
        mpc_configs = self.apply_yaml_init(mpc_configs, self.mpc_params)
        self.create_controls(self.frame_mpc, mpc_configs, "mpc")

    def create_controls(self, parent_frame, configs, target_node_name):
        canvas = tk.Canvas(parent_frame)
        scrollbar = ttk.Scrollbar(parent_frame, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for cfg in configs:
            self.create_row(scroll_frame, cfg, target_node_name)

    def create_row(self, parent, cfg, target):
        frame = tk.Frame(parent, pady=5)
        frame.pack(fill="x", padx=10)

        tk.Label(
            frame, text=cfg["name"], width=22, anchor="w", font=("Consolas", 10)
        ).pack(side="left")

        var = (
            tk.IntVar(value=cfg["init"])
            if cfg["type"] == int
            else tk.DoubleVar(value=cfg["init"])
        )

        entry = tk.Entry(frame, textvariable=var, width=6, justify="right")
        entry.pack(side="left", padx=5)
        entry.bind(
            "<Return>",
            lambda e, n=cfg["name"], t=cfg["type"]: self.node.send_parameter(
                target, n, var.get(), t
            ),
        )

        tk.Scale(
            frame,
            from_=cfg["min"],
            to=cfg["max"],
            orient="horizontal",
            resolution=cfg["res"],
            variable=var,
            showvalue=0,
            length=200,
            command=lambda v, n=cfg["name"], t=cfg["type"]: self.node.send_parameter(
                target, n, v, t
            ),
        ).pack(side="left", fill="x", expand=True)


def ros_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = TunerNode()
    t = threading.Thread(target=ros_thread, args=(node,), daemon=True)
    t.start()

    root = tk.Tk()
    App(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
