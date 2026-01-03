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

        # 클라이언트 2: Frenet Planner
        # [주의] YAML 파일의 노드 이름은 frenet_planner지만,
        # launch 파일이나 cpp 노드 초기화 시 이름이 다를 수 있으므로 확인 필요.
        # 일단 frenet_planner.cpp에서는 "frenet_planner"로 초기화됨.
        self.cli_frenet = self.create_client(
            SetParameters, "/frenet_planner/set_parameters"
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

        if target_node == "mpc":
            if self.cli_mpc.service_is_ready():
                future = self.cli_mpc.call_async(req)
                future.add_done_callback(self.callback_done)
        else:
            if self.cli_frenet.service_is_ready():
                future = self.cli_frenet.call_async(req)
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

        # 탭 2: Frenet Planner
        self.frame_frenet = ttk.Frame(self.notebook)
        self.notebook.add(self.frame_frenet, text="Frenet Planner")
        self.setup_frenet_tab()

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

                    # Frenet 파라미터 파싱
                    if "frenet_planner" in data:
                        self.frenet_params = data["frenet_planner"].get(
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

    def setup_frenet_tab(self):
        # 1. Cool Start 버튼 (상단 배치)
        btn_frame = tk.Frame(self.frame_frenet, pady=10, bg="#ffcccc")
        btn_frame.pack(fill="x", side="top")

        btn_reset = tk.Button(
            btn_frame,
            text="🔥 COOL START (RESET STATE) 🔥",
            command=self.trigger_reset,
            bg="red",
            fg="white",
            font=("Arial", 12, "bold"),
        )
        btn_reset.pack(pady=5, padx=20, fill="x")

        # 2. 파라미터 리스트
        frenet_configs = [
            {
                "name": "target_speed",
                "min": 0.0,
                "max": 5.0,
                "init": 1.0,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "boost_speed",
                "min": 0.0,
                "max": 5.0,
                "init": 1.5,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "base_lookahead",
                "min": 0.1,
                "max": 3.0,
                "init": 0.6,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "max_lookahead",
                "min": 0.5,
                "max": 5.0,
                "init": 1.2,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "weight_heading",
                "min": 0.0,
                "max": 2.0,
                "init": 0.7,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "weight_path",
                "min": 0.0,
                "max": 2.0,
                "init": 0.3,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "error_gain",
                "min": 0.0,
                "max": 1.0,
                "init": 0.2,
                "res": 0.05,
                "type": float,
            },
            {
                "name": "steer_alpha",
                "min": 0.0,
                "max": 1.0,
                "init": 0.1,
                "res": 0.05,
                "type": float,
            },
            {
                "name": "static_margin_front",
                "min": 0.0,
                "max": 5.0,
                "init": 0.7,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "static_margin_rear",
                "min": 0.0,
                "max": 5.0,
                "init": 0.3,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "predict_time",
                "min": 0.1,
                "max": 5.0,
                "init": 1.0,
                "res": 0.1,
                "type": float,
            },
            {
                "name": "predict_step",
                "min": 0.05,
                "max": 1.0,
                "init": 0.2,
                "res": 0.05,
                "type": float,
            },
        ]
        # YAML 값 적용
        frenet_configs = self.apply_yaml_init(frenet_configs, self.frenet_params)
        self.create_controls(self.frame_frenet, frenet_configs, "frenet")

    def trigger_reset(self):
        # Frenet Node에 reset_trigger = True를 보냄
        self.node.send_parameter("frenet", "reset_trigger", True, bool)
        print(">>> Reset Trigger Sent!")

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
