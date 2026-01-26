#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
import threading
import sys


class MPCTunerNode(Node):
    def __init__(self):
        super().__init__("mpc_tuner_gui")

        self.get_logger().info("Starting MPC Tuner GUI (Multi-Tab Edition)...")

        # 1. 파라미터 선언 (Launch/YAML에서 넘어온 cav_ids 수신)
        self.declare_parameter("cav_ids", [1, 2, 3, 4])
        self.cav_ids = self.get_parameter("cav_ids").value

        # 2. MPC 파라미터 이름 목록 (기본값 정의용)
        self.param_names = [
            "Q_pos",
            "Q_heading",
            "R_v",
            "R_w",
            "max_velocity",
            "max_accel",
            "max_angular_vel",
            "horizon",
        ]

        # 기본값 (슬롯별 파라미터가 없을 경우 사용)
        self.default_values = {
            "Q_pos": 15.0,
            "Q_heading": 8.0,
            "R_v": 0.5,
            "R_w": 0.5,
            "max_velocity": 3.0,
            "max_accel": 2.0,
            "max_angular_vel": 2.0,
            "horizon": 20,
        }

        # 3. ★ 슬롯별 파라미터 선언 및 로드
        # Launch에서 slot01_Q_pos, slot02_Q_pos 등의 형식으로 전달됨
        self.slot_values = {}  # {slot_idx: {param_name: value}}

        for slot_idx in range(1, 5):  # slot 01 ~ 04
            slot_str = f"{slot_idx:02d}"
            self.slot_values[slot_idx] = {}

            for param_name in self.param_names:
                full_param_name = f"slot{slot_str}_{param_name}"
                default_val = self.default_values[param_name]

                # 파라미터 선언
                self.declare_parameter(full_param_name, default_val)

                # 값 로드
                try:
                    val = self.get_parameter(full_param_name).value
                    self.slot_values[slot_idx][param_name] = val
                    self.get_logger().info(f"  Loaded {full_param_name} = {val}")
                except Exception as e:
                    self.slot_values[slot_idx][param_name] = default_val
                    self.get_logger().warn(
                        f"  Failed to load {full_param_name}, using default: {default_val}"
                    )

        self.srv_clients = {}

        # 4. 각 CAV 노드의 set_parameters 서비스 클라이언트 생성
        self.get_logger().info(f"Target CAV IDs: {self.cav_ids}")
        for cid in self.cav_ids:
            id_str = f"{cid:02d}"
            node_name = f"/mpc_tracker_cav{id_str}"
            service_name = f"{node_name}/set_parameters"

            cli = self.create_client(SetParameters, service_name)
            self.srv_clients[cid] = cli

            if not cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f"Service {service_name} not ready yet.")
            else:
                self.get_logger().info(f"Connected to {node_name}")

    def get_slot_initial_value(self, slot_idx, param_name):
        """슬롯별 초기값 반환 (1-based index)"""
        if slot_idx in self.slot_values and param_name in self.slot_values[slot_idx]:
            return self.slot_values[slot_idx][param_name]
        return self.default_values.get(param_name, 0)

    def send_parameter_to_id(self, target_cav_id, name, value, param_type):
        """특정 CAV ID에게만 파라미터 변경 요청을 보냅니다."""
        if target_cav_id not in self.srv_clients:
            self.get_logger().error(f"CAV {target_cav_id} client not found!")
            return

        cli = self.srv_clients[target_cav_id]
        if not cli.service_is_ready():
            self.get_logger().warn(f"Service for CAV {target_cav_id} is not ready.")
            return

        req = SetParameters.Request()
        param = Parameter()
        param.name = name

        if param_type == int:
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        elif param_type == float:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
        elif param_type == bool:
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = bool(value)

        req.parameters = [param]
        future = cli.call_async(req)


class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("BISA Multi-CAV MPC Tuner")
        self.root.geometry("600x750")

        # 1. 파라미터 설정 메타데이터
        self.configs = [
            {"name": "Q_pos", "min": 0.0, "max": 200.0, "res": 0.1, "type": float},
            {"name": "Q_heading", "min": 0.0, "max": 200.0, "res": 0.1, "type": float},
            {"name": "R_v", "min": 0.0, "max": 20.0, "res": 0.1, "type": float},
            {"name": "R_w", "min": 0.0, "max": 20.0, "res": 0.1, "type": float},
            {
                "name": "max_velocity",
                "min": 0.0,
                "max": 10.0,
                "res": 0.1,
                "type": float,
            },
            {"name": "max_accel", "min": 0.0, "max": 10.0, "res": 0.1, "type": float},
            {
                "name": "max_angular_vel",
                "min": 0.0,
                "max": 10.0,
                "res": 0.1,
                "type": float,
            },
            {"name": "horizon", "min": 5, "max": 200, "res": 1, "type": int},
        ]

        # 2. 탭 컨트롤 (Notebook) 생성
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(expand=True, fill="both", padx=5, pady=5)

        # 3. 각 CAV ID 별로 탭 생성
        self.create_tabs_for_cavs()

        # 하단 상태바
        self.status_var = tk.StringVar()
        self.status_var.set(f"Active Nodes: {self.node.cav_ids}")
        lbl_status = tk.Label(
            self.root,
            textvariable=self.status_var,
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            bg="#dddddd",
        )
        lbl_status.pack(side=tk.BOTTOM, fill=tk.X)

    def create_tabs_for_cavs(self):
        """cav_ids 리스트를 순회하며 탭을 생성합니다."""
        for index, cid in enumerate(self.node.cav_ids):
            # 탭 프레임 생성
            tab_frame = ttk.Frame(self.notebook)
            tab_title = f"CAV {cid:02d}"  # 예: CAV 01, CAV 24
            self.notebook.add(tab_frame, text=tab_title)

            # ★ 슬롯 인덱스 전달 (1-based: index 0 -> slot 1)
            slot_idx = index + 1
            self.create_controls_for_tab(tab_frame, cid, slot_idx)

    def create_controls_for_tab(self, parent_frame, cav_id, slot_idx):
        """특정 CAV ID를 위한 컨트롤들을 생성합니다."""

        # 스크롤 기능 추가
        canvas = tk.Canvas(parent_frame)
        scrollbar = ttk.Scrollbar(parent_frame, orient="vertical", command=canvas.yview)
        scroll_content = ttk.Frame(canvas)

        scroll_content.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scroll_content, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # 타이틀 (슬롯 정보 포함)
        tk.Label(
            scroll_content,
            text=f"Tuning Parameters for CAV {cav_id:02d} (Slot {slot_idx:02d})",
            font=("Arial", 12, "bold"),
            fg="#333333",
            pady=10,
        ).pack()

        # 각 파라미터 행 생성
        for cfg in self.configs:
            self.create_row(scroll_content, cfg, cav_id, slot_idx)

    def create_row(self, parent, cfg, cav_id, slot_idx):
        frame = tk.Frame(parent, pady=5)
        frame.pack(fill="x", padx=10)

        # 1. 라벨
        tk.Label(
            frame, text=cfg["name"], width=16, anchor="w", font=("Consolas", 10, "bold")
        ).pack(side="left")

        # 2. ★ 슬롯별 초기값 가져오기
        init_val = self.node.get_slot_initial_value(slot_idx, cfg["name"])

        if cfg["type"] == int:
            var = tk.IntVar(value=int(init_val))
        else:
            var = tk.DoubleVar(value=float(init_val))

        # 3. 입력창 (Entry)
        entry = tk.Entry(
            frame, textvariable=var, width=6, justify="right", font=("Arial", 10)
        )
        entry.pack(side="left", padx=5)

        # 엔터키 누르면 전송 (특정 CAV ID로)
        entry.bind(
            "<Return>",
            lambda e, c=cav_id, n=cfg["name"], t=cfg[
                "type"
            ], v=var: self.node.send_parameter_to_id(c, n, v.get(), t),
        )

        # 4. 슬라이더 (Scale) - 움직일 때마다 즉시 전송
        scale = tk.Scale(
            frame,
            from_=cfg["min"],
            to=cfg["max"],
            orient="horizontal",
            resolution=cfg["res"],
            variable=var,
            showvalue=0,
            length=220,
            command=lambda val, c=cav_id, n=cfg["name"], t=cfg[
                "type"
            ]: self.node.send_parameter_to_id(c, n, val, t),
        )
        scale.pack(side="left", fill="x", expand=True)


def ros_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTunerNode()

    # ROS 통신을 위한 별도 스레드 실행
    t = threading.Thread(target=ros_thread, args=(node,), daemon=True)
    t.start()

    # GUI 실행 (메인 스레드)
    root = tk.Tk()
    app = App(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
