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

        self.get_logger().info("Starting MPC Tuner GUI (Tkinter Style)...")
        self.get_logger().info("Parameters will be synced to ALL active CAVs.")

        # 1. 파라미터 선언 (Launch/YAML에서 넘어온 초기값 수신)
        self.declare_parameter('cav_ids', [1, 2, 3, 4])
        
        # MPC 제어 파라미터 (YAML의 mpc_settings 값들이 여기로 들어옴)
        self.declare_parameter('Q_pos', 15.0)
        self.declare_parameter('Q_heading', 8.0)
        self.declare_parameter('R_v', 0.5)
        self.declare_parameter('R_w', 0.5)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('horizon', 20)

        # 2. 제어 대상 CAV ID 확인
        self.cav_ids = self.get_parameter('cav_ids').value
        
        # [수정] self.clients는 ROS2 Node의 예약어이므로 self.srv_clients로 변경
        self.srv_clients = {}

        # 3. 각 CAV 노드의 set_parameters 서비스 클라이언트 생성
        self.get_logger().info(f"Target CAV IDs: {self.cav_ids}")
        for cid in self.cav_ids:
            id_str = f"{cid:02d}"
            node_name = f"/mpc_tracker_cav{id_str}"
            service_name = f"{node_name}/set_parameters"
            
            cli = self.create_client(SetParameters, service_name)
            # [수정] 변수명 변경 반영
            self.srv_clients[cid] = cli
            
            # 서비스 연결 대기 (너무 오래 걸리면 스킵)
            if not cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f"⚠️ Service {service_name} not ready yet.")
            else:
                self.get_logger().info(f"✅ Connected to {node_name}")

    def broadcast_parameter(self, name, value, param_type):
        """
        변경된 파라미터 값을 모든 연결된 CAV 노드에게 전송합니다.
        """
        # 요청 객체 생성
        req = SetParameters.Request()
        param = Parameter()
        param.name = name

        # 타입에 따른 값 설정
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

        # 모든 클라이언트에 비동기 전송
        # [수정] 변수명 변경 반영
        for cid, cli in self.srv_clients.items():
            if cli.service_is_ready():
                future = cli.call_async(req)
                # 결과 콜백은 필요시 추가
            else:
                pass 

class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("BISA Multi-CAV MPC Tuner")
        self.root.geometry("550x700")

        # 1. 파라미터 설정 리스트 (GUI 구성용)
        # init 값은 노드 파라미터(YAML 로드됨)에서 가져옵니다.
        self.configs = [
            {"name": "Q_pos",           "min": 0.0, "max": 200.0, "res": 1.0, "type": float},
            {"name": "Q_heading",       "min": 0.0, "max": 200.0, "res": 1.0, "type": float},
            {"name": "R_v",             "min": 0.0, "max": 20.0,  "res": 0.1, "type": float},
            {"name": "R_w",             "min": 0.0, "max": 20.0,  "res": 0.1, "type": float},
            {"name": "max_velocity",    "min": 0.0, "max": 10.0,  "res": 0.1, "type": float},
            {"name": "max_accel",       "min": 0.0, "max": 10.0,  "res": 0.1, "type": float},
            {"name": "max_angular_vel", "min": 0.0, "max": 10.0,  "res": 0.1, "type": float},
            {"name": "horizon",         "min": 5,   "max": 200,   "res": 1,   "type": int},
        ]

        # 2. 초기값 로드 (Node -> Config)
        print(">>> Loading initial values from Launch/YAML...")
        for cfg in self.configs:
            try:
                val = self.node.get_parameter(cfg["name"]).value
                cfg["init"] = val
                print(f"   - {cfg['name']}: {val}")
            except Exception as e:
                print(f"   ! Failed to get {cfg['name']}: {e}")
                cfg["init"] = 0

        # 3. GUI 생성
        self.create_controls(self.root, self.configs)

        # 하단 상태바
        self.status_var = tk.StringVar()
        self.status_var.set(f"Controlling: {self.node.cav_ids}")
        lbl_status = tk.Label(self.root, textvariable=self.status_var, bd=1, relief=tk.SUNKEN, anchor=tk.W, bg="#dddddd")
        lbl_status.pack(side=tk.BOTTOM, fill=tk.X)

    def create_controls(self, parent, configs):
        # 스크롤 가능한 캔버스 영역 생성
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 타이틀
        tk.Label(scroll_frame, text="MPC Parameters (All Vehicles)", font=("Arial", 14, "bold"), pady=15).pack()

        # 각 파라미터 행 생성
        for cfg in configs:
            self.create_row(scroll_frame, cfg)

    def create_row(self, parent, cfg):
        frame = tk.Frame(parent, pady=8)
        frame.pack(fill="x", padx=15)

        # 1. 라벨
        tk.Label(frame, text=cfg["name"], width=18, anchor="w", font=("Consolas", 11, "bold")).pack(side="left")

        # 2. 변수 (Int/Double)
        if cfg["type"] == int:
            var = tk.IntVar(value=int(cfg["init"]))
        else:
            var = tk.DoubleVar(value=float(cfg["init"]))

        # 3. 입력창 (Entry)
        entry = tk.Entry(frame, textvariable=var, width=6, justify="right", font=("Arial", 10))
        entry.pack(side="left", padx=10)
        
        # 엔터키 누르면 전송
        entry.bind("<Return>", lambda e, n=cfg["name"], t=cfg["type"], v=var: 
                   self.node.broadcast_parameter(n, v.get(), t))

        # 4. 슬라이더 (Scale) - 움직일 때마다 즉시 전송
        scale = tk.Scale(
            frame,
            from_=cfg["min"],
            to=cfg["max"],
            orient="horizontal",
            resolution=cfg["res"],
            variable=var,
            showvalue=0,
            length=200,
            command=lambda val, n=cfg["name"], t=cfg["type"]: 
                self.node.broadcast_parameter(n, val, t)
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

if __name__ == '__main__':
    main()