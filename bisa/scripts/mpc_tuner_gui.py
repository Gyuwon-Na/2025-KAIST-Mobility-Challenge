#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
import threading

class MPCTunerGUI(Node):
    def __init__(self):
        super().__init__('mpc_tuner_gui')
        
        # 1. 타겟 노드 설정 (mpc_path_tracker의 파라미터를 바꿈)
        self.target_node = 'mpc_path_tracker'
        self.client = self.create_client(SetParameters, f'/{self.target_node}/set_parameters')
        
        # 서비스가 준비될 때까지 대기 (타임아웃 설정)
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Service /{self.target_node}/set_parameters not available.')
        else:
            self.get_logger().info(f'Connected to /{self.target_node}/set_parameters')

    def send_parameter(self, name, value, param_type):
        """ROS 2 파라미터 업데이트 요청 전송"""
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        
        # 타입에 따른 값 설정
        if param_type == int:
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        else:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
            
        req.parameters = [param]
        
        # 비동기 호출 (GUI 멈춤 방지)
        future = self.client.call_async(req)
        future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        try:
            result = future.result()
            # 성공 여부 확인 (필요시 로그 출력)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("MPC Real-time Tuner")
        self.root.geometry("550x700")
        
        # 파라미터 정의 (이름, 최소값, 최대값, 초기값, 해상도, 타입)
        self.params_config = [
            # --- MPC 제어 설정 ---
            {'name': 'horizon',             'min': 5,   'max': 100,  'init': 30,   'res': 1,    'type': int},
            {'name': 'dt',                  'min': 0.01,'max': 0.2,  'init': 0.02, 'res': 0.005,'type': float},
            {'name': 'lookahead_base',      'min': 0.1, 'max': 5.0,  'init': 0.3,  'res': 0.1,  'type': float},
            {'name': 'lookahead_per_speed', 'min': 0.0, 'max': 2.0,  'init': 0.2,  'res': 0.05, 'type': float},
            
            # --- 가중치 설정 (Weights) ---
            {'name': 'Q_pos',               'min': 0.0, 'max': 100.0,'init': 20.0, 'res': 0.5,  'type': float},
            {'name': 'Q_heading',           'min': 0.0, 'max': 100.0,'init': 10.0, 'res': 0.5,  'type': float},
            {'name': 'R_v',                 'min': 0.0, 'max': 10.0, 'init': 0.5,  'res': 0.1,  'type': float},
            {'name': 'R_w',                 'min': 0.0, 'max': 10.0, 'init': 0.5,  'res': 0.1,  'type': float},
            
            # --- 차량 제약 조건 (Constraints) ---
            {'name': 'max_velocity',        'min': 0.0, 'max': 10.0, 'init': 3.0,  'res': 0.1,  'type': float},
            {'name': 'max_accel',           'min': 0.0, 'max': 10.0, 'init': 2.0,  'res': 0.1,  'type': float},
            {'name': 'max_angular_vel',     'min': 0.0, 'max': 5.0,  'init': 2.0,  'res': 0.1,  'type': float},
        ]

        # UI 생성
        self.vars = {}
        self.create_widgets()

    def create_widgets(self):
        # 타이틀
        lbl_title = tk.Label(self.root, text="BISA MPC Parameters", font=("Arial", 14, "bold"))
        lbl_title.pack(pady=10)

        # 스크롤 가능한 프레임 (파라미터가 많으므로)
        canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # 각 파라미터별 위젯 생성
        for cfg in self.params_config:
            self.create_param_row(scrollable_frame, cfg)

    def create_param_row(self, parent, cfg):
        name = cfg['name']
        p_type = cfg['type']
        
        # 프레임 (한 줄)
        frame = tk.Frame(parent, pady=5)
        frame.pack(fill="x", padx=10)
        
        # 1. 라벨 (이름)
        lbl = tk.Label(frame, text=name, width=18, anchor='w', font=("Consolas", 10))
        lbl.pack(side="left")

        # 변수 생성 (DoubleVar 또는 IntVar)
        if p_type == int:
            var = tk.IntVar(value=cfg['init'])
        else:
            var = tk.DoubleVar(value=cfg['init'])
        
        self.vars[name] = var

        # 2. 엔트리 (숫자 입력)
        entry = tk.Entry(frame, textvariable=var, width=6, justify='right')
        entry.pack(side="left", padx=5)
        
        # 엔트리에서 Enter 키 누르면 업데이트
        entry.bind('<Return>', lambda event, n=name, t=p_type: self.on_entry_update(n, t))

        # 3. 슬라이더 (트랙바)
        scale = tk.Scale(frame, from_=cfg['min'], to=cfg['max'], 
                         orient="horizontal", resolution=cfg['res'],
                         variable=var, showvalue=0, length=200,
                         command=lambda v, n=name, t=p_type: self.on_slider_update(v, n, t))
        scale.pack(side="left", fill="x", expand=True)

    def on_slider_update(self, value, name, p_type):
        """슬라이더를 움직일 때 호출"""
        # 즉시 반영
        self.node.send_parameter(name, value, p_type)

    def on_entry_update(self, name, p_type):
        """엔트리에 숫자를 쓰고 엔터를 쳤을 때 호출"""
        try:
            val = self.vars[name].get()
            self.node.send_parameter(name, val, p_type)
        except Exception:
            pass # 숫자가 아닌 값 입력 시 무시


def ros_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = MPCTunerGUI()
    
    # ROS 통신을 별도 스레드에서 실행 (GUI 멈춤 방지)
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