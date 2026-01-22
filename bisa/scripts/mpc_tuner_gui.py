#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import cv2
import numpy as np

class MPCTunerGUI(Node):
    def __init__(self):
        super().__init__('mpc_tuner_gui')
        
        self.get_logger().info("===========================================")
        self.get_logger().info("MPC Parameter Tuner GUI v3.0")
        self.get_logger().info("Horizon + Lookahead Control")
        self.get_logger().info("===========================================")
        
        # 파라미터 클라이언트
        self.param_client = self.create_client(
            SetParameters, 
            '/mpc_path_tracker_cpp/set_parameters'
        )
        
        # 파라미터 초기값
        self.params = {
            'Q_pos': 954.0,
            'Q_heading': 956.0,
            'R_v': 0.6,
            'R_w': 0.6,
            'max_velocity': 1.2,
            'max_accel': 1.5,
            'max_angular_vel': 2.8,
            'horizon': 160
        }
        
        # 파라미터 범위 및 설명
        self.param_info = {
            'Q_pos': {
                'min': 1, 'max': 1000, 'scale': 10,
                'desc': 'Path Accuracy (↑=Stick to path)',
                'unit': '',
                'type': 'float'
            },
            'Q_heading': {
                'min': 1, 'max': 1000, 'scale': 10,
                'desc': 'Heading Accuracy (↑=Precise angle)',
                'unit': '',
                'type': 'float'
            },
            'R_v': {
                'min': 1, 'max': 50, 'scale': 10,
                'desc': 'Speed Smoothness (↑=Constant speed)',
                'unit': '',
                'type': 'float'
            },
            'R_w': {
                'min': 1, 'max': 50, 'scale': 10,
                'desc': 'Turn Smoothness (↑=Smooth turn)',
                'unit': '',
                'type': 'float'
            },
            'max_velocity': {
                'min': 10, 'max': 60, 'scale': 10,
                'desc': 'Top Speed Limit',
                'unit': 'm/s',
                'type': 'float'
            },
            'max_accel': {
                'min': 5, 'max': 60, 'scale': 10,
                'desc': 'Max Acceleration',
                'unit': 'm/s²',
                'type': 'float'
            },
            'max_angular_vel': {
                'min': 5, 'max': 60, 'scale': 10,
                'desc': 'Max Rotation Speed',
                'unit': 'rad/s',
                'type': 'float'
            },
            'horizon': {
                'min': 1, 'max': 3000, 'scale': 1,
                'desc': 'Prediction Steps (↓=Cut inside, ↑=Wide turn)',
                'unit': 'steps',
                'type': 'int'
            }
        }
        
        self.create_gui()
        
        # Timer for CV2 update
        self.timer = self.create_timer(0.05, self.update_gui)
        
        self.get_logger().info("✓ GUI Ready!")
    
    def create_gui(self):
        cv2.namedWindow('MPC Parameter Tuner', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('MPC Parameter Tuner', 1000, 750)
        
        for param_name, info in self.param_info.items():
            initial = int(self.params[param_name] * info['scale'])
            cv2.createTrackbar(
                param_name, 
                'MPC Parameter Tuner',
                initial,
                info['max'],
                lambda x, name=param_name: self.on_trackbar_change(name, x)
            )
    
    def on_trackbar_change(self, param_name, value):
        info = self.param_info[param_name]
        scale = info['scale']
        
        # 최소값 제한
        if value < info['min']:
            value = info['min']
            cv2.setTrackbarPos(param_name, 'MPC Parameter Tuner', value)
        
        # 값 업데이트
        if info['type'] == 'int':
            new_value = int(value)
        else:
            new_value = value / scale
        
        self.params[param_name] = new_value
        
        # ROS2 파라미터 설정
        self.set_ros_parameter(param_name, new_value)
        
        if info['type'] == 'int':
            self.get_logger().info(f"Updated {param_name} = {new_value}")
        else:
            self.get_logger().info(f"Updated {param_name} = {new_value:.1f}")
    
    def set_ros_parameter(self, name, value):
        if not self.param_client.wait_for_service(timeout_sec=0.1):
            return
        
        param = Parameter()
        param.name = name
        
        if self.param_info[name]['type'] == 'int':
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        else:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
        
        request = SetParameters.Request()
        request.parameters = [param]
        
        future = self.param_client.call_async(request)
    
    def update_gui(self):
        img = np.zeros((750, 1000, 3), dtype=np.uint8)
        
        # 제목
        cv2.putText(img, 'MPC Parameter Tuner v3.0', (20, 35), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        
        # Horizon 특별 표시
        horizon_val = int(self.params['horizon'])
        prediction_time = horizon_val * 0.001  # dt = 0.001
        cv2.putText(img, f'Horizon: {horizon_val} steps = {prediction_time*1000:.1f}ms prediction', 
                    (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
        
        # 현재 값
        y_offset = 110
        
        for param_name, value in self.params.items():
            info = self.param_info[param_name]
            
            # 값 표시
            if info['type'] == 'int':
                value_text = f"{int(value)}"
            else:
                value_text = f"{value:.1f}"
            
            # 파라미터 이름
            cv2.putText(img, f"{param_name}:", (20, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # 값 (강조)
            color = (0, 255, 0) if param_name != 'horizon' else (255, 150, 0)
            cv2.putText(img, value_text, (220, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            
            # 단위
            if info['unit']:
                cv2.putText(img, info['unit'], (300, y_offset), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # 설명
            cv2.putText(img, info['desc'], (400, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)
            
            y_offset += 75
        
        # 하단 가이드
        y_offset += 10
        cv2.line(img, (20, y_offset), (980, y_offset), (100, 100, 100), 1)
        y_offset += 30
        
        cv2.putText(img, 'Understanding Horizon:', (20, y_offset), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)
        y_offset += 30
        
        tips = [
            "Horizon = How many steps MPC predicts into future (Integer only!)",
            "  Small (5-10):  Short prediction -> Cut inside corners (aggressive)",
            "  Large (20-30): Long prediction  -> Wide smooth turns (safe)",
            "",
            "Similar to Lookahead Distance but in TIME steps, not meters!",
            "",
            "Press ESC or Q to close"
        ]
        
        for tip in tips:
            if tip == "":
                y_offset += 15
            else:
                cv2.putText(img, tip, (30, y_offset), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
                y_offset += 22
        
        cv2.imshow('MPC Parameter Tuner', img)
        
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            self.get_logger().info("Closing GUI...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MPCTunerGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
