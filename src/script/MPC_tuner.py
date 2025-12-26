import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import json
import threading


class MPCTuner(Node):
    def __init__(self):
        super().__init__("mpc_tuner")
        self.pub = self.create_publisher(String, "/mpc_params", 10)

        # GUI 설정 (메인 스레드에서 실행됨)
        self.root = tk.Tk()
        self.root.title("MPC Real-time Tuner")
        self.root.geometry("400x650")

        # === Curve Mode Parameters ===
        tk.Label(
            self.root, text="=== [Curve Mode] ===", font=("bold", 12), fg="blue"
        ).pack(pady=5)
        self.c_speed = self.create_slider("Target Speed", 0.1, 1.5, 0.4, 0.1)
        self.c_cte = self.create_slider("W_CTE (Path Fit)", 1000, 100000, 50000, 1000)
        self.c_delta = self.create_slider("W_Delta (Steer Cost)", 0, 1000, 0, 10)
        self.c_dd = self.create_slider("W_Delta_D (Smoothness)", 0, 5000, 0, 10)
        self.c_look = self.create_slider("Look Ahead (pts)", 10, 80, 35, 1)

        # === Straight Mode Parameters ===
        tk.Label(
            self.root, text="=== [Straight Mode] ===", font=("bold", 12), fg="black"
        ).pack(pady=10)
        self.s_speed = self.create_slider("Target Speed", 0.1, 2.0, 0.5, 0.1)
        self.s_cte = self.create_slider("W_CTE", 1000, 50000, 20000, 1000)
        self.s_dd = self.create_slider("W_Delta_D (Stability)", 0, 10000, 2000, 100)
        self.s_look = self.create_slider("Look Ahead (pts)", 30, 100, 70, 1)

        # [수정 포인트] ROS 타이머 대신 Tkinter 타이머 사용
        # 100ms마다 self.tk_publish_loop 함수를 실행 (메인 스레드 유지)
        self.root.after(100, self.tk_publish_loop)

    def create_slider(self, label, min_val, max_val, init_val, step):
        frame = tk.Frame(self.root)
        frame.pack(fill="x", padx=10, pady=2)
        tk.Label(frame, text=label, width=20, anchor="w").pack(side="left")
        var = tk.DoubleVar(value=init_val)
        scale = tk.Scale(
            frame,
            variable=var,
            from_=min_val,
            to=max_val,
            resolution=step,
            orient="horizontal",
        )
        scale.pack(side="right", expand=True, fill="x")
        return var

    def tk_publish_loop(self):
        """메인 스레드에서 실행되는 루프"""
        self.publish_params()
        # 100ms 후에 다시 이 함수를 호출 (재귀적 스케줄링)
        self.root.after(100, self.tk_publish_loop)

    def publish_params(self):
        # 여기서 get()을 호출해도 메인 스레드이므로 안전함
        data = {
            # Curve
            "c_speed": self.c_speed.get(),
            "c_cte": self.c_cte.get(),
            "c_delta": self.c_delta.get(),
            "c_dd": self.c_dd.get(),
            "c_look": int(self.c_look.get()),
            # Straight
            "s_speed": self.s_speed.get(),
            "s_cte": self.s_cte.get(),
            "s_dd": self.s_dd.get(),
            "s_look": int(self.s_look.get()),
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTuner()

    # [수정 포인트] ROS 통신을 위한 스핀을 별도 스레드로 분리
    # GUI가 멈추지 않도록 ROS는 백그라운드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # [수정 포인트] GUI는 반드시 메인 스레드에서 실행
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
