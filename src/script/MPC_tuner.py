import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import messagebox
import json
import threading
import os


class MPCTuner(Node):
    def __init__(self):
        super().__init__("mpc_tuner")
        self.pub = self.create_publisher(String, "/mpc_params", 10)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(current_dir, "mpc_config.json")

        self.root = tk.Tk()
        self.root.title("MPC Config Manager")
        self.root.geometry("400x700")

        self.config_data = self.load_config()

        # Curve UI
        tk.Label(
            self.root, text="=== [Curve Mode] ===", font=("bold", 12), fg="blue"
        ).pack(pady=5)
        self.c_speed = self.create_slider(
            "Speed", 0.1, 1.5, self.config_data["curve"]["speed"], 0.1
        )
        self.c_cte = self.create_slider(
            "W_CTE", 1000, 100000, self.config_data["curve"]["w_cte"], 1000
        )
        self.c_delta = self.create_slider(
            "W_Delta", 0, 1000, self.config_data["curve"]["w_delta"], 10
        )
        self.c_dd = self.create_slider(
            "W_Delta_D", 0, 5000, self.config_data["curve"]["w_delta_d"], 10
        )
        self.c_look = self.create_slider(
            "Look Ahead", 10, 80, self.config_data["curve"]["look_ahead"], 1
        )

        # Straight UI
        tk.Label(
            self.root, text="=== [Straight Mode] ===", font=("bold", 12), fg="black"
        ).pack(pady=10)
        self.s_speed = self.create_slider(
            "Speed", 0.1, 2.0, self.config_data["straight"]["speed"], 0.1
        )
        self.s_cte = self.create_slider(
            "W_CTE", 1000, 50000, self.config_data["straight"]["w_cte"], 1000
        )
        self.s_dd = self.create_slider(
            "W_Delta_D", 0, 10000, self.config_data["straight"]["w_delta_d"], 100
        )
        self.s_look = self.create_slider(
            "Look Ahead", 30, 100, self.config_data["straight"]["look_ahead"], 1
        )

        # Save Button
        tk.Button(
            self.root,
            text="💾 SAVE CONFIG 💾",
            command=self.save_config,
            bg="#4CAF50",
            fg="white",
            font=("bold", 12),
            height=2,
        ).pack(pady=20, fill="x", padx=20)

        self.root.after(100, self.tk_publish_loop)

    def load_config(self):
        try:
            with open(self.config_path, "r") as f:
                return json.load(f)
        except:
            return {
                "curve": {
                    "speed": 0.4,
                    "w_cte": 50000,
                    "w_delta": 0,
                    "w_delta_d": 100,
                    "look_ahead": 35,
                },
                "straight": {
                    "speed": 0.5,
                    "w_cte": 20000,
                    "w_delta_d": 2000,
                    "look_ahead": 70,
                },
            }

    def save_config(self):
        data = {
            "curve": {
                "speed": self.c_speed.get(),
                "w_cte": self.c_cte.get(),
                "w_delta": self.c_delta.get(),
                "w_delta_d": self.c_dd.get(),
                "look_ahead": int(self.c_look.get()),
            },
            "straight": {
                "speed": self.s_speed.get(),
                "w_cte": self.s_cte.get(),
                "w_delta_d": self.s_dd.get(),
                "look_ahead": int(self.s_look.get()),
            },
        }
        try:
            with open(self.config_path, "w") as f:
                json.dump(data, f, indent=4)
            messagebox.showinfo("Success", "Configuration Saved!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save: {e}")

    def create_slider(self, label, min_val, max_val, init_val, step):
        frame = tk.Frame(self.root)
        frame.pack(fill="x", padx=10, pady=2)
        tk.Label(frame, text=label, width=15, anchor="w").pack(side="left")
        var = tk.DoubleVar(value=init_val)
        tk.Scale(
            frame,
            variable=var,
            from_=min_val,
            to=max_val,
            resolution=step,
            orient="horizontal",
        ).pack(side="right", expand=True, fill="x")
        return var

    def tk_publish_loop(self):
        data = {
            "c_speed": self.c_speed.get(),
            "c_cte": self.c_cte.get(),
            "c_delta": self.c_delta.get(),
            "c_dd": self.c_dd.get(),
            "c_look": int(self.c_look.get()),
            "s_speed": self.s_speed.get(),
            "s_cte": self.s_cte.get(),
            "s_dd": self.s_dd.get(),
            "s_look": int(self.s_look.get()),
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)
        self.root.after(100, self.tk_publish_loop)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTuner()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
