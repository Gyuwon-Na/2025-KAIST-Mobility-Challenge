import numpy as np
from scipy.optimize import minimize
import json
import os


class MPCSolver:
    def __init__(self):
        self.dt = 0.05
        self.N = 12
        self.Lf = 2.67

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(current_dir, "../mpc_config.json")

        self.load_config()

        self.prev_u = np.zeros(self.N * 2)
        self.prev_v = 0.0

    def load_config(self):
        try:
            with open(self.config_path, "r") as f:
                config = json.load(f)

            self.param_curve = {
                "w_cte": config["curve"]["w_cte"],
                "w_epsi": 0.0,  # 점 추종에서는 사용 안 함
                "w_v": 10.0,
                "w_delta": config["curve"]["w_delta"],
                "w_delta_d": config["curve"]["w_delta_d"],
                "speed": config["curve"]["speed"],
            }

            self.param_straight = {
                "w_cte": config["straight"]["w_cte"],
                "w_epsi": 0.0,
                "w_v": 50.0,
                "w_delta": 100.0,
                "w_delta_d": config["straight"]["w_delta_d"],
                "speed": config["straight"]["speed"],
            }
            print(f"[MPC] Configuration loaded successfully.")
        except Exception as e:
            print(f"[MPC Error] Config load failed: {e}")
            self.param_curve = {
                "w_cte": 50000,
                "w_epsi": 0,
                "w_v": 10,
                "w_delta": 0,
                "w_delta_d": 100,
                "speed": 0.4,
            }
            self.param_straight = {
                "w_cte": 20000,
                "w_epsi": 0,
                "w_v": 50,
                "w_delta": 100,
                "w_delta_d": 2000,
                "speed": 0.5,
            }

        self.set_mode("straight")

    def update_params(self, new_params):
        self.param_straight["w_cte"] = new_params.get(
            "s_cte", self.param_straight["w_cte"]
        )
        self.param_straight["w_delta_d"] = new_params.get(
            "s_dd", self.param_straight["w_delta_d"]
        )
        self.param_straight["speed"] = new_params.get(
            "s_speed", self.param_straight["speed"]
        )

        self.param_curve["w_cte"] = new_params.get("c_cte", self.param_curve["w_cte"])
        self.param_curve["w_delta"] = new_params.get(
            "c_delta", self.param_curve["w_delta"]
        )
        self.param_curve["w_delta_d"] = new_params.get(
            "c_dd", self.param_curve["w_delta_d"]
        )
        self.param_curve["speed"] = new_params.get("c_speed", self.param_curve["speed"])

    def set_mode(self, mode):
        p = self.param_curve if mode == "curve" else self.param_straight
        self.w_cte = p["w_cte"]
        self.w_v = p["w_v"]
        self.w_delta = p.get("w_delta", 100.0)
        self.w_delta_d = p["w_delta_d"]
        self.target_speed = p["speed"]

    def predict_state(self, state, delta, a, dt):
        x, y, psi, v = state
        v = max(0.1, v)
        x += v * np.cos(psi) * dt
        y += v * np.sin(psi) * dt
        psi += (v / self.Lf) * delta * dt
        v += a * dt
        return [x, y, psi, v]

    def cost_function(self, u_flat, initial_state, ref_traj):
        """
        ref_traj: 로컬 좌표계로 변환된 목표 점 리스트 (N개)
        """
        u = u_flat.reshape((self.N, 2))
        cost = 0.0
        state = list(initial_state)

        for i in range(self.N):
            delta, a = u[i]
            state = self.predict_state(state, delta, a, self.dt)
            px, py, psi, v = state

            # [핵심] 점과 점 사이의 거리 오차 (CTE 대체)
            ref_x, ref_y = ref_traj[i]
            dist_err = (px - ref_x) ** 2 + (py - ref_y) ** 2

            cost += self.w_cte * dist_err
            cost += self.w_v * (v - self.target_speed) ** 2
            cost += self.w_delta * (delta**2)

            if i > 0:
                cost += self.w_delta_d * (delta - u[i - 1, 0]) ** 2

        return float(cost)

    def solve(self, current_state, ref_traj):
        u0 = np.zeros(self.N * 2)
        u0[:-2] = self.prev_u[2:]
        u0[-2:] = self.prev_u[-2:]
        bounds = [(-1.2, 1.2), (-1.0, 1.0)] * self.N

        try:
            res = minimize(
                self.cost_function,
                u0,
                args=(current_state, ref_traj),
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": 20, "ftol": 1e-3},
            )
            self.prev_u = res.x
            u_opt = res.x.reshape((self.N, 2))

            delta_cmd = u_opt[0, 0]
            v_cmd = max(0.2, current_state[3] + u_opt[0, 1] * self.dt)
            w_cmd = v_cmd * np.tan(delta_cmd) / self.Lf

            pred_path = []
            temp = [0.0, 0.0, 0.0, current_state[3]]
            for i in range(self.N):
                temp = self.predict_state(temp, u_opt[i, 0], u_opt[i, 1], self.dt)
                pred_path.append([temp[0], temp[1]])

            return v_cmd, w_cmd, pred_path
        except Exception:
            self.prev_u = np.zeros(self.N * 2)
            return 0.2, 0.0, []
