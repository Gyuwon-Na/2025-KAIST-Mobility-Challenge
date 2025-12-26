import numpy as np
from scipy.optimize import minimize


class MPCSolver:
    def __init__(self):
        self.dt = 0.05
        self.N = 12
        self.Lf = 2.67

        # [튜닝용 변수 저장소] 초기값 설정
        self.param_straight = {
            "w_cte": 20000.0,
            "w_epsi": 20000.0,
            "w_v": 50.0,
            "w_delta": 2000.0,
            "w_delta_d": 2000.0,
            "speed": 0.5,
        }
        self.param_curve = {
            "w_cte": 50000.0,
            "w_epsi": 30000.0,
            "w_v": 10.0,
            "w_delta": 0.0,
            "w_delta_d": 0.0,
            "speed": 0.4,
        }

        # 현재 적용된 파라미터
        self.target_speed = 0.5
        self.w_cte = 20000.0
        self.w_epsi = 20000.0
        self.w_v = 50.0
        self.w_delta = 100.0
        self.w_delta_d = 2000.0

        self.prev_u = np.zeros(self.N * 2)
        self.prev_v = 0.0

    def update_params(self, new_params):
        """Tuner 노드에서 받은 파라미터로 덮어쓰기"""
        # Straight 파라미터 업데이트
        self.param_straight["w_cte"] = new_params.get(
            "s_cte", self.param_straight["w_cte"]
        )
        self.param_straight["w_delta_d"] = new_params.get(
            "s_dd", self.param_straight["w_delta_d"]
        )
        self.param_straight["speed"] = new_params.get(
            "s_speed", self.param_straight["speed"]
        )

        # Curve 파라미터 업데이트
        self.param_curve["w_cte"] = new_params.get("c_cte", self.param_curve["w_cte"])
        self.param_curve["w_delta"] = new_params.get(
            "c_delta", self.param_curve["w_delta"]
        )
        self.param_curve["w_delta_d"] = new_params.get(
            "c_dd", self.param_curve["w_delta_d"]
        )
        self.param_curve["speed"] = new_params.get("c_speed", self.param_curve["speed"])

    def set_mode(self, mode):
        """현재 모드에 맞춰 파라미터 적용"""
        if mode == "curve":
            p = self.param_curve
        else:
            p = self.param_straight

        self.w_cte = p["w_cte"]
        self.w_epsi = p.get("w_epsi", self.w_cte)  # epsi는 cte와 비슷하게 따라감
        self.w_v = p["w_v"]
        self.w_delta = p["w_delta"]
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

    def cost_function(self, u_flat, initial_state, coeffs):
        u = u_flat.reshape((self.N, 2))
        cost = 0.0
        state = list(initial_state)

        for i in range(self.N):
            delta, a = u[i]
            state = self.predict_state(state, delta, a, self.dt)
            px, py, psi, v = state

            f_x = np.polyval(coeffs, px)
            f_prime = 3 * coeffs[0] * px**2 + 2 * coeffs[1] * px + coeffs[2]
            psides = np.arctan(f_prime)

            cost += self.w_cte * (f_x - py) ** 2
            cost += self.w_epsi * (psi - psides) ** 2
            cost += self.w_v * (v - self.target_speed) ** 2
            cost += self.w_delta * (delta**2)

            if i > 0:
                cost += self.w_delta_d * (delta - u[i - 1, 0]) ** 2

        return float(cost)

    def solve(self, current_state, coeffs):
        u0 = np.zeros(self.N * 2)
        u0[:-2] = self.prev_u[2:]
        u0[-2:] = self.prev_u[-2:]
        bounds = [(-1.2, 1.2), (-0.5, 0.5)] * self.N

        try:
            res = minimize(
                self.cost_function,
                u0,
                args=(current_state, coeffs),
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": 30, "ftol": 1e-4},
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
        except Exception as e:
            self.prev_u = np.zeros(self.N * 2)
            return 0.2, 0.0, []
