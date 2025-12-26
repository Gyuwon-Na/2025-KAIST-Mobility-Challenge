import numpy as np
from scipy.optimize import minimize


class MPCSolver:
    def __init__(self):
        self.dt = 0.05
        self.N = 12
        self.Lf = 2.67
        self.target_speed = 0.5

        # [수정 1] 오실레이션 방지를 위한 가중치 튜닝
        self.w_cte = 15000.0
        self.w_epsi = 15000.0
        self.w_v = 50.0
        self.w_delta = 100.0
        self.w_delta_d = 5000.0  # [핵심] 핸들 급조작 방지 -> 오실레이션 억제

        # [수정 2] Warm Start용 변수
        self.prev_u = np.zeros(self.N * 2)

        # [에러 수정] 누락되었던 속도 저장 변수 복구
        self.prev_v = 0.0

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
        # [수정 3] Warm Start: 이전 해를 Shift하여 초기값으로 사용
        u0 = np.zeros(self.N * 2)
        u0[:-2] = self.prev_u[2:]
        u0[-2:] = self.prev_u[-2:]

        bounds = [(-1.0, 1.0), (-0.5, 0.5)] * self.N

        try:
            res = minimize(
                self.cost_function,
                u0,
                args=(current_state, coeffs),
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": 30, "ftol": 1e-4},
            )

            # [수정 4] 현재 해를 저장 (다음 스텝 Warm Start용)
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
            print(f"[MPC Error] {e}")
            self.prev_u = np.zeros(self.N * 2)
            return 0.2, 0.0, []
