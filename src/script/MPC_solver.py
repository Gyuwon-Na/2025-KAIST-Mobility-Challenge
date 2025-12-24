import numpy as np
from scipy.optimize import minimize

class MPCSolver:
    def __init__(self):
        self.dt = 0.1
        self.N = 10    # 예측 구간 (짧게 유지)
        
        # [수정됨] 맵이 작으므로 속도를 5.0 -> 1.0 (m/s)로 대폭 낮춤
        self.target_speed = 1.0 
        
        # 가중치 [Lat, Head, Vel, Steer]
        self.weights = [1.0, 1.0, 1.0, 0.1] 
        
        self.prev_v = 0.0
        self.prev_w = 0.0

    def set_strategy(self, mode='straight'):
        if mode == 'straight':
            self.weights = [0.5, 0.5, 2.0, 5.0]
            # 직선에서도 2.0 m/s를 넘지 않도록
            self.target_speed = 2.0 
        elif mode == 'curve':
            self.weights = [5.0, 2.0, 0.5, 0.1]
            # 커브에서는 아주 천천히
            self.target_speed = 0.8
        elif mode == 'intersection':
            self.weights = [1.0, 1.0, 1.5, 0.01]
            self.target_speed = 1.0

    def predict_state(self, state, v, w, dt):
        x, y, yaw = state
        x_new = x + v * np.cos(yaw) * dt
        y_new = y + v * np.sin(yaw) * dt
        yaw_new = yaw + w * dt
        return [x_new, y_new, yaw_new]

    def cost_function(self, u_flat, current_state, ref_path):
        u = u_flat.reshape((self.N, 2))
        cost = 0.0
        temp_state = list(current_state)

        for i in range(self.N):
            v, w = u[i]
            temp_state = self.predict_state(temp_state, v, w, self.dt)
            
            idx = min(i, len(ref_path)-1)
            ref_x, ref_y, ref_yaw = ref_path[idx]

            lat_err = np.sqrt((temp_state[0] - ref_x)**2 + (temp_state[1] - ref_y)**2)
            head_err = temp_state[2] - ref_yaw
            while head_err > np.pi: head_err -= 2*np.pi
            while head_err < -np.pi: head_err += 2*np.pi

            w_lat, w_head, w_vel, w_steer = self.weights
            
            cost += w_lat * (lat_err**2)
            cost += w_head * (head_err**2)
            cost += w_vel * ((v - self.target_speed)**2)
            cost += w_steer * (w**2)

        return cost

    def solve(self, current_state, ref_path_segment):
        u0 = np.array([self.target_speed, 0.0] * self.N)
        # [수정됨] 최대 속도 제한도 2.5 정도로 낮춤
        bounds = [(0.0, 2.5), (-2.0, 2.0)] * self.N

        try:
            res = minimize(self.cost_function, u0, args=(current_state, ref_path_segment),
                           method='SLSQP', bounds=bounds, options={'ftol': 1e-3, 'maxiter': 10})
            
            u_optimal = res.x.reshape((self.N, 2))
            v_mpc, w_mpc = u_optimal[0]

            pred_path = []
            temp_state = list(current_state)
            for i in range(self.N):
                v, w = u_optimal[i]
                temp_state = self.predict_state(temp_state, v, w, self.dt)
                pred_path.append([temp_state[0], temp_state[1]])
            
            return v_mpc, w_mpc, pred_path

        except Exception as e:
            print(f"[MPC Solver Error] {e}")
            return 0.0, 0.0, []

    def apply_smoothing(self, v_new, w_new):
        alpha = 0.8
        beta = 0.7
        v_final = alpha * v_new + (1 - alpha) * self.prev_v
        w_final = beta * w_new + (1 - beta) * self.prev_w
        self.prev_v = v_final
        self.prev_w = w_final
        return v_final, w_final