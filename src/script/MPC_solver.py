import numpy as np
from scipy.optimize import minimize

class MPCSolver:
    def __init__(self):
        # 제어 주기를 0.02s(50Hz)로 촘촘하게 설정하여 반응 속도 극대화
        self.dt = 0.05 
        self.N = 12          # 약 0.6초 앞을 예측 (급커브 대응용)
        self.Lf = 2.67       # 차량 축거
        self.target_speed = 0.5  # 코너 통과를 위해 속도를 낮춤
        
        # [가중치 재설정] 조향 방해 요소 제거
        self.w_cte = 20000.0   # 경로 이탈 벌점 (최상)
        self.w_epsi = 20000.0  # 방향 오차 벌점 (최상)
        self.w_v = 10.0
        self.w_delta = 0.0     # 조향 사용 벌점 제거 (마음껏 꺾어라)
        self.w_delta_d = 0.0   # 조향 변화 벌점 제거 (최대한 빠르게 꺾어라)
        
        self.prev_v = 0.0
        self.prev_w = 0.0

    def predict_state(self, state, delta, a, dt):
        x, y, psi, v = state
        v = max(0.1, v) # 전진 속도 보장
        x += v * np.cos(psi) * dt
        y += v * np.sin(psi) * dt
        psi += (v / self.Lf) * delta * dt
        v += a * dt
        return [x, y, psi, v]

    def cost_function(self, u_flat, initial_state, coeffs):
        u = u_flat.reshape((self.N, 2))
        cost = 0.0
        # initial_state는 이제 무조건 [0, 0, 0, v] 임
        state = list(initial_state) 

        for i in range(self.N):
            delta, a = u[i]
            state = self.predict_state(state, delta, a, self.dt)
            px, py, psi, v = state

            # 수식 f(x)를 통한 오차 계산 (이미 수평으로 정렬된 coeffs 사용)
            f_x = np.polyval(coeffs, px)
            f_prime = 3*coeffs[0]*px**2 + 2*coeffs[1]*px + coeffs[2]
            psides = np.arctan(f_prime)

            # CTE(거리오차)와 EPSI(각도오차)
            cost += self.w_cte * (f_x - py)**2
            cost += self.w_epsi * (psi - psides)**2
            cost += self.w_v * (v - self.target_speed)**2
            cost += self.w_delta * (delta**2)
            
            if i > 0:
                cost += self.w_delta_d * (delta - u[i-1, 0])**2

        return float(cost)

    def solve(self, current_state, coeffs):
        # 초기값: 현재 헤딩 방향으로 전진 시도
        u0 = np.zeros(self.N * 2)
        # 조향 한계를 1.2 rad(약 68도)까지 대폭 확대하여 물리적 최대 조향 유도
        bounds = [(-1.2, 1.2), (-0.5, 0.5)] * self.N 
        
        try:
            res = minimize(self.cost_function, u0, args=(current_state, coeffs),
                           method='SLSQP', bounds=bounds, options={'maxiter': 50})
            u_opt = res.x.reshape((self.N, 2))
            
            delta_cmd = u_opt[0, 0]
            v_cmd = max(0.2, current_state[3] + u_opt[0, 1] * self.dt)
            
            # 각속도 w 계산: 시뮬레이터가 인식할 수 있는 최종 값
            w_cmd = v_cmd * np.tan(delta_cmd) / self.Lf
            
            pred_path = []
            temp = [0.0, 0.0, 0.0, current_state[3]]
            for i in range(self.N):
                temp = self.predict_state(temp, u_opt[i, 0], u_opt[i, 1], self.dt)
                pred_path.append([temp[0], temp[1]])
                
            return v_cmd, w_cmd, pred_path
        except:
            return 0.2, 0.0, []