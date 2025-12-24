import csv
import math
import numpy as np

def load_path_csv(file_path):
    """CSV 로드 및 Yaw 각도 계산 (x, y, yaw)"""
    path_points = []
    try:
        # newline='' 옵션 추가 (줄바꿈 호환성 강화)
        with open(file_path, 'r', newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            temp_xy = []
            for row in reader:
                if not row or len(row) < 2: continue
                try:
                    s_x = row[0].strip()
                    s_y = row[1].strip()
                    if not s_x or not s_y: continue
                    
                    x, y = float(s_x), float(s_y)
                    if math.isnan(x) or math.isnan(y): continue
                    temp_xy.append([x, y])
                except ValueError: continue
        
        np_xy = np.array(temp_xy)
        if len(np_xy) == 0: return np.array([])

        for i in range(len(np_xy)):
            x, y = np_xy[i]
            if i < len(np_xy) - 1:
                dx = np_xy[i+1][0] - x
                dy = np_xy[i+1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = path_points[-1][2] if path_points else 0.0
            path_points.append([x, y, yaw])
            
        return np.array(path_points)
    except Exception as e:
        print(f"[ERROR] CSV Load Error: {e}")
        return np.array([])

def euler_from_quaternion(q):
    """
    Quaternion -> Yaw 변환 (강제 정규화 포함)
    시뮬레이터가 비정상적인 쿼터니언(크기 > 1)을 보낼 때를 대비합니다.
    """
    # 1. 쿼터니언 정규화 (Normalization)
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm == 0:
        return 0.0
    
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm

    # 2. Yaw(Z-axis rotation) 계산
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z