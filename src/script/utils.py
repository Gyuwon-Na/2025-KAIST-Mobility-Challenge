import csv
import math
import numpy as np


def load_path_csv(file_path):
    """CSV 파일을 로드하고 각 점에서의 Yaw(방향)를 계산합니다."""
    path_points = []
    try:
        with open(file_path, "r", newline="") as f:
            reader = csv.reader(f)
            first_row = next(reader, None)
            temp_xy = []
            if first_row:
                try:
                    temp_xy.append([float(first_row[0]), float(first_row[1])])
                except ValueError:
                    pass
            for row in reader:
                if not row or len(row) < 2:
                    continue
                try:
                    temp_xy.append([float(row[0]), float(row[1])])
                except ValueError:
                    continue

        np_xy = np.array(temp_xy)
        if len(np_xy) == 0:
            return np.array([])

        final_path = []
        for i in range(len(np_xy)):
            x, y = np_xy[i]
            if i < len(np_xy) - 1:
                dx = np_xy[i + 1][0] - x
                dy = np_xy[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = final_path[-1][2] if final_path else 0.0
            final_path.append([x, y, yaw])
        return np.array(final_path)
    except Exception as e:
        print(f"[ERROR] CSV Load Error: {e}")
        return np.array([])


def euler_from_quaternion(q):
    """Quaternion 데이터를 Yaw 각도로 변환합니다."""
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm == 0:
        return 0.0
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def global_to_local(ego_pose, global_points):
    """
    차량의 현재 위치와 방향을 기준으로 글로벌 좌표를 로컬 좌표로 변환합니다.
    이 과정을 통해 지도가 차량 정면을 향하도록 회전합니다.
    """
    rx, ry, ryaw = ego_pose
    c, s = np.cos(ryaw), np.sin(ryaw)

    local_points = []
    for pt in global_points:
        dx = pt[0] - rx
        dy = pt[1] - ry
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        local_points.append([lx, ly])

    return np.array(local_points)
