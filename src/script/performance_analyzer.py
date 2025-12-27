import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import shutil
import glob
import argparse

# 경로 설정
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "../log"))
PATH_DIR = os.path.normpath(os.path.join(CURRENT_DIR, "../path"))
CURRENT_LOG_FILE = os.path.join(LOG_DIR, "trajectory_log.csv")
GLOBAL_PATH_FILE = os.path.join(PATH_DIR, "problem1-1_CAV1.csv")
SAVED_LOGS_DIR = os.path.join(LOG_DIR, "saved_runs")


class PerformanceAnalyzer:
    def __init__(self):
        self.global_path = self.load_csv(GLOBAL_PATH_FILE)[:, :2]  # x, y only
        if not os.path.exists(SAVED_LOGS_DIR):
            os.makedirs(SAVED_LOGS_DIR)

    def load_csv(self, filepath):
        data = []
        try:
            with open(filepath, "r") as f:
                reader = csv.reader(f)
                next(reader, None)  # Skip Header
                for row in reader:
                    if row and len(row) >= 2:
                        try:
                            # x, y, mode(옵션)
                            data.append([float(row[0]), float(row[1])])
                        except:
                            pass
            return np.array(data)
        except Exception as e:
            print(f"[Err] Failed to load {filepath}: {e}")
            return np.array([])

    def calculate_metrics(self, traj_data):
        """
        주행 데이터를 분석하여 성적표를 만듭니다.
        """
        if len(traj_data) == 0:
            return None

        # 1. CTE (Cross Track Error) 계산
        # 주행 경로의 각 점마다 글로벌 경로에서 가장 가까운 점까지의 거리를 구함
        cte_list = []
        for pt in traj_data:
            dists = np.linalg.norm(self.global_path - pt, axis=1)
            min_dist = np.min(dists)
            cte_list.append(min_dist)

        cte_arr = np.array(cte_list)

        # 지표 1: RMSE (평균 오차)
        rmse = np.sqrt(np.mean(cte_arr**2))

        # 지표 2: Max Error (최대 이탈)
        max_error = np.max(cte_arr)

        # 지표 3: Oscillation Score (진동 점수)
        # CTE의 변화량(미분값)의 합 -> 차가 좌우로 많이 움직일수록 커짐
        oscillation = np.sum(np.abs(np.diff(cte_arr))) / len(cte_arr) * 100
        # *100은 보기 편하게 스케일링

        return {
            "rmse": rmse,
            "max_error": max_error,
            "oscillation": oscillation,
            "steps": len(traj_data),
        }

    def save_current_run(self, name):
        """현재 로그를 이름과 함께 저장"""
        if not os.path.exists(CURRENT_LOG_FILE):
            print("❌ 현재 로그 파일이 없습니다. 주행을 먼저 하세요.")
            return

        target_path = os.path.join(SAVED_LOGS_DIR, f"{name}.csv")
        shutil.copy(CURRENT_LOG_FILE, target_path)
        print(f"✅ 현재 주행 기록이 '{name}'(으)로 저장되었습니다.")

        # 저장 후 바로 분석 결과 출력
        self.analyze_run(target_path, name)

    def analyze_run(self, filepath, name):
        data = self.load_csv(filepath)
        metrics = self.calculate_metrics(data)
        if metrics:
            print(f"\n📊 [Analysis Result: {name}]")
            print(f"   - 📏 RMSE (정확도): {metrics['rmse']:.4f} m (낮을수록 좋음)")
            print(f"   - ⚠️ Max Error (최대이탈): {metrics['max_error']:.4f} m")
            print(
                f"   - 〰️ Oscillation (진동): {metrics['oscillation']:.4f} 점 (낮을수록 부드러움)"
            )
            print(f"   - ⏱️ Data Points: {metrics['steps']}")
        else:
            print(f"⚠️ 데이터가 부족하여 분석할 수 없습니다: {name}")
        return metrics, data

    def compare_all(self):
        """저장된 모든 로그를 비교"""
        log_files = glob.glob(os.path.join(SAVED_LOGS_DIR, "*.csv"))
        if not log_files:
            print("❌ 저장된 기록이 없습니다. 'save' 명령어로 기록을 먼저 남기세요.")
            return

        plt.figure(figsize=(12, 10))
        plt.plot(
            self.global_path[:, 0],
            self.global_path[:, 1],
            "k--",
            linewidth=2,
            label="Global Path",
            alpha=0.3,
        )

        results = []
        print(f"\n🏆 [Performance Leaderboard]")
        print(f"{'Name':<20} | {'RMSE':<10} | {'Max Err':<10} | {'Oscillation':<10}")
        print("-" * 60)

        for log in log_files:
            name = os.path.basename(log).replace(".csv", "")
            metrics, data = self.analyze_run(log, name)  # Prints individual details
            if metrics:
                results.append((name, metrics, data))

        # 테이블 정렬 출력
        # RMSE 기준으로 정렬
        results.sort(key=lambda x: x[1]["rmse"])

        for name, m, data in results:
            print(
                f"{name:<20} | {m['rmse']:.4f} m   | {m['max_error']:.4f} m   | {m['oscillation']:.4f}"
            )
            plt.plot(
                data[:, 0],
                data[:, 1],
                label=f"{name} (Err: {m['rmse']:.3f}m)",
                linewidth=1.5,
            )

        plt.title("Trajectory Comparison")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid(True, which="both", linestyle="--", alpha=0.5)
        plt.axis("equal")

        print("\n📈 비교 그래프를 표시합니다...")
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MPC Performance Analyzer")
    subparsers = parser.add_subparsers(dest="command")

    # Save command
    save_parser = subparsers.add_parser("save", help="Save current run")
    save_parser.add_argument(
        "name", type=str, help="Name of the configuration (e.g. 'cte_1000')"
    )

    # Compare command
    compare_parser = subparsers.add_parser("compare", help="Compare all saved runs")

    args = parser.parse_args()

    analyzer = PerformanceAnalyzer()

    if args.command == "save":
        analyzer.save_current_run(args.name)
    elif args.command == "compare":
        analyzer.compare_all()
    else:
        parser.print_help()
