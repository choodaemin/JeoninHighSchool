import sys
import numpy as np
import pyqtgraph as pg
import math
import time
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import QThread, Signal
from rplidar import RPLidar, RPLidarException
from scipy.optimize import least_squares

# --- [설정값] ---
PORT_NAME = 'COM7'
BAUDRATE = 115200
MAX_DISTANCE_MM = 2500     
MOVE_THRESHOLD_MM = 25.0   # 필터를 조금 더 높여 안정성 확보
GRID_SIZE_MM = 50.0        

class NavigationThread(QThread):
    update_data = Signal(dict)

    def __init__(self):
        super().__init__()
        self.running = True
        self.state = np.array([0.0, 0.0, 0.0]) 
        self.prev_points = None
        self.path_history = [(0.0, 0.0)]

    def transform_points(self, points, x, y, theta):
        c, s = np.cos(theta), np.sin(theta)
        rot_matrix = np.array([[c, -s], [s, c]])
        return (points @ rot_matrix.T) + np.array([x, y])

    def scan_matching_odometry(self, current_points):
        if self.prev_points is None:
            self.prev_points = current_points
            return 0.0, 0.0, 0.0
        
        # [중요] 연산 부하를 줄이기 위해 샘플링 비율을 더 높임 (매 20번째 점 사용)
        curr_sample = current_points[::20] 
        prev_sample = self.prev_points[::10]

        def error_func(params):
            dx, dy, dtheta = params
            transformed = self.transform_points(curr_sample, dx, dy, dtheta)
            # 가장 가까운 점 찾기 연산 최적화
            dists = [np.min(np.linalg.norm(prev_sample - p, axis=1)) for p in transformed]
            return dists

        # ftol을 높여 계산 시간을 단축 (버퍼 밀림 방지 핵심)
        res = least_squares(error_func, [0, 0, 0], 
                            bounds=([-100, -100, -0.2], [100, 100, 0.2]), 
                            ftol=0.2, xtol=0.2)
        self.prev_points = current_points
        return res.x

    def run(self):
        while self.running:
            lidar = None
            try:
                lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)
                print("Lidar Connected. Starting scan...")
                
                # 데이터 읽기 시작
                for scan in lidar.iter_scans(max_buf_meas=500):
                    if not self.running: break
                    
                    points_local = []
                    for _, angle, dist in scan:
                        if 150 < dist < MAX_DISTANCE_MM:
                            rad = np.deg2rad(angle)
                            points_local.append([dist * np.sin(rad), dist * np.cos(rad)])

                    if len(points_local) < 15: continue
                    curr_points_np = np.array(points_local)

                    # 이동량 계산
                    dx, dy, dtheta = self.scan_matching_odometry(curr_points_np)
                    
                    dist_moved = math.sqrt(dx**2 + dy**2)
                    is_moving = dist_moved > MOVE_THRESHOLD_MM or abs(np.rad2deg(dtheta)) > 1.0

                    if is_moving:
                        c, s = np.cos(self.state[2]), np.sin(self.state[2])
                        self.state[0] += dx * c - dy * s
                        self.state[1] += dx * s + dy * c
                        self.state[2] += dtheta
                        self.path_history.append((self.state[0], self.state[1]))

                    global_points = self.transform_points(curr_points_np, self.state[0], self.state[1], self.state[2])
                    self.update_data.emit({
                        'map_x': global_points[:, 0], 'map_y': global_points[:, 1],
                        'path': self.path_history,
                        'robot_pos': self.state.copy(),
                        'moved': is_moving
                    })

            except (RPLidarException, Exception) as e:
                print(f"Communication Error ({e}). Reconnecting in 1s...")
                # if lidar:
                #     lidar.stop()
                #     lidar.disconnect()
                time.sleep(1) # 재연결 대기시간
            finally:    
                if lidar:
                    try:
                        lidar.stop()
                        lidar.disconnect()
                    except Exception:
                        pass

class MappingVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Perception System (Stable)")
        self.resize(1000, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.status_label = QLabel("Status: Connecting...")
        self.status_label.setStyleSheet("background: #222; color: #0f0; padding: 10px; font-family: Consolas;")
        layout.addWidget(self.status_label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.setBackground('k')
        layout.addWidget(self.plot_widget)

        self.map_scatter = pg.ScatterPlotItem(size=2, brush=pg.mkBrush(255, 255, 255, 80))
        self.curr_obs_scatter = pg.ScatterPlotItem(size=5, brush=pg.mkBrush(0, 255, 255, 255))
        self.path_line = self.plot_widget.plot(pen=pg.mkPen('g', width=2))
        self.robot_marker = pg.ScatterPlotItem(size=15, symbol='t', brush='y')

        for item in [self.map_scatter, self.curr_obs_scatter, self.robot_marker]:
            self.plot_widget.addItem(item)

        self.map_grid = {} 
        self.thread = NavigationThread()
        self.thread.update_data.connect(self.update_view)
        self.thread.start()

    def update_view(self, data):
        if data['moved']:
            new_x, new_y = [], []
            for x, y in zip(data['map_x'], data['map_y']):
                grid_key = (round(x / GRID_SIZE_MM), round(y / GRID_SIZE_MM))
                if grid_key not in self.map_grid:
                    self.map_grid[grid_key] = True
                    new_x.append(x)
                    new_y.append(y)
            if new_x:
                self.map_scatter.addPoints(x=new_x, y=new_y)

        self.curr_obs_scatter.setData(x=data['map_x'], y=data['map_y'])
        path_np = np.array(data['path'])
        self.path_line.setData(path_np[:, 0], path_np[:, 1])

        rx, ry, rt = data['robot_pos']
        self.robot_marker.setData(x=[rx], y=[ry])
        self.status_label.setText(f"POS: X={rx:.1f} Y={ry:.1f} | GRIDS: {len(self.map_grid)}")

    def closeEvent(self, event):
        self.thread.running = False
        self.thread.wait()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MappingVisualizer()
    window.show()
    sys.exit(app.exec())