"""
mapper.py
OAK-D-Lite + LiDAR 데이터를 누적하여 2D 맵을 생성

- LiDAR: 360도 벽/장애물 포인트 → 맵에 누적
- OAK-D-Lite: 정면 깊이 정보 → 맵에 보완
- 로봇이 이동하면 맵도 함께 업데이트 (단순 누적 방식)
"""

import numpy as np  # NumPy 라이브러리 (배열 계산)
import math  # 수학 함수 (삼각함수 등)
import time  # 시간 관련 함수 (타임스탬프)
from dataclasses import dataclass, field  # 데이터 클래스 정의
from typing import List, Tuple, Optional  # 타입 힌트
import config  # 설정 파일에서 상수 가져옴


# 셀 상태값 상수 정의
CELL_UNKNOWN    = 0.0  # 미지 영역
CELL_FREE       = 0.3  # 빈 공간
CELL_WALL       = 0.8  # 벽
CELL_OBSTACLE   = 1.0  # 장애물
CELL_GLASS_WALL = 0.6   # 유리벽/투명 장애물


# 맵 셀을 나타내는 데이터 클래스
@dataclass
class MapCell:
    value: float = CELL_UNKNOWN     # 현재 셀 상태 값
    hit_count: int = 0              # 장애물로 감지된 횟수
    free_count: int = 0             # 빈 공간으로 감지된 횟수
    last_updated: float = 0.0       # 마지막 업데이트 시간


# 2D 점유 격자 맵을 관리하는 클래스
class OccupancyMap:
    """
    2D 점유 격자 맵.
    맵 자체는 월드 좌표계에 고정되고, 로봇은 set_robot_pose()로 전달받은
    오도메트리를 따라 맵 위를 이동하며, 센서 데이터가 그 위치 기준으로 누적된다.
    """

    def __init__(self):
        # 맵 해상도 설정 (미터/셀)
        self.resolution = config.FUSION_GRID_RESOLUTION
        # 맵 크기 설정 (미터)
        self.width_m    = config.FUSION_GRID_WIDTH_M   # 맵 가로 크기 (m)
        self.height_m   = config.FUSION_GRID_HEIGHT_M  # 맵 세로 크기 (m)

        # 셀 단위 크기 계산
        self.width_cells  = int(self.width_m  / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)

        # 맵 그리드 초기화: 0=미지, 0.3=빈공간, 0.8=벽, 1.0=장애물
        self.grid = np.full(
            (self.height_cells, self.width_cells),
            CELL_UNKNOWN,
            dtype=np.float32,
        )
        # [정적 맵 원본 보관] 불러온 사전 맵과 실시간 감지 센서 맵의 색상 구분을 위한 배열
        self.static_grid    = np.full_like(self.grid, CELL_UNKNOWN)
        # 신뢰도 카운터 및 장애물 메모리 타임스탬프 초기화
        self.hit_count      = np.zeros_like(self.grid, dtype=np.int32)    # 장애물 감지 카운트
        self.free_count     = np.zeros_like(self.grid, dtype=np.int32)    # 빈 공간 감지 카운트
        self.last_hit_time  = np.zeros_like(self.grid, dtype=np.float64)  # 장애물이 마지막으로 감지된 시간 (s)
        self.OBSTACLE_MEMORY_SEC = 5.0  # 장애물 메모리 보존 시간 (5초 동안 책상 상판을 라이다가 지우지 못하도록 보호)
        self.STATIC_PROMOTE_HIT_COUNT = 8  # 이 횟수를 초과해 일관되게 감지되면 static_grid(영구 지도)에 편입

        # 로봇 현재 위치 (맵 중앙에서 시작하며, set_robot_pose 호출 시 실제 이동을 따라 갱신됨)
        self.robot_col = self.width_cells  // 2
        self.robot_row = self.height_cells // 2

        # 월드 좌표 원점(오도메트리 0,0 지점)에 해당하는 셀 - 최초 1회 고정
        self._origin_col = self.robot_col
        self._origin_row = self.robot_row

        # 로봇 현재 자세 (맵 좌표계: x=우측+, y=전방+ / 헤딩: 전방 0도, 우측 +90도)
        self.robot_world_x    = 0.0
        self.robot_world_y    = 0.0
        self.robot_heading_deg = 0.0

        # 마지막 업데이트 시간
        self.last_update = time.time()

    # ── 로봇 자세(오도메트리) 반영 ────────────────────────────────
    def set_robot_pose(self, x_fwd_m: float, y_left_m: float, heading_deg: float):
        """
        BLE 오도메트리로 받은 로봇의 현재 자세를 맵에 반영한다.

        입력 좌표계(PathRecommender/BLE): 전방=+X, 좌측=+Y, 헤딩 0도=전방 / +90도=우측
        맵 좌표계: x=우측+, y=전방+ (world_to_cell 이 이 축을 기준으로 동작)
        """
        self.robot_world_x     = -y_left_m   # 좌측(+Y) → 맵의 우측(+x) 기준으로 부호 반전
        self.robot_world_y     = x_fwd_m     # 전방(+X) → 맵의 전방(+y)
        self.robot_heading_deg = heading_deg

        # 고정 원점 셀로부터의 변위로 현재 로봇 셀 위치를 갱신
        self.robot_col = self._origin_col + int(round(self.robot_world_x / self.resolution))
        self.robot_row = self._origin_row - int(round(self.robot_world_y / self.resolution))

    # ── 좌표 변환 ─────────────────────────────────────────────────
    # 월드 좌표(미터)를 맵 셀 인덱스로 변환하는 메서드
    def world_to_cell(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """직교 좌표(m) → 격자 인덱스 (row, col). 로봇의 현재 위치 기준."""
        # 로봇 위치를 기준으로 셀 인덱스 계산
        col = self.robot_col + int(round(x_m / self.resolution))  # x → col
        row = self.robot_row - int(round(y_m / self.resolution))  # y → row (y축 반전)
        return row, col

    # 맵 셀 인덱스를 월드 좌표(미터)로 변환하는 메서드
    def cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """격자 인덱스 → 직교 좌표(m)"""
        # 셀 인덱스를 로봇 기준 월드 좌표로 변환
        x_m = (col - self.robot_col) * self.resolution  # col → x
        y_m = (self.robot_row - row) * self.resolution  # row → y (y축 반전)
        return x_m, y_m

    # 주어진 셀 인덱스가 맵 범위 내인지 확인하는 메서드
    def in_bounds(self, row: int, col: int) -> bool:
        # 행과 열이 맵 크기 내에 있는지 체크
        return 0 <= row < self.height_cells and 0 <= col < self.width_cells

    # ── LiDAR 스캔 업데이트 ───────────────────────────────────────
    # LiDAR 스캔 데이터를 맵에 반영하는 메서드
    def update_from_lidar(self, scan) -> int:
        """
        LiDAR 스캔 포인트를 맵에 반영.
        레이 캐스팅으로 포인트까지의 경로를 빈 공간으로 마킹하되,
        카메라 등으로 최근 등록된 장애물(책상 등)은 강제로 지워지지 않도록 보호.
        """
        if scan is None or not scan.points:  # 스캔 데이터가 없으면
            return 0  # 업데이트 없음

        now = time.time()
        updated = 0  # 업데이트된 셀 수
        for point in scan.points:  # 각 포인트에 대해
            if point.distance_m <= 0:  # 거리가 유효하지 않으면
                continue  # 건너뜀

            # 포인트의 극좌표를 직교좌표로 변환 (로봇 헤딩만큼 회전시켜 월드 방향으로 정렬)
            rad   = math.radians(point.angle_deg + self.robot_heading_deg)  # 로봇 헤딩 보정 후 라디안 변환
            x_end = point.distance_m * math.sin(rad)  # 끝점 x (로봇 현재 위치 기준 상대 좌표)
            y_end = point.distance_m * math.cos(rad)  # 끝점 y (로봇 현재 위치 기준 상대 좌표)

            # 끝점을 셀 인덱스로 변환
            end_row, end_col = self.world_to_cell(x_end, y_end)

            # 레이 캐스팅: 로봇 위치에서 포인트까지의 경로를 빈 공간으로 마킹
            ray_cells = self._bresenham(  # Bresenham 알고리즘으로 경로 셀 계산
                self.robot_row, self.robot_col,  # 시작점: 로봇
                end_row, end_col,  # 끝점
            )
            for r, c in ray_cells[:-1]:  # 끝점 제외한 경로 셀들
                if self.in_bounds(r, c):  # 맵 범위 내이면
                    # [장애물 메모리 보호] 최근 5초 이내에 감지된 장애물 셀은 라이다 빈 공간 광선이 지우지 못하도록 보호!
                    if (now - self.last_hit_time[r, c] < self.OBSTACLE_MEMORY_SEC) and (self.grid[r, c] >= CELL_WALL):
                        continue

                    self.free_count[r, c] += 1  # 빈 공간 카운트 증가
                    self._update_cell(r, c, now)  # 셀 상태 업데이트

            # 끝점은 장애물로 마킹
            if self.in_bounds(end_row, end_col):  # 범위 내이면
                self.hit_count[end_row, end_col] += 1  # 장애물 카운트 증가
                self.last_hit_time[end_row, end_col] = now  # 마지막 감지 타임스탬프 갱신
                self._update_cell(end_row, end_col, now)  # 셀 상태 업데이트
                updated += 1  # 업데이트 수 증가

        # 마지막 업데이트 시간 기록
        self.last_update = now
        return updated  # 업데이트된 셀 수 반환

    # ── OAK 깊이맵 업데이트 ───────────────────────────────────────
    # OAK 카메라 데이터를 맵에 반영하는 메서드
    def update_from_oak(self, oak_frame) -> int:
        """
        OAK-D-Lite 장애물 정보를 맵에 반영 (책상 상판 등).
        정면 시야각 내 장애물을 메모리 보호와 함께 등록.
        """
        if oak_frame is None or not oak_frame.obstacles:  # OAK 데이터가 없으면
            return 0  # 업데이트 없음

        now = time.time()
        updated = 0  # 업데이트된 셀 수
        for obs in oak_frame.obstacles:  # 각 장애물에 대해
            if obs.is_wall:  # 벽이면 건너뜀
                continue

            # 장애물의 극좌표를 직교좌표로 변환 (로봇 헤딩만큼 회전시켜 월드 방향으로 정렬)
            rad   = math.radians(obs.angle_deg + self.robot_heading_deg)
            x_end = obs.distance_m * math.sin(rad)
            y_end = obs.distance_m * math.cos(rad)

            # 끝점을 셀 인덱스로 변환
            end_row, end_col = self.world_to_cell(x_end, y_end)

            if self.in_bounds(end_row, end_col):  # 범위 내이면
                # 신뢰도에 따라 가중치 적용
                weight = int(obs.confidence * 4) + 2
                self.hit_count[end_row, end_col] += weight  # 가중치만큼 장애물 카운트 증가
                self.free_count[end_row, end_col] = 0       # 기존 free 카운트 리셋 (확실한 장애물 선언)
                self.last_hit_time[end_row, end_col] = now  # 5초 메모리 보호 타임스탬프 갱신
                self.grid[end_row, end_col] = CELL_OBSTACLE # 즉시 장애물 부여

                # [책상 상판 두께 팽창] 장애물 주변 3x3 반경도 함께 메모리 보호 등록
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        nr, nc = end_row + dr, end_col + dc
                        if self.in_bounds(nr, nc):
                            self.last_hit_time[nr, nc] = now
                            self.hit_count[nr, nc] += 1
                            self._update_cell(nr, nc, now)

                updated += 1  # 업데이트 수 증가

        return updated  # 업데이트된 셀 수 반환

    # ── 셀 상태 갱신 ──────────────────────────────────────────────
    # 셀의 상태를 hit/free 카운트 비율 및 장애물 메모리로 결정하는 내부 메서드
    def _update_cell(self, row: int, col: int, now: Optional[float] = None):
        """hit/free 카운트 비율, 5초 장애물 메모리 보호, 영구 승격 여부로 셀 상태 결정"""
        if now is None:
            now = time.time()

        hit  = self.hit_count[row, col]  # 장애물 감지 횟수
        free = self.free_count[row, col]  # 빈 공간 감지 횟수
        total = hit + free  # 총 감지 횟수

        # [영구 승격] 충분히 여러 번(STATIC_PROMOTE_HIT_COUNT회 초과), 일관되게(비율 0.40 초과)
        # 장애물로 확인된 셀은 static_grid(영구 지도)에 편입시킨다.
        # 이후엔 센서 시야를 벗어나거나 다음 세션에 다시 켜도 계속 기억된다.
        # (사람처럼 움직이는 대상은 한 셀에서 hit_count가 이 정도까지 쌓이기 전에 자리를 벗어나므로
        #  자연히 승격되지 않는다.)
        if hit > self.STATIC_PROMOTE_HIT_COUNT and total > 0 and (hit / total) > 0.40:
            self.static_grid[row, col] = CELL_WALL
            self.grid[row, col] = CELL_WALL
            return

        # [핵심] 최근 5초 이내에 카메라/라이다가 장애물로 등록한 셀은 강제로 장애물 유지!
        if (now - self.last_hit_time[row, col] < self.OBSTACLE_MEMORY_SEC) and (hit >= 2):
            self.grid[row, col] = CELL_OBSTACLE
            return

        if total == 0:  # 감지 기록이 없으면
            return  # 상태 유지

        hit_ratio = hit / total  # 장애물 비율 계산

        if hit_ratio > 0.40:  # 장애물 비율이 높으면 (기존 0.55에서 0.40으로 민감도 향상)
            # 위의 영구 승격 분기에서 이미 hit > STATIC_PROMOTE_HIT_COUNT 인 경우를 처리했으므로
            # 여기 도달하는 건 항상 승격 기준에 못 미치는(아직 확신이 약한) 일반 장애물이다.
            self.grid[row, col] = CELL_OBSTACLE
        elif hit_ratio < 0.15:  # 빈 공간 비율이 압도적일 때만 빈 공간으로 설정
            self.grid[row, col] = CELL_FREE  # 빈 공간으로 설정
        # 그 사이는 현재 상태 유지 (불확실 영역)

    def set_cell(self, row: int, col: int, value: float):
        """특정 셀의 점유 상태 값을 직접 설정 (범위 내부일 때)"""
        if self.in_bounds(row, col):
            self.grid[row, col] = value

    # ── Bresenham 레이 캐스팅 ─────────────────────────────────────
    # 두 셀 사이의 선분을 따라 모든 셀을 반환하는 정적 메서드 (Bresenham 알고리즘)
    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
        """두 격자 좌표 사이의 셀 목록 반환 (Bresenham 선분 알고리즘)"""
        cells = []  # 셀 리스트
        dr = abs(r1 - r0)  # 행 차이
        dc = abs(c1 - c0)  # 열 차이
        sr = 1 if r1 > r0 else -1  # 행 방향
        sc = 1 if c1 > c0 else -1  # 열 방향
        err = dr - dc  # 오류 값 초기화

        r, c = r0, c0  # 시작점
        max_steps = max(dr, dc) + 1  # 최대 스텝 수

        for _ in range(max_steps):  # 최대 스텝까지 반복
            cells.append((r, c))  # 현재 셀 추가
            if r == r1 and c == c1:  # 끝점에 도달하면
                break  # 종료
            e2 = 2 * err  # 오류 값 계산
            if e2 > -dc:  # 행 방향 조정 필요
                err -= dc
                r   += sr
            if e2 < dr:  # 열 방향 조정 필요
                err += dr
                c   += sc

        return cells  # 셀 리스트 반환

    # ── 맵 통계 ───────────────────────────────────────────────────
    # 맵의 통계 정보를 반환하는 메서드
    def stats(self) -> dict:
        total = self.width_cells * self.height_cells  # 총 셀 수
        obstacle_cells = np.sum(self.grid >= CELL_OBSTACLE)  # 장애물 셀 수
        wall_cells     = np.sum((self.grid >= CELL_WALL) & (self.grid < CELL_OBSTACLE))  # 벽 셀 수
        free_cells     = np.sum((self.grid > 0) & (self.grid < CELL_WALL))  # 빈 공간 셀 수
        unknown_cells  = np.sum(self.grid == CELL_UNKNOWN)  # 미지 셀 수
        return {  # 통계 딕셔너리 반환
            "total": total,  # 총 셀 수
            "obstacle": int(obstacle_cells),  # 장애물 셀 수
            "wall":     int(wall_cells),     # 벽 셀 수
            "free":     int(free_cells),     # 빈 공간 셀 수
            "unknown":  int(unknown_cells),  # 미지 셀 수
            "explored_pct": round((1 - unknown_cells / total) * 100, 1),  # 탐색된 비율 (%)
        }

    # ── 전방 장애물 크기(폭, 길이) 측정 ──────────────────────────────
    def get_front_obstacle_dimensions(
        self,
        max_dist_m: float = 1.5,
        half_width_m: float = 0.8
    ) -> Tuple[float, float, float]:
        """
        로봇 전방 관심 영역(ROI, 로봇의 현재 헤딩 기준) 내 장애물 클러스터의
        실제 물리적 크기(가로 폭, 세로 깊이, 최근접 거리)를 측정.

        맵 격자 자체는 회전하지 않으므로, 로봇 주변을 넉넉히 포함하는
        월드좌표 정사각형 윈도우를 먼저 자른 뒤 각 셀을 로봇 헤딩만큼
        역회전시켜 "로봇 로컬 전방/좌우" 기준으로 판정한다.

        Returns:
            (width_m, length_m, nearest_dist_m)
        """
        y_min, y_max = 0.05, max_dist_m

        # 회전된 ROI(사각형)를 항상 포함할 수 있는 정사각형 탐색 반경
        search_radius_m = math.hypot(half_width_m, max_dist_m)
        r_span = int(math.ceil(search_radius_m / self.resolution)) + 1

        r_start = max(0, self.robot_row - r_span)
        r_end   = min(self.height_cells, self.robot_row + r_span + 1)
        c_start = max(0, self.robot_col - r_span)
        c_end   = min(self.width_cells, self.robot_col + r_span + 1)

        window = self.grid[r_start:r_end, c_start:c_end]
        obs_rows, obs_cols = np.where(window >= CELL_WALL)

        if obs_rows.size == 0:
            return 0.40, 0.45, 999.0  # 장애물 감지 안 됨 (기본값)

        global_rows = obs_rows + r_start
        global_cols = obs_cols + c_start

        # 로봇 기준 월드 좌표 (x=우측+, y=전방+, 헤딩 회전 미반영)
        x_world = (global_cols - self.robot_col) * self.resolution
        y_world = (self.robot_row - global_rows) * self.resolution

        # 로봇 헤딩만큼 역회전 -> 로봇 로컬 좌표(x=로봇 우측+, y=로봇 전방+)
        h = math.radians(self.robot_heading_deg)
        cos_h, sin_h = math.cos(h), math.sin(h)
        x_local = x_world * cos_h - y_world * sin_h
        y_local = x_world * sin_h + y_world * cos_h

        # 로봇 로컬 기준 전방 ROI(가로 ±half_width_m, 세로 y_min~y_max) 안의 셀만 채택
        in_roi = (
            (x_local >= -half_width_m) & (x_local <= half_width_m) &
            (y_local >= y_min) & (y_local <= y_max)
        )
        if not np.any(in_roi):
            return 0.40, 0.45, 999.0  # 장애물 감지 안 됨 (기본값)

        xs = x_local[in_roi]
        ys = y_local[in_roi]

        width_m  = float(np.max(xs) - np.min(xs) + self.resolution)
        length_m = float(np.max(ys) - np.min(ys) + self.resolution)
        nearest_m = float(np.min(ys))

        # 최소 물리 크기 보정 (단일 픽셀 노이즈 방어: 최소 0.20m)
        width_m  = max(0.25, width_m)
        length_m = max(0.25, length_m)

        return width_m, length_m, nearest_m

    # ── 맵 저장 및 불러오기 (File I/O) ──────────────────────────────
    def _resolve_path(self, filepath: str) -> str:
        """상대 경로 입력 시 mapper.py가 위치한 폴더 기준으로 절대 경로 변환"""
        import os
        if not os.path.isabs(filepath):
            base_dir = os.path.dirname(os.path.abspath(__file__))
            return os.path.join(base_dir, filepath)
        return filepath

    def save_map(self, filepath: str = "saved_map.npz") -> bool:
        """
        누적된 영구 지도(static_grid)를 압축 파일(.npz)과 시각화 이미지(.png)로 저장.
        static_grid는 사전에 불러온 지도 + 이번 세션에서 충분히 반복 확인되어
        영구 승격된(hit_count > STATIC_PROMOTE_HIT_COUNT) 셀들로 구성된다.
        self.grid(순간 스냅샷)를 저장하면 사람 등 일시적으로 지나가던 대상이
        그대로 박제될 수 있어 static_grid를 저장한다.
        """
        filepath = self._resolve_path(filepath)
        try:
            np.savez_compressed(
                filepath,
                grid=self.static_grid,
                hit_count=self.hit_count,
                free_count=self.free_count,
                resolution=self.resolution,
                width_m=self.width_m,
                height_m=self.height_m,
                robot_col=self.robot_col,
                robot_row=self.robot_row,
                timestamp=time.time(),
            )
            # 사람이 열어볼 수 있는 PNG 이미지로도 함께 저장
            img_path = filepath.rsplit(".", 1)[0] + ".png"
            norm_map = np.full(self.static_grid.shape, 128, dtype=np.uint8) # 기본 미지: 128
            norm_map[self.static_grid == CELL_FREE] = 230      # 빈 공간: 밝은 흰색/회색
            norm_map[self.static_grid == CELL_WALL] = 80       # 벽: 진한 회색
            norm_map[self.static_grid >= CELL_OBSTACLE] = 0    # 장애물: 검정색

            import cv2
            cv2.imwrite(img_path, norm_map)
            print(f"[OccupancyMap] 💾 맵 저장 완료: {filepath} & {img_path}")
            return True
        except Exception as e:
            print(f"[OccupancyMap] ❌ 맵 저장 실패: {e}")
            return False

    def load_map(self, filepath: str = "saved_map.npz") -> bool:
        """저장된 .npz 맵 파일을 불러와 현재 맵 및 static_grid로 복원"""
        import os
        filepath = self._resolve_path(filepath)
        if not os.path.exists(filepath):
            print(f"[OccupancyMap] ⚠️ 저장된 맵 파일이 존재하지 않습니다: {filepath}")
            return False

        try:
            data = np.load(filepath)
            loaded_grid = data["grid"]
            if loaded_grid.shape == self.grid.shape:
                self.grid[:]        = loaded_grid
                self.static_grid[:] = loaded_grid  # 사전 정적 지도 기준점으로 보관
                if "hit_count" in data:
                    self.hit_count[:] = data["hit_count"]
                if "free_count" in data:
                    self.free_count[:] = data["free_count"]
                self.last_hit_time[:] = time.time()  # 메모리 보호 시간 갱신
                print(f"[OccupancyMap] 📂 맵 불러오기 성공: {filepath} (크기: {self.grid.shape})")
                return True
            else:
                print(f"[OccupancyMap] ⚠️ 맵 격자 해상도/크기 불일치 (현재: {self.grid.shape}, 로드: {loaded_grid.shape})")
                return False
        except Exception as e:
            print(f"[OccupancyMap] ❌ 맵 불러오기 오류: {e}")
            return False

    def prepare_frame(self):
        """
        매 프레임 센서 업데이트 전 호출: 사전 저장된 정적 맵(static_grid)을 베이스로 유지하되,
        최근 OBSTACLE_MEMORY_SEC(기본 5초) 이내에 확인된 장애물 셀은 이번 프레임에 센서
        시야를 벗어났더라도 그대로 유지한다 (OAK처럼 시야각이 좁은 센서가 잠깐 다른 곳을
        보는 사이 장애물이 지도에서 사라지는 것을 방지).
        """
        now = time.time()
        protected = (self.hit_count >= 2) & ((now - self.last_hit_time) < self.OBSTACLE_MEMORY_SEC)
        self.grid[:] = np.where(protected, CELL_OBSTACLE, self.static_grid)

    # 맵 전체를 초기화하는 메서드 (사용자 초기화 버튼 클릭 시)
    def reset(self):
        self.grid[:]          = CELL_UNKNOWN  # 그리드 초기화
        self.static_grid[:]   = CELL_UNKNOWN  # 정적 맵 초기화
        self.hit_count[:]     = 0             # 히트 카운트 초기화
        self.free_count[:]    = 0             # 프리 카운트 초기화
        self.last_hit_time[:] = 0.0           # 장애물 메모리 초기화
