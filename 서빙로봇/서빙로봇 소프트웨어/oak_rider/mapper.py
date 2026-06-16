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
    로봇 위치가 맵 중앙에 고정되고, 센서 데이터가 누적된다.
    """

    def __init__(self):
        # 맵 해상도 설정 (미터/셀)
        self.resolution = config.FUSION_GRID_RESOLUTION
        # 맵 크기 설정 (미터)
        self.width_m    = 10.0     # 맵 가로 크기 (m)
        self.height_m   = 10.0    # 맵 세로 크기 (m)

        # 셀 단위 크기 계산
        self.width_cells  = int(self.width_m  / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)

        # 맵 그리드 초기화: 0=미지, 0.3=빈공간, 0.8=벽, 1.0=장애물
        self.grid = np.full(
            (self.height_cells, self.width_cells),
            CELL_UNKNOWN,
            dtype=np.float32,
        )
        # 신뢰도 카운터 초기화
        self.hit_count  = np.zeros_like(self.grid, dtype=np.int32)  # 장애물 감지 카운트
        self.free_count = np.zeros_like(self.grid, dtype=np.int32)  # 빈 공간 감지 카운트

        # 로봇 위치 설정 (맵 중앙)
        self.robot_col = self.width_cells  // 2
        self.robot_row = self.height_cells // 2

        # 마지막 업데이트 시간
        self.last_update = time.time()

    # ── 좌표 변환 ─────────────────────────────────────────────────
    # 월드 좌표(미터)를 맵 셀 인덱스로 변환하는 메서드
    def world_to_cell(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """직교 좌표(m) → 격자 인덱스 (row, col). 로봇 위치 기준."""
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
        레이 캐스팅으로 포인트까지의 경로를 빈 공간으로 마킹.
        Returns: 업데이트된 셀 수
        """
        if scan is None or not scan.points:  # 스캔 데이터가 없으면
            return 0  # 업데이트 없음

        updated = 0  # 업데이트된 셀 수
        for point in scan.points:  # 각 포인트에 대해
            if point.distance_m <= 0:  # 거리가 유효하지 않으면
                continue  # 건너뜀

            # 포인트의 극좌표를 직교좌표로 변환
            rad   = math.radians(point.angle_deg)  # 각도를 라디안으로
            x_end = point.distance_m * math.sin(rad)  # 끝점 x
            y_end = point.distance_m * math.cos(rad)  # 끝점 y

            # 끝점을 셀 인덱스로 변환
            end_row, end_col = self.world_to_cell(x_end, y_end)

            # 레이 캐스팅: 로봇 위치에서 포인트까지의 경로를 빈 공간으로 마킹
            ray_cells = self._bresenham(  # Bresenham 알고리즘으로 경로 셀 계산
                self.robot_row, self.robot_col,  # 시작점: 로봇
                end_row, end_col,  # 끝점
            )
            for r, c in ray_cells[:-1]:  # 끝점 제외한 경로 셀들
                if self.in_bounds(r, c):  # 맵 범위 내이면
                    self.free_count[r, c] += 1  # 빈 공간 카운트 증가
                    self._update_cell(r, c)  # 셀 상태 업데이트

            # 끝점은 장애물로 마킹
            if self.in_bounds(end_row, end_col):  # 범위 내이면
                self.hit_count[end_row, end_col] += 1  # 장애물 카운트 증가
                self._update_cell(end_row, end_col)  # 셀 상태 업데이트
                updated += 1  # 업데이트 수 증가

        # 마지막 업데이트 시간 기록
        self.last_update = time.time()
        return updated  # 업데이트된 셀 수 반환

    # ── OAK 깊이맵 업데이트 ───────────────────────────────────────
    # OAK 카메라 데이터를 맵에 반영하는 메서드
    def update_from_oak(self, oak_frame) -> int:
        """
        OAK-D-Lite 장애물 정보를 맵에 반영.
        정면 시야각 내 장애물만 처리.
        """
        if oak_frame is None or not oak_frame.obstacles:
            return 0

        updated = 0
        for obs in oak_frame.obstacles:
            if obs.is_wall:
                continue

            rad   = math.radians(obs.angle_deg)
            x_end = obs.distance_m * math.sin(rad)
            y_end = obs.distance_m * math.cos(rad)

            end_row, end_col = self.world_to_cell(x_end, y_end)

            # [수정된 부분] 주변 5x5 셀(반지름 2칸)을 모두 장애물로 부풀리기 (Inflation)
            INFLATION_RADIUS = 2  # 필요에 따라 1(3x3) 또는 3(7x7)으로 조절하세요.
            weight = int(obs.confidence * 2) + 1

            for dr in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                for dc in range(-INFLATION_RADIUS, INFLATION_RADIUS + 1):
                    nr = end_row + dr
                    nc = end_col + dc
                    
                    if self.in_bounds(nr, nc):
                        self.hit_count[nr, nc] += weight
                        self._update_cell(nr, nc)
                        updated += 1

        return updated

    # ── 셀 상태 갱신 ──────────────────────────────────────────────
    # 셀의 상태를 hit/free 카운트 비율로 결정하는 내부 메서드
    def _update_cell(self, row: int, col: int):
        """hit/free 카운트 비율로 셀 상태 결정"""
        hit  = self.hit_count[row, col]  # 장애물 감지 횟수
        free = self.free_count[row, col]  # 빈 공간 감지 횟수
        total = hit + free  # 총 감지 횟수

        if total == 0:  # 감지 기록이 없으면
            return  # 상태 유지

        hit_ratio = hit / total  # 장애물 비율 계산

        if hit_ratio > 0.55:  # 장애물 비율이 높으면
            # 벽인지 일반 장애물인지 구분 (히트 수가 많으면 벽)
            self.grid[row, col] = CELL_WALL if hit > 8 else CELL_OBSTACLE
        elif hit_ratio < 0.2:  # 빈 공간 비율이 높으면
            self.grid[row, col] = CELL_FREE  # 빈 공간으로 설정
        # 그 사이는 현재 상태 유지 (불확실 영역)

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

    # 맵을 초기화하는 메서드
    def reset(self):
        self.grid[:]       = CELL_UNKNOWN  # 그리드 초기화
        self.hit_count[:]  = 0  # 히트 카운트 초기화
        self.free_count[:] = 0  # 프리 카운트 초기화
