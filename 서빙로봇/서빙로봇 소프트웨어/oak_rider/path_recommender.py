"""
path_recommender.py
경로 추천 모듈 — 직진 우선, 장애물 회피

기본 원칙:
  1. 기본 상태는 항상 직진(Front)
  2. 정면에 장애물이 일정 거리(AVOID_DIST_M) 안에 들어오면 → 비어있는 방향으로 회피
  3. 장애물이 사라지면 → 다시 직진으로 복귀
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from mapper import OccupancyMap, CELL_WALL, CELL_OBSTACLE


@dataclass
class DirectionScore:
    angle_deg: float
    score: float
    clearance_m: float
    has_unknown: bool
    label: str


@dataclass
class PathRecommendation:
    best_angle_deg: float
    best_label: str
    reason: str
    scores: List[DirectionScore]
    is_stuck: bool = False


class PathRecommender:

    DIRECTIONS = [
        (0,    "Front"),
        (30,   "Front-R"),
        (-30,  "Front-L"),
        (60,   "Right-F"),
        (-60,  "Left-F"),
        (90,   "Right"),
        (-90,  "Left"),
    ]

    # ── 설정 ─────────────────────────────────────────────────────
    AVOID_DIST_M   = 1.5    # 이 거리 안에 장애물이 들어오면 회피 (m)
    MAX_RANGE_M    = 3.0    # 각 방향 검사 최대 거리 (m)
    ROBOT_RADIUS_M = 0.25   # 로봇 반경 (m)

    def __init__(self, occ_map: OccupancyMap):
        self.occ_map = occ_map
        self.footprint_offsets: List[Tuple[int, int]] = []
        self._precompute_footprint()

    def _precompute_footprint(self):
        """로봇 원형 바디의 격자 오프셋을 미리 계산"""
        res = self.occ_map.resolution
        r_cells = int(round(self.ROBOT_RADIUS_M / res))
        r_sq = (self.ROBOT_RADIUS_M / res) ** 2
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if dr * dr + dc * dc <= r_sq:
                    self.footprint_offsets.append((dr, dc))

    # ── 메인 추천 ─────────────────────────────────────────────────
    def recommend(self) -> PathRecommendation:
        # 1) 모든 방향의 안전거리 측정
        scores: List[DirectionScore] = []
        for angle, label in self.DIRECTIONS:
            clearance = self._measure_clearance(angle)
            scores.append(DirectionScore(
                angle_deg   = angle,
                score       = clearance,
                clearance_m = clearance,
                has_unknown = False,
                label       = label,
            ))

        # 2) 정면 안전거리 확인
        front = scores[0]  # (0도, "Front")

        if front.clearance_m >= self.AVOID_DIST_M:
            # ── 정면이 안전하면 무조건 직진 ───────────────────────
            best = front
            reason = f"Front clear ({front.clearance_m:.1f}m)"
        else:
            # ── 정면에 장애물 → 가장 멀리 뚫린 방향으로 회피 ─────
            # 직진에 가까운 방향을 약간 우선 (각도 작은 순으로 이미 정렬됨)
            alternatives = sorted(scores[1:], key=lambda s: s.clearance_m, reverse=True)
            best = alternatives[0]

            # 안전거리가 가장 긴 후보들 중 직진에 가까운 것 선택
            top_clearance = best.clearance_m
            similar = [s for s in alternatives if top_clearance - s.clearance_m < 0.5]
            if len(similar) > 1:
                best = min(similar, key=lambda s: abs(s.angle_deg))

            reason = f"Obstacle at {front.clearance_m:.1f}m → {best.label} ({best.clearance_m:.1f}m)"

        return PathRecommendation(
            best_angle_deg = best.angle_deg,
            best_label     = best.label,
            reason         = reason,
            scores         = scores,
            is_stuck       = best.clearance_m < 0.5,
        )

    # ── 특정 방향의 안전거리 측정 ─────────────────────────────────
    def _measure_clearance(self, angle_deg: float) -> float:
        """
        해당 방향으로 로봇 바디를 전진시키며 장애물까지의 거리를 측정.
        mapper의 grid 값(비율 검증 완료)을 기준으로 판단.
        """
        rad    = math.radians(angle_deg)
        step_m = self.occ_map.resolution
        clearance = 0.0

        dist = step_m
        while dist <= self.MAX_RANGE_M:
            x_m = dist * math.sin(rad)
            y_m = dist * math.cos(rad)
            row_c, col_c = self.occ_map.world_to_cell(x_m, y_m)

            if not self.occ_map.in_bounds(row_c, col_c):
                break

            # Footprint 내부에 확정 장애물이 있는지 검사
            hit = False
            for dr, dc in self.footprint_offsets:
                r = row_c + dr
                c = col_c + dc
                if not self.occ_map.in_bounds(r, c):
                    hit = True
                    break
                cell = float(self.occ_map.grid[r, c])
                if cell >= CELL_WALL:  # 0.8 이상 = mapper가 확정한 벽/장애물
                    hit = True
                    break

            if hit:
                break

            clearance = dist
            dist += step_m

        return clearance

    def forward_clearance(self) -> float:
        return self._measure_clearance(0)
