"""
map_visualizer.py
2D 맵과 경로 추천을 OpenCV 창으로 시각화

색상 코드:
  흰색  = 빈 공간 (free)
  검정  = 미지 영역 (unknown)
  회색  = 벽 (wall)
  주황  = 장애물 (obstacle)
  파랑  = 로봇 위치
  초록  = 추천 방향 화살표
  빨강  = 위험 방향
"""

import cv2  # OpenCV 라이브러리 (이미지 처리 및 그리기)
import numpy as np  # NumPy 라이브러리 (배열 계산)
import math  # 수학 함수 (삼각함수 등)
from typing import Optional  # 타입 힌트용 모듈
from mapper import OccupancyMap, CELL_FREE, CELL_WALL, CELL_OBSTACLE, CELL_UNKNOWN  # 맵 관련 클래스와 상수들
from path_recommender import PathRecommendation, DirectionScore  # 경로 추천 관련 클래스들


# 맵 시각화를 담당하는 클래스
class MapVisualizer:

    # 클래스 상수들: 시각화 크기와 색상 설정
    MAP_SIZE_PX = 600       # 시각화 창 크기 (픽셀 단위)
    PANEL_H     = 180       # 하단 정보 패널 높이 (픽셀)

    # 셀 색상 정의 (BGR 형식)
    COLOR_UNKNOWN  = (30,  30,  30)   # 미지 영역: 어두운 회색
    COLOR_FREE     = (210, 210, 210)  # 빈 공간: 밝은 회색
    COLOR_WALL     = (90,  90,  90)   # 벽: 중간 회색
    COLOR_OBSTACLE = (40,  120, 220)  # 장애물: 주황색
    COLOR_ROBOT    = (220, 180,  30)  # 로봇 위치: 파란색
    COLOR_BEST_DIR = (60,  200,  60)  # 최적 방향: 초록색
    COLOR_BAD_DIR  = (40,   40, 200)  # 나쁜 방향: 빨간색
    COLOR_GRID     = (50,   50,  50)  # 격자선: 어두운 회색

    # 초기화 메서드: OccupancyMap 객체를 받아서 시각화 준비
    def __init__(self, occ_map: OccupancyMap):
        self.occ_map   = occ_map  # 점유 맵 객체 저장
        self.px_per_cell = self.MAP_SIZE_PX / occ_map.width_cells  # 셀당 픽셀 수 계산

    # 전체 시각화 이미지를 렌더링하는 메서드
    def render(
        self,
        recommendation: Optional[PathRecommendation] = None,  # 경로 추천 결과 (선택적)
        fps: float = 0.0,  # FPS 값
        lidar_points: int = 0,  # LiDAR 포인트 수
    ) -> np.ndarray:
        """전체 시각화 이미지 반환 (맵 + 정보 패널)"""
        # 맵 이미지와 패널 이미지를 각각 렌더링
        map_img   = self._render_map()  # 맵 부분 렌더링
        panel_img = self._render_panel(recommendation, fps, lidar_points)  # 정보 패널 렌더링
        combined  = np.vstack([map_img, panel_img])  # 맵과 패널을 세로로 결합
        return combined  # 결합된 이미지 반환

    # ── 맵 렌더링 ─────────────────────────────────────────────────
    # 맵 부분을 렌더링하는 내부 메서드
    def _render_map(self) -> np.ndarray:
        S  = self.MAP_SIZE_PX  # 맵 크기
        px = self.px_per_cell  # 픽셀당 셀 비율
        canvas = np.zeros((S, S, 3), dtype=np.uint8)  # 빈 캔버스 생성 (검은색 배경)

        grid = self.occ_map.grid  # 점유 맵의 그리드 데이터
        H, W = grid.shape  # 그리드의 높이와 너비

        # 셀 색상 채우기 (벡터 연산 대신 루프로 각 셀 처리)
        cell_px = max(1, int(px))  # 셀당 픽셀 수 (최소 1)
        for row in range(H):  # 각 행에 대해
            for col in range(W):  # 각 열에 대해
                val = grid[row, col]  # 셀 값 가져오기
                # 셀 값에 따라 색상 결정
                if val == CELL_UNKNOWN:
                    color = self.COLOR_UNKNOWN  # 미지 영역
                elif val < CELL_WALL:
                    color = self.COLOR_FREE  # 빈 공간
                elif val < CELL_OBSTACLE:
                    color = self.COLOR_WALL  # 벽
                # 유리벽 등 특수 케이스 추가
                elif val == 0.6:
                    color = (200, 220, 255)  # 유리벽: 하늘색
                else:
                    color = self.COLOR_OBSTACLE  # 장애물

                # 픽셀 좌표 계산 및 색상 채우기
                y1 = int(row * px)  # 시작 y 좌표
                x1 = int(col * px)  # 시작 x 좌표
                y2 = min(y1 + cell_px, S)  # 끝 y 좌표 (범위 제한)
                x2 = min(x1 + cell_px, S)  # 끝 x 좌표 (범위 제한)
                canvas[y1:y2, x1:x2] = color  # 해당 영역에 색상 채우기

        # 격자선 그리기 (5m 간격)
        grid_m = 1.0  # 격자 간격 (미터)
        grid_cells = int(grid_m / self.occ_map.resolution)  # 격자 간격을 셀 단위로 변환
        for i in range(0, W, grid_cells):  # 세로선 그리기
            x = int(i * px)  # x 좌표 계산
            cv2.line(canvas, (x, 0), (x, S), self.COLOR_GRID, 1)  # 세로선 그리기
        for j in range(0, H, grid_cells):  # 가로선 그리기
            y = int(j * px)  # y 좌표 계산
            cv2.line(canvas, (0, y), (S, y), self.COLOR_GRID, 1)  # 가로선 그리기

        # 로봇 위치 표시
        rx = int(self.occ_map.robot_col * px + px / 2)  # 로봇 x 좌표 (셀 중심)
        ry = int(self.occ_map.robot_row * px + px / 2)  # 로봇 y 좌표 (셀 중심)
        cv2.circle(canvas, (rx, ry), 8, self.COLOR_ROBOT, -1)  # 로봇 위치 원 채우기
        cv2.circle(canvas, (rx, ry), 8, (255, 255, 255), 1)  # 로봇 위치 원 테두리 (흰색)

        # 구역 원 표시 (위험/경고/안전 존)
        import config  # 설정 파일 임포트
        for zone_m, color in [  # 각 존의 반지름과 색상
            (config.ZONE_DANGER_M,  (40,  40, 200)),   # 위험 존: 빨간색
            (config.ZONE_WARNING_M, (40, 160, 220)),   # 경고 존: 주황색
            (config.ZONE_SAFE_M,    (60, 200,  60)),   # 안전 존: 초록색
        ]:
            r_px = int(zone_m / self.occ_map.resolution * px)  # 반지름을 픽셀로 변환
            cv2.circle(canvas, (rx, ry), r_px, color, 1, cv2.LINE_AA)  # 원 그리기

        return canvas  # 렌더링된 맵 이미지 반환

    # 맵 위에 경로 추천 화살표를 그리는 메서드
    def draw_recommendation(
        self,
        canvas: np.ndarray,  # 기존 캔버스
        recommendation: Optional[PathRecommendation],  # 경로 추천 결과
    ) -> np.ndarray:
        """맵 위에 방향 화살표 오버레이"""
        if recommendation is None:  # 추천이 없으면 그대로 반환
            return canvas

        S  = self.MAP_SIZE_PX  # 맵 크기
        px = self.px_per_cell  # 픽셀당 셀 비율
        rx = int(self.occ_map.robot_col * px + px / 2)  # 로봇 x 좌표
        ry = int(self.occ_map.robot_row * px + px / 2)  # 로봇 y 좌표

        # 모든 방향 점수에 대해 화살표 표시
        for ds in recommendation.scores:  # 각 방향 점수에 대해
            rad    = math.radians(ds.angle_deg)  # 각도를 라디안으로 변환
            length = int(ds.clearance_m / self.occ_map.resolution * px * 0.8)  # 화살표 길이 계산
            ex     = int(rx + length * math.sin(rad))  # 끝점 x 좌표
            ey     = int(ry - length * math.cos(rad))  # 끝점 y 좌표
            ex     = max(0, min(S - 1, ex))  # 범위 제한
            ey     = max(0, min(S - 1, ey))  # 범위 제한

            # 최적 방향이면 굵은 초록 화살표, 아니면 얇은 빨강 선
            if ds.angle_deg == recommendation.best_angle_deg:  # 최적 방향
                cv2.arrowedLine(canvas, (rx, ry), (ex, ey),  # 화살표 그리기
                                self.COLOR_BEST_DIR, 3, tipLength=0.25,
                                line_type=cv2.LINE_AA)
                # 레이블 표시
                lx = ex + 8 if ex < S - 60 else ex - 80  # 레이블 x 위치
                ly = ey - 8  # 레이블 y 위치
                cv2.putText(canvas, recommendation.best_label,  # 텍스트 표시
                            (lx, ly), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, self.COLOR_BEST_DIR, 1, cv2.LINE_AA)
            else:  # 다른 방향
                thickness = 1  # 선 굵기
                cv2.line(canvas, (rx, ry), (ex, ey),  # 선 그리기
                         self.COLOR_BAD_DIR, thickness, cv2.LINE_AA)

        return canvas  # 수정된 캔버스 반환

    # ── 정보 패널 ─────────────────────────────────────────────────
    # 하단 정보 패널을 렌더링하는 내부 메서드
    def _render_panel(
        self,
        recommendation: Optional[PathRecommendation],  # 경로 추천
        fps: float,  # FPS
        lidar_points: int,  # LiDAR 포인트 수
    ) -> np.ndarray:
        S = self.MAP_SIZE_PX  # 패널 너비
        panel = np.full((self.PANEL_H, S, 3), 20, dtype=np.uint8)  # 어두운 배경 패널 생성

        stats = self.occ_map.stats()  # 맵 통계 가져오기
        font  = cv2.FONT_HERSHEY_SIMPLEX  # 폰트 설정

        # 왼쪽: 맵 통계 표시
        lines_left = [
            f"Explored: {stats['explored_pct']}%",  # 탐색된 영역 비율
            f"Wall: {stats['wall']}  Obstacle: {stats['obstacle']}",  # 벽과 장애물 수
            f"Free: {stats['free']} cells",  # 빈 셀 수
            f"LiDAR pts: {lidar_points}  FPS: {fps:.1f}",  # LiDAR 포인트와 FPS
        ]
        for i, line in enumerate(lines_left):  # 각 라인 표시
            cv2.putText(panel, line, (12, 28 + i * 30),  # 텍스트 위치
                        font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        # 오른쪽: 경로 추천 표시
        if recommendation:  # 추천이 있으면
            stuck_txt = "  [STUCK]" if recommendation.is_stuck else ""  # 막힘 표시
            cv2.putText(  # 추천 방향 표시
                panel,
                f"Recommend: {recommendation.best_label}{stuck_txt}",
                (S // 2, 28), font, 0.65,
                (60, 220, 60) if not recommendation.is_stuck else (40, 40, 220),  # 색상: 초록 또는 빨강
                1, cv2.LINE_AA,
            )
            reason_lines = self._wrap_text(recommendation.reason, max_chars=38)  # 이유 텍스트 줄바꿈
            for i, ln in enumerate(reason_lines):  # 이유 표시
                cv2.putText(panel, ln, (S // 2, 60 + i * 26),
                            font, 0.44, (160, 200, 160), 1, cv2.LINE_AA)

            # 방향 점수 바 차트
            bar_y = 120  # 바 시작 y
            bar_w = S // 2 - 20  # 바 너비
            max_score = max((s.score for s in recommendation.scores), default=1)  # 최대 점수
            for j, ds in enumerate(recommendation.scores):  # 각 방향에 대해
                bx    = S // 2 + j * (bar_w // len(recommendation.scores))  # 바 x 시작
                ratio = max(0, ds.score) / max(max_score, 0.01)  # 점수 비율
                bar_h = int(ratio * 50)  # 바 높이
                color = self.COLOR_BEST_DIR if ds.angle_deg == recommendation.best_angle_deg \
                        else (80, 100, 140)  # 색상: 최적이면 초록, 아니면 회색
                cv2.rectangle(  # 바 사각형 그리기
                    panel,
                    (bx + 2, bar_y + 50 - bar_h),  # 시작 좌표
                    (bx + bar_w // len(recommendation.scores) - 2, bar_y + 50),  # 끝 좌표
                    color, -1,  # 채우기
                )

        # 범례 표시 (색상과 라벨)
        legend = [
            ((210, 210, 210), "Free"),      # 빈 공간
            ((90,   90,  90), "Wall"),      # 벽
            ((40,  120, 220), "Obstacle"),  # 장애물
            ((220, 180,  30), "Robot"),     # 로봇
        ]
        lx = 12  # 범례 시작 x
        ly = self.PANEL_H - 20  # 범례 y
        for color, label in legend:  # 각 범례 항목
            cv2.rectangle(panel, (lx, ly - 10), (lx + 12, ly + 2), color, -1)  # 색상 사각형
            cv2.putText(panel, label, (lx + 16, ly),  # 라벨 텍스트
                        font, 0.38, (170, 170, 170), 1)
            lx += 70  # 다음 항목으로 이동

        return panel  # 패널 이미지 반환

    # 텍스트를 줄바꿈하는 정적 메서드
    @staticmethod
    def _wrap_text(text: str, max_chars: int) -> list:
        words  = text.split()  # 단어로 분리
        lines  = []  # 줄 리스트
        current = ""  # 현재 줄
        for word in words:  # 각 단어에 대해
            if len(current) + len(word) + 1 <= max_chars:  # 현재 줄에 추가 가능
                current += (" " if current else "") + word  # 단어 추가
            else:  # 새 줄 시작
                if current:
                    lines.append(current)  # 현재 줄 추가
                current = word  # 새 줄 시작
        if current:
            lines.append(current)  # 마지막 줄 추가
        return lines[:3]  # 최대 3줄 반환
 