"""
main_mapping.py
OAK-D-Lite + LiDAR + BLE 전체 멀티프로세싱 통합 버전

프로세스 구조:
  ┌─────────────────────────────────────────────┐
  │  메인 프로세스                                │
  │  - 맵 업데이트 (mapper.py)                   │
  │  - 객체 분류 (object_classifier.py)          │
  │  - 경로 추천 (path_recommender.py)           │
  │  - 시각화   (map_visualizer.py)              │
  └────┬──────────────┬──────────────┬───────────┘
       │ Queue         │ Queue        │ Queue
  ┌────▼────┐    ┌─────▼─────┐  ┌───▼────────┐
  │ LiDAR   │    │ OAK 프로세스│  │ BLE 프로세스│
  │ 프로세스 │    │ (선택)     │  │            │
  └─────────┘    └───────────┘  └────────────┘

실행:
    python main_mapping.py              # LiDAR + BLE
    python main_mapping.py --oak        # OAK + LiDAR + BLE
    python main_mapping.py --mock       # 시뮬레이션
    python main_mapping.py --no-ble     # BLE 없이

조작:
    'r' : 맵 초기화
    's' : 현재 화면 저장
    'q' : 종료
"""

import argparse
import time
import math
import multiprocessing
import cv2
import numpy as np

import config
from lidar_processor   import LidarProcessor
from mapper            import OccupancyMap, CELL_WALL, CELL_OBSTACLE, CELL_FREE
from path_recommender  import PathRecommender
from map_visualizer    import MapVisualizer
from object_classifier import ObjectClassifier, ObjectType
from BLE_processor     import ble_processor


# ── OAK 워커 함수 (독립 프로세스) ────────────────────────────────
def _oak_worker(frame_queue: multiprocessing.Queue, stop_event):
    """
    OAK-D-Lite 를 독립 프로세스에서 실행.
    프레임을 frame_queue 에 넣어 메인 프로세스로 전달.
    """
    try:
        from oak_processor import OakProcessor
        oak = OakProcessor()
        oak.start()
        print("[OAK 프로세스] 시작")

        while not stop_event.is_set():
            frame = oak.get_frame()
            if frame is not None:
                # 큐에 최신 프레임만 유지
                while frame_queue.qsize() > 2:
                    try:
                        frame_queue.get_nowait()
                    except Exception:
                        break
                try:
                    frame_queue.put_nowait(frame)
                except Exception:
                    pass
            time.sleep(0.01)

    except Exception as e:
        print(f"[OAK 프로세스] 오류: {e}")
    finally:
        print("[OAK 프로세스] 종료")


# ── 메인 함수 ─────────────────────────────────────────────────────
def main(use_mock: bool = False, use_oak: bool = False, use_ble: bool = True):
    print("=" * 55)
    print("  Serving Robot - Full Multiprocessing Mode")
    print(f"  Mode: {'Mock' if use_mock else 'Hardware'}"
          f"  OAK: {'ON' if use_oak else 'OFF'}"
          f"  BLE: {'ON' if use_ble else 'OFF'}")
    print("  Keys: r=reset  s=save  o=toggle mock(only in mock mode)  q=quit")
    print("=" * 55)

    # ── 프로세스 초기화 ───────────────────────────────────────────

    # 1. LiDAR 프로세스
    lidar_proc = LidarProcessor(use_mock=use_mock)
    lidar_proc.start()

    # 2. OAK 프로세스 (선택)
    oak_frame_queue = None
    oak_stop_event  = None
    oak_process     = None
    if use_oak:
        oak_frame_queue = multiprocessing.Queue()
        oak_stop_event  = multiprocessing.Event()
        oak_process     = multiprocessing.Process(
            target = _oak_worker,
            args   = (oak_frame_queue, oak_stop_event),
            daemon = True,
        )
        oak_process.start()
        print("[Main] OAK 프로세스 시작")

    # 3. BLE 프로세스 (선택)
    ble_proc = None
    if use_ble:
        ble_proc = ble_processor()
        ble_proc.start()

    # ── 메인 프로세스 전용 모듈 초기화 ───────────────────────────
    occ_map     = OccupancyMap()
    recommender = PathRecommender(occ_map)
    visualizer  = MapVisualizer(occ_map)
    classifier  = ObjectClassifier()

    # 루프 제어 변수
    target_dt          = 1.0 / config.TARGET_FPS
    frame_count        = 0
    fps_timer          = time.time()
    fps_display        = 0.0
    save_count         = 0
    classified_objects = []
    recommendation     = None
    last_sent_label    = ""
    oak_frame          = None

    print("[Main] 루프 시작")

    try:
        while True:
            t0 = time.time()

            # ── 1. 센서 데이터 수집 ───────────────────────────────

            # LiDAR: 프로세스 큐에서 최신 스캔 수신
            lidar_scan = lidar_proc.get_scan()

            # OAK: 프로세스 큐에서 최신 프레임 수신
            if oak_frame_queue is not None:
                latest_frame = None
                while True:
                    try:
                        latest_frame = oak_frame_queue.get_nowait()
                    except Exception:
                        break
                if latest_frame is not None:
                    oak_frame = latest_frame

            # ── 2. 객체 분류 (3프레임마다) ───────────────────────
            if frame_count % 3 == 0:
                classified_objects = classifier.classify(lidar_scan, oak_frame)

            # ── 3. 맵 업데이트 ────────────────────────────────────
            occ_map.update_from_lidar(lidar_scan)

            if oak_frame and classified_objects:
                _update_map_from_classified(occ_map, classified_objects)
            elif oak_frame:
                occ_map.update_from_oak(oak_frame)

            # ── 4. 경로 추천 (5프레임마다) ───────────────────────
            if frame_count % 5 == 0:
                recommendation = recommender.recommend()

                # BLE 전송: 방향이 바뀐 경우에만
                if (ble_proc is not None
                        and recommendation is not None
                        and recommendation.best_label != last_sent_label):
                    ble_proc.send(recommendation.best_label)
                    last_sent_label = recommendation.best_label

            # ── 5. BLE 응답 수신 ──────────────────────────────────
            if ble_proc is not None:
                response = ble_proc.get_response()
                if response and not response.startswith("__"):
                    print(f"\n[Arduino] {response}")

            # ── 6. 시각화 ─────────────────────────────────────────
            lidar_pts = len(lidar_scan.points) if lidar_scan else 0

            map_img  = visualizer.render(recommendation, fps_display, lidar_pts)
            map_view = map_img[:MapVisualizer.MAP_SIZE_PX].copy()
            map_view = visualizer.draw_recommendation(map_view, recommendation)
            map_img[:MapVisualizer.MAP_SIZE_PX] = map_view

            oak_vis = classifier.visualize(oak_frame, classified_objects)
            oak_vis = cv2.resize(
                oak_vis,
                (MapVisualizer.MAP_SIZE_PX,
                 MapVisualizer.MAP_SIZE_PX + MapVisualizer.PANEL_H),
            )

            _draw_object_stats(map_img, classified_objects)

            # FPS 표시
            cv2.putText(map_img, f"FPS:{fps_display:.1f}", (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (100, 255, 100), 1)

            # BLE 상태 표시
            if ble_proc is not None:
                ble_txt   = "BLE:ON" if ble_proc.is_alive() else "BLE:OFF"
                ble_color = (60, 220, 60) if ble_proc.is_alive() else (40, 40, 200)
                cv2.putText(map_img, ble_txt, (8, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, ble_color, 1)

            # 프로세스 상태 표시
            lidar_alive = lidar_proc._process and lidar_proc._process.is_alive()
            lidar_txt   = "LiDAR:ON" if lidar_alive else "LiDAR:OFF"
            cv2.putText(map_img, lidar_txt, (8, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (60, 220, 60) if lidar_alive else (40, 40, 200), 1)

            combined = np.hstack([map_img, oak_vis])
            cv2.imshow("2D Map + Object Detection [Multiprocess]", combined)

            # ── 7. 키 입력 ────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                occ_map.reset()
                print("\n[Main] 맵 초기화")
            elif key == ord('o'):
                if use_mock:
                    lidar_proc.toggle_mock_obstacles()
            elif key == ord('s'):
                fname = f"map_save_{save_count:03d}.png"
                cv2.imwrite(fname, combined)
                save_count += 1
                print(f"\n[Main] 저장: {fname}")

            # 콘솔 출력 (20프레임마다)
            if frame_count % 20 == 0 and recommendation:
                n_wall   = sum(1 for o in classified_objects if o.object_type == ObjectType.WALL)
                n_obs    = sum(1 for o in classified_objects if o.object_type == ObjectType.OBSTACLE)
                n_person = sum(1 for o in classified_objects if o.object_type == ObjectType.PERSON)
                stats = occ_map.stats()
                print(
                    f"\rExplored={stats['explored_pct']}%  "
                    f"Wall={n_wall} Obs={n_obs} Person={n_person}  "
                    f"-> {recommendation.best_label}"
                    + (f"  [BLE:{last_sent_label}]" if last_sent_label else ""),
                    end="", flush=True,
                )

            # FPS 계산
            frame_count += 1
            if frame_count % 30 == 0:
                now = time.time()
                fps_display = 30 / max(now - fps_timer, 1e-6)
                fps_timer   = now

            elapsed = time.time() - t0
            time.sleep(max(0.0, target_dt - elapsed))

    except KeyboardInterrupt:
        print("\n[Main] 종료 요청")

    finally:
        print("[Main] 프로세스 정리 중...")
        lidar_proc.stop()

        if oak_stop_event:
            oak_stop_event.set()
        if oak_process and oak_process.is_alive():
            oak_process.join(timeout=3.0)
            if oak_process.is_alive():
                oak_process.terminate()

        if ble_proc:
            ble_proc.stop()

        cv2.destroyAllWindows()

        stats = occ_map.stats()
        print(f"\n[최종 통계]")
        print(f"  탐색률  : {stats['explored_pct']}%")
        print(f"  벽      : {stats['wall']} cells")
        print(f"  장애물  : {stats['obstacle']} cells")
        print(f"  빈공간  : {stats['free']} cells")
        print("[Main] 완료")


# ── 분류 결과 → 맵 반영 ──────────────────────────────────────────
def _update_map_from_classified(occ_map: OccupancyMap, objects):
    for obj in objects:
        rad = math.radians(obj.angle_deg)
        x_m = obj.distance_m * math.sin(rad)
        y_m = obj.distance_m * math.cos(rad)

        row, col = occ_map.world_to_cell(x_m, y_m)
        if not occ_map.in_bounds(row, col):
            continue

        weight = int(obj.confidence * 3) + 1

        if obj.object_type == ObjectType.DENIED:
            occ_map.hit_count[row, col]  = max(0, occ_map.hit_count[row, col] - weight * 3)
            occ_map.free_count[row, col] += weight * 2
        elif obj.object_type == ObjectType.WALL:
            occ_map.hit_count[row, col]  += weight * 2
            occ_map.free_count[row, col]  = max(0, occ_map.free_count[row, col] - 1)
        else:
            occ_map.hit_count[row, col]  += weight

        # grid 값을 직접 덮어쓰지 않고, 비율 판정(_update_cell)에 맡김
        occ_map._update_cell(row, col)


# ── 객체 통계 오버레이 ────────────────────────────────────────────
def _draw_object_stats(img: np.ndarray, objects):
    n_wall   = sum(1 for o in objects if o.object_type == ObjectType.WALL)
    n_obs    = sum(1 for o in objects if o.object_type == ObjectType.OBSTACLE)
    n_person = sum(1 for o in objects if o.object_type == ObjectType.PERSON)

    items = [
        (f"Wall:     {n_wall}",   (120, 120, 120)),
        (f"Obstacle: {n_obs}",    (40,  180, 255)),
        (f"Person:   {n_person}", (60,  220,  60)),
    ]
    h = img.shape[0]
    for i, (text, color) in enumerate(items):
        cv2.putText(img, text, (8, h - 100 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA)


# ── 진입점 ───────────────────────────────────────────────────────
if __name__ == "__main__":
    # Windows 멀티프로세싱 필수
    multiprocessing.freeze_support()

    parser = argparse.ArgumentParser()
    parser.add_argument("--mock",   action="store_true", help="시뮬레이션 모드")
    parser.add_argument("--oak",    action="store_true", help="OAK-D-Lite 사용")
    parser.add_argument("--no-ble", action="store_true", help="BLE 비활성화")
    args = parser.parse_args()

    main(use_mock=args.mock, use_oak=args.oak, use_ble=not args.no_ble)
