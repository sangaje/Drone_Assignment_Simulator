"""간단한 드론 시뮬레이션 테스트
- 두 개 태스크 시나리오
- 다중 태스크 시나리오(여러 작업을 한 드론에 연속 할당)

실행 방법:
  - 두 개 태스크: python3 test.py                (기본)
  - 다중 태스크:  python3 test.py --scenario multi
"""

import argparse
import os
import sys
import types

# Bypass dronesim/__init__.py heavy imports by pre-creating a lightweight package module
pkg_root = os.path.join(os.path.dirname(__file__), 'dronesim')
if 'dronesim' not in sys.modules:
    pkg = types.ModuleType('dronesim')
    pkg.__path__ = [pkg_root]  # mark as package and point to source dir
    sys.modules['dronesim'] = pkg

# Some optional deps (matplotlib/pandas) are not required for these tests; stub them
if 'matplotlib' not in sys.modules:
    sys.modules['matplotlib'] = types.ModuleType('matplotlib')
    sys.modules['matplotlib.pyplot'] = types.ModuleType('matplotlib.pyplot')
if 'pandas' not in sys.modules:
    sys.modules['pandas'] = types.ModuleType('pandas')

from dronesim.energy import BatteryStatus, WattHour
from dronesim.geo import GeoPoint
from dronesim.mission import DeliveryState, DeliveryTask
from dronesim.unit import Second, Time
from dronesim.vehicles import DeliveryDrone


def print_state(drone: DeliveryDrone, task: DeliveryTask, time: Time, step: int):
    """현재 상태 출력"""
    # DeliveryState 이름 매핑
    task_state_name = {
        DeliveryState.ASSIGNED: "ASSIGNED(할당됨)",
        DeliveryState.GO_PICKUP: "GO_PICKUP(픽업지로 이동중)",
        DeliveryState.SERVICE_PICKUP: "SERVICE_PICKUP(픽업중)",
        DeliveryState.GO_DROPOFF: "GO_DROPOFF(배달지로 이동중)",
        DeliveryState.SERVICE_DROPOFF: "SERVICE_DROPOFF(배달중)",
        DeliveryState.DONE: "DONE(완료)",
        DeliveryState.ABORTED: "ABORTED(취소됨)"
    }.get(task.current_state, f"Unknown({task.current_state})")

    print(f"\n[Step {step}] 시간: {time}")
    print(f"  드론 상태: {drone.current_state}")
    print(f"  태스크 ID: {task.id}")
    print(f"  태스크 상태: {task_state_name} (완료: {task.done})")

    # Task 상세 정보
    print("  태스크 상세:")
    print(f"    - origin (픽업지): lat={float(task.origin.latitude):.6f}, lon={float(task.origin.longitude):.6f}")
    print(f"    - destination (배달지): lat={float(task.destination.latitude):.6f}, lon={float(task.destination.longitude):.6f}")
    print(f"    - order_time: {task.order_time}")
    print(f"    - pickup_time: {task.pickup_time}")
    if hasattr(task, 'event_time') and task.event_time:
        print("    - 상태별 진입 시간:")
        for state, event_time in task.event_time.items():
            if event_time is not None:
                state_name = {
                    DeliveryState.ASSIGNED: "할당",
                    DeliveryState.GO_PICKUP: "픽업이동",
                    DeliveryState.SERVICE_PICKUP: "픽업중",
                    DeliveryState.GO_DROPOFF: "배달이동",
                    DeliveryState.SERVICE_DROPOFF: "배달중",
                    DeliveryState.DONE: "완료",
                }.get(state, str(state))
                print(f"      {state_name}: {event_time}")

    print(f"  배터리: {drone.battery.current}")
    print(f"  드론 위치: lat={float(drone.position.latitude):.6f}, lon={float(drone.position.longitude):.6f}")

    # 드론 목적지 정보
    if drone.current_destination:
        print(f"  드론 목적지 (설정됨): lat={float(drone.current_destination.latitude):.6f}, lon={float(drone.current_destination.longitude):.6f}")
        # 태스크 목적지와 비교
        if task.current_state in [DeliveryState.GO_PICKUP, DeliveryState.SERVICE_PICKUP]:
            expected = task.origin
            match = (abs(float(drone.current_destination.latitude) - float(expected.latitude)) < 0.0001 and
                    abs(float(drone.current_destination.longitude) - float(expected.longitude)) < 0.0001)
            print(f"    → {'✅ 픽업지와 일치' if match else '❌ 픽업지와 불일치'}")
        elif task.current_state in [DeliveryState.GO_DROPOFF, DeliveryState.SERVICE_DROPOFF]:
            expected = task.destination
            match = (abs(float(drone.current_destination.latitude) - float(expected.latitude)) < 0.0001 and
                    abs(float(drone.current_destination.longitude) - float(expected.longitude)) < 0.0001)
            print(f"    → {'✅ 배달지와 일치' if match else '❌ 배달지와 불일치'}")
    else:
        print("  드론 목적지: ❌ 설정 안됨")
        if task.current_state == DeliveryState.GO_PICKUP:
            print(f"    → ⚠️  픽업지로 가야 함: lat={float(task.origin.latitude):.6f}, lon={float(task.origin.longitude):.6f}")
        elif task.current_state == DeliveryState.GO_DROPOFF:
            print(f"    → ⚠️  배달지로 가야 함: lat={float(task.destination.latitude):.6f}, lon={float(task.destination.longitude):.6f}")

    print(f"  _is_on_base: {drone._is_on_base}, _is_going_to_base: {drone._is_going_to_base}")
    print(f"  task_queue: {len(drone.task_queue)}, current_tasks: {len(drone.current_tasks)}")


def print_task_budget(drone: DeliveryDrone, task: DeliveryTask, label: str):
    """주어진 태스크를 추가했을 때의 예상 총거리/에너지와 배터리 여유율을 출력."""
    d, e = drone.estimate_mission_budget(task)
    d_km = float(d) / 1000.0
    curr = float(drone.battery.current)
    cap = float(drone.battery.capacity)
    next_percent = 100.0 * (curr - float(e)) / cap
    print(f"  [{label}] 예상 총거리: {d_km:.3f} km | 예상 에너지: {float(e):.1f} (SI) | 예상 잔량: {next_percent:.1f}%")

def run_two():
    """두 개 태스크 시나리오 실행"""
    print("=" * 60)
    print("드론 배달 시뮬레이션 시작!")
    print("=" * 60)

    # 1. 드론 초기화
    battery = BatteryStatus(capacity=WattHour(1000), current=WattHour(1000))
    home_base = GeoPoint.from_deg(37.5665, 126.9780)  # 서울 시청
    drone = DeliveryDrone(pos=home_base, battery=battery)

    # 2. 배달 작업 2개 생성
    pickup_location = GeoPoint.from_deg(37.5700, 126.9800)  # 픽업 위치
    delivery_location = GeoPoint.from_deg(37.5750, 126.9850)  # 배달 위치
    task = DeliveryTask(
        id = 1,
        origin=pickup_location,
        destination=delivery_location,
        order_time=Second(0),
        pickup_time=Second(5)  # 5초 후 픽업 가능
    )
    pickup_location = GeoPoint.from_deg(37.5701, 126.9801)  # 픽업 위치
    delivery_location = GeoPoint.from_deg(37.5751, 126.9851)  # 배달 위치
    task2 = DeliveryTask(
        id = 2,
        origin=pickup_location,
        destination=delivery_location,
        order_time=Second(0),
        pickup_time=Second(5)  # 5초 후 픽업 가능
    )

    print("\n초기 설정:")
    print(f"  홈베이스:    lat={float(home_base.latitude):.6f}, lon={float(home_base.longitude):.6f}")
    print(f"  픽업 위치:   lat={float(pickup_location.latitude):.6f}, lon={float(pickup_location.longitude):.6f}")
    print(f"  배달 위치:   lat={float(delivery_location.latitude):.6f}, lon={float(delivery_location.longitude):.6f}")

    # 3. 작업 할당
    print("\n할당 전 비용(예상):")
    print_task_budget(drone, task, "T1")
    print_task_budget(drone, task2, "T2")

    print("\n작업 할당...")
    ok1 = drone.assign(task)
    # 첫 번째 할당 이후, 두 번째 태스크의 예상 비용도 다시 확인
    print("  T1 할당 직후 비용(예상, T2 추가 시):")
    print_task_budget(drone, task2, "T2")
    ok2 = drone.assign(task2)
    print(f"  결과: T1={'성공' if ok1 else '실패'}, T2={'성공' if ok2 else '실패'}")

    # 4. 시뮬레이션 실행
    current_time = Second(0)
    dt = Second(1.0)  # 1초씩 업데이트
    max_iterations = 2000

    print("\n" + "=" * 60)
    print("시뮬레이션 시작 (2개 태스크)")
    print("=" * 60)

    # 초기 상태 출력 (두 태스크)
    print_state(drone, task, current_time, 0)
    print_state(drone, task2, current_time, 0)

    # 상태 변화 추적: 드론 상태 변화시에만 출력
    prev_drone_state = drone.current_state

    # 자연스러운 플로우에서 Task.next/enter_grounded 호출 추적을 위해 모니터링 래퍼 부착
    orig_next = DeliveryTask.next
    def spy_next(self, now=None):
        prev = self.current_state
        result = orig_next(self, now)
        new = self.current_state
        print(f"[Task.next] t={float(current_time)} task={getattr(self,'id',id(self))} {prev.name} -> {new.name}")
        return result

    orig_enter_grounded = DeliveryDrone.enter_grounded
    def spy_enter_grounded(self, now):
        cs = getattr(self._current_mission, 'current_state', None)
        csn = cs.name if hasattr(cs, 'name') else cs
        print(f"[enter_grounded] t={float(now)} mission_state={csn}")
        return orig_enter_grounded(self, now)

    DeliveryTask.next = spy_next
    DeliveryDrone.enter_grounded = spy_enter_grounded

    try:
        for i in range(1, max_iterations + 1):
            # 드론 업데이트 (update 메서드가 비어있으므로 수동으로 호출)
            drone.timer_update(dt)  # 타이머 업데이트
            drone.vehicle_update(dt, current_time)  # 상태 머신 업데이트
            current_time = current_time + dt

            # 상태 변화가 있을 때마다 출력
            if drone.current_state != prev_drone_state:
                # 드론 상태가 바뀐 경우에만 두 태스크 상태 함께 출력
                print_state(drone, task, current_time, i)
                print_state(drone, task2, current_time, i)
                prev_drone_state = drone.current_state

            # 작업이 완료되면 종료
            if task.done and task2.done:
                # 마지막 스냅샷
                print_state(drone, task, current_time, i)
                print_state(drone, task2, current_time, i)
                print("\n" + "=" * 60)
                print(f"✅ 2개 배달 완료! (총 {current_time} 소요)")
                print("=" * 60)
                break
        else:
            print("\n" + "=" * 60)
            print(f"⚠️  시뮬레이션 종료 (최대 반복 {max_iterations}회 도달)")
            print(f"   마지막 드론 상태: {drone.current_state}")
            print(f"   마지막 태스크 상태: {task.current_state}")
            print("=" * 60)
    finally:
        # 모니터링 원복
        DeliveryTask.next = orig_next
        DeliveryDrone.enter_grounded = orig_enter_grounded


def print_multi_state(drone: DeliveryDrone, tasks: list[DeliveryTask], time: Time, step: int):
    """다중 태스크 시나리오 상태 요약 출력"""
    done_cnt = sum(1 for t in tasks if t.done)
    total = len(tasks)
    print(f"\n[Step {step}] 시간: {time} | 완료 {done_cnt}/{total}")
    print(f"  드론 상태: {drone.current_state} | 위치: lat={float(drone.position.latitude):.6f}, lon={float(drone.position.longitude):.6f}")
    if drone.current_destination:
        print(
            f"  목적지: lat={float(drone.current_destination.latitude):.6f}, lon={float(drone.current_destination.longitude):.6f}"
        )
    print(f"  배터리: {drone.battery.current} | queue: {len(drone.task_queue)} | current: {len(drone.current_tasks)}")
    # 간단한 태스크별 상태 요약(최대 5개 표시)
    for idx, t in enumerate(tasks[:5]):
        print(
            f"   - T{idx}: {t.current_state.name:>15s} | done={t.done} | O=({float(t.origin.latitude):.4f},{float(t.origin.longitude):.4f}) -> D=({float(t.destination.latitude):.4f},{float(t.destination.longitude):.4f})"
        )


def run_multi(n_tasks: int = 5):
    """다중 태스크 시나리오 실행: 한 드론에 여러 태스크를 배치하고 처리되는지 확인"""
    print("=" * 60)
    print("드론 배달 시뮬레이션(다중 태스크) 시작!")
    print("=" * 60)

    # 1. 드론 초기화 (여유 있는 배터리)
    battery = BatteryStatus(capacity=WattHour(2000), current=WattHour(2000))
    home_base = GeoPoint.from_deg(37.5665, 126.9780)  # 서울 시청
    drone = DeliveryDrone(pos=home_base, battery=battery)

    # 2. 여러 배달 작업 생성 (홈베이스 주변 격자로 분포)
    tasks: list[DeliveryTask] = []
    base_lat = float(home_base.latitude)
    base_lon = float(home_base.longitude)
    # 약 ~수백미터 수준의 오프셋
    offsets = [
        (0.003, 0.003),
        (0.004, -0.002),
        (-0.003, 0.004),
        (-0.004, -0.003),
        (0.005, 0.000),
    ]
    if n_tasks > len(offsets):
        # 필요시 오프셋을 순환 사용
        from itertools import cycle, islice
        offsets = list(islice(cycle(offsets), n_tasks))
    else:
        offsets = offsets[:n_tasks]

    for idx, (dlat, dlon) in enumerate(offsets):
        pickup = GeoPoint.from_deg(base_lat + dlat, base_lon + dlon)
        drop = GeoPoint.from_deg(base_lat + dlat * 1.2, base_lon + dlon * 1.2)
        task = DeliveryTask(
            origin=pickup,
            destination=drop,
            order_time=Second(0),
            pickup_time=Second(5 + 3 * idx),  # 각 태스크에 조금씩 다른 픽업 가능 시간
        )
        tasks.append(task)

    print("\n초기 설정(다중 태스크):")
    print(f"  홈베이스: lat={base_lat:.6f}, lon={base_lon:.6f}")
    print(f"  태스크 수: {len(tasks)}")

    # 3. 작업 일괄 할당 (기지 대기 중 수락 → 큐에 쌓인 뒤 출발 시 배치로 로딩)
    print("\n작업 일괄 할당...")
    assign_ok = 0
    for t in tasks:
        if drone.assign(t):
            assign_ok += 1
    print(f"  결과: {assign_ok}/{len(tasks)} 건 할당됨")

    # 4. 시뮬레이션 실행
    current_time = Second(0)
    dt = Second(1.0)  # 1초씩 업데이트
    max_iterations = 2000

    print("\n" + "=" * 60)
    print("시뮬레이션 시작(다중 태스크)")
    print("=" * 60)

    print_multi_state(drone, tasks, current_time, 0)
    prev_drone_state = drone.current_state

    for i in range(1, max_iterations + 1):
        drone.timer_update(dt)
        drone.vehicle_update(dt, current_time)
        current_time = current_time + dt

        # 드론 상태 변화시에만 출력
        if drone.current_state != prev_drone_state:
            print_multi_state(drone, tasks, current_time, i)
            prev_drone_state = drone.current_state

        # 모두 완료되면 종료
        if all(t.done for t in tasks):
            print("\n" + "=" * 60)
            print(f"✅ 모든 배달 완료! (총 {current_time} 소요) | 완료 {sum(1 for t in tasks if t.done)}/{len(tasks)}")
            print(f"{drone.current_tasks}")
            print("=" * 60)
            break
    else:
        print("\n" + "=" * 60)
        print(f"⚠️  시뮬레이션 종료 (최대 반복 {max_iterations}회 도달)")
        print(f"   마지막 드론 상태: {drone.current_state}")
        print(f"   완료된 태스크: {sum(1 for t in tasks if t.done)}/{len(tasks)}")
        print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DeliveryDrone simple tests")
    parser.add_argument(
        "--scenario",
        choices=["two", "multi"],
        default="two",
        help="테스트 시나리오 선택 (two|multi)",
    )
    parser.add_argument("--n_tasks", type=int, default=5, help="multi 시나리오에서 생성할 태스크 수")
    args = parser.parse_args()

    if args.scenario == "two":
        run_two()
    else:
        run_multi(args.n_tasks)
