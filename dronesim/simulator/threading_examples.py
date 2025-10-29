"""Simple multi-threading examples for drone simulation.

This module demonstrates various Python multi-threading patterns
that can be applied to drone simulation scenarios.
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
import queue
import random
import threading
import time


@dataclass
class DroneData:
    """Simple drone data structure for examples."""
    id: int
    position: tuple
    battery: float
    status: str


class BasicMultiThreading:
    """Basic multi-threading examples."""

    @staticmethod
    def basic_thread_example():
        """기본 스레드 생성 및 실행 예제."""
        print("=== 기본 스레드 예제 ===")

        def drone_mission(drone_id: int, duration: float):
            print(f"드론 {drone_id}: 미션 시작")
            time.sleep(duration)
            print(f"드론 {drone_id}: 미션 완료")
            return f"드론 {drone_id} 결과"

        # 스레드 생성
        threads = []
        for i in range(3):
            thread = threading.Thread(
                target=drone_mission,
                args=(i, random.uniform(1, 3))
            )
            threads.append(thread)
            thread.start()

        # 모든 스레드 완료 대기
        for thread in threads:
            thread.join()

        print("모든 드론 미션 완료\n")

    @staticmethod
    def thread_pool_example():
        """ThreadPoolExecutor 사용 예제."""
        print("=== 스레드 풀 예제 ===")

        def process_drone_data(drone_id: int) -> DroneData:
            # 드론 데이터 처리 시뮬레이션
            time.sleep(random.uniform(0.5, 2.0))
            return DroneData(
                id=drone_id,
                position=(random.randint(0, 100), random.randint(0, 100)),
                battery=random.uniform(20, 100),
                status="active"
            )

        # 스레드 풀로 여러 드론 데이터 처리
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(process_drone_data, i) for i in range(6)]

            # 결과 수집
            results = []
            for future in as_completed(futures):
                result = future.result()
                results.append(result)
                print(f"드론 {result.id} 처리 완료: {result}")

        print("모든 드론 데이터 처리 완료\n")


class ProducerConsumerExample:
    """Producer-Consumer 패턴 예제."""

    def __init__(self, max_queue_size: int = 10):
        self.task_queue = queue.Queue(maxsize=max_queue_size)
        self.result_queue = queue.Queue()
        self.running = True

    def producer(self, num_tasks: int):
        """작업 생성자 - 드론 작업을 큐에 추가."""
        for i in range(num_tasks):
            task = {
                'id': i,
                'type': 'mission',
                'data': f'드론 작업 {i}',
                'priority': random.randint(1, 5)
            }
            self.task_queue.put(task)
            print(f"작업 {i} 생성됨")
            time.sleep(0.1)  # 작업 생성 간격

        # 종료 신호
        for _ in range(3):  # 워커 수만큼
            self.task_queue.put(None)

    def consumer(self, worker_id: int):
        """작업 소비자 - 드론 작업을 처리."""
        while self.running:
            try:
                task = self.task_queue.get(timeout=1)

                if task is None:  # 종료 신호
                    break

                # 작업 처리 시뮬레이션
                print(f"워커 {worker_id}: 작업 {task['id']} 처리 중...")
                time.sleep(random.uniform(0.5, 2.0))

                # 결과 저장
                result = {
                    'task_id': task['id'],
                    'worker_id': worker_id,
                    'result': f"작업 {task['id']} 완료",
                    'processing_time': time.time()
                }
                self.result_queue.put(result)

                self.task_queue.task_done()

            except queue.Empty:
                continue

    def run_example(self):
        """Producer-Consumer 예제 실행."""
        print("=== Producer-Consumer 예제 ===")

        # 워커 스레드 시작
        workers = []
        for i in range(3):
            worker = threading.Thread(target=self.consumer, args=(i,))
            worker.start()
            workers.append(worker)

        # 프로듀서 스레드 시작
        producer_thread = threading.Thread(target=self.producer, args=(10,))
        producer_thread.start()

        # 프로듀서 완료 대기
        producer_thread.join()

        # 모든 작업 완료 대기
        self.task_queue.join()

        # 워커 스레드 종료 대기
        for worker in workers:
            worker.join()

        # 결과 출력
        print("\n=== 처리 결과 ===")
        while not self.result_queue.empty():
            result = self.result_queue.get()
            print(f"워커 {result['worker_id']}: {result['result']}")

        print("Producer-Consumer 예제 완료\n")


class ThreadSafeCounter:
    """스레드 안전한 카운터 예제."""

    def __init__(self):
        self._value = 0
        self._lock = threading.Lock()

    def increment(self):
        """카운터 증가 (스레드 안전)."""
        with self._lock:
            self._value += 1

    def decrement(self):
        """카운터 감소 (스레드 안전)."""
        with self._lock:
            self._value -= 1

    @property
    def value(self):
        """현재 값 조회 (스레드 안전)."""
        with self._lock:
            return self._value


class DroneFleetManager:
    """스레드 안전한 드론 플리트 관리자."""

    def __init__(self):
        self.drones: dict[int, DroneData] = {}
        self.lock = threading.RLock()  # 재진입 가능한 락
        self.active_counter = ThreadSafeCounter()

    def add_drone(self, drone: DroneData) -> bool:
        """드론 추가 (스레드 안전)."""
        with self.lock:
            if drone.id in self.drones:
                return False

            self.drones[drone.id] = drone
            if drone.status == "active":
                self.active_counter.increment()
            return True

    def remove_drone(self, drone_id: int) -> bool:
        """드론 제거 (스레드 안전)."""
        with self.lock:
            if drone_id not in self.drones:
                return False

            drone = self.drones.pop(drone_id)
            if drone.status == "active":
                self.active_counter.decrement()
            return True

    def update_drone_status(self, drone_id: int, new_status: str) -> bool:
        """드론 상태 업데이트 (스레드 안전)."""
        with self.lock:
            if drone_id not in self.drones:
                return False

            old_status = self.drones[drone_id].status
            self.drones[drone_id].status = new_status

            # 활성 카운터 업데이트
            if old_status == "active" and new_status != "active":
                self.active_counter.decrement()
            elif old_status != "active" and new_status == "active":
                self.active_counter.increment()

            return True

    def get_drone_count(self) -> dict[str, int]:
        """드론 수 조회 (스레드 안전)."""
        with self.lock:
            return {
                'total': len(self.drones),
                'active': self.active_counter.value
            }

    def get_all_drones(self) -> list[DroneData]:
        """모든 드론 정보 조회 (스레드 안전한 복사본)."""
        with self.lock:
            return list(self.drones.values())


def thread_safety_example():
    """스레드 안전성 예제."""
    print("=== 스레드 안전성 예제 ===")

    fleet_manager = DroneFleetManager()

    def add_drones(start_id: int, count: int):
        """드론 추가 작업."""
        for i in range(count):
            drone = DroneData(
                id=start_id + i,
                position=(random.randint(0, 100), random.randint(0, 100)),
                battery=random.uniform(20, 100),
                status="active" if random.random() > 0.3 else "inactive"
            )
            success = fleet_manager.add_drone(drone)
            print(f"드론 {drone.id} 추가: {'성공' if success else '실패'}")
            time.sleep(0.1)

    def update_drone_statuses():
        """드론 상태 업데이트 작업."""
        for _ in range(20):
            drones = fleet_manager.get_all_drones()
            if drones:
                drone = random.choice(drones)
                new_status = "active" if drone.status == "inactive" else "inactive"
                success = fleet_manager.update_drone_status(drone.id, new_status)
                print(f"드론 {drone.id} 상태 변경: {new_status} ({'성공' if success else '실패'})")
            time.sleep(0.15)

    def monitor_fleet():
        """플리트 모니터링 작업."""
        for _ in range(15):
            counts = fleet_manager.get_drone_count()
            print(f"플리트 현황 - 총 {counts['total']}대, 활성 {counts['active']}대")
            time.sleep(0.5)

    # 동시에 여러 작업 실행
    threads = [
        threading.Thread(target=add_drones, args=(0, 10)),
        threading.Thread(target=add_drones, args=(100, 8)),
        threading.Thread(target=update_drone_statuses),
        threading.Thread(target=monitor_fleet)
    ]

    # 모든 스레드 시작
    for thread in threads:
        thread.start()

    # 모든 스레드 완료 대기
    for thread in threads:
        thread.join()

    # 최종 상태 출력
    final_counts = fleet_manager.get_drone_count()
    print(f"\n최종 플리트 현황: 총 {final_counts['total']}대, 활성 {final_counts['active']}대")
    print("스레드 안전성 예제 완료\n")


def main():
    """모든 예제 실행."""
    print("Python 멀티스레딩 예제 시작\n")

    # 기본 예제들
    BasicMultiThreading.basic_thread_example()
    BasicMultiThreading.thread_pool_example()

    # Producer-Consumer 패턴
    pc_example = ProducerConsumerExample()
    pc_example.run_example()

    # 스레드 안전성 예제
    thread_safety_example()

    print("모든 예제 완료!")


if __name__ == "__main__":
    main()
