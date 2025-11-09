"""DeliveryDrone: state-aware routing and package workflow.

- Picks next waypoint by task state and distance (pickup vs dropoff).
- Coordinates simple ground services (pickup timing, dropoff completion).
- Estimates remaining route/energy for assignment strategies.

See example notebooks under `example/` for end-to-end usage.
"""

from collections.abc import Sequence

from dronesim.energy import BatteryStatus
from dronesim.energy.unit import Energy, WattHour
from dronesim.geo import GeoPoint
from dronesim.mission import DeliveryState, DeliveryTask
from dronesim.unit import KilometersPerHour, Length, Minute, Power, Time, Velocity, Watt
from dronesim.unit.unit_distance import Kilometer

from .drone import Drone, DroneState

DEFAULT_VELOCITY = KilometersPerHour(50.0)
DEFAULT_TRANSITION_DURATION = Minute(1.0)
DEFAULT_CONSUMPTION = Watt(0.0)
DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE = (
    20.0  # Minimum battery percentage to be operational
)


class DeliveryDrone(Drone[DeliveryTask]):
    """Delivery drone with state-aware routing and simple ground services.

    Highlights:
    - Picks next waypoint by state (pickup vs dropoff) and distance.
    - Tracks pickup/dropoff on the ground; cleans up DONE missions.
    - Offers route/energy estimation for assignment strategies.
    """

    _current_mission: DeliveryTask | None = None
    _deliveries_per_charge: int
    _deliveries_count: int
    _is_going_to_base: bool
    _is_on_base: bool
    _start_travel_time: Time | None
    _pakage_count: int
    battery_usage_history: list[tuple[Time, Time, Energy]]

    def __init__(
        self,
        pos: GeoPoint,
        battery: BatteryStatus,
        velocity: Velocity = DEFAULT_VELOCITY,
        transition_duration: Time = DEFAULT_TRANSITION_DURATION,
        power_idle: Power = DEFAULT_CONSUMPTION,
        power_vtol: Power = DEFAULT_CONSUMPTION,
        power_transit: Power = DEFAULT_CONSUMPTION,
        power_per_pakage: Power = DEFAULT_CONSUMPTION,
        operational_battery_percentage: float = DEFAULT_OPERATIONAL_BATTERY_PERCENTAGE,
        base_pos: dict[int, GeoPoint] | None = None,
        max_task_queue_size: int = 0,
        deliveries_per_charge: int = 1,
    ):
        """Initialize drone with position, battery, power, and base config."""
        super().__init__(
            pos,
            battery,
            velocity,
            transition_duration,
            power_idle,
            power_vtol,
            power_transit,
            operational_battery_percentage,
            base_pos,
            max_task_queue_size,
        )
        self._deliveries_per_charge = deliveries_per_charge
        self._deliveries_count = 0
        self._is_going_to_base = False
        self._is_on_base = True
        self._start_travel_time = None
        self.battery_usage_history = []
        self._pakage_count = 0
        self.power_per_pakage = power_per_pakage
        self._is_dropping_off = False

    def vehicle_update(self, dt, now):
        # Apply simple per‑package transit energy model while airborne
        if self.current_state != DroneState.GROUNDED:
            consume_energy = (
                WattHour.from_si(self._pakage_count * float(dt)) * self.power_per_pakage
            )
            self.battery.consume_energy(consume_energy)
        return super().vehicle_update(dt, now)

    def get_nearest_base(self, last_point: GeoPoint | None = None) -> GeoPoint:
        """Return the base closest to current position or to `last_point`."""
        if last_point is None:
            k, v = min(
                self.base_pos.items(), key=lambda t: self.position.distance_to(t[1])
            )
        else:
            k, v = min(
                self.base_pos.items(), key=lambda t: last_point.distance_to(t[1])
            )

        return v

    def _try_assign_new_destination(self, now: Time) -> None:
        """Choose next waypoint.

        - If both exist, pick nearer of: next pickup (ASSIGNED/GO_PICKUP) vs next dropoff (GO_DROPOFF).
        - Keep per-task precedence (pickup before that task's dropoff).
        - If none left, return to nearest base.
        """
        if len(self.current_tasks) == 0 and len(self.task_queue) == 0:
            self._is_dropping_off = False
            if not self.is_operational() or not self._is_on_base:
                v = self.get_nearest_base()
                self.current_destination = v
                self._is_going_to_base = True
                self._start_flight(now)
            return

        if self._start_travel_time is None:
            self._is_on_base = False
            self._start_travel_time = now

        if len(self.current_tasks) == 0:
            self.current_tasks = list(self.task_queue)
            self._deliveries_count += len(self.current_tasks)
            self.task_queue.clear()

        # Find the task with the minimum state
        min_state = min(self.current_tasks, key=lambda t: t.current_state).current_state
        todo_task_list = [
            task for task in self.current_tasks if task.current_state == min_state
        ]

        # Assign destination based on the minimum state task
        if min_state == DeliveryState.ASSIGNED:
            min_distance_task: DeliveryTask = min(
                todo_task_list, key=lambda t: self.position.distance_to(t.origin)
            )
            self.current_destination = min_distance_task.origin
            self._current_mission = min_distance_task
            self._current_mission.next(now)
            self._start_flight(now)
        elif min_state == DeliveryState.GO_DROPOFF:
            min_distance_task: DeliveryTask = min(
                todo_task_list, key=lambda t: self.position.distance_to(t.destination)
            )
            self._is_dropping_off = True
            self.current_destination = min_distance_task.destination
            self._current_mission = min_distance_task
            self._start_flight(now)
        else:
            return

    def estimate_mission_budget(
        self, new_tasks: DeliveryTask | Sequence[DeliveryTask]
    ) -> tuple[Length, Energy, list[Length]]:
        """Estimate remaining (distance, energy) including optional new_task.

        - Builds a simple greedy route from current position:
          pending pickups → pending dropoffs → nearest base.
        - If `new_task` is provided, appends its (origin, destination).
        - Includes tasks currently in `task_queue` for a conservative estimate.
        """
        new_tasks = new_tasks if isinstance(new_tasks, Sequence) else [new_tasks]

        # 1) 현재 작업에서 픽업/드롭 지점 수집
        pick_up_points: list[GeoPoint] = [
            t.origin
            for t in self.current_tasks
            if t.current_state < DeliveryState.SERVICE_PICKUP
        ]
        drop_off_points: list[GeoPoint] = [
            t.destination
            for t in self.current_tasks
            if t.current_state < DeliveryState.SERVICE_DROPOFF
        ]

        # 2) 현재 작업의 경로 정렬(기존 규칙 그대로)
        if pick_up_points:
            pick_up_points.sort(key=lambda t: self.position.distance_to(t))
            drop_off_points.sort(key=lambda t: pick_up_points[-1].distance_to(t))
        else:
            drop_off_points.sort(key=lambda t: self.position.distance_to(t))

        # 3) 신규/대기 작업에서 사후 경유지 후보 수집
        post_pick_up_points: list[GeoPoint] = [t.origin for t in new_tasks + list(self.task_queue)]
        post_drop_off_points: list[GeoPoint] = [t.destination for t in new_tasks + list(self.task_queue)]

        # 4) [중요 변경] 사후 '픽업' 정렬 피벗을 통일
        #    드롭오프가 하나라도 있으면 마지막 드롭오프, 아니면 현재 위치
        pivot_for_new_pickups: GeoPoint = drop_off_points[-1] if drop_off_points else self.position
        post_pick_up_points.sort(key=lambda t: pivot_for_new_pickups.distance_to(t))

        # 5) 사후 '드롭오프'는 마지막 사후 픽업을 기준으로 정렬(동일)
        if post_pick_up_points:
            post_drop_off_points.sort(key=lambda t: post_pick_up_points[-1].distance_to(t))
        else:
            # 사후 픽업이 전혀 없을 때의 안전 기준(희소 케이스)
            post_drop_off_points.sort(key=lambda t: pivot_for_new_pickups.distance_to(t))

        # 6) 최종 경로 구성
        route: list[GeoPoint] = pick_up_points + drop_off_points
        route += post_pick_up_points + post_drop_off_points
        route += [self.get_nearest_base(route[-1])]

        # 7) 총거리/에너지 계산(기존과 동일한 모델)
        total_distance: Length = Kilometer(0)
        prev_point: GeoPoint = self.position
        for point in route:
            total_distance += prev_point.distance_to(point)
            prev_point = point

        time = float(total_distance) / float(self.velocity)
        total_energy = WattHour.from_si(float(self.power_transit + 1.0 * self.power_per_pakage) * time)
        return total_distance, total_energy

    def on_grounded(self, dt: Time, now: Time) -> None:
        """Handle ground services and assign next destination when idle.

        - If in SERVICE_* state: perform pickup/dropoff and advance task state.
        - Otherwise, choose the next destination or remain on base charging.
        """

        def wait_for_pickup(current_mission: DeliveryTask):
            if current_mission.current_state is DeliveryState.SERVICE_PICKUP:
                #TODO : check pickup time
                if current_mission.pickup_time <= now:
                    self._pakage_count += 1
                    self._current_mission.next(now)

        def wait_for_dropoff(current_mission: DeliveryTask):
            # Preserve original behavior: advance again if at SERVICE_DROPOFF
            if current_mission.current_state == DeliveryState.SERVICE_DROPOFF:
                self._current_mission.next(now)
                self._pakage_count -= 1
            # Clean up if mission completed
            if current_mission.current_state == DeliveryState.DONE:
                self.current_tasks.remove(self._current_mission)
                self._current_mission = None

        super().on_grounded(dt, now)

        if self._is_on_base:
            # Auto-replace battery while on base
            self.battery.replace_battery()

        if self.current_destination is None:
            if (
                self._current_mission
                and self._current_mission.current_state in DeliveryTask.ground_task()
            ):
                wait_for_pickup(self._current_mission)
                wait_for_dropoff(self._current_mission)
                return

            self._try_assign_new_destination(now)

            return

    def enter_grounded(self, now: Time) -> None:
        """On landing: record trip, advance mission state, clean up if done."""
        if self._is_going_to_base:
            self._is_going_to_base = False
            self._deliveries_count = 0
            self._is_on_base = True
            battery_useage = self.battery.capacity - self.battery.current
            self.battery_usage_history.append(
                (self._start_travel_time, now, battery_useage)
            )
            self._start_travel_time = None

        super().enter_grounded(now)

        if self._current_mission:
            # Advance one step on touch-down
            self._current_mission.next(now)

    def can_accept_task(self) -> bool:
        """Return True if drone is operational and not returning to base."""
        # c1 = (self._deliveries_count + len(self.task_queue)) <= self._deliveries_per_charge
        c1 = True
        c2 = not self._is_going_to_base
        return c1 and c2 and self.is_operational()

    def assign(self, task):
        """Battery-gated enqueue: accept task if budget allows, then queue it."""
        if self.can_accept_task():
            # d, e = self.estimate_mission_budget(task)
            # # battery_usage = float(d) * float(self.power_transit) / float(self.velocity)
            # battery_usage = float(e)

            # next_battery = float(self.battery.current) - battery_usage
            # if (
            #     # 100 * next_battery / float(self.battery.capacity)
            #     # < self.operational_battery_percentage
            #     next_battery < 0
            # ):
            #     return False

            return super().assign(task)
        return False

    def unassign(self, task):
        """Remove task from current tasks or queue."""
        if task in self.current_tasks:
            self.current_tasks.remove(task)
            if task == self._current_mission:
                self._current_mission = None
            return True
        if task in list(self.task_queue):
            self.task_queue.remove(task)
            return True
        return False

    @property
    def is_busy(self) -> bool:
        """Busy when working or explicitly returning to base."""
        return super().is_busy or self._is_going_to_base

    def package_usage(self) -> int:
        """Rough load metric = active tasks + queued tasks."""
        return len(self.current_tasks) + len(self.task_queue)
