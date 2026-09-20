"""Unit tests for defensive sensor debouncing and recovery engine."""

from maze_solver.core.recovery import (
    FaultDetector,
    RecoveryController,
    RecoveryState,
    TelemetryFrame,
)
from maze_solver.core.sensor_filter import (
    RawDistances,
    SensorFilter,
    SensorFilterConfig,
)
from maze_solver.core.types import MovementCommand


def test_sensor_filter_smoothing() -> None:
    """Test EMA filter dampens analog sensor fluctuations."""
    config = SensorFilterConfig(alpha=0.5, wall_detect_threshold_cm=12.0)
    filter_engine = SensorFilter(config)

    # Initial state is 30cm
    assert filter_engine.filtered_distances.front == 30.0

    # Feed reading of 10.0 cm
    sensations = filter_engine.update(RawDistances(front=10.0, left=30.0, right=30.0))
    # 0.5 * 10 + 0.5 * 30 = 20.0
    assert abs(filter_engine.filtered_distances.front - 20.0) < 1e-4
    # Because 20.0 > 12.0, wall is not detected yet
    assert sensations.front is False


def test_sensor_filter_hysteresis_and_debouncing() -> None:
    """Test that hysteresis prevents oscillation and single cycle spikes are rejected."""
    config = SensorFilterConfig(
        alpha=0.8,
        wall_detect_threshold_cm=12.0,
        wall_clear_threshold_cm=16.0,
        min_consecutive_cycles=2,
    )
    filter_engine = SensorFilter(config)

    # Feed wall reading: cycle 1
    s1 = filter_engine.update(RawDistances(front=5.0, left=30.0, right=30.0))
    # Filtered value drops, but min_consecutive_cycles=2 prevents immediate state flip
    assert s1.front is False

    # Feed wall reading: cycle 2 (meets min_consecutive_cycles)
    s2 = filter_engine.update(RawDistances(front=5.0, left=30.0, right=30.0))
    assert s2.front is True

    # Single transient spike (e.g. 17cm for 1 cycle) should not immediately clear wall
    s3 = filter_engine.update(RawDistances(front=18.0, left=30.0, right=30.0))
    assert s3.front is True


def test_fault_detector_nominal() -> None:
    """Test nominal forward motion does not trigger any fault."""
    detector = FaultDetector()
    frame = TelemetryFrame(
        command=MovementCommand.FORWARD,
        encoder_delta_cm=2.0,
        gyro_yaw_rate_dps=5.0,
        front_dist_cm=25.0,
        left_dist_cm=8.0,
        right_dist_cm=8.0,
    )
    state = detector.evaluate(frame)
    assert state == RecoveryState.NORMAL


def test_fault_detector_stall() -> None:
    """Test stall is detected when wheels turn but mouse is stuck against wall."""
    detector = FaultDetector(stall_cycles_threshold=3)

    stuck_frame = TelemetryFrame(
        command=MovementCommand.FORWARD,
        encoder_delta_cm=2.0,  # Encoders spinning
        gyro_yaw_rate_dps=0.0,
        front_dist_cm=5.0,  # Stuck 5cm from wall
        left_dist_cm=8.0,
        right_dist_cm=8.0,
    )

    assert detector.evaluate(stuck_frame) == RecoveryState.NORMAL
    assert detector.evaluate(stuck_frame) == RecoveryState.NORMAL
    # Third cycle exceeds threshold -> STALL_DETECTED
    assert detector.evaluate(stuck_frame) == RecoveryState.STALL_DETECTED


def test_fault_detector_slip() -> None:
    """Test sudden large rotational gyro rate while commanded forward flags slip."""
    detector = FaultDetector()
    slip_frame = TelemetryFrame(
        command=MovementCommand.FORWARD,
        encoder_delta_cm=2.0,
        gyro_yaw_rate_dps=180.0,  # High spin rate!
        front_dist_cm=20.0,
        left_dist_cm=8.0,
        right_dist_cm=8.0,
    )
    assert detector.evaluate(slip_frame) == RecoveryState.SLIP_DETECTED


def test_recovery_controller_full_sequence() -> None:
    """Test recovery state machine progresses through HALT -> REVERSE -> SQUARE -> RESUME."""
    controller = RecoveryController()

    stall_frame = TelemetryFrame(
        command=MovementCommand.FORWARD,
        encoder_delta_cm=2.0,
        gyro_yaw_rate_dps=0.0,
        front_dist_cm=4.0,
        left_dist_cm=7.0,
        right_dist_cm=9.0,
    )

    # 1. Trigger stall
    controller.fault_detector._last_front_dist = 4.0
    controller.fault_detector._stall_counter = 2  # prime it
    state, action = controller.process_telemetry(stall_frame)
    assert state == RecoveryState.STALL_DETECTED
    assert action is not None and action.action_type == "HALT"

    # 2. Step 1 of recovery: REVERSE back-off
    state, action = controller.process_telemetry(stall_frame)
    assert state == RecoveryState.RECENTERING
    assert action is not None and action.action_type == "REVERSE"
    assert action.distance_cm < 0

    # 3. Step 2 of recovery: SQUARE_WALL alignment
    state, action = controller.process_telemetry(stall_frame)
    assert state == RecoveryState.RECENTERING
    assert action is not None and action.action_type == "SQUARE_WALL"

    # 4. Step 3 of recovery: RESUME nominal operation
    state, action = controller.process_telemetry(stall_frame)
    assert state == RecoveryState.RECOVERED
    assert action is not None and action.action_type == "RESUME"
