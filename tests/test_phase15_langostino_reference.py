"""Phase 15C functional tests for the Langostino offline reference
fixtures. See docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md for what is
verified from swarm-subnet/Langostino's own repo metadata versus what
this module infers.
"""
from __future__ import annotations

import math

from swarm_sim.contracts import HealthState
from swarm_sim.external import langostino_reference as lr


class TestLidarMapping:
    def test_normal_reading_maps_to_a_single_range_return(self):
        obs = lr.map_lidar_to_sensor_observation(lr.lidar_distance_observation(distance_m=3.5))
        assert obs.dropout is False
        assert obs.range_returns_m == (3.5,)
        assert obs.occluded == (False,)

    def test_missing_lidar_maps_to_whole_observation_dropout(self):
        obs = lr.map_lidar_to_sensor_observation(lr.missing_lidar())
        assert obs.dropout is True
        assert obs.range_returns_m == ()
        assert obs.occluded == ()

    def test_mapped_observation_satisfies_the_real_sensor_observation_contract(self):
        # SensorObservation.__post_init__ validates itself - constructing
        # it at all (no exception) is the real proof this mapping is
        # contract-shaped, not just "looks right."
        lr.map_lidar_to_sensor_observation(lr.lidar_distance_observation())
        lr.map_lidar_to_sensor_observation(lr.missing_lidar())


class TestTelemetryMapping:
    def test_normal_telemetry_is_healthy_and_estimator_valid(self):
        vs = lr.map_telemetry_to_vehicle_state(lr.gps_attitude_altitude_telemetry(), sim_time_s=0.0)
        assert vs.estimator_valid is True
        assert vs.health_state == HealthState.OK

    def test_invalid_altitude_degrades_without_crashing(self):
        """Regression: VehicleState.__post_init__ requires every
        position component to be finite (unlike CandidateCommand's
        deliberately-permissive NaN-allowed pattern) - passing a NaN
        altitude straight through used to raise ValueError inside
        VehicleState's own constructor instead of degrading gracefully."""
        vs = lr.map_telemetry_to_vehicle_state(lr.invalid_altitude_telemetry(), sim_time_s=0.0)
        assert vs.estimator_valid is False
        assert vs.health_state == HealthState.DEGRADED
        assert math.isfinite(vs.position_m[2])   # sentinel, never NaN

    def test_missing_gps_fix_degrades(self):
        fixture = lr.gps_attitude_altitude_telemetry(gps_fix=False)
        vs = lr.map_telemetry_to_vehicle_state(fixture, sim_time_s=0.0)
        assert vs.estimator_valid is False
        assert vs.health_state == HealthState.DEGRADED

    def test_altitude_bounds_reject_absurd_values(self):
        assert lr.is_altitude_valid(10_000.0) is False
        assert lr.is_altitude_valid(-1000.0) is False
        assert lr.is_altitude_valid(float("nan")) is False
        assert lr.is_altitude_valid(float("inf")) is False
        assert lr.is_altitude_valid(1.5) is True


class TestLinkStatusMapping:
    def test_connected_recent_heartbeat_is_ok(self):
        fixture = lr.LangostinoLinkFixture(vehicle_id="drone0", timestamp_s=0.0,
                                            companion_computer_connected=True, last_heartbeat_s=0.0)
        assert lr.map_link_status_to_health_state(fixture, now_s=0.1) == HealthState.OK

    def test_companion_computer_link_loss_is_degraded_not_failed(self):
        """DEGRADED, not FAILED: the flight controller (INAV in
        Langostino's real architecture) keeps its own onboard failsafes
        even if the companion Pi is unreachable, so losing that link
        alone is a real but recoverable degradation - see the mapping
        function's own docstring for the full rationale."""
        assert lr.map_link_status_to_health_state(lr.companion_computer_link_loss(), now_s=0.0) == HealthState.DEGRADED

    def test_stale_heartbeat_is_treated_as_link_loss(self):
        fixture = lr.LangostinoLinkFixture(vehicle_id="drone0", timestamp_s=10.0,
                                            companion_computer_connected=True, last_heartbeat_s=0.0)
        assert lr.map_link_status_to_health_state(fixture, now_s=10.0, max_heartbeat_age_s=2.0) == HealthState.DEGRADED

    def test_emergency_landing_request_is_failed(self):
        """A deliberate operator/companion-computer decision, not a mere
        timeout - mapped to FAILED, matching how this project already
        treats an operator-abort-class event elsewhere."""
        assert lr.map_link_status_to_health_state(lr.emergency_landing_request(), now_s=0.0) == HealthState.FAILED


class TestStaleness:
    def test_recent_observation_is_not_stale(self):
        assert lr.is_observation_stale(timestamp_s=9.5, now_s=10.0, max_age_s=2.0) is False

    def test_old_observation_is_stale(self):
        assert lr.is_observation_stale(timestamp_s=0.0, now_s=10.0, max_age_s=2.0) is True

    def test_stale_observation_fixture_is_actually_stale(self):
        fixture = lr.stale_observation(stale_timestamp_s=-10.0)
        assert lr.is_observation_stale(fixture.timestamp_s, now_s=0.0, max_age_s=2.0) is True
