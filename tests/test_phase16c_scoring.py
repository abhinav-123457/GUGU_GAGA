"""Phase 16C: FlightScorer - altitude statistics (all samples and before each drone's first contact) and the
contact root-cause attribution. See swarm_sim/flight_scoring.py."""
import pytest

from swarm_sim.flight_scoring import ROOT_CAUSES, TILT_LIMIT_RAD, FlightScorer


def _scorer(n=2):
    return FlightScorer(n, target_altitude_m=3.0, floor_alt_m=0.5, ceiling_alt_m=8.0, settle_time_s=2.0)


def _feed(s, i, samples, dt=0.1, t0=2.0, tilt=(0.0, 0.0)):
    for k, z in enumerate(samples):
        s.record_tick(t0 + k * dt, i, z, tilt)


class TestAltitudeStatistics:
    def test_samples_before_the_settle_time_are_ignored(self):
        s = _scorer()
        for k in range(10):
            s.record_tick(k * 0.1, 0, 9.0)                  # t < 2 s
        assert s.summary()["altitude"]["samples"] == 0

    def test_error_std_and_extremes(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 3.1, 2.9, 3.0])
        a = s.summary()["altitude"]
        assert a["samples"] == 4 and a["max_abs_error_m"] == pytest.approx(0.1)
        assert a["min_m"] == 2.9 and a["max_m"] == 3.1 and a["outside_band_fraction"] == 0.0

    def test_the_band_fraction_counts_floor_and_ceiling_violations(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 0.2, 9.0, 3.0])
        assert s.summary()["altitude"]["outside_band_fraction"] == pytest.approx(0.5)

    def test_median_of_the_per_drone_stds(self):
        s = _scorer(3)
        _feed(s, 0, [3.0, 3.0, 3.0]); _feed(s, 1, [2.9, 3.1, 2.9, 3.1]); _feed(s, 2, [2.0, 4.0, 2.0, 4.0])
        a = s.summary()["altitude"]
        assert a["median_per_drone_std_m"] == pytest.approx(0.1)

    def test_a_drone_with_no_samples_does_not_break_the_summary(self):
        s = _scorer(2)
        _feed(s, 0, [3.0, 3.0])
        assert s.summary()["per_drone"][1] == {"drone": 1, "samples": 0}


class TestPreContactStatistics:
    def test_the_pre_contact_block_excludes_the_fall_that_follows_a_contact(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 3.0, 3.0])                          # t = 2.0, 2.1, 2.2
        s.record_contact(2.25, "drone_drone", [0])
        _feed(s, 0, [1.0, 0.1, 0.0], t0=2.3)                   # the fall
        a = s.summary()["altitude"]
        assert a["max_abs_error_m"] == pytest.approx(3.0)      # all samples see the fall ...
        assert a["pre_contact"]["max_abs_error_m"] == pytest.approx(0.0)      # ... the pre-contact block does not
        assert a["pre_contact"]["samples"] == 3

    def test_a_drone_that_never_touched_anything_is_fully_counted(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 2.5, 3.0])
        a = s.summary()["altitude"]
        assert a["pre_contact"]["samples"] == 3 and a["pre_contact"]["max_abs_error_m"] == pytest.approx(0.5)

    def test_it_is_per_drone(self):
        s = _scorer(2)
        _feed(s, 0, [3.0, 3.0, 3.0]); _feed(s, 1, [3.0, 3.0, 3.0])
        s.record_contact(2.05, "drone_obstacle", [0])
        _feed(s, 0, [0.0], t0=2.3)
        _feed(s, 1, [2.6], t0=2.3)
        assert s.summary()["altitude"]["pre_contact"]["max_abs_error_m"] == pytest.approx(0.4)   # drone 1 only

    def test_an_altitude_excursion_before_the_contact_is_still_seen(self):
        """A drone that climbed away or sank before hitting something must not be excused by the exclusion."""
        s = _scorer(1)
        _feed(s, 0, [3.0, 4.5, 5.0])
        s.record_contact(2.35, "drone_ground", [0], altitude_m=0.0)
        assert s.summary()["altitude"]["pre_contact"]["max_abs_error_m"] == pytest.approx(2.0)


class TestContactAttribution:
    def test_root_causes_are_the_documented_set(self):
        assert ROOT_CAUSES == ("vertical_control", "attitude_loss", "obstacle", "swarm")

    def test_an_upright_ground_strike_is_a_vertical_control_failure(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 2.0, 1.0], tilt=(0.05, 0.02))
        s.record_contact(2.3, "drone_ground", [0])
        assert s.summary()["contacts"]["first_contact_root_cause_per_drone"]["vertical_control"] == 1

    def test_a_ground_strike_after_a_tumble_is_attitude_loss_not_vertical_control(self):
        s = _scorer(1)
        _feed(s, 0, [3.0, 2.9, 2.0], tilt=(TILT_LIMIT_RAD + 0.3, 0.0))
        s.record_contact(2.3, "drone_ground", [0])
        roots = s.summary()["contacts"]["first_contact_root_cause_per_drone"]
        assert roots["attitude_loss"] == 1 and roots["vertical_control"] == 0

    def test_a_tilt_long_before_the_strike_does_not_count(self):
        s = _scorer(1)
        s.record_tick(2.0, 0, 3.0, (1.5, 0.0))
        for k in range(1, 30):
            s.record_tick(2.0 + k * 0.1, 0, 3.0, (0.0, 0.0))
        s.record_contact(5.0, "drone_ground", [0])
        assert s.summary()["contacts"]["first_contact_root_cause_per_drone"]["vertical_control"] == 1

    def test_pitch_counts_as_well_as_roll(self):
        s = _scorer(1)
        s.record_tick(2.0, 0, 3.0, (0.0, -TILT_LIMIT_RAD - 0.1))
        s.record_contact(2.1, "drone_ground", [0])
        assert s.summary()["contacts"]["first_contact_root_cause_per_drone"]["attitude_loss"] == 1

    def test_obstacle_and_swarm_contacts_attribute_to_their_own_cause(self):
        s = _scorer(3)
        s.record_contact(3.0, "drone_obstacle", [0])
        s.record_contact(3.1, "drone_drone", [1, 2])
        roots = s.summary()["contacts"]["first_contact_root_cause_per_drone"]
        assert roots["obstacle"] == 1 and roots["swarm"] == 2

    def test_only_the_first_contact_of_a_drone_decides_its_root_cause(self):
        s = _scorer(1)
        s.record_contact(3.0, "drone_obstacle", [0])
        s.record_contact(3.2, "drone_ground", [0])            # the fall after the collision
        c = s.summary()["contacts"]
        assert c["first_contact_root_cause_per_drone"] == {"vertical_control": 0, "attitude_loss": 0, "obstacle": 1,
                                                           "swarm": 0}
        assert c["post_contact_ground_events"] == 1 and c["events_by_category"]["drone_ground"] == 1

    def test_first_contact_records_time_and_altitude(self):
        s = _scorer(1)
        s.record_contact(4.5, "drone_ground", [0], altitude_m=0.02)
        rec = s.summary()["contacts"]["first_contacts"][0]
        assert (rec["drone"], rec["t"], rec["altitude_m"]) == (0, 4.5, 0.02)
