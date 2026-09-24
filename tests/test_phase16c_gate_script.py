"""Phase 16C: the validation-gate script's pure parts, plus one tiny end-to-end run.
See scripts/run_phase16c_gate.py."""
import csv
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase16c_gate as gate  # noqa: E402


class TestScenarioGrid:
    def test_modes_times_profiles(self):
        s = gate.build_scenarios(["legacy", "altitude_hold"], ["truth_state", "vio"])
        assert [x.name for x in s] == ["legacy/truth_state", "legacy/vio", "altitude_hold/truth_state", "altitude_hold/vio"]

    def test_unknown_mode_or_profile_is_rejected(self):
        with pytest.raises(ValueError, match="flight_control_mode"):
            gate.build_scenarios(["autopilot"], ["vio"])
        with pytest.raises(ValueError, match="unknown profile"):
            gate.build_scenarios(["legacy"], ["gps"])

    def test_the_old_failure_profiles_carry_the_old_vertical_noise(self):
        from swarm_sim.estimation import PROFILES
        old, three = gate.extra_profile("flow_rf_oldvz"), gate.extra_profile("stress_vz3")
        assert old.sensors[0].vz_noise_std_mps == 0.10 and three.sensors[0].vz_noise_std_mps == 0.15
        assert old.sensors[0].vel_noise_std_mps == PROFILES["flow_rf"].sensors[0].vel_noise_std_mps

    def test_the_test_only_profiles_never_leak_into_the_global_table(self):
        from swarm_sim.estimation import PROFILES, get_profile
        assert "flow_rf_oldvz" not in PROFILES
        with gate.profile_available("flow_rf_oldvz"):
            assert get_profile("flow_rf_oldvz").sensors[0].vz_noise_std_mps == 0.10
        assert "flow_rf_oldvz" not in PROFILES and "stress_vz3" not in PROFILES
        with gate.profile_available("vio"):                       # shipped profiles need nothing
            assert "vio" in PROFILES
        assert "vio" in PROFILES

    def test_the_profile_table_is_restored_even_if_the_run_raises(self):
        from swarm_sim.estimation import PROFILES
        with pytest.raises(RuntimeError):
            with gate.profile_available("stress_vz3"):
                raise RuntimeError("boom")
        assert "stress_vz3" not in PROFILES

    def test_the_test_only_profiles_are_known_scenario_profiles(self):
        assert [s.name for s in gate.build_scenarios(["altitude_hold"], ["flow_rf_oldvz"])] == ["altitude_hold/flow_rf_oldvz"]

    def test_config_carries_the_scenario(self):
        cfg = gate.build_config(gate.Scenario("altitude_hold/vio", "altitude_hold", "vio"), 5, 9.0, 3)
        assert (cfg.flight_control_mode, cfg.localization_mode, cfg.estimator_profile) == ("altitude_hold", "estimated", "vio")
        base = gate.build_config(gate.Scenario("legacy/truth_state", "legacy", "truth_state"), 1, 6.0, 2)
        assert base.localization_mode == "truth_state" and base.flight_control_mode == "legacy"

    def test_every_run_column_is_unique(self):
        assert len(set(gate.RUN_COLUMNS)) == len(gate.RUN_COLUMNS)


def _row(scenario, seed, **kw):
    row = {c: None for c in gate.RUN_COLUMNS}
    row.update(scenario=scenario, seed=seed, contact_steps=0, first_contact_vertical_control=0,
               first_contact_attitude_loss=0, first_contact_obstacle=0, first_contact_swarm=0,
               alt_median_drone_std_m=0.01, alt_max_abs_error_m=0.05, pre_median_drone_std_m=0.01,
               pre_max_abs_error_m=0.05, pre_min_m=2.95, pre_max_m=3.05)
    row.update(kw)
    return row


class TestSummaryAndGate:
    def test_summary_counts_contacts_and_takes_worst_cases(self):
        rows = [_row("a", 1, contact_steps=5, first_contact_vertical_control=2, first_contact_attitude_loss=1,
                     alt_max_abs_error_m=0.4, alt_min_m=2.7),
                _row("a", 2, first_contact_obstacle=1, alt_max_abs_error_m=0.1, alt_min_m=2.9)]
        s = gate.summarize(rows)["a"]
        assert s["runs"] == 2 and s["runs_with_contact"] == 1
        assert (s["drones_first_contact_vertical_control"], s["drones_first_contact_obstacle"],
                s["drones_first_contact_attitude_loss"]) == (2, 1, 1)
        assert s["alt_worst_abs_error_m"] == 0.4 and s["alt_min_m"] == 2.7

    def test_g2_passes_when_all_three_criteria_hold(self):
        rows = [_row("altitude_hold/vio", k) for k in range(3)]
        g = gate.evaluate_gate_g2(rows, gate.summarize(rows))["altitude_hold/vio"]
        assert g["pass"] and all(g["checks"].values())

    @pytest.mark.parametrize("bad, failing", [
        (dict(first_contact_vertical_control=1), "no_vertical_control_contacts"),
        (dict(pre_max_abs_error_m=0.9), "within_band"),
        (dict(pre_median_drone_std_m=0.4), "median_altitude_std_ok"),
    ])
    def test_g2_fails_on_each_criterion_separately(self, bad, failing):
        rows = [_row("altitude_hold/vio", 1), _row("altitude_hold/vio", 2, **bad)]
        if failing == "median_altitude_std_ok":
            rows = [_row("altitude_hold/vio", k, **bad) for k in range(3)]
        g = gate.evaluate_gate_g2(rows, gate.summarize(rows))["altitude_hold/vio"]
        assert not g["pass"] and g["checks"][failing] is False

    def test_the_gate_only_judges_altitude_hold_rows(self):
        rows = [_row("legacy/vio", 1, first_contact_vertical_control=3)]
        assert gate.evaluate_gate_g2(rows, gate.summarize(rows)) == {}

    def test_obstacle_swarm_and_tumble_contacts_are_reported_but_not_gated(self):
        rows = [_row("altitude_hold/vio", 1, first_contact_obstacle=2, first_contact_swarm=2,
                     first_contact_attitude_loss=1, contact_steps=40)]
        assert gate.evaluate_gate_g2(rows, gate.summarize(rows))["altitude_hold/vio"]["pass"]

    def test_the_altitude_criteria_use_the_pre_contact_numbers_not_the_post_crash_fall(self):
        """A drone that collides and then falls has a huge all-samples error; the gate must not blame the
        altitude controller for it - but an excursion BEFORE the contact must still fail the gate."""
        crashed = [_row("altitude_hold/vio", 1, first_contact_swarm=2, alt_max_abs_error_m=3.0, alt_min_m=0.0)]
        assert gate.evaluate_gate_g2(crashed, gate.summarize(crashed))["altitude_hold/vio"]["pass"]
        drifted = [_row("altitude_hold/vio", 1, first_contact_swarm=2, alt_max_abs_error_m=3.0, pre_max_abs_error_m=0.8)]
        assert not gate.evaluate_gate_g2(drifted, gate.summarize(drifted))["altitude_hold/vio"]["pass"]

    def test_print_table_handles_none(self, capsys):
        rows = [_row("legacy/truth_state", 1, alt_median_drone_std_m=None, alt_max_abs_error_m=None,
                     pre_median_drone_std_m=None, pre_max_abs_error_m=None, pre_min_m=None, pre_max_m=None)]
        summary = gate.summarize(rows)
        gate.print_table(summary, gate.evaluate_gate_g2(rows, summary))
        assert "legacy/truth_state" in capsys.readouterr().out


def test_tiny_end_to_end_run_writes_all_outputs(tmp_path):
    out = tmp_path / "gate"
    code = gate.main(["--profiles", "truth_state,stress", "--num-seeds", "1", "--duration", "5", "--num-drones", "3",
                      "--out-dir", str(out)])
    assert code == 0
    rows = list(csv.DictReader(open(out / "runs.csv")))
    assert [r["scenario"] for r in rows] == ["legacy/truth_state", "legacy/stress", "altitude_hold/truth_state",
                                             "altitude_hold/stress"]
    assert list(rows[0]) == gate.RUN_COLUMNS
    hold = [r for r in rows if r["scenario"].startswith("altitude_hold")]
    assert all(float(r["alt_std_m"]) < 0.05 for r in hold)
    summary = json.load(open(out / "summary.json"))
    assert set(summary["gate_g2"]) == {"altitude_hold/truth_state", "altitude_hold/stress"}
    manifests = json.load(open(out / "manifests.json"))
    assert manifests["altitude_hold/stress"]["model_id"] == "flight=altitude_hold/localization=estimated/profile=stress"
