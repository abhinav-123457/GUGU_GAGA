"""Phase 16B: the scenario-sweep script's pure parts, plus one tiny end-to-end
run. See scripts/run_phase16b_scenarios.py."""
import csv
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase16b_scenarios as sweep  # noqa: E402


class TestScenarioGrid:
    def test_baseline_plus_profiles_times_k_values(self):
        scenarios = sweep.build_scenarios(["ideal", "vio"], [0.0, 3.0])
        assert [s.name for s in scenarios] == ["truth_state", "ideal/k=0", "ideal/k=3", "vio/k=0", "vio/k=3"]
        assert scenarios[0].localization_mode == "truth_state" and scenarios[1].localization_mode == "estimated"

    def test_baseline_can_be_omitted(self):
        assert [s.name for s in sweep.build_scenarios(["vio"], [0.0], include_baseline=False)] == ["vio/k=0"]

    def test_unknown_profile_is_rejected(self):
        with pytest.raises(ValueError, match="unknown profile"):
            sweep.build_scenarios(["gps"], [0.0])

    def test_config_carries_the_scenario(self):
        s = sweep.build_scenarios(["stress"], [3.0], include_baseline=False)[0]
        cfg = sweep.build_config(s, seed=7, duration_s=12.0, num_drones=5)
        assert (cfg.localization_mode, cfg.estimator_profile, cfg.safety_pose_sigma_geofence_k) == ("estimated", "stress", 3.0)
        assert (cfg.seed, cfg.duration_sec, cfg.num_drones) == (7, 12.0, 5)
        base = sweep.build_config(sweep.build_scenarios([], [])[0], seed=1, duration_s=6.0, num_drones=3)
        assert base.localization_mode == "truth_state" and base.safety_pose_sigma_geofence_k == 0.0

    def test_every_run_column_is_unique(self):
        assert len(set(sweep.RUN_COLUMNS)) == len(sweep.RUN_COLUMNS)


def _row(scenario, seed, **kw):
    row = {c: None for c in sweep.RUN_COLUMNS}
    row.update(scenario=scenario, seed=seed)
    row.update(kw)
    return row


class TestSummarize:
    def test_means_and_worst_cases_per_scenario(self):
        rows = [_row("a", 1, victims_found=2, loc_mean_error_m=1.0, loc_max_error_m=3.0, true_geofence_max_excursion_m=0.0),
                _row("a", 2, victims_found=4, loc_mean_error_m=3.0, loc_max_error_m=9.0, true_geofence_max_excursion_m=2.5),
                _row("b", 1, victims_found=1)]
        out = sweep.summarize(rows)
        assert list(out) == ["a", "b"]
        assert out["a"]["runs"] == 2 and out["a"]["seeds"] == [1, 2]
        assert out["a"]["victims_found"] == 3.0 and out["a"]["loc_mean_error_m"] == 2.0
        assert out["a"]["loc_worst_error_m"] == 9.0 and out["a"]["true_geofence_worst_excursion_m"] == 2.5

    def test_altitude_range_takes_the_lowest_minimum_and_highest_maximum_across_runs(self):
        out = sweep.summarize([_row("a", 1, alt_min_m=2.5, alt_max_m=3.5), _row("a", 2, alt_min_m=1.2, alt_max_m=3.1)])
        assert out["a"]["true_altitude_min_m"] == 1.2 and out["a"]["true_altitude_max_m"] == 3.5

    def test_missing_values_stay_none(self):
        out = sweep.summarize([_row("truth", 1, victims_found=1)])
        assert out["truth"]["loc_mean_error_m"] is None and out["truth"]["loc_worst_error_m"] is None

    def test_print_table_handles_none(self, capsys):
        sweep.print_table(sweep.summarize([_row("truth", 1, victims_found=1), _row("x", 1, victims_found=2,
                                                                                    loc_mean_error_m=1.0)]))
        text = capsys.readouterr().out
        assert "truth" in text and "-" in text


def test_tiny_end_to_end_run_writes_all_outputs(tmp_path):
    out = tmp_path / "sweep"
    code = sweep.main(["--profiles", "ideal,stress", "--k-values", "0", "--num-seeds", "1", "--duration", "3",
                       "--num-drones", "3", "--out-dir", str(out)])
    assert code == 0
    rows = list(csv.DictReader(open(out / "runs.csv")))
    assert [r["scenario"] for r in rows] == ["truth_state", "ideal/k=0", "stress/k=0"]
    assert list(rows[0]) == sweep.RUN_COLUMNS
    assert float(rows[1]["loc_max_error_m"]) < 1e-9 and float(rows[2]["loc_mean_error_m"]) > 0.0
    assert 0.0 < float(rows[1]["alt_min_m"]) <= float(rows[1]["alt_max_m"])         # true altitude range is reported
    summary = json.load(open(out / "summary.json"))
    assert set(summary["scenarios"]) == {"truth_state", "ideal/k=0", "stress/k=0"}
    manifests = json.load(open(out / "manifests.json"))
    assert manifests["stress/k=0"]["model_id"] == "localization=estimated/profile=stress/k=0"
    assert len(manifests["stress/k=0"]["map_id"]) == 12
    assert any(name.startswith("odometry/") for name in manifests["stress/k=0"]["subsystem_seeds"])
    assert not any(name.startswith("odometry/") for name in manifests["truth_state"]["subsystem_seeds"])
