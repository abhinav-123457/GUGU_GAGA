"""Phase 16B: the localisation flags on run_mission.py's CLI."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import run_mission  # noqa: E402


def _parse(*argv):
    return run_mission.build_parser().parse_args(list(argv))


def test_defaults_keep_the_legacy_pipeline():
    args = _parse()
    assert args.localization == "truth_state" and args.geofence_sigma_k == 0.0
    assert args.estimator_profile == "fused" and args.estimator_noise_scale == 1.0


@pytest.mark.parametrize("profile", ["ideal", "vio", "flow_rf", "lidar", "fused", "stress"])
def test_every_estimator_profile_is_selectable(profile):
    assert _parse("--localization", "estimated", "--estimator-profile", profile).estimator_profile == profile


def test_unknown_localization_or_profile_is_rejected():
    for bad in (["--localization", "gps"], ["--estimator-profile", "gps"]):
        with pytest.raises(SystemExit):
            _parse(*bad)


def test_numeric_flags_parse():
    args = _parse("--estimator-noise-scale", "0.3", "--geofence-sigma-k", "3")
    assert args.estimator_noise_scale == 0.3 and args.geofence_sigma_k == 3.0
