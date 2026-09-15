"""CLI/parser tests for run_mission.py's --flock-model choice - added
after Phase 3 review found config.py/SwarmController accept "olfati_saber"
but the CLI's argparse choices didn't. Parser-only: never runs a mission.
"""
import pytest

from run_mission import build_parser


@pytest.mark.parametrize("model", ["couzin", "boids", "vicsek", "olfati_saber"])
def test_flock_model_choice_accepted(model):
    args = build_parser().parse_args(["--flock-model", model])
    assert args.flock_model == model


def test_flock_model_default_is_couzin():
    args = build_parser().parse_args([])
    assert args.flock_model == "couzin"


def test_unknown_flock_model_rejected():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--flock-model", "not_a_real_model"])
