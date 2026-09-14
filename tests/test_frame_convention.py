"""Fixture documenting this project's current PyBullet<->ENU axis
convention (see swarm_sim.contracts.Frame's docstring). PyBullet itself
has no notion of ENU/NED - this is mission.py's application-level choice,
written down once so Phase 6's adapter can test the exact mapping instead
of each call site re-deriving it. No frame conversion happens in Phase 1;
this test only pins down that the documented convention exists and hasn't
silently changed shape.
"""
from swarm_sim.contracts import Frame, PYBULLET_LOCAL_ENU_AXIS_CONVENTION


def test_frame_enum_distinguishes_enu_ned_body():
    assert Frame.LOCAL_ENU is not Frame.LOCAL_NED
    assert Frame.LOCAL_ENU is not Frame.BODY
    assert Frame.LOCAL_NED is not Frame.BODY


def test_documented_pybullet_enu_axis_convention_fixture():
    assert set(PYBULLET_LOCAL_ENU_AXIS_CONVENTION.keys()) == {"x", "y", "z"}
    for axis, description in PYBULLET_LOCAL_ENU_AXIS_CONVENTION.items():
        assert isinstance(description, str) and len(description) > 0
