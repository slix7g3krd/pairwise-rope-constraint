import pytest
from physics import rope_constraint as mod

CASES = [
    ("mass", 'm.mass_ratio(1.0, 4.0) == approx(0.25)'),
    ("position", 'm.position_correction(0.0, 1.0, 0.5) == approx(0.5)'),
    ("velocity", 'm.velocity_correction(1.0, 2.0) == approx(3.0)'),
    ("swept", 'm.swept_radius(2.0, 0.5) == approx(1.0)'),
    ("wind", 'm.wind_force(1.0, 2.0) == approx(3.0)'),
    ("redistribute", 'm.redistribute_length(10.0, 2) == approx(5.0)'),
    ("substeps", 'm.adaptive_substeps(2.0, 0.5) == 4'),
    ("network", 'm.network_shape([1, 2, 3], 2) == [1, 2]'),
    ("damping_heavy", 'm.segment_damping(1.0, "heavy") == approx(0.5)'),
    ("normal", 'm.collision_normal((0.0, 1.0), True) == approx((0.0, -1.0))'),
    ("order", 'm.deterministic_order([3, 1, 2]) == [1, 2, 3]'),
    ("threshold", 'm.local_threshold(1.0, 2.0) == approx(2.0)'),
    ("bounds", 'm.bounds_update([0.0, 1.0, 3.0]) == approx((0.0, 3.0))'),
    ("substeps_one", 'm.adaptive_substeps(0.1, 0.5) == 1'),
    ("damping_light", 'm.segment_damping(1.0, "light") == approx(1.0)')
]

@pytest.mark.parametrize("name,expr", CASES, ids=[c[0] for c in CASES])
def test_contract(name, expr):
    assert eval(expr, {"m": mod, "approx": pytest.approx})
