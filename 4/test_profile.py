import sys

sys.path.insert(0, "/mnt/user-data/outputs")

from trajectory import profile


def test_basic_motion():
    """Test basic trapezoidal motion trajectory"""
    d, s, t, ta = 100, 0, [0, 0.5, 1, 1.5, 2], 0.5
    pos, vel, acc = profile(d, s, t, ta)

    assert pos[0] == s, f"Start position should be {s}"
    assert abs(pos[-1] - (s + d)) < 0.01, f"End position should be {s + d}"
    assert vel[0] == 0, "Start velocity should be 0"
    assert abs(vel[-1]) < 0.01, "End velocity should be 0"
    print("✓ Basic motion test passed")


def test_zero_displacement():
    """Test with zero displacement"""
    d, s, t, ta = 0, 10, [0, 0.5, 1], 0.5
    pos, vel, acc = profile(d, s, t, ta)

    assert all(p == s for p in pos), "All positions should equal start"
    print("✓ Zero displacement test passed")


def test_negative_displacement():
    """Test with negative displacement"""
    d, s, t, ta = -50, 100, [0, 0.25, 0.5, 0.75, 1], 0.25
    pos, vel, acc = profile(d, s, t, ta)

    assert pos[0] == s, "Start position correct"
    assert abs(pos[-1] - (s + d)) < 0.01, "End position correct for negative move"
    print("✓ Negative displacement test passed")


def test_symmetry():
    """Test acceleration symmetry"""
    d, s, t, ta = 100, 0, [0, 0.5, 1, 1.5, 2], 0.5
    pos, vel, acc = profile(d, s, t, ta)

    assert abs(acc[0] + acc[-1]) < 0.01, "Accel should be symmetric"
    print("✓ Symmetry test passed")


if __name__ == "__main__":
    test_basic_motion()
    test_zero_displacement()
    test_negative_displacement()
    test_symmetry()
    print("\n✅ All trajectory tests passed!")
