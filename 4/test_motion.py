import sys
sys.path.insert(0, '/mnt/user-data/outputs')

from motion import motion

def test_basic_motion():
    """Test basic motion planning"""
    d, i, al, vl = 100, 0.1, 50, 100
    t, ta = motion(d, i, al, vl)
    
    assert t[0] == 0, "Should start at t=0"
    assert t[-1] >= 0, "Should have positive end time"
    assert ta > 0, "Ta should be positive"
    print(f"✓ Basic motion: tf={t[-1]:.2f}, ta={ta:.2f}")

def test_triangular_profile():
    """Test when displacement is small (triangular profile)"""
    d, i, al, vl = 10, 0.05, 50, 100
    t, ta = motion(d, i, al, vl)
    
    # Triangular: d = al * ta^2, so ta = sqrt(d/al)
    expected_ta = (abs(d) / al) ** 0.5
    assert abs(ta - expected_ta) < 0.01, f"Ta should be ~{expected_ta:.3f}"
    print(f"✓ Triangular profile: ta={ta:.3f}")

def test_trapezoidal_profile():
    """Test when displacement is large (trapezoidal profile)"""
    d, i, al, vl = 1000, 0.1, 50, 100
    t, ta = motion(d, i, al, vl)
    
    expected_ta = vl / al
    assert abs(ta - expected_ta) < 0.01, f"Ta should be {expected_ta}"
    print(f"✓ Trapezoidal profile: ta={ta:.2f}")

def test_different_displacements():
    """Test that larger displacement gives longer time"""
    d1, d2, i, al, vl = 50, 100, 0.1, 50, 100
    t1, ta1 = motion(d1, i, al, vl)
    t2, ta2 = motion(d2, i, al, vl)
    
    assert t2[-1] > t1[-1], "Larger displacement should take longer"
    print(f"✓ Displacement scaling: d={d1} → tf={t1[-1]:.2f}, d={d2} → tf={t2[-1]:.2f}")

def test_interval_step():
    """Test that time array respects interval"""
    d, i, al, vl = 100, 0.2, 50, 100
    t, ta = motion(d, i, al, vl)
    
    for j in range(len(t) - 1):
        step = t[j + 1] - t[j]
        assert step <= i + 0.01, f"Step should be <= interval"
    print(f"✓ Interval test: {len(t)} time steps")

if __name__ == "__main__":
    test_basic_motion()
    test_triangular_profile()
    test_trapezoidal_profile()
    test_different_displacements()
    test_interval_step()
    print("\n✅ All motion tests passed!")
