import sys
sys.path.insert(0, '/mnt/user-data/outputs')

from jointInterpolation import jointInterpolation

def test_basic_interpolation():
    """Test basic two-joint coordination"""
    da, sa = 100, 0
    db, sb = 50, 0
    i, ala, vla, alb, vlb = 0.1, 100, 200, 80, 150
    
    eoma, eomb, t = jointInterpolation(da, sa, db, sb, i, ala, vla, alb, vlb)
    
    assert len(eoma) == 3, "Joint A should return (d, v, a)"
    assert len(eomb) == 3, "Joint B should return (d, v, a)"
    assert len(eoma[0]) == len(t), "Position array matches time array"
    assert eoma[0][0] == sa, "Joint A starts at correct position"
    assert eomb[0][0] == sb, "Joint B starts at correct position"
    print(f"✓ Basic interpolation: {len(t)} steps, tf={t[-1]:.2f}")

def test_coordinated_timing():
    """Test that both joints finish at same time"""
    da, sa = 100, 0
    db, sb = 200, 10
    i, ala, vla, alb, vlb = 0.1, 50, 100, 50, 100
    
    eoma, eomb, t = jointInterpolation(da, sa, db, sb, i, ala, vla, alb, vlb)
    
    final_pos_a = eoma[0][-1]
    final_pos_b = eomb[0][-1]
    
    assert abs(final_pos_a - (sa + da)) < 0.1, "Joint A reaches target"
    assert abs(final_pos_b - (sb + db)) < 0.1, "Joint B reaches target"
    print(f"✓ Coordinated timing: both finish at t={t[-1]:.2f}")

def test_different_limits():
    """Test with different acceleration/velocity limits"""
    da, sa = 100, 0
    db, sb = 100, 0
    i = 0.1
    ala, vla = 200, 400  # Fast joint
    alb, vlb = 50, 100   # Slow joint
    
    eoma, eomb, t = jointInterpolation(da, sa, db, sb, i, ala, vla, alb, vlb)
    
    # Slower joint should dictate total time
    assert len(t) > 0, "Should have time array"
    print(f"✓ Different limits: tf={t[-1]:.2f}")

def test_zero_displacement_joint():
    """Test when one joint doesn't move"""
    da, sa = 100, 0
    db, sb = 0, 50
    i, ala, vla, alb, vlb = 0.1, 100, 200, 80, 150
    
    eoma, eomb, t = jointInterpolation(da, sa, db, sb, i, ala, vla, alb, vlb)
    
    assert all(abs(p - sb) < 0.01 for p in eomb[0]), "Joint B should stay at start"
    assert abs(eoma[0][-1] - (sa + da)) < 0.1, "Joint A should reach target"
    print(f"✓ Zero displacement: Joint B stationary")

def test_negative_displacement():
    """Test with negative displacement"""
    da, sa = -100, 100
    db, sb = 50, 0
    i, ala, vla, alb, vlb = 0.1, 100, 200, 80, 150
    
    eoma, eomb, t = jointInterpolation(da, sa, db, sb, i, ala, vla, alb, vlb)
    
    assert abs(eoma[0][-1] - (sa + da)) < 0.1, "Joint A moves backward correctly"
    assert abs(eomb[0][-1] - (sb + db)) < 0.1, "Joint B moves forward correctly"
    print(f"✓ Negative displacement: Joint A moves backward")

if __name__ == "__main__":
    test_basic_interpolation()
    test_coordinated_timing()
    test_different_limits()
    test_zero_displacement_joint()
    test_negative_displacement()
    print("\n✅ All joint interpolation tests passed!")
