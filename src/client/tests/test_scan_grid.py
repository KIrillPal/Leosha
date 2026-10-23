import math

import numpy as np

from leo_client.hardware.scan_grid import angle_in_sector, resample


def test_sector_wraps_across_the_back():
    assert angle_in_sector(170.0, 150.0, -150.0)
    assert angle_in_sector(-170.0, 150.0, -150.0)
    assert not angle_in_sector(0.0, 150.0, -150.0)


def test_resample_uses_a_fixed_counter_clockwise_grid():
    angles = np.linspace(-math.pi, math.pi, 40, endpoint=False)
    ranges = np.full(40, 1.5)
    scan = resample(
        angles,
        ranges,
        num_rays=36,
        range_min=0.02,
        range_max=6.0,
        capture_time_ns=1,
        scan_time_sec=0.1,
        blind_start_deg=150.0,
        blind_end_deg=-150.0,
        blind_max_m=0.12,
    )
    assert scan.angle_min == -math.pi
    assert abs(scan.angle_increment - (2 * math.pi / 36)) < 1e-9
    assert len(scan.ranges) == 36
    assert np.isfinite(scan.ranges[18])


def test_blind_zone_becomes_nan():
    angles = np.array([math.radians(170.0)])
    ranges = np.array([0.05])
    scan = resample(
        angles,
        ranges,
        num_rays=36,
        range_min=0.02,
        range_max=6.0,
        capture_time_ns=1,
        scan_time_sec=0.1,
        blind_start_deg=150.0,
        blind_end_deg=-150.0,
        blind_max_m=0.12,
    )
    assert np.isnan(scan.ranges).any()
