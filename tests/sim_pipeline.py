"""Shared test helper: run the analysis stages on a simulated session (as in test_integrate)."""
from types import SimpleNamespace

from powerkettle.anchors import AnchorConfig, detect_anchors, refine_rests
from powerkettle.bell import BellProfile
from powerkettle.calibration import calibrate
from powerkettle.imu import scale_run
from powerkettle.integrate import power, segment_position, segment_velocity
from powerkettle.kinematics import com_acceleration
from powerkettle.orientation import estimate_orientation
from powerkettle.simulate import SimConfig, simulate

BELL = BellProfile()


def run_stages(moves, cfg=SimConfig(), bell=BELL):
    """Scale, calibrate, orientation, COM acceleration, anchors, velocity / position,
    refine_rests, then velocity / position / power once more on the refined anchors."""
    run, truth = simulate(moves, bell, cfg)
    imu = scale_run(run)
    calib = calibrate(imu, bell)
    q, bias = estimate_orientation(imu, calib, return_bias=True)
    kin = com_acceleration(imu, q, calib, bias)
    raw_anchors = detect_anchors(imu, q, kin, calib)
    v = segment_velocity(kin.a_com, imu.t, raw_anchors)
    pos = segment_position(v, imu.t, raw_anchors)
    anchors = refine_rests(raw_anchors, pos, AnchorConfig())
    v = segment_velocity(kin.a_com, imu.t, anchors)
    pos = segment_position(v, imu.t, anchors)
    P = power(kin.a_com, v, bell.mass_kg)
    return SimpleNamespace(imu=imu, truth=truth, calib=calib, q=q, kin=kin, anchors=anchors,
                           v=v, pos=pos, P=P)
