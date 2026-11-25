"""Camera arrival trigger on synthetic belt frames."""

from __future__ import annotations

import numpy as np

from machine_vision.services.arrival import ArrivalDetector, ArrivalSettings

H, W = 360, 640


def _background() -> np.ndarray:
    frame = np.full((H, W, 3), (40, 90, 40), np.uint8)   # green belt
    frame[:, :120] = (120, 120, 120)                    # grey board
    frame[:, -120:] = (120, 120, 120)
    return frame


def _frame(centre_y: float | None, height: int = 120) -> np.ndarray:
    frame = _background()
    if centre_y is not None:
        top = int(centre_y - height / 2)
        frame[max(0, top):max(0, top + height), 200:440] = (90, 150, 200)  # cardboard
    return frame


def _run(positions, settings=None, height=120, dt=1 / 30):
    detector = ArrivalDetector(_background(), settings or ArrivalSettings(latency_s=0.0))
    fired = []
    for index, centre in enumerate(positions):
        if detector.update(_frame(centre, height), index * dt):
            fired.append(index)
    return detector, fired


def _fully_inside(centre, height=120, margin=0.02):
    return centre - height / 2 >= margin * H and centre + height / 2 <= (1 - margin) * H


def test_fires_once_centred_and_only_when_fully_inside_either_direction():
    down = list(range(-60, 420, 12))
    for positions in (down, list(reversed(down))):
        _det, fired = _run(positions)
        assert len(fired) == 1
        centre = positions[fired[0]]
        assert _fully_inside(centre)
        assert abs(centre - H / 2) <= 12


def test_parcel_longer_than_the_view_stops_once_with_its_leading_end_inside():
    # 2026-11-25: a parcel laid diagonally spans ~0.95 of the view and is never fully
    # inside. It is stopped once, when its leading end is about to leave, trailing end cut.
    for height in (345, 380):                         # 0.96 and 1.06 of the 360 px view
        positions = list(range(-200, 560, 10))
        det, fired = _run(positions, height=height)
        assert len(fired) == 1, height
        centre = positions[fired[0]]
        top, bottom = centre - height / 2, centre + height / 2
        assert top <= 0.01 * H and bottom <= 0.98 * H, (height, top, bottom)


def test_fully_insidable_parcel_never_uses_the_overlong_rule():
    det, fired = _run(list(range(-200, 560, 10)), height=300)
    assert len(fired) == 1 and not det.state.overlong


def test_latency_compensation_stops_earlier_but_still_inside():
    positions = list(range(-60, 420, 6))   # 6 px/frame at 30 fps = 180 px/s
    _d, plain = _run(positions, ArrivalSettings(latency_s=0.0))
    _d, lead = _run(positions, ArrivalSettings(latency_s=0.2))
    # 0.2 s x 180 px/s = 36 px = 6 frames earlier
    assert plain[0] - lead[0] == 6
    assert _fully_inside(positions[lead[0]])


def test_long_parcel_stops_before_its_front_leaves_the_view():
    # 300 px parcel in a 360 px view: fully inside only briefly, never centred late.
    positions = list(range(-200, 560, 4))
    _d, fired = _run(positions, ArrivalSettings(latency_s=0.0), height=300)
    assert len(fired) == 1
    assert _fully_inside(positions[fired[0]], height=300)


def test_empty_belt_and_small_debris_never_fire():
    detector = ArrivalDetector(_background(), ArrivalSettings())
    for index in range(30):
        frame = _background()
        frame[100:106, 300:306] = (255, 255, 255)   # glare speck
        assert not detector.update(frame, index / 30)


def test_static_blob_never_fires_and_empty_view_still_rearms():
    # 2026-11-24: the belt runs while the detector is fed, so a parcel always moves.
    # A blob resting fully inside (glare, shadow, parcel held by hand) is not stopped.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    events = []
    sequence = [100.0] * 10 + [None] * 5 + list(range(-60, 420, 12)) + [None] * 5 \
        + list(range(-60, 420, 12))
    for index, centre in enumerate(sequence):
        if detector.update(_frame(centre), index / 30):
            events.append(index)
    # Each moving parcel fires once; the resting one never; an empty view re-arms.
    assert len(events) == 2
    assert events[0] > 15 and events[1] > 15 + 40 + 5


def _placed_by_hand(fps: float, speed_px_s: float, height: int = 200, arm_frames: int = 6):
    """Parcel put on the belt head *under* the camera: an arm covers it from the top
    edge for the first frames, then lets go (blob shrinks, centroid jumps)."""
    frames, centre, step = [], 10 + height / 2, speed_px_s / fps
    while centre - height / 2 < H:
        frame = _frame(centre, height)
        if len(frames) < arm_frames:
            frame[: int(centre - height / 2) + 40, 280:360] = (30, 30, 160)   # sleeve
        frames.append((frame, centre))
        centre += step
    return frames


def test_hand_leaving_the_parcel_does_not_fake_a_fast_parcel():
    # 2026-11-23 field fault: the centroid jump when the hand left read as a very fast
    # parcel, the stop was predicted to overshoot and the camera never fired.
    for fps in (30, 12, 8):
        detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.15))
        fired = [c for i, (f, c) in enumerate(_placed_by_hand(fps, 60.0))
                 if detector.update(f, i / fps)]
        assert len(fired) == 1, fps
        # Where the belt really stops (0.15 s later) the parcel is still fully inside.
        assert _fully_inside(fired[0] + 60.0 * 0.15, height=200), (fps, fired)


def test_slow_camera_still_catches_a_long_parcel():
    # 300 px parcel in a 360 px view at 8 fps: only one or two frames show it whole.
    for speed in (60.0, 120.0):
        positions = [-200 + speed / 8 * k for k in range(60)]
        detector, fired = _run(positions, ArrivalSettings(latency_s=0.0), height=300, dt=1 / 8)
        assert len(fired) == 1, speed
        assert _fully_inside(positions[fired[0]], height=300)


def test_overshoot_reports_how_far_the_leading_end_is_out():
    detector = ArrivalDetector(_background(), ArrivalSettings(edge_margin=0.02))
    out = detector.measure(_frame(H - 40, height=120))     # leading (bottom) end cut off
    inside = detector.measure(_frame(H / 2, height=120))
    assert detector.overshoot(out, +1) > 0   # end cut at the border: only "out" is known
    assert detector.overshoot(inside, +1) <= 0
    assert detector.overshoot(inside, 0) == 0.0


def test_camera_overshoot_waits_for_fresh_frames_after_the_stop():
    import threading
    import time

    from machine_vision.ui.auto_cycle import camera_overshoot
    from machine_vision.ui.camera_worker import FrameBuffer

    detector = ArrivalDetector(_background(), ArrivalSettings())
    detector.state.heading = 1.0
    frames = FrameBuffer(4)
    frames.append(_frame(H / 2))                       # stale frame: parcel still inside

    def camera():
        for _ in range(3):
            time.sleep(0.02)
            frames.append(_frame(H - 40))              # after the stop: front end out

    threading.Thread(target=camera).start()
    assert camera_overshoot(frames, detector, timeout_s=1.0)() > 0
    assert camera_overshoot(FrameBuffer(2), detector, timeout_s=0.05)() is None


def _moved_view() -> np.ndarray:
    moved = np.roll(_background(), 150, axis=1)
    return (moved.astype(np.int16) + (30, -20, 40)).clip(0, 255).astype(np.uint8)


def test_moved_camera_is_learnt_and_the_next_parcel_still_stops():
    # Reference shot before the camera moved: every frame differs edge to edge. The
    # difference does not move, so it is learnt into the reference within ~4 s.
    detector = ArrivalDetector(_moved_view() * 0 + _background(), ArrivalSettings(latency_s=0.0))
    for k in range(150):                                   # 5 s, belt running, no parcel
        assert not detector.update(_moved_view(), k / 30)
        assert not detector.background_mismatch(k / 30, hold_s=6.0)
    assert detector.measure(_moved_view()) is None         # view learnt as background
    fired = []
    for k, centre in enumerate(range(-60, 420, 12)):
        frame = _moved_view()
        top = int(centre - 60)
        frame[max(0, top):max(0, top + 120), 200:440] = (90, 150, 200)
        if detector.update(frame, 5 + k / 30):
            fired.append(centre)
    assert len(fired) == 1 and _fully_inside(fired[0])


def test_moved_camera_is_reported_when_learning_is_off():
    detector = ArrivalDetector(_background(), ArrivalSettings(static_absorb_s=0.0))
    states = [detector.update(_moved_view(), k / 30) or detector.background_mismatch(k / 30)
              for k in range(120)]
    assert any(states) and not any(states[:80])        # reported after 3 s, never fired
    # A real parcel never spans the whole view: no false alarm.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    for k, centre in enumerate(range(-200, 560, 4)):
        detector.update(_frame(centre, 300), k / 8)
        assert not detector.background_mismatch(k / 8)


def _with_glare(frame: np.ndarray) -> np.ndarray:
    frame = frame.copy()
    band = frame[210:250, 160:480].astype(np.int16) + 120     # lamp reflection on the belt
    frame[210:250, 160:480] = band.clip(0, 255).astype(np.uint8)
    return frame


def test_every_parcel_stops_although_a_static_glare_never_leaves_the_view():
    # 2026-11-24 field fault (logs/arrival/arrival_20261124_1950*.csv): after the first
    # stop the view never emptied (a static changed band), the detector stayed disarmed
    # and the next parcel (a rejected one) ran through. The cycle now re-arms per parcel.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    t, stops = 0.0, []
    for _ in range(30):                                     # glare appears, belt running
        detector.update(_with_glare(_background()), t)
        t += 1 / 30
    for parcel, colour in enumerate([(90, 150, 200), (60, 110, 160), (90, 150, 200)]):
        detector.rearm()                                    # cycle: next parcel
        for centre in range(-60, 480, 10):
            frame = _background()
            top = int(centre - 60)
            frame[max(0, top):max(0, top + 120), 200:440] = colour
            if detector.update(_with_glare(frame), t):
                stops.append((parcel, centre))
                t += 2.0                                    # belt stopped for inspection
                break
            t += 1 / 30
    assert [p for p, _ in stops] == [0, 1, 2]
    assert all(_fully_inside(c) for _, c in stops)


def test_lamp_off_is_reported():
    detector = ArrivalDetector(_background(), ArrivalSettings())
    dark = (_background() * 0.35).astype(np.uint8)
    states = [detector.update(dark, k / 30) or detector.light_fault(k / 30) for k in range(90)]
    assert any(states[61:]) and not any(states[:59])
    # Normal parcels (+ the brightness of a big carton) do not trip it.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    for k, centre in enumerate(range(-200, 560, 6)):
        detector.update(_frame(centre, 300), k / 30)
        assert not detector.light_fault(k / 30)


def test_moving_lamp_reflection_is_not_a_parcel_but_a_carton_is():
    # 2026-11-25 field fault: on the glossy wavy belt the lamp reflection is a dim,
    # blurred band that moves and changes shape while the belt runs; it was stopped
    # as a "parcel". A carton (bright, sharp) still fires. Real belt: dark green (L ~15-20).
    import cv2

    def belt():
        frame = np.full((H, W, 3), (15, 35, 15), np.uint8)
        frame[:, :120] = (120, 120, 120)
        frame[:, -120:] = (120, 120, 120)
        return frame

    def glare(frame, centre, width):
        layer = np.zeros(frame.shape[:2], np.float32)
        top = int(centre - 30)
        layer[max(0, top):max(0, top + 60), 320 - width:320 + width] = 1.0
        layer = cv2.GaussianBlur(layer, (0, 0), 10)
        out = frame.astype(np.float32) + layer[..., None] * np.array([30, 48, 30], np.float32)
        return out.clip(0, 255).astype(np.uint8)

    detector = ArrivalDetector(belt(), ArrivalSettings(latency_s=0.0))
    for k, centre in enumerate(range(-40, 420, 6)):           # moves and changes shape
        assert not detector.update(glare(belt(), centre, 80 + k % 40), k / 30), centre
    detector.rearm()
    fired = []
    for k, centre in enumerate(range(-60, 420, 12)):
        frame = belt()
        top = int(centre - 60)
        frame[max(0, top):max(0, top + 120), 200:440] = (90, 150, 200)    # carton
        if detector.update(glare(frame, 380 - 3 * k, 100), 3 + k / 30):
            fired.append(centre)
    assert len(fired) == 1 and _fully_inside(fired[0])


def test_overlong_parcel_with_a_jumping_leading_end_still_stops():
    # 2026-11-25 lần 3: corner + glare made the leading end read 0.92 -> 0.85 -> 1.00, so
    # the single "about to leave" frame was skipped and the parcel ran through.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.12))
    fired = []
    for k, top in enumerate(range(-300, 100, 4)):           # 380 px parcel, 4 px/frame
        frame = _background()
        bottom = top + 380
        if k % 3 == 1:
            bottom -= 25                                     # corner read short now and then
        frame[max(0, top):max(0, min(H, bottom)), 200:440] = (90, 150, 200)
        if detector.update(frame, k / 30):
            fired.append(bottom / H)
    assert len(fired) == 1 and fired[0] <= 0.98, fired


def test_passed_downstream_only_when_a_moving_blob_reaches_the_far_edge():
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    for k, centre in enumerate(range(-60, 200, 12)):         # enters, still in the view
        detector.update(_frame(centre), k / 30)
    assert not detector.state.passed_downstream
    for k, centre in enumerate(range(200, 460, 12)):         # leaves at the bottom
        detector.update(_frame(centre), 1 + k / 30)
    assert detector.state.passed_downstream
    detector.rearm()
    assert not detector.state.passed_downstream


def test_overlong_parcel_whose_leading_end_jumps_out_still_stops_late():
    # Leading end seen inside (trailing cut), next frame read beyond the edge: stop now.
    detector = ArrivalDetector(_background(), ArrivalSettings(latency_s=0.0))
    fired = []
    tops = list(range(-300, -40, 6)) + [-10]           # last step: 380 px parcel jumps out
    for k, top in enumerate(tops):
        frame = _background()
        frame[max(0, top):max(0, min(H, top + 380)), 200:440] = (90, 150, 200)
        if detector.update(frame, k / 30):
            fired.append(top)
    assert len(fired) == 1 and detector.state.overlong
