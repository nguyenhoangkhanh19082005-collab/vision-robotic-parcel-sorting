"""Run recorder: one folder per parcel with arrival frames, burst and meta.json."""

from __future__ import annotations

import json

import numpy as np

from machine_vision.ui.run_recorder import RunRecorder


def test_parcel_folders_hold_frames_burst_and_meta(tmp_path):
    recorder = RunRecorder(tmp_path, keep=5, min_free_mb=0)
    frame = np.full((720, 1280, 3), 60, np.uint8)
    for counter in range(1, 9):
        recorder.frame(counter, counter / 30, frame, present=counter > 3)
    recorder.fired(8, 8 / 30, {"start": 0.2, "end": 0.8})
    recorder.burst([frame, frame])
    recorder.result({"decision": "PASS", "qr_text": "SPX1"})
    recorder.next_parcel()
    for counter in range(9, 12):                 # empty belt only: nothing written
        recorder.frame(counter, counter / 30, frame, present=False)
    recorder.close()

    runs = list(tmp_path.iterdir())
    assert len(runs) == 1
    parcels = sorted(p.name for p in runs[0].iterdir())
    assert parcels == ["p01"]
    p01 = runs[0] / "p01"
    assert sorted(f.name for f in (p01 / "arrival").glob("*.jpg")) == [
        f"f{c:07d}.jpg" for c in range(4, 9)    # only the last ``keep`` frames
    ]
    stamps = json.loads((p01 / "arrival" / "stamps.json").read_text(encoding="utf-8"))
    assert stamps["f0000008.jpg"] == round(8 / 30, 4)
    assert sorted(f.name for f in (p01 / "burst").iterdir()) == ["b00.jpg", "b01.jpg"]
    meta = json.loads((p01 / "meta.json").read_text(encoding="utf-8"))
    assert meta["fired_frame"] == 8 and meta["results"][0]["decision"] == "PASS"


def test_recording_stops_when_the_disk_is_nearly_full(tmp_path):
    recorder = RunRecorder(tmp_path, min_free_mb=10**12)
    recorder.burst([np.zeros((8, 8, 3), np.uint8)])
    recorder.close()
    assert not recorder.enabled
    assert not any(tmp_path.rglob("*.jpg"))


def test_frames_after_the_trigger_are_kept(tmp_path):
    import time

    from machine_vision.ui.camera_worker import FrameBuffer

    frames = FrameBuffer(4)
    recorder = RunRecorder(tmp_path, keep=50, min_free_mb=0)
    frame = np.full((72, 128, 3), 60, np.uint8)
    for counter in range(1, 4):
        recorder.frame(counter, counter / 30, frame, present=True)
        frames.append(frame)
    recorder.fired(3, 0.1, {}, frames.latest, tail_s=0.3)
    for _ in range(5):                       # camera keeps running while the belt brakes
        time.sleep(0.03)
        frames.append(frame)
    time.sleep(0.35)
    recorder.close()
    p01 = next(tmp_path.iterdir()) / "p01"
    meta = json.loads((p01 / "meta.json").read_text(encoding="utf-8"))
    assert meta["tail_frames"] == 5
    assert len(list((p01 / "arrival").glob("*.jpg"))) == 8
