"""Camera trigger: decide when a parcel moving on the belt may be stopped for inspection.

Replaces the photo sensor at the inspection point (the sensor sits at the pick
point, where position accuracy matters). Every live frame is compared with the
empty-belt reference on a small downscaled copy (~2 ms); the largest changed blob
is the parcel.

Stop rule (2026-11-22, operator requirement): the STOP is sent **only when the
whole parcel is inside the view**: both its ends along the travel axis are at least
``edge_margin`` away from the edges of the ROI, now *and* after the stop latency
(``velocity x latency_s``, camera + USB + UDP + motor). Among those moments the
trigger waits until the parcel centre is predicted to reach ``center_line``, so the
label sits in the middle of the image, unless waiting longer would let the leading
end leave the view; then it stops at once. A parcel resting fully inside the view
(placed by hand, cycle started with a parcel under the camera) triggers too.

The rule does not depend on the travel direction (velocity sign is measured).
After a trigger the detector is *disarmed* until the view has been empty for
``empty_frames`` frames, so the inspected parcel never triggers twice.

2026-11-23 (missed stops on the real belt): the belt head is inside the view, so
parcels are placed by hand under the camera. When the hand leaves, the blob
(parcel + arm) shrinks at once and its *centroid* jumps; the old velocity (centroid
over 6 frames) read that as a very fast parcel, predicted it would leave the view
during the stop latency and never fired. Velocity now comes from the parcel *ends*
(median of frame-to-frame steps, an end clamped at the ROI edge is ignored), so
one jump does not count. The "last chance" test uses the measured frame interval
instead of an assumed 30 fps (the GUI camera ran at ~12 fps). If the parcel is
inside now but the next frame would be too late, it fires now: the cycle then backs
the belt up if the leading end overshot (``CameraStationCycle._recenter``).

2026-11-24 evening (only the first parcel of a run was ever stopped): after a trigger
the detector waited for an *empty* view to re-arm, but the view never emptied
(a glare band / shadow / lighting change stayed "changed" for 30-40 s), so the next
parcel, e.g. a rejected one, ran through unstopped. Now:

* ``rearm()`` is called by the cycle before every new parcel (the inspected one has
  been carried away by then), no empty view needed.
* **Only a moving blob can trigger.** The detector runs only while the belt runs
  forward, so a real parcel always moves; a changed region that stays put (glare,
  shadow, a lamp switched on, a moved camera) is not a parcel. A component whose
  pixels have nearly all been unchanged for ``static_still_s`` is skipped, and after
  ``static_absorb_s`` every changed pixel that stayed unchanged that long is copied
  into the reference (the reference "learns" it, also where it touches a parcel).
* 2026-11-25 (empty belt stopped as a "parcel" after ~5 s, logs/runs/20261125_1059*):
  the lamp's reflection on the glossy, wavy belt is a bright band that *moves and
  changes shape* while the belt runs. A parcel (carton + label) is far brighter and
  sharper: measured component medians, glare L 18-77 / texture 4.5-16, real parcels
  L 130-150 / texture 19.8-26.5. A component counts as a parcel only if its median
  lightness >= ``object_min_lightness`` or its median texture >= ``object_min_texture``;
  dim, smooth pixels (glare body, belt where the glare used to be) are dropped from the
  mask before the components are built so they cannot merge with a parcel.
* 2026-11-25 (lần 2, parcels laid diagonally ran through unstopped): rotated ~35 deg a
  parcel spans ~0.95 of the view along the belt and is never fully inside. A parcel
  whose trailing end is still cut by the upstream edge when its leading end is about
  to leave ("overlong") is stopped then: most of it is in view.
  Lần 3: the leading end of such a parcel jumped 0.92 -> 0.85 -> 1.00 (corner + glare)
  and the one "about to leave" frame was skipped. It now stops as soon as the blob
  covers >= 0.86 of the view with its trailing end cut and its leading end (after
  the stop latency) past ``overlong_stop_at``.
* ``passed_downstream``: a moving blob reached the downstream edge since the last
  ``rearm()``. The cycle uses it to tell a parcel the camera missed from a hand at
  the pick sensor (lần 3: such a "missed parcel" was recorded while the real
  parcel was still under the camera).
* ``light_fault()``: the global brightness against the reference left
  ``light_gain_range`` (lamp off) for ``light_fault_s`` -> the cycle stops with a
  clear message instead of running blind.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass(frozen=True)
class ArrivalSettings:
    roi: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)  # x, y, w, h fractions
    axis: str = "y"                  # image axis the belt moves along ("x" or "y")
    center_line: float = 0.5         # preferred stop position of the parcel centre
    edge_margin: float = 0.02        # parcel ends must stay this far inside the ROI
    latency_s: float = 0.12          # frame-to-belt-stop delay to compensate
    width: int = 320                 # processing width (px)
    difference_threshold: float = 25.0
    min_area_ratio: float = 0.04     # blob area / ROI area to count as a parcel
    confirm_frames: int = 2          # consecutive frames with a parcel before trusting it
    empty_frames: int = 3            # consecutive empty frames to re-arm
    match_brightness: bool = True    # compensate global lighting drift vs the reference
    moving_speed: float = 0.03       # |velocity| (ROI/s) that proves the blob rides the belt
    still_tolerance: float = 12.0    # frame-to-frame change below this: pixel is still
    static_still_s: float = 1.0      # pixel unchanged this long counts as still
    static_fraction: float = 0.9     # component this still -> not a parcel, skipped
    static_absorb_s: float = 4.0     # still pixels of a skipped component -> reference (0: off)
    light_gain_range: tuple[float, float] = (0.5, 2.0)  # brightness vs reference outside: lamp off?
    object_min_lightness: float = 100.0  # component median L (0-255, after gain) of a parcel ...
    object_min_texture: float = 30.0     # ... or its median texture (glare reached 17.8, parcels 14-27)
    glare_max_lightness: float = 80.0    # pixels this dim ...
    glare_max_texture: float = 8.0       # ... and this smooth never count as parcel
    overlong_stop_at: float = 0.93       # overlong parcel: stop once its leading end gets here
    light_fault_s: float = 2.0

    @classmethod
    def from_config(cls, config: dict | None) -> ArrivalSettings:
        config = dict(config or {})
        for key in ("roi", "light_gain_range"):
            if key in config:
                config[key] = tuple(float(v) for v in config[key])
        known = {k: config[k] for k in cls.__dataclass_fields__ if k in config}
        return cls(**known)


@dataclass(frozen=True)
class Blob:
    centre: float   # along the axis, fraction of the ROI
    start: float    # low end along the axis
    end: float      # high end along the axis
    area_ratio: float


@dataclass
class ArrivalState:
    armed: bool = True
    present: bool = False
    triggered: bool = False
    inside: bool = False             # whole parcel inside the view (with margin)
    position: float | None = None    # centroid along the axis, 0..1 of the ROI
    velocity: float = 0.0            # ROI fractions per second (signed)
    area_ratio: float = 0.0
    frame_interval: float = 1 / 30   # measured seconds between analysed frames
    heading: float = 0.0             # travel direction (+1/-1) at the last trigger
    full_view_since: float | None = None  # a changed blob spans the whole view since then
    moved: bool = False              # the current blob has been seen riding the belt
    static_ratio: float = 0.0        # share of changed pixels skipped as static (diagnostics)
    absorbed: int = 0                # reference pixels learnt so far (diagnostics)
    glare_skipped: int = 0           # frames in which a dim/smooth moving blob was ignored
    blob_lightness: float = 0.0      # median L / texture of the picked blob (diagnostics)
    blob_texture: float = 0.0
    overlong: bool = False           # last trigger: parcel longer than the view (diagnostics)
    passed_downstream: bool = False  # a moving blob reached the downstream edge since rearm
    lead_was_in: bool = False        # this overlong blob was seen with its leading end inside
    blob: Blob | None = None         # last measurement (diagnostics, recentering)
    history: list[tuple[float, Blob]] = field(default_factory=list)


class ArrivalDetector:
    def __init__(self, background: np.ndarray, settings: ArrivalSettings | None = None):
        self.settings = settings or ArrivalSettings()
        if self.settings.axis not in {"x", "y"}:
            raise ValueError("arrival.axis phải là 'x' hoặc 'y'")
        self._reference = self._prepare(background)
        self.last_gain = 1.0
        self.state = ArrivalState()
        self._present_count = 0
        self._empty_count = 0
        self._previous: np.ndarray | None = None   # last analysed frame (Lab, small)
        self._previous_t = 0.0
        self._still = np.zeros(self._reference.shape[:2], np.float32)  # seconds unchanged
        self._light_bad_since: float | None = None

    # -- image ------------------------------------------------------------------
    def _crop(self, frame: np.ndarray) -> np.ndarray:
        x, y, w, h = self.settings.roi
        height, width = frame.shape[:2]
        x0, y0 = int(x * width), int(y * height)
        x1, y1 = int((x + w) * width), int((y + h) * height)
        return frame[y0:y1, x0:x1]

    def _small(self, frame: np.ndarray) -> np.ndarray:
        crop = self._crop(frame)
        scale = self.settings.width / max(1, crop.shape[1])
        small = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        return cv2.GaussianBlur(small, (5, 5), 0)

    @staticmethod
    def _lab(small: np.ndarray) -> np.ndarray:
        return cv2.cvtColor(small, cv2.COLOR_BGR2LAB).astype(np.float32)

    def _prepare(self, frame: np.ndarray) -> np.ndarray:
        small = self._small(frame)
        self._reference_gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        return self._lab(small)

    def _match_brightness(self, small: np.ndarray) -> np.ndarray:
        """Undo a global lighting change (lamp, daylight) against the empty-belt reference.

        2026-11-23: frames shot 1 h apart differed 2x in brightness, so the white rails
        and the grey board all looked "changed". On the camera image the ratio
        current/reference is the same for every background pixel; its histogram peak
        is that gain even when a parcel covers half the view (parcel ratios spread out).
        """
        if not self.settings.match_brightness:
            return small
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        ratio = np.log((gray + 4.0) / (self._reference_gray + 4.0))
        hist, edges = np.histogram(ratio, bins=120, range=(-1.5, 1.5))
        peak = int(np.argmax(hist))
        gain = float(np.exp((edges[peak] + edges[peak + 1]) / 2))
        self.last_gain = gain
        if abs(gain - 1.0) < 0.03:
            return small
        return np.clip(small.astype(np.float32) / gain, 0, 255).astype(np.uint8)

    @staticmethod
    def _metric(delta: np.ndarray) -> np.ndarray:
        # Same lightness + chroma metric as the damage segmentation.
        return np.maximum(delta[:, :, 0], 2 * np.maximum(delta[:, :, 1], delta[:, :, 2]))

    @staticmethod
    def _gradient(lightness: np.ndarray) -> np.ndarray:
        return cv2.magnitude(cv2.Sobel(lightness, cv2.CV_32F, 1, 0), cv2.Sobel(lightness, cv2.CV_32F, 0, 1))

    def _difference(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(current Lab, difference to the reference, texture added on top of the reference)."""
        small = self._small(frame)
        if small.shape[:2] != self._reference.shape[:2]:
            raise ValueError("Khung hình và ảnh nền khác kích thước")
        current = self._lab(self._match_brightness(small))
        texture = cv2.blur(np.maximum(
            self._gradient(current[:, :, 0]) - self._gradient(self._reference[:, :, 0]), 0), (9, 9))
        return current, self._metric(np.abs(current - self._reference)), texture

    def measure(self, frame: np.ndarray) -> Blob | None:
        """Largest changed, moving-parcel-like blob along the travel axis, or None."""
        return self._pick(*self._difference(frame))[0]

    def _pick(
        self, current: np.ndarray, difference: np.ndarray, texture: np.ndarray
    ) -> tuple[Blob | None, np.ndarray]:
        """(largest parcel-like non-static component as a Blob, mask of the static components)."""
        s = self.settings
        lightness = current[:, :, 0]
        changed = difference > s.difference_threshold
        glare = (lightness < s.glare_max_lightness) & (texture < s.glare_max_texture)
        mask = np.uint8(changed & ~glare) * 255
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        static = np.zeros(mask.shape, bool)
        index = None
        still = self._still >= s.static_still_s
        for candidate in 1 + np.argsort(-stats[1:, cv2.CC_STAT_AREA]):
            if float(stats[candidate, cv2.CC_STAT_AREA]) / mask.size < s.min_area_ratio:
                break
            pixels = labels == candidate
            if float(still[pixels].mean()) >= s.static_fraction:
                static |= pixels      # glare / shadow / moved fixture: not on the belt
                continue
            median_l = float(np.median(lightness[pixels]))
            median_t = float(np.median(texture[pixels]))
            if median_l < s.object_min_lightness and median_t < s.object_min_texture:
                self.state.glare_skipped += 1   # moving lamp reflection: dim and smooth
                continue
            self.state.blob_lightness, self.state.blob_texture = median_l, median_t
            index = int(candidate)
            break
        total = max(1, int(np.count_nonzero(mask)))
        self.state.static_ratio = float(np.count_nonzero(static & (mask > 0))) / total
        if index is None:
            return None, static
        area_ratio = float(stats[index, cv2.CC_STAT_AREA]) / mask.size
        if self.settings.axis == "y":
            length = mask.shape[0]
            start, size = stats[index, cv2.CC_STAT_TOP], stats[index, cv2.CC_STAT_HEIGHT]
            centre = centroids[index][1]
        else:
            length = mask.shape[1]
            start, size = stats[index, cv2.CC_STAT_LEFT], stats[index, cv2.CC_STAT_WIDTH]
            centre = centroids[index][0]
        # +1 px: a blob touching the far edge ends exactly at ``length``.
        return Blob(float(centre) / length, float(start) / length,
                    float(start + size) / length, area_ratio), static

    def _track_still(self, current: np.ndarray, timestamp: float) -> None:
        """Per pixel: how long it has looked the same (belt running, frames analysed)."""
        if self._previous is not None:
            # A gap (belt stopped for inspection, frames not analysed) counts as one
            # short step: a pixel unchanged across it keeps its time, a changed one resets.
            dt = min(max(timestamp - self._previous_t, 0.0), 0.2)
            same = self._metric(np.abs(current - self._previous)) < self.settings.still_tolerance
            self._still = np.where(same, self._still + dt, 0.0).astype(np.float32)
        self._previous, self._previous_t = current, timestamp

    def _check_light(self, timestamp: float) -> None:
        low, high = self.settings.light_gain_range
        if not self.settings.match_brightness or low <= self.last_gain <= high:
            self._light_bad_since = None
        elif self._light_bad_since is None:
            self._light_bad_since = timestamp

    def light_fault(self, timestamp: float) -> bool:
        """True when the view has been far darker/brighter than the reference (lamp off)."""
        since = self._light_bad_since
        return since is not None and timestamp - since >= self.settings.light_fault_s

    # -- state machine ---------------------------------------------------------------
    def reset(self, armed: bool = True) -> None:
        self.state = ArrivalState(armed=armed)
        self._present_count = 0
        self._empty_count = 0

    def rearm(self) -> None:
        """Ready for the next parcel (called by the cycle once the last one was carried away).

        Keeps the learnt reference and the per-pixel still times; the velocity history
        spans a stop and is dropped.
        """
        st = self.state
        st.armed = True
        st.moved = False
        st.passed_downstream = False
        st.lead_was_in = False
        st.history.clear()
        self._present_count = 0

    # Ends closer than this to the ROI border are clamped (parcel still entering or
    # leaving); their motion says nothing about the belt speed.
    _CLAMP = 0.005

    @classmethod
    def _end_velocity(cls, history: list[tuple[float, Blob]]) -> float:
        """Belt speed (ROI fractions / s, signed) from the parcel ends.

        Median of the frame-to-frame steps of each end: an end clamped at the ROI
        border is skipped; a sudden jump of one end (hand leaving the parcel, shadow)
        is one outlier among many steps. The belt moves both ends alike, so the end
        that moves more wins: 2026-11-25 lần 3, the leading end stuck at a glare band
        for 8 frames while the trailing end ran at 0.4 view/s, and the pooled median
        (0.05) made the parcel look stopped.
        """
        starts: list[float] = []
        ends: list[float] = []
        for (t0, a), (t1, b) in zip(history, history[1:]):
            dt = max(t1 - t0, 1e-3)
            if a.start > cls._CLAMP and b.start > cls._CLAMP:
                starts.append((b.start - a.start) / dt)
            if a.end < 1 - cls._CLAMP and b.end < 1 - cls._CLAMP:
                ends.append((b.end - a.end) / dt)
        speeds = [float(np.median(v)) for v in (starts, ends) if len(v) >= 2]
        if not speeds:
            pooled = starts + ends
            return float(np.median(pooled)) if pooled else 0.0
        return max(speeds, key=abs)

    def background_mismatch(self, timestamp: float, hold_s: float = 3.0) -> bool:
        """True when the whole view has differed from the empty-belt reference for ``hold_s``.

        2026-11-24 field fault: the camera had been moved; every frame showed one blob
        from edge to edge, so no parcel was ever "fully inside" and the belt never
        stopped, silently. No parcel spans the whole view along the belt for seconds
        (it could not be inspected anyway): the reference must be re-captured.
        """
        since = self.state.full_view_since
        return since is not None and timestamp - since >= hold_s

    def overshoot(self, blob: Blob | None, heading: float) -> float:
        """How far (ROI fraction) the leading end sticks out past the margin; <= 0: inside."""
        if blob is None or heading == 0:
            return 0.0
        low, high = self.settings.edge_margin, 1.0 - self.settings.edge_margin
        return blob.end - high if heading > 0 else low - blob.start

    def update(self, frame: np.ndarray, timestamp: float) -> bool:
        """Feed one frame; return True exactly once when the parcel may be stopped."""
        s, st = self.settings, self.state
        current, difference, texture = self._difference(frame)
        self._check_light(timestamp)
        self._track_still(current, timestamp)
        blob, static = self._pick(current, difference, texture)
        if s.static_absorb_s > 0:
            # Per pixel, not per component: a glare band touching a parcel merges with it
            # into one (not static) component, but its pixels alone have not changed for
            # seconds. A riding parcel's pixels are only still while it covers them.
            learn = (difference > s.difference_threshold) & (self._still >= s.static_absorb_s)
            if learn.any():
                self._reference[learn] = current[learn]
                st.absorbed += int(np.count_nonzero(learn))
        st.triggered = False
        st.blob = blob
        spans = blob is not None and blob.start <= 0.01 and blob.end >= 0.99 and blob.area_ratio >= 0.25
        if not spans and static.mean() >= 0.25:
            # Skipped as static but covering the whole view (learning off / not done yet).
            along = static.any(axis=1 if s.axis == "y" else 0)
            edge = max(1, along.size // 100)
            spans = bool(along[:edge].any() and along[-edge:].any())
        if not spans:
            st.full_view_since = None
        elif st.full_view_since is None:
            st.full_view_since = timestamp
        if blob is None:
            st.present = st.inside = st.moved = st.lead_was_in = False
            st.position = None
            st.area_ratio = 0.0
            st.history.clear()
            self._present_count = 0
            self._empty_count += 1
            if not st.armed and self._empty_count >= s.empty_frames:
                st.armed = True
            return False
        self._empty_count = 0
        self._present_count += 1
        st.present = True
        st.position = blob.centre
        st.area_ratio = blob.area_ratio
        st.history.append((timestamp, blob))
        del st.history[:-8]
        st.velocity = self._end_velocity(st.history)
        if len(st.history) >= 3 and abs(st.velocity) >= s.moving_speed:
            st.moved = True
            st.heading = float(np.sign(st.velocity))
        if st.moved and st.heading != 0:
            at_edge = blob.end >= 1 - self._CLAMP if st.heading > 0 else blob.start <= self._CLAMP
            st.passed_downstream = st.passed_downstream or at_edge
        if len(st.history) >= 2:
            times = [t for t, _ in st.history]
            st.frame_interval = float(np.median(np.diff(times)))
        low, high = s.edge_margin, 1.0 - s.edge_margin
        st.inside = blob.start >= low and blob.end <= high
        if not st.armed or self._present_count < s.confirm_frames:
            return False
        # The belt runs while this is called: a blob that never moved (glare, shadow,
        # parcel held by hand) is not a parcel riding to the pick point.
        if not st.moved or abs(st.velocity) < s.moving_speed:
            return False
        heading = float(np.sign(st.velocity))
        shift = st.velocity * s.latency_s
        later = blob.end + shift + st.velocity * st.frame_interval if heading > 0 \
            else blob.start + shift + st.velocity * st.frame_interval
        too_late_next = later > high if heading > 0 else later < low
        if not st.inside:
            # Overlong: trailing end still cut by the upstream edge, leading end inside
            # and about to leave. It cannot get any more of itself into view.
            trailing_cut = blob.start <= self._CLAMP if heading > 0 else blob.end >= 1 - self._CLAMP
            leading_in = blob.end <= high if heading > 0 else blob.start >= low
            # Only a blob that really spans (nearly) the whole view: a hand letting go
            # makes an end jump, the velocity spikes and a short blob looks "too late".
            spans_view = blob.end - blob.start >= (high - low) - 0.10
            # Measured position, no latency shift: a parcel that still fits (~0.87 of
            # the view) must get the normal fully-inside stop first (lần 3 p07).
            # 0.93 (lần 5: a 0.91-long parcel still fits fully inside two frames later).
            lead = blob.end if heading > 0 else 1.0 - blob.start
            fire = trailing_cut and leading_in and spans_view and (
                too_late_next or lead >= s.overlong_stop_at)
            # Late: its leading end was inside and has now jumped out (lần 3: 0.92 ->
            # 0.85 -> 1.00 between frames). Stop now; the cycle backs the overshoot up.
            late = trailing_cut and not leading_in and st.lead_was_in
            if trailing_cut and leading_in and spans_view:
                st.lead_was_in = True
            fire = fire or late
            st.overlong = fire
        else:
            st.overlong = False
            st.lead_was_in = False
            # Where the parcel will be when the belt has actually stopped (shift), and
            # where it would stop if we waited for the next analysed frame (later).
            centred = heading * (blob.centre + shift - s.center_line) >= 0
            # Inside now: stop once centred, or now if the next frame would be too late
            # (any overshoot from the stop latency is backed up by the cycle).
            fire = centred or too_late_next
        if fire:
            st.armed = False
            st.triggered = True
            st.heading = float(np.sign(st.velocity))
        return bool(fire)
