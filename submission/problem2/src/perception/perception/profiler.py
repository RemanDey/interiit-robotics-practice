"""Lightweight per-stage latency + RAM profiler for the perception node.

No ROS dependencies: pure Python + optional psutil / torch.
The ROS node owns DiagnosticArray publishing, CSV and summary JSON;
this helper only tracks numbers with minimal overhead.
"""

from __future__ import annotations

import os
import statistics
import time
from collections import defaultdict, deque


def _read_rss_mb_fallback() -> float | None:
    """Fallback RSS reader via /proc when psutil is unavailable."""
    try:
        with open("/proc/self/status", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    # Format: "VmRSS:	  123456 kB"
                    parts = line.split()
                    return float(parts[1]) / 1024.0
    except Exception:
        return None
    return None


class Profiler:
    """Rolling-window stage timer + process RSS tracker."""

    # Stages instrumented in Object3DMapperNode.perception_callback.
    STAGES = (
        "convert_ms",
        "infer_track_ms",
        "mask_centroid_ms",
        "depth_sample_ms",
        "project_ms",
        "tf_lookup_ms",
        "map_update_ms",
        "debug_plot_publish_ms",
        "total_e2e_ms",
    )

    def __init__(self, window: int = 100) -> None:
        self._window = max(1, int(window))
        self._samples: dict[str, deque] = {
            s: deque(maxlen=self._window) for s in self.STAGES
        }
        self._fps_samples: deque = deque(maxlen=self._window)
        self._last_frame_t: float | None = None
        self.frame_count: int = 0
        self.tf_fail_count: int = 0
        self.depth_reject_count: int = 0

        # --- RAM tracking ---
        self._psutil_process = None
        try:
            import psutil  # type: ignore

            self._psutil_process = psutil.Process(os.getpid())
        except Exception:
            self._psutil_process = None

        self.rss_mb: float = 0.0
        self.peak_rss_mb: float = 0.0
        self.model_load_mb: float = 0.0
        self._baseline_mb: float = 0.0
        self.sample_ram()  # establish baseline immediately
        self._baseline_mb = self.rss_mb

    # -- timing API -----------------------------------------------------
    @staticmethod
    def now_ns() -> int:
        return time.perf_counter_ns()

    @staticmethod
    def elapsed_ms(t0_ns: int, t1_ns: int) -> float:
        return (t1_ns - t0_ns) / 1e6

    def record(self, stage: str, ms: float) -> None:
        if stage in self._samples:
            self._samples[stage].append(float(ms))

    def tick_frame(self) -> float:
        """Call once per perception_callback. Returns instantaneous FPS."""
        now = time.perf_counter()
        fps = 0.0
        if self._last_frame_t is not None:
            dt = now - self._last_frame_t
            if dt > 0:
                fps = 1.0 / dt
                self._fps_samples.append(fps)
        self._last_frame_t = now
        self.frame_count += 1
        return fps

    # -- RAM API ----------------------------------------------------------
    def sample_ram(self) -> float:
        rss = None
        if self._psutil_process is not None:
            try:
                rss = self._psutil_process.memory_info().rss / (1024.0 * 1024.0)
            except Exception:
                rss = None
        if rss is None:
            rss = _read_rss_mb_fallback()
        if rss is not None:
            self.rss_mb = float(rss)
            if self.rss_mb > self.peak_rss_mb:
                self.peak_rss_mb = self.rss_mb
        return self.rss_mb

    def mark_model_loaded(self) -> float:
        """Call right after YOLO weights are loaded. Returns overhead MB."""
        before = self._baseline_mb
        self.sample_ram()
        self.model_load_mb = self.rss_mb - before
        return self.model_load_mb

    @staticmethod
    def gpu_mem_mb() -> float | None:
        """torch CUDA peak MB if available, else None (CPU/Jetson-safe)."""
        try:
            import torch  # type: ignore

            if torch.cuda.is_available():
                return float(torch.cuda.max_memory_allocated() / (1024.0 * 1024.0))
        except Exception:
            pass
        return None

    # -- aggregation -------------------------------------------------------
    def _avg(self, stage: str) -> float:
        q = self._samples[stage]
        return float(sum(q) / len(q)) if q else 0.0

    def _p95(self, stage: str) -> float:
        q = sorted(self._samples[stage])
        if not q:
            return 0.0
        idx = min(len(q) - 1, int(0.95 * len(q)))
        return float(q[idx])

    def snapshot(self) -> dict:
        """Flat dict of avg/p95 per stage + fps/ram. Safe to call at 1 Hz."""
        self.sample_ram()
        fps_avg = (
            float(sum(self._fps_samples) / len(self._fps_samples))
            if self._fps_samples
            else 0.0
        )
        stages = {}
        for s in self.STAGES:
            stages[f"{s}.avg"] = round(self._avg(s), 3)
            stages[f"{s}.p95"] = round(self._p95(s), 3)
        gpu = self.gpu_mem_mb()
        return {
            "frames.count": self.frame_count,
            "fps.avg": round(fps_avg, 2),
            "fps.instant": round(self._fps_samples[-1], 2)
            if self._fps_samples
            else 0.0,
            **stages,
            "ram.rss_mb": round(self.rss_mb, 2),
            "ram.peak_mb": round(self.peak_rss_mb, 2),
            "ram.model_load_mb": round(self.model_load_mb, 2),
            "ram.delta_mb": round(self.rss_mb - self._baseline_mb, 2),
            "gpu.peak_mb": round(gpu, 2) if gpu is not None else None,
            "tf.fail_count": self.tf_fail_count,
            "depth.reject_count": self.depth_reject_count,
        }

    def fps_avg(self) -> float:
        if not self._fps_samples:
            return 0.0
        return float(statistics.fmean(self._fps_samples))
