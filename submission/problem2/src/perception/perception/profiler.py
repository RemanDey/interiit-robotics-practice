#!/usr/bin/env python3
"""All-in-one latency + RAM benchmark for the perception node.

Launch behaviour:
  * Imported by ``semantic_segmentation.py`` -> attach(node=...) arms the
    /diagnostics publisher (DiagnosticArray @ stats_hz), per-frame CSV and
    shutdown summary JSON. Just ``ros2 run perception perception_segmentor``
    and you get all three outputs with zero extra flags.
  * Run directly (``python3 profiler.py``) -> offline self-test that simulates
    frames and writes profiling.csv + profiling_summary.json in cwd, printing
    the same snapshot the node would publish. No ROS required.

The node stays thin: create Profiler(), attach(), tick/record per stage,
write_csv_row() per frame, publish_diagnostics() on timer, shutdown() on exit.
"""

from __future__ import annotations

import csv
import json
import os
import socket
import statistics
import time
from collections import deque
from contextlib import contextmanager

try:  # optional: only needed for DiagnosticArray publishing
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
except Exception:  # offline use without ROS
    DiagnosticArray = None  # type: ignore
    DiagnosticStatus = None  # type: ignore
    KeyValue = None  # type: ignore


def _read_rss_mb_fallback() -> float | None:
    try:
        with open("/proc/self/status", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return float(line.split()[1]) / 1024.0
    except Exception:
        return None
    return None


class Profiler:
    """Rolling-window stage timers + RAM + diagnostics/CSV/JSON outputs."""

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

    CSV_FIELDS = [
        "frame", "stamp_ros_sec", "fps_instant",
        "convert_ms", "infer_track_ms", "mask_centroid_ms",
        "depth_sample_ms", "project_ms", "tf_lookup_ms",
        "map_update_ms", "debug_plot_publish_ms", "total_e2e_ms",
        "n_tracks", "rss_mb",
    ]

    # Keep in sync with diagnostic_msgs DiagnosticStatus values.
    OK, WARN, ERROR, STALE = 0, 1, 2, 3

    def __init__(
        self,
        node=None,
        window: int = 100,
        enabled: bool = True,
        diagnostics_topic: str = "/diagnostics",
        csv_path: str = "profiling.csv",
        summary_path: str = "profiling_summary.json",
        stats_hz: float = 1.0,
        model_name: str = "",
        component_name: str = "perception: semantic_segmentation",
    ) -> None:
        self._window = max(1, int(window))
        self._samples: dict[str, deque] = {s: deque(maxlen=self._window) for s in self.STAGES}
        self._fps_samples: deque = deque(maxlen=self._window)
        self._last_frame_t: float | None = None
        self.frame_count: int = 0
        self.tf_fail_count: int = 0
        self.depth_reject_count: int = 0

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
        self.sample_ram()
        self._baseline_mb = self.rss_mb

        # ROS/output wiring (attached later via attach() if node not ready yet).
        self._node = None
        self._diag_pub = None
        self._diag_timer = None
        self._csv_file = None
        self._csv_writer = None
        self.enabled = enabled
        self.diagnostics_topic = diagnostics_topic
        self.csv_path = csv_path
        self.summary_path = summary_path
        self.stats_hz = stats_hz
        self.model_name = model_name
        self.component_name = component_name
        try:
            self._hostname = socket.gethostname()
        except Exception:
            self._hostname = "unknown"

        if node is not None:
            self.attach(
                node=node,
                enabled=enabled,
                diagnostics_topic=diagnostics_topic,
                csv_path=csv_path,
                summary_path=summary_path,
                stats_hz=stats_hz,
                model_name=model_name,
                component_name=component_name,
            )
        elif enabled and csv_path:
            # Offline mode: still open CSV so `python3 profiler.py` produces it.
            self._open_csv(csv_path)

    # -- attach -----------------------------------------------------------
    def attach(
        self,
        node,
        enabled: bool = True,
        diagnostics_topic: str = "/diagnostics",
        csv_path: str = "profiling.csv",
        summary_path: str = "profiling_summary.json",
        stats_hz: float = 1.0,
        model_name: str = "",
        component_name: str = "perception: semantic_segmentation",
    ):
        """Arm all outputs from the node. Call once after params are loaded."""
        self._node = node
        self.enabled = enabled
        self.diagnostics_topic = diagnostics_topic
        self.csv_path = csv_path
        self.summary_path = summary_path
        self.stats_hz = stats_hz
        if model_name:
            self.model_name = model_name
        if component_name:
            self.component_name = component_name
        if not enabled:
            return self
        self._open_csv(csv_path)
        if node is not None and DiagnosticArray is not None:
            try:
                self._diag_pub = node.create_publisher(DiagnosticArray, diagnostics_topic, 10)
                period = 1.0 / stats_hz if stats_hz and stats_hz > 0 else 1.0
                # Timer callback delegates back here with live track count if
                # the node exposes `tracked_objects`.
                self._diag_timer = node.create_timer(period, self._on_timer)
            except Exception as e:
                try:
                    node.get_logger().warn(f"Profiler diagnostics disabled: {e}")
                except Exception:
                    pass
                self._diag_pub = None
                self._diag_timer = None
        return self

    def _open_csv(self, path: str) -> None:
        self._close_csv()
        try:
            self._csv_file = open(path, "w", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self.CSV_FIELDS)
            self._csv_writer.writeheader()
        except Exception as e:
            if self._node is not None:
                try:
                    self._node.get_logger().warn(f"Profiling CSV disabled: {e}")
                except Exception:
                    pass
            self._csv_file = None
            self._csv_writer = None

    def _close_csv(self) -> None:
        try:
            if self._csv_file is not None:
                try:
                    self._csv_file.flush()
                finally:
                    self._csv_file.close()
        except Exception:
            pass
        finally:
            self._csv_file = None
            self._csv_writer = None

    # -- timing API ---------------------------------------------------------
    @staticmethod
    def now_ns() -> int:
        return time.perf_counter_ns()

    @staticmethod
    def elapsed_ms(t0_ns: int, t1_ns: int) -> float:
        return (t1_ns - t0_ns) / 1e6

    @contextmanager
    def stage(self, name: str):
        """Single-span timer: ``with prof.stage('convert_ms'): ...``."""
        t0 = self.now_ns()
        try:
            yield
        finally:
            self.record(name, self.elapsed_ms(t0, self.now_ns()))

    def record(self, stage: str, ms: float) -> None:
        if stage in self._samples:
            self._samples[stage].append(float(ms))

    def tick_frame(self) -> float:
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

    # -- RAM API --------------------------------------------------------------
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
        before = self._baseline_mb
        self.sample_ram()
        self.model_load_mb = self.rss_mb - before
        return self.model_load_mb

    @staticmethod
    def gpu_mem_mb() -> float | None:
        try:
            import torch  # type: ignore

            if torch.cuda.is_available():
                return float(torch.cuda.max_memory_allocated() / (1024.0 * 1024.0))
        except Exception:
            pass
        return None

    # -- aggregation ------------------------------------------------------------
    def _avg(self, stage: str) -> float:
        q = self._samples[stage]
        return float(sum(q) / len(q)) if q else 0.0

    def _p95(self, stage: str) -> float:
        q = sorted(self._samples[stage])
        if not q:
            return 0.0
        return float(q[min(len(q) - 1, int(0.95 * len(q)))])

    def snapshot(self) -> dict:
        self.sample_ram()
        fps_avg = float(sum(self._fps_samples) / len(self._fps_samples)) if self._fps_samples else 0.0
        out: dict = {
            "frames.count": self.frame_count,
            "fps.avg": round(fps_avg, 2),
            "fps.instant": round(self._fps_samples[-1], 2) if self._fps_samples else 0.0,
        }
        for s in self.STAGES:
            out[f"{s}.avg"] = round(self._avg(s), 3)
            out[f"{s}.p95"] = round(self._p95(s), 3)
        gpu = self.gpu_mem_mb()
        out.update(
            {
                "ram.rss_mb": round(self.rss_mb, 2),
                "ram.peak_mb": round(self.peak_rss_mb, 2),
                "ram.model_load_mb": round(self.model_load_mb, 2),
                "ram.delta_mb": round(self.rss_mb - self._baseline_mb, 2),
                "gpu.peak_mb": round(gpu, 2) if gpu is not None else None,
                "tf.fail_count": self.tf_fail_count,
                "depth.reject_count": self.depth_reject_count,
            }
        )
        return out

    def fps_avg(self) -> float:
        return float(statistics.fmean(self._fps_samples)) if self._fps_samples else 0.0

    # -- CSV -----------------------------------------------------------------------
    def write_csv_row(self, stamp_ros_sec: float, fps_instant: float,
                      stage: dict, total_ms: float, n_tracks: int) -> None:
        if not self.enabled or self._csv_writer is None:
            return
        try:
            self._csv_writer.writerow(
                {
                    "frame": self.frame_count,
                    "stamp_ros_sec": f"{stamp_ros_sec:.6f}",
                    "fps_instant": f"{fps_instant:.2f}",
                    "convert_ms": f"{stage.get('convert_ms', 0):.3f}",
                    "infer_track_ms": f"{stage.get('infer_track_ms', 0):.3f}",
                    "mask_centroid_ms": f"{stage.get('mask_centroid_ms', 0):.3f}",
                    "depth_sample_ms": f"{stage.get('depth_sample_ms', 0):.3f}",
                    "project_ms": f"{stage.get('project_ms', 0):.3f}",
                    "tf_lookup_ms": f"{stage.get('tf_lookup_ms', 0):.3f}",
                    "map_update_ms": f"{stage.get('map_update_ms', 0):.3f}",
                    "debug_plot_publish_ms": f"{stage.get('debug_plot_publish_ms', 0):.3f}",
                    "total_e2e_ms": f"{total_ms:.3f}",
                    "n_tracks": n_tracks,
                    "rss_mb": f"{self.rss_mb:.1f}",
                }
            )
        except Exception as e:
            if self._node is not None:
                try:
                    self._node.get_logger().warn(f"CSV write failed: {e}", throttle_duration_sec=5.0)
                except Exception:
                    pass

    # -- diagnostics ------------------------------------------------------------------
    def level_for_snapshot(self, snap: dict) -> tuple[int, str]:
        fps = snap.get("fps.avg", 0.0)
        peak = snap.get("ram.peak_mb", 0.0)
        if fps <= 0.0 and snap.get("frames.count", 0) < 5:
            return self.STALE, "warming up"
        if fps < 5.0 or (peak or 0) >= 8192:
            return self.ERROR, f"LOW_FPS_OR_OOM fps={fps} peak={peak}MB"
        if fps < 10.0 or (peak or 0) >= 6144:
            return self.WARN, f"near budget fps={fps} peak={peak}MB"
        return self.OK, f"OK fps={fps} infer_p95={snap.get('infer_track_ms.p95', 0)}ms"

    def build_diagnostic_array(self, snap: dict, live_tracks: int = 0):
        """Build DiagnosticArray (None when ROS msgs unavailable)."""
        if DiagnosticArray is None:
            return None
        level, message = self.level_for_snapshot(snap)
        status = DiagnosticStatus()
        # ROS 2 octet fields are bytes of length 1 (b'\x00'...), not ints.
        status.level = bytes([level]) if isinstance(level, int) else level
        status.name = self.component_name
        status.message = message
        status.hardware_id = f"{self._hostname}:{os.path.basename(str(self.model_name))}"
        for key in [
            "frames.count", "fps.avg", "fps.instant",
            "convert_ms.avg", "infer_track_ms.avg", "infer_track_ms.p95",
            "mask_centroid_ms.avg", "depth_sample_ms.avg", "project_ms.avg",
            "tf_lookup_ms.avg", "map_update_ms.avg",
            "debug_plot_publish_ms.avg", "total_e2e_ms.avg", "total_e2e_ms.p95",
            "ram.rss_mb", "ram.peak_mb", "ram.model_load_mb", "ram.delta_mb",
            "gpu.peak_mb", "tf.fail_count", "depth.reject_count",
        ]:
            kv = KeyValue()
            kv.key = key
            val = snap.get(key, 0)
            kv.value = "" if val is None else str(val)
            status.values.append(kv)
        kv = KeyValue()
        kv.key = "tracks.live_count"
        kv.value = str(live_tracks)
        status.values.append(kv)
        arr = DiagnosticArray()
        if self._node is not None:
            try:
                arr.header.stamp = self._node.get_clock().now().to_msg()
            except Exception:
                pass
        arr.status = [status]
        return arr

    def _on_timer(self) -> None:
        live = 0
        if self._node is not None:
            try:
                live = len(getattr(self._node, "tracked_objects", {}))
            except Exception:
                live = 0
        self.publish_diagnostics(live_tracks=live)

    def publish_diagnostics(self, live_tracks: int = 0) -> None:
        if not self.enabled:
            return
        snap = self.snapshot()
        arr = self.build_diagnostic_array(snap, live_tracks=live_tracks)
        if arr is not None and self._diag_pub is not None:
            try:
                self._diag_pub.publish(arr)
            except Exception:
                pass
        # Throttled console log in both ROS and offline modes.
        msg = (
            f"[bench] fps={snap['fps.avg']} total_p95={snap['total_e2e_ms.p95']}ms "
            f"infer_p95={snap['infer_track_ms.p95']}ms rss={snap['ram.rss_mb']}MB "
            f"peak={snap['ram.peak_mb']}MB tracks={live_tracks}"
        )
        if self._node is not None:
            try:
                self._node.get_logger().info(msg, throttle_duration_sec=5.0)
                return
            except Exception:
                pass
        print(msg)

    # -- summary / shutdown --------------------------------------------------------------
    def write_summary(self, model: str = "", live_tracks: int = 0) -> dict:
        snap = self.snapshot()
        summary = {
            "model": model or self.model_name,
            "snapshot": snap,
            "live_tracks": live_tracks,
            "edge_budget": {
                "fps_target": 10.0,
                "ram_limit_mb": 8192,
                "fps_pass": snap.get("fps.avg", 0.0) >= 10.0,
                "ram_pass": (snap.get("ram.peak_mb", 0.0) or 0.0) < 8192,
            },
        }
        if self.enabled and self.summary_path:
            try:
                with open(self.summary_path, "w") as f:
                    json.dump(summary, f, indent=2)
            except Exception as e:
                if self._node is not None:
                    try:
                        self._node.get_logger().warn(f"Summary write failed: {e}")
                    except Exception:
                        pass
            try:
                if self._csv_file is not None:
                    self._csv_file.flush()
            except Exception:
                pass
        return summary

    def shutdown(self, model: str = "", live_tracks: int = 0) -> dict:
        summary = self.write_summary(model=model, live_tracks=live_tracks)
        self._close_csv()
        return summary


def main() -> None:
    """Offline self-test: ``python3 profiler.py`` writes CSV + JSON + snapshot."""
    import argparse

    ap = argparse.ArgumentParser(description="Profiler offline self-test (no ROS needed)")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--csv", default="profiling.csv")
    ap.add_argument("--summary", default="profiling_summary.json")
    args = ap.parse_args()

    prof = Profiler(enabled=True, csv_path=args.csv, summary_path=args.summary,
                    model_name="self-test")
    prof.mark_model_loaded()
    for i in range(args.frames):
        fps = prof.tick_frame()
        stage = {k: 0.0 for k in Profiler.STAGES if k != "total_e2e_ms"}
        with prof.stage("convert_ms") as _:
            time.sleep(0.001)
        # Simulate per-object accumulation then single record per frame.
        t0 = Profiler.now_ns()
        time.sleep(0.004)
        stage_ms = Profiler.elapsed_ms(t0, Profiler.now_ns())
        prof.record("convert_ms", Profiler.elapsed_ms(t0, Profiler.now_ns()))
        prof.record("infer_track_ms", 5.0 + (i % 3))
        prof.record("mask_centroid_ms", stage_ms)
        prof.record("depth_sample_ms", 0.4)
        prof.record("project_ms", 0.1)
        prof.record("tf_lookup_ms", 0.3)
        prof.record("map_update_ms", 0.2)
        prof.record("debug_plot_publish_ms", 1.0)
        prof.record("total_e2e_ms", 8.0 + (i % 2))
        prof.write_csv_row(time.time(), fps,
                           {**{k: 1.0 for k in Profiler.STAGES}, **stage,
                            "convert_ms": stage_ms, "infer_track_ms": 5.0 + (i % 3)},
                           8.0 + (i % 2), n_tracks=i % 4)
        time.sleep(0.01)
    prof.publish_diagnostics(live_tracks=3)
    summary = prof.shutdown(model="self-test", live_tracks=3)
    print(json.dumps(summary, indent=2))
    print(f"Wrote {args.csv} + {args.summary}")


if __name__ == "__main__":
    main()
