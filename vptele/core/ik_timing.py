"""ROS-independent per-call timing capture; no change to solver/history policy."""
import csv
import json
import math
import os

STAGES = (
    "context_build_us", "phi1_us", "phi2_us", "motion_selection_us",
    "offset_execution_us", "actuator_conversion_us",
)
SERVER_TIMES = ("selector_call_us", "selector_elapsed_us") + STAGES
TIMES = ("ik_latency_us",) + SERVER_TIMES
FIELDS = ("frame_index", "source_timestamp", "method", "startup_call",
          "call_success", "ik_status", "timing_valid", "target_is_moving",
          "timing_runtime_mode") + TIMES


def capture(frame, method, elapsed_us, response=None, error=None, startup=False):
    row = dict(frame_index=frame.index, source_timestamp=frame.timestamp,
               method=method, startup_call=startup,
               call_success=bool(getattr(response, "success", False)),
               ik_status=str(error) if error is not None else str(getattr(response, "message", "")),
               timing_valid=False, target_is_moving="", timing_runtime_mode="",
               ik_latency_us=elapsed_us)
    row.update({name: "" for name in SERVER_TIMES})
    valid = bool(getattr(response, "timing_valid", False))
    values = [getattr(response, name, None) for name in SERVER_TIMES]
    if valid and all(isinstance(v, (float, int)) and math.isfinite(v) and v >= 0 for v in values):
        row.update(zip(SERVER_TIMES, values))
        row.update(timing_valid=True,
                   target_is_moving=bool(response.target_is_moving),
                   timing_runtime_mode=str(response.timing_runtime_mode))
    return row


def percentile(values, p):
    values = sorted(values)
    index = (len(values)-1)*p/100.0
    low = int(index)
    high = min(low+1, len(values)-1)
    return values[low] + (index-low)*(values[high]-values[low])


def summarize(rows):
    def group(row):
        if not row["call_success"]:
            return "failed_calls"
        if not row["timing_valid"]:
            return "unavailable_timing"
        if not row["target_is_moving"]:
            return "static_calls"
        if ":hold_previous:" in row["ik_status"]:
            return "moving_hold"
        return "moving_non_hold"
    groups = {"all_calls": rows, "startup_calls": [r for r in rows if r["startup_call"]]}
    for name in ("failed_calls", "unavailable_timing", "static_calls", "moving_hold", "moving_non_hold"):
        groups[name] = [r for r in rows if not r["startup_call"] and group(r) == name]
    output = {}
    for name, selected in groups.items():
        stats = {}
        for metric in TIMES:
            valid = [r[metric] for r in selected if isinstance(r[metric], (int, float))
                     and math.isfinite(r[metric]) and r[metric] >= 0]
            # Zero-stage fields cannot establish that the stage was measured.
            samples = [v for v in valid if v > 0] if metric in STAGES else valid
            stats[metric] = dict(count=len(samples), missing_count=len(selected)-len(valid),
                                 zero_count=sum(v == 0 for v in valid))
            stats[metric].update({k: None for k in ("mean", "p50", "p95", "p99", "max")})
            if samples:
                stats[metric].update(mean=sum(samples)/len(samples), p50=percentile(samples, 50),
                                     p95=percentile(samples, 95), p99=percentile(samples, 99), max=max(samples))
        output[name] = dict(calls=len(selected), metrics_us=stats)
    return dict(schema_version=1, units="microseconds", groups=output,
                notes=["Startup calls are excluded from non-startup groups.",
                       "Zero stage durations mean unrecorded/not reached and are excluded from stage percentiles.",
                       "Stage intervals are the library's instrumentation scopes; do not sum them into exclusive percentages.",
                       "Client call time includes ROS round trip, not CSV I/O or console printing.",
                       "Timing rows describe IK calls, not proof that a command was published."])


class TimingRecorder:
    def __init__(self, output_path):
        stem = os.path.splitext(output_path)[0]
        self.path = stem + "_timing.csv"
        self.summary_path = stem + "_timing_summary.json"
        self.summary_csv_path = stem + "_timing_summary.csv"
        if any(os.path.exists(p) for p in (self.path,self.summary_path,self.summary_csv_path)):
            raise FileExistsError("refusing to overwrite timing files")
        self.file = open(self.path, "x", newline="", encoding="utf-8")
        self.rows = []

    def record(self, row):
        self.rows.append(row)

    def close(self):
        if self.file.closed:
            return
        try:
            writer = csv.DictWriter(self.file, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        finally:
            self.file.close()
        with open(self.summary_path, "x", encoding="utf-8") as stream:
            summary = summarize(self.rows)
            json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        with open(self.summary_csv_path, "x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("group", "metric_us", "calls", "count",
                "missing_count", "zero_count", "mean", "p50", "p95", "p99", "max"))
            writer.writeheader()
            for group, detail in summary["groups"].items():
                for metric, stats in detail["metrics_us"].items():
                    writer.writerow(dict(group=group,metric_us=metric,calls=detail["calls"],**stats))
