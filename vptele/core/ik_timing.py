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
PAPER_STAGES = ("stage1_baseline_us", "stage2_gate_us", "stage3_motion_domain_us",
                "stage4_wrist_candidates_us", "stage5_temporal_selection_us")
PAPER_FLAGS = tuple("stage%d_executed" % i for i in range(1, 6))
PAPER_TIMES = PAPER_STAGES + ("selector_other_us",)
PAPER_META = ("timing_schema_version", "paper_timing_valid", "paper_timing_error",
              "selector_mode", "wrist_skip_reason") + PAPER_FLAGS
RESPONSE_FIELDS = ("timing_valid", "target_is_moving", "timing_runtime_mode") + SERVER_TIMES + PAPER_META + PAPER_TIMES
TIMES = ("ik_latency_us",) + SERVER_TIMES + PAPER_TIMES
FIELDS = ("frame_index", "source_timestamp", "method", "startup_call",
          "call_success", "ik_status", "ik_latency_us") + RESPONSE_FIELDS


def capture(frame, method, elapsed_us, response=None, error=None, startup=False):
    row = dict(frame_index=frame.index, source_timestamp=frame.timestamp,
               method=method, startup_call=startup,
               call_success=bool(getattr(response, "success", False)),
               ik_status=str(error) if error is not None else str(getattr(response, "message", "")),
               timing_valid=False, target_is_moving="", timing_runtime_mode="",
               ik_latency_us=elapsed_us)
    row.update({name: "" for name in SERVER_TIMES})
    row.update({name: "" for name in PAPER_META + PAPER_TIMES})
    row.update(paper_timing_valid=False)
    valid = bool(getattr(response, "timing_valid", False))
    values = [getattr(response, name, None) for name in SERVER_TIMES]
    if valid and all(isinstance(v, (float, int)) and math.isfinite(v) and v >= 0 for v in values):
        row.update(zip(SERVER_TIMES, values))
        row.update(timing_valid=True,
                   target_is_moving=bool(response.target_is_moving),
                   timing_runtime_mode=str(response.timing_runtime_mode))
    version = getattr(response, "timing_schema_version", 0)
    row.update(timing_schema_version=version,
               selector_mode=str(getattr(response, "selector_mode", "")),
               wrist_skip_reason=str(getattr(response, "wrist_skip_reason", "")))
    if version == 2 and getattr(response, "paper_timing_valid", False):
        durations = [getattr(response, key, None) for key in PAPER_TIMES]
        flags = [getattr(response, key, None) for key in PAPER_FLAGS]
        good = row["timing_valid"] and all(
            isinstance(v, (float, int)) and math.isfinite(v) and v >= 0 for v in durations)
        good = good and all(isinstance(v, (bool, int)) and v in (0, 1) for v in flags)
        if good:
            good = all(flag or duration == 0 for flag, duration in zip(flags, durations))
            good = good and math.isclose(sum(durations), row["selector_elapsed_us"],
                                         abs_tol=1e-5, rel_tol=1e-9)
        if good:
            row.update(zip(PAPER_TIMES, durations))
            row.update(zip(PAPER_FLAGS, map(bool, flags)))
            row["paper_timing_valid"] = True
        else:
            row["paper_timing_error"] = "invalid_fields_or_exclusive_sum_mismatch"
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
            eligible = [r for r in selected if metric not in PAPER_TIMES or r.get("paper_timing_valid", False)]
            valid = [r[metric] for r in eligible if isinstance(r.get(metric), (int, float))
                     and math.isfinite(r[metric]) and r[metric] >= 0]
            # Zero-stage fields cannot establish that the stage was measured.
            samples = [v for v in valid if v > 0] if metric in STAGES else valid
            executed_count = skipped_count = execution_rate = mean_all_calls = None
            if metric in PAPER_STAGES:
                flag = PAPER_FLAGS[PAPER_STAGES.index(metric)]
                samples = [r[metric] for r in eligible if r[flag]]
                executed_count = len(samples)
                skipped_count = len(valid) - executed_count
                execution_rate = executed_count / len(valid) if valid else None
                mean_all_calls = sum(valid) / len(valid) if valid else None
            stats[metric] = dict(count=len(samples), missing_count=len(selected)-len(valid),
                                 zero_count=sum(v == 0 for v in valid),
                                 executed_count=executed_count, skipped_count=skipped_count,
                                 execution_rate=execution_rate, mean_all_calls=mean_all_calls)
            stats[metric].update({k: None for k in ("mean", "p50", "p95", "p99", "max")})
            if samples:
                stats[metric].update(mean=sum(samples)/len(samples), p50=percentile(samples, 50),
                                     p95=percentile(samples, 95), p99=percentile(samples, 99), max=max(samples))
        paper = [r for r in selected if r.get("paper_timing_valid", False)]
        paper_total = sum(r["selector_elapsed_us"] for r in paper)
        shares = {key: sum(r[key] for r in paper) / paper_total if paper_total else None
                  for key in PAPER_TIMES}
        output[name] = dict(calls=len(selected), metrics_us=stats,
                            paper_valid_calls=len(paper), paper_time_fractions=shares,
                            paper_invalid_calls=sum(bool(r.get("paper_timing_error")) for r in selected))
    return dict(schema_version=2, units="microseconds", groups=output,
                notes=["Startup calls are excluded from non-startup groups.",
                       "Legacy stage zeros are ambiguous; legacy stages must not be summed with paper stages.",
                       "Paper stage mean/percentiles/max use executed calls, including true zero durations; rejected candidates count too.",
                       "Paper execution_rate denominator is valid paper calls in the group; mean_all_calls includes skipped zeros.",
                       "Paper stages are exclusive; their sum plus selector_other_us equals selector_elapsed_us. IK is included.",
                       "Guard/experimental profiles are excluded from paper statistics, not from total-time statistics.",
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
                "missing_count", "zero_count", "executed_count", "skipped_count", "execution_rate",
                "mean_all_calls", "mean", "p50", "p95", "p99", "max"))
            writer.writeheader()
            for group, detail in summary["groups"].items():
                for metric, stats in detail["metrics_us"].items():
                    writer.writerow(dict(group=group,metric_us=metric,calls=detail["calls"],**stats))
