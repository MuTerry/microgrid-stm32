"""Recompute the archived bench evidence; no hardware access or extra packages.

Run from any directory. JSON is written to stdout, never into the raw records.
CSV ranges refer to data rows (excluding the header); ADC ranges refer to
complete reports found in order, including valid reports after damaged prefixes.
All ranges below are one-based and inclusive. They compare separate plateaus,
not synchronized samples. Window choices are documented in the test report.
"""

import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
from statistics import mean


DATA = Path(__file__).resolve().parent / "data" / "2026-11-08"
FIELDS = ("avg", "min", "max", "mV", "n", "err", "safe", "fault")
REPORT = re.compile(
    r"PA1 avg=(\d+) min=(\d+) max=(\d+) mV=(\d+) "
    r"n=(\d+) err=(\d+) safe=(\d+) fault=(\d+)"
)
WINDOWS = {
    "reverse.csv": [(41, 70), (111, 140), (181, 210)],
    "reverse.txt": [(121, 240), (521, 640), (881, 1000)],
    "vcc-cycle.txt": [(121, 240), (361, 480), (641, 760)],
    "pa1-cycle.csv": [(31, 60), (91, 120), (141, 170)],
    "pa1-cycle.txt": [(121, 240), (341, 460), (601, 720)],
}
FILES = (
    "pa1-adc-2026-11-08.txt", "pa1-dmm-2026-11-08.csv",
    "reverse.csv", "reverse.txt", "vcc-cycle.csv", "vcc-cycle.txt",
    "pa1-cycle.csv", "pa1-cycle.txt",
    "pa1-ground.txt", "pa1-mid.csv", "pa1-mid.txt",
    "pa1-mid-probe-off.txt", "pa1-mid-probe-on.txt",
    "acs-zero-check.csv", "acs-zero-check.txt",
)


def stats(values):
    return {"count": len(values), "min": min(values),
            "mean": mean(values), "max": max(values)}


def analyze():
    result = {"selection": "manual interior plateaus; no timestamp alignment",
              "files": {}}
    adc_values = {}
    for name in FILES:
        raw = (DATA / name).read_bytes()
        text = raw.decode("utf-8-sig")
        info = {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}
        if name.endswith(".csv"):
            rows = list(csv.DictReader(io.StringIO(text)))
            values = [float(row["Value"]) for row in rows]
            info["time_first"] = rows[0]["Date/Time"]
            info["time_last"] = rows[-1]["Date/Time"]
            info["function_units"] = sorted(set(
                (row["Function"], row["Unit"]) for row in rows))
        else:
            rows = [dict(zip(FIELDS, map(int, match.groups())))
                    for match in REPORT.finditer(text)]
            values = [row["avg"] for row in rows]
            adc_values[name] = values
            info["states_n_err_safe_fault"] = sorted(set(
                tuple(row[key] for key in FIELDS[4:]) for row in rows))
            info["nonmatching_line_numbers"] = [
                i for i, line in enumerate(text.splitlines(), 1)
                if line.strip() and not REPORT.fullmatch(line.strip())]
            info["unparsed_fragments"] = [
                {"line": i, "text": REPORT.sub("", line).strip()}
                for i, line in enumerate(text.splitlines(), 1)
                if REPORT.sub("", line).strip()]
            info["within_report_peak_to_peak_counts"] = stats([
                row["max"] - row["min"] for row in rows])
        info["all"] = stats(values)
        info["windows"] = []
        for lo, hi in WINDOWS.get(name, []):
            if not 1 <= lo <= hi <= len(rows):
                raise ValueError(f"Invalid window: {name} {lo}:{hi}")
            window = {"first": lo, "last": hi, **stats(values[lo-1:hi])}
            if name.endswith(".csv"):
                window.update(time_first=rows[lo-1]["Date/Time"],
                              time_last=rows[hi-1]["Date/Time"])
            info["windows"].append(window)
        result["files"][name] = info

    dmm = [w["mean"] for w in result["files"]["pa1-cycle.csv"]["windows"]]
    adc = [w["mean"] for w in result["files"]["pa1-cycle.txt"]["windows"]]
    dv = dmm[1] - (dmm[0] + dmm[2]) / 2
    dc = adc[1] - (adc[0] + adc[2]) / 2
    vref, current = 3.278, -0.479  # user-reported rail/current, not calibrated constants
    result["pa1_cycle_comparison"] = {
        "assumed_vref_V": vref, "reported_current_A": current,
        "dmm_delta_mV": dv * 1000, "adc_delta_counts": dc,
        "adc_estimated_delta_mV": dc * vref / 4095 * 1000,
        "delta_difference_mV": (dc * vref / 4095 - dv) * 1000,
        "dmm_return_shift_mV": (dmm[2] - dmm[0]) * 1000,
        "adc_return_shift_counts": adc[2] - adc[0],
        "dmm_coarse_sensitivity_mV_per_A": dv * 1000 / current,
        "adc_coarse_sensitivity_mV_per_A": dc * vref / 4095 * 1000 / current,
        "estimated_adc_minus_dmm_mV": [
            (code * vref / 4095 - voltage) * 1000
            for voltage, code in zip(dmm, adc)],
    }
    result["input_control_comparisons"] = {}
    # User-reported rails bracket the recordings; no continuous VREF record.
    for base, references in (("pa1-mid", [3.299, 3.300]),
                             ("acs-zero-check", [3.285])):
        dmm_mean = result["files"][base + ".csv"]["all"]["mean"]
        adc_mean = result["files"][base + ".txt"]["all"]["mean"]
        result["input_control_comparisons"][base] = [
            {"assumed_vref_V": reference,
             "adc_estimated_V": adc_mean * reference / 4095,
             "adc_minus_dmm_mV": (adc_mean * reference / 4095 - dmm_mean) * 1000}
            for reference in references]
    result["probe_report_order_comparisons"] = {}
    for name in ("pa1-mid.txt", "pa1-mid-probe-off.txt", "pa1-mid-probe-on.txt"):
        a, b = adc_values[name][:-1], adc_values[name][1:]
        ma, mb = mean(a), mean(b)
        denominator = math.sqrt(sum((x-ma)**2 for x in a) * sum((y-mb)**2 for y in b))
        result["probe_report_order_comparisons"][name] = {
            "lag1_correlation": (sum((x-ma)*(y-mb) for x, y in zip(a, b))
                                 / denominator if denominator else None),
            "mean_absolute_adjacent_step_counts": mean(abs(y-x) for x, y in zip(a, b)),
        }
    return result


if __name__ == "__main__":
    print(json.dumps(analyze(), ensure_ascii=True, indent=2))
