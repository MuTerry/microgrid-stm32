"""Summarize the archived PA1 serial logs and replay the run 11 filter."""

import hashlib
import json
from pathlib import Path
import re
from statistics import mean, stdev


DATA = Path(__file__).resolve().parent / "data" / "2026-11-19-to-21"
REPORT = re.compile(
    r"PA1 avg=(\d+) min=(\d+) max=(\d+) mV=(\d+) "
    r"n=(\d+) err=(\d+) safe=(\d+) fault=(\d+)"
)
CURRENT = re.compile(r"(?m)^CURRENT mA=([+-]\d+)\r?$")
AVERAGE = re.compile(r"(?m)^CURRENT_AVG mA=([+-]\d+) n=(\d+)\r?$")
ZERO = re.compile(r"(?m)^ZERO OK avg=([\d.]+) groups=(\d+)\r?$")


def run_number(path):
    match = re.search(r"-run(\d+)$", path.stem)
    return int(match.group(1) if match else path.stem)


def replay_average(values, size):
    for stop in range(size, len(values) + 1):
        total = sum(values[stop - size:stop])
        magnitude = (abs(total) + size // 2) // size
        yield -magnitude if total < 0 else magnitude


def windows(values, ranges):
    result = []
    for first, last in ranges:
        assert 1 <= first <= last <= len(values)
        part = values[first - 1:last]
        result.append({"first": first, "last": last, "mean": mean(part),
                       "sample_stdev": stdev(part)})
    return result


def analyze():
    result = {"files": {}}
    files = sorted(DATA.glob("*.txt"), key=run_number)
    assert [run_number(path) for path in files] == list(range(1, 19))
    for path in files:
        raw = path.read_bytes()
        text = raw.decode("utf-8-sig")
        reports = [tuple(map(int, row)) for row in REPORT.findall(text)]
        current = [int(ma) for ma in CURRENT.findall(text)]
        averages = [(int(ma), int(size)) for ma, size in AVERAGE.findall(text)]
        zeros = list(ZERO.finditer(text))
        avgs = [report[0] for report in reports]
        result["files"][path.name] = {
            "sha256": hashlib.sha256(raw).hexdigest(),
            "bytes": len(raw),
            "PA1_reports": len(reports),
            "PA1_min_max_avg_counts": [min(avgs), max(avgs)],
            "states_n_err_safe_fault": sorted(set(row[4:] for row in reports)),
            "CURRENT_reports": len(current),
            "CURRENT_AVG_reports": len(averages),
            "ZERO_OK_counts": [float(match[1]) for match in zeros],
        }

        run = run_number(path)
        if run == 9:
            # One-based CURRENT report positions in the file, including before z.
            result["run09_CURRENT_mA"] = windows(current, [(31, 135), (151, 270), (291, 405)])
        elif run == 11:
            assert len(zeros) == 1
            after_zero = text[zeros[0].end():]
            values = [int(ma) for ma in CURRENT.findall(after_zero)]
            observed = [(int(ma), int(size)) for ma, size in AVERAGE.findall(after_zero)]
            expected = list(replay_average(values, 8))
            assert len(expected) == len(observed) == 175
            assert all(size == 8 for _, size in observed)
            assert expected == [ma for ma, _ in observed]
            first_average = AVERAGE.search(after_zero)
            assert len(CURRENT.findall(after_zero[:first_average.start()])) == 8
            # Same report endpoints for both series. Exclude switch transients.
            ranges = [(15, 40), (72, 107), (136, 170)]
            result["run11_after_zero"] = {
                "CURRENT_reports": len(values),
                "first_average_at_CURRENT_report": 8,
                "exact_filter_matches": len(expected),
                "CURRENT_mA": windows(values, ranges),
                "CURRENT_AVG_mA": windows([ma for ma, _ in observed],
                                          [(first - 7, last - 7) for first, last in ranges]),
            }
        elif run == 16:
            filtered = [ma for ma, _ in averages]
            result["run16_CURRENT_AVG_mA"] = windows(
                filtered, [(1, len(filtered)), (1, 60), (len(filtered) - 59, len(filtered))])

    return result


if __name__ == "__main__":
    print(json.dumps(analyze(), indent=2))
