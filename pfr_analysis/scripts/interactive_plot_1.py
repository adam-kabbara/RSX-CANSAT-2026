#!/usr/bin/env python3
"""
Interactive XY plotting utility for comma-separated flight logs.

Usage (interactive):
  python3 scripts/interactive_plot.py

Usage (CLI):
  python3 scripts/interactive_plot.py <file> <x_col> <y_col> "<title>" [--save out.png]

Columns are 0-based indices. Time strings like HH:MM:SS in the X column are parsed.
Missing or non-numeric values are skipped.
"""
import sys
import math
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


def parse_time_to_dt(s: str):
    s = s.strip()
    if not s:
        return None
    fmts = ["%H:%M:%S", "%H:%M:%S.%f"]
    for f in fmts:
        try:
            dt = datetime.strptime(s, f)
            # if only a time (year 1900), attach today's date for plotting ranges
            if dt.year == 1900:
                today = datetime.today()
                dt = datetime(year=today.year, month=today.month, day=today.day, hour=dt.hour, minute=dt.minute, second=dt.second, microsecond=dt.microsecond)
            return dt
        except Exception:
            continue
    return None


def to_float_safe(s: str):
    try:
        return float(s)
    except Exception:
        return None


def _next_tick_above(vmin, vmax, ticks):
    """Return an upper limit that is one tick above data max when possible."""
    if not ticks:
        if vmax == vmin:
            return vmax + (abs(vmax) * 0.1 if vmax != 0 else 1.0)
        return vmax + (vmax - vmin) * 0.1
    bigger = [t for t in ticks if t > vmax]
    if bigger:
        return min(bigger)
    # no tick above max: extrapolate using last tick spacing
    ticks_sorted = sorted(ticks)
    if len(ticks_sorted) >= 2:
        step = ticks_sorted[-1] - ticks_sorted[-2]
        if step > 0:
            return ticks_sorted[-1] + step
    if vmax == vmin:
        return vmax + (abs(vmax) * 0.1 if vmax != 0 else 1.0)
    return vmax + (vmax - vmin) * 0.1


def load_xy(path, x_idx, y_idx):
    xs = []
    ys = []
    x_is_time = False
    any_dt = False
    total = 0
    skipped_missing = 0
    skipped_parse = 0
    raw_ys = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            total += 1
            ln = ln.strip()
            if not ln:
                continue
            parts = [p for p in ln.split(",")]
            # require both columns exist
            if len(parts) <= max(x_idx, y_idx):
                skipped_missing += 1
                continue
            x_raw = parts[x_idx].strip()
            y_raw = parts[y_idx].strip()
            if x_raw == "" or y_raw == "":
                skipped_missing += 1
                continue
            # try parse x as time first
            x_dt = parse_time_to_dt(x_raw)
            if x_dt is not None:
                x_val = x_dt
                x_is_time = True
                any_dt = True
            else:
                x_val = to_float_safe(x_raw)
            # attempt numeric parse for y, but keep raw if not numeric
            y_val = to_float_safe(y_raw)
            if x_val is None:
                skipped_parse += 1
                continue
            xs.append(x_val)
            if y_val is None:
                ys.append(None)
                raw_ys.append(y_raw)
                skipped_parse += 1
            else:
                ys.append(y_val)
                raw_ys.append(None)
    # if we detected any datetime x values, keep only datetime rows
    if any_dt:
        # Collect rows where x is datetime, and handle numeric or raw categorical Ys
        collected_x = []
        collected_y_num = []
        collected_y_raw = []
        for xv, yv, raw in zip(xs, ys, raw_ys):
            if not isinstance(xv, datetime):
                continue
            collected_x.append(xv)
            if yv is not None:
                collected_y_num.append(yv)
                collected_y_raw.append(None)
            else:
                collected_y_num.append(None)
                collected_y_raw.append(raw)

        # If we have numeric Ys among these, filter out None numeric rows
        numeric_count = sum(1 for v in collected_y_num if v is not None)
        if numeric_count > 0:
            final_xs = []
            final_ys = []
            for xv, yv in zip(collected_x, collected_y_num):
                if yv is None:
                    continue
                final_xs.append(xv)
                final_ys.append(yv)
            stats = {"total": total, "used": len(final_xs), "skipped_missing": skipped_missing, "skipped_parse": skipped_parse}
            return final_xs, final_ys, True, stats

        # Otherwise, if categorical raw Ys exist, encode them
        if any(v is not None for v in collected_y_raw):
            mapping = {}
            encoded = []
            for raw in collected_y_raw:
                if raw is None:
                    encoded.append(None)
                    continue
                if raw not in mapping:
                    mapping[raw] = len(mapping)
                encoded.append(mapping[raw])
            final_xs = []
            final_ys = []
            for xv, ev in zip(collected_x, encoded):
                if ev is None:
                    continue
                final_xs.append(xv)
                final_ys.append(float(ev))
            stats = {"total": total, "used": len(final_xs), "skipped_missing": skipped_missing, "skipped_parse": skipped_parse, "categorical_mapping": mapping}
            return final_xs, final_ys, True, stats

        # nothing usable
        stats = {"total": total, "used": 0, "skipped_missing": skipped_missing, "skipped_parse": skipped_parse}
        return [], [], True, stats

    # If no numeric Ys were parsed but we have raw categorical Ys, encode them
    numeric_count = sum(1 for v in ys if v is not None)
    if numeric_count == 0 and any(v is not None for v in raw_ys):
        # build mapping
        unique = []
        mapping = {}
        encoded = []
        for raw in raw_ys:
            if raw is None:
                encoded.append(None)
                continue
            if raw not in mapping:
                mapping[raw] = len(mapping)
                unique.append(raw)
            encoded.append(mapping[raw])
        # filter out entries where encoded is None
        final_xs = []
        final_ys = []
        for xv, ev in zip(xs, encoded):
            if ev is None:
                continue
            final_xs.append(xv)
            final_ys.append(float(ev))
        stats = {"total": total, "used": len(final_xs), "skipped_missing": skipped_missing, "skipped_parse": skipped_parse, "categorical_mapping": mapping}
        return final_xs, final_ys, x_is_time, stats

    stats = {"total": total, "used": sum(1 for v in ys if v is not None), "skipped_missing": skipped_missing, "skipped_parse": skipped_parse}
    # filter out None y values
    final_xs = []
    final_ys = []
    for xv, yv in zip(xs, ys):
        if yv is None:
            continue
        final_xs.append(xv)
        final_ys.append(yv)
    return final_xs, final_ys, x_is_time, stats


def plot_xy(xs, ys, x_is_time, title, xlabel, ylabel, savepath=None, stats=None):
    plt.close("all")
    fig, ax = plt.subplots(figsize=(9, 5))
    if x_is_time:
        ax.plot(xs, ys, "-o", markersize=4)
        dates = mdates.date2num(xs)
        # Force UTC axis to 1-minute major ticks
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
        fig.autofmt_xdate()
        # set x limits to data extent with one extra minute at the end
        if len(dates) > 0:
            xmin_dt = min(xs)
            xmax_dt = max(xs)
            xmin_floor = xmin_dt.replace(second=0, microsecond=0)
            xmax_ceil = xmax_dt.replace(second=0, microsecond=0)
            if xmax_ceil <= xmax_dt:
                xmax_ceil = xmax_ceil + timedelta(minutes=1)
            # extra minute so max point is not at the right edge
            xmax_with_headroom = xmax_ceil + timedelta(minutes=1)
            ax.set_xlim(mdates.date2num(xmin_floor), mdates.date2num(xmax_with_headroom))
    else:
        ax.plot(xs, ys, "-o", markersize=4)
        # set numeric x limits to data extent with one extra tick at end
        if len(xs) > 0:
            try:
                xmin = min(xs)
                xmax = max(xs)
                if math.isfinite(xmin) and math.isfinite(xmax):
                    fig.canvas.draw()
                    xticks = [t for t in ax.get_xticks() if math.isfinite(t)]
                    xupper = _next_tick_above(xmin, xmax, xticks)
                    ax.set_xlim(xmin, xupper)
            except Exception:
                pass
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle=':', alpha=0.6)
    # Y limits / categorical labels
    if stats and "categorical_mapping" in stats:
        mapping = stats["categorical_mapping"]
        inv = {v: k for k, v in mapping.items()}
        ticks = sorted(inv.keys())
        labels = [inv[i] for i in ticks]
        ax.set_yticks(ticks)
        ax.set_yticklabels(labels)
        # set y limits to cover ticks
        if ticks:
            ax.set_ylim(min(ticks) - 0.5, max(ticks) + 0.5)
    else:
        if len(ys) > 0:
            try:
                ymin = min(ys)
                ymax = max(ys)
                if math.isfinite(ymin) and math.isfinite(ymax):
                    fig.canvas.draw()
                    yticks = [t for t in ax.get_yticks() if math.isfinite(t)]
                    yupper = _next_tick_above(ymin, ymax, yticks)
                    ax.set_ylim(ymin, yupper)
            except Exception:
                pass
    if savepath:
        fig.tight_layout()
        fig.savefig(savepath)
        print(f"Saved plot to {savepath}")
    else:
        plt.show()


def prompt_loop(default_path="pfr_analysis/flight_data/flight_logs.rebuilt.txt"):
    path = input(f"Log file (default: {default_path}): ").strip() or default_path
    while True:
        try:
            x_idx = int(input("X column index (0-based): ").strip())
            y_idx = int(input("Y column index (0-based): ").strip())
        except Exception:
            print("Invalid index. Try again.")
            continue
        title = input("Plot title (optional): ").strip() or f"Col {y_idx} vs Col {x_idx}"
        xlabel = input("X label (optional): ").strip() or f"Col {x_idx}"
        ylabel = input("Y label (optional): ").strip() or f"Col {y_idx}"
        save = input("Save to file? (enter path or leave empty to display): ").strip()
        xs, ys, is_time, stats = load_xy(path, x_idx, y_idx)
        if not xs:
            print("No valid points found for those columns.\nStats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
        else:
            print("Stats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
            if "categorical_mapping" in stats:
                print("Categorical mapping:\n" + "\n".join([f'{k} -> {v}' for k, v in stats['categorical_mapping'].items()]))
            # print sample points
            def fmt_point(xv, yv):
                if isinstance(xv, datetime):
                    xs_s = xv.strftime("%H:%M:%S")
                else:
                    xs_s = str(xv)
                if "categorical_mapping" in stats:
                    # invert mapping
                    inv = {v: k for k, v in stats['categorical_mapping'].items()}
                    y_s = inv.get(int(yv), str(yv)) if yv is not None else ""
                else:
                    y_s = str(yv)
                return f"({xs_s}, {y_s})"

            pts = list(zip(xs, ys))
            if pts:
                head = pts[:5]
                tail = pts[-5:]
                print("First points:", ", ".join(fmt_point(x, y) for x, y in head))
                if len(pts) > 5:
                    print("Last points:", ", ".join(fmt_point(x, y) for x, y in tail))
            plot_xy(xs, ys, is_time, title, xlabel, ylabel, savepath=(save or None), stats=stats)
        again = input("Plot another? [y/N]: ").strip().lower()
        if again != "y":
            break


def main(argv):
    if len(argv) >= 5:
        path = argv[1]
        x_idx = int(argv[2])
        y_idx = int(argv[3])
        title = argv[4]
        savepath = None
        if "--save" in argv:
            si = argv.index("--save")
            if si + 1 < len(argv):
                savepath = argv[si + 1]
        xs, ys, is_time, stats = load_xy(path, x_idx, y_idx)
        if not xs:
            print("No valid data found for those columns.\nStats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
            return 1
        print("Stats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
        if "categorical_mapping" in stats:
            print("Categorical mapping:\n" + "\n".join([f'{k} -> {v}' for k, v in stats['categorical_mapping'].items()]))
        # show point samples
        pts = list(zip(xs, ys))
        def fmt_point_main(xv, yv):
            if isinstance(xv, datetime):
                xs_s = xv.strftime("%H:%M:%S")
            else:
                xs_s = str(xv)
            if "categorical_mapping" in stats:
                inv = {v: k for k, v in stats['categorical_mapping'].items()}
                y_s = inv.get(int(yv), str(yv)) if yv is not None else ""
            else:
                y_s = str(yv)
            return f"({xs_s}, {y_s})"
        if pts:
            head = pts[:5]
            tail = pts[-5:]
            print("First points:", ", ".join(fmt_point_main(x, y) for x, y in head))
            if len(pts) > 5:
                print("Last points:", ", ".join(fmt_point_main(x, y) for x, y in tail))
        xlabel = f"Col {x_idx}"
        ylabel = f"Col {y_idx}"
        plot_xy(xs, ys, is_time, title, xlabel, ylabel, savepath=savepath, stats=stats)
        return 0
    else:
        prompt_loop()
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
