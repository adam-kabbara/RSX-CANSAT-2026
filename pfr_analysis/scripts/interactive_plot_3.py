#!/usr/bin/env python3
"""
Interactive XY plotting utility for comma-separated flight logs.

Usage (interactive):
  python3 scripts/interactive_plot.py

Usage (CLI):
    python3 scripts/interactive_plot.py <file> <x_col> <y_col1>[,<y_col2>[,<y_col3>]] "<title>" [--save out.png]

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


def load_multi_xy(path, x_idx, y_indices):
    all_xs = []
    series_by_col = {y_idx: [] for y_idx in y_indices}
    x_is_time = False
    any_dt = False
    total = 0
    skipped_missing = 0
    skipped_parse = 0
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for ln in f:
            total += 1
            ln = ln.strip()
            if not ln:
                continue
            parts = [p for p in ln.split(",")]
            # require x and all requested y columns to exist
            if len(parts) <= max([x_idx] + y_indices):
                skipped_missing += 1
                continue
            x_raw = parts[x_idx].strip()
            if x_raw == "":
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
            if x_val is None:
                skipped_parse += 1
                continue
            row_has_any_value = False
            for y_idx in y_indices:
                if len(parts) <= y_idx:
                    skipped_missing += 1
                    continue
                y_raw = parts[y_idx].strip()
                if y_raw == "":
                    skipped_missing += 1
                    continue
                y_val = to_float_safe(y_raw)
                if y_val is None:
                    skipped_parse += 1
                else:
                    series_by_col[y_idx].append((x_val, y_val))
                    row_has_any_value = True
            if row_has_any_value:
                all_xs.append(x_val)

    stats = {"total": total, "skipped_missing": skipped_missing, "skipped_parse": skipped_parse}
    if not all_xs:
        stats["used"] = 0
        return [], {}, x_is_time, stats
    stats["used"] = len(all_xs)
    return all_xs, series_by_col, x_is_time, stats


def plot_xy(xs, series_by_col, x_is_time, title, xlabel, ylabel, savepath=None, stats=None):
    plt.close("all")
    fig, ax = plt.subplots(figsize=(9, 5))
    legend_labels = []
    all_x_values = []
    all_y_values = []
    plotted_any = False
    y_min_candidate = None
    y_max_candidate = None
    # Plot one or more Y series.
    if isinstance(series_by_col, dict):
        for y_idx, points in series_by_col.items():
            if not points:
                continue
            xs_for_series = [xv for xv, _ in points]
            ys_for_series = [yv for _, yv in points]
            line, = ax.plot(xs_for_series, ys_for_series, "-o", markersize=4, label=f"Col {y_idx}")
            legend_labels.append(line.get_label())
            all_x_values.extend(xs_for_series)
            all_y_values.extend(ys_for_series)
            plotted_any = True
    else:
        ax.plot(xs, series_by_col, "-o", markersize=4, label=ylabel)
        legend_labels.append(ylabel)
        all_x_values.extend(xs)
        all_y_values.extend(series_by_col)
        plotted_any = True

    if x_is_time and all_x_values:
        ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
        fig.autofmt_xdate()
        xmin_dt = min(all_x_values)
        xmax_dt = max(all_x_values)
        xmin_floor = xmin_dt.replace(second=0, microsecond=0)
        xmax_ceil = xmax_dt.replace(second=0, microsecond=0)
        if xmax_ceil <= xmax_dt:
            xmax_ceil = xmax_ceil + timedelta(minutes=1)
        xmax_with_headroom = xmax_ceil + timedelta(minutes=1)
        ax.set_xlim(mdates.date2num(xmin_floor), mdates.date2num(xmax_with_headroom))
    elif all_x_values:
        try:
            xmin = min(all_x_values)
            xmax = max(all_x_values)
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
    if all_y_values:
        try:
            ymin = min(all_y_values)
            ymax = max(all_y_values)
            if math.isfinite(ymin) and math.isfinite(ymax):
                fig.canvas.draw()
                yticks = [t for t in ax.get_yticks() if math.isfinite(t)]
                yupper = _next_tick_above(ymin, ymax, yticks)
                ax.set_ylim(ymin, yupper)
        except Exception:
            pass
    if legend_labels:
        ax.legend(loc="best")
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
            y_text = input("Y column index/indices (0-based, comma-separated up to 3): ").strip()
            y_indices = [int(v.strip()) for v in y_text.split(",") if v.strip()]
        except Exception:
            print("Invalid index. Try again.")
            continue
        title = input("Plot title (optional): ").strip() or f"Cols {', '.join(str(v) for v in y_indices)} vs Col {x_idx}"
        xlabel = input("X label (optional): ").strip() or f"Col {x_idx}"
        ylabel = input("Y label (optional): ").strip() or " / ".join(f"Col {v}" for v in y_indices)
        save = input("Save to file? (enter path or leave empty to display): ").strip()
        xs, series_by_col, is_time, stats = load_multi_xy(path, x_idx, y_indices)
        if not xs:
            print("No valid points found for those columns.\nStats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
        else:
            print("Stats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
            # print sample points from first available series
            first_series = next((series_by_col[y] for y in y_indices if series_by_col.get(y)), [])
            def fmt_point(xv, yv):
                if isinstance(xv, datetime):
                    xs_s = xv.strftime("%H:%M:%S")
                else:
                    xs_s = str(xv)
                y_s = str(yv)
                return f"({xs_s}, {y_s})"

            pts = list(zip(xs, first_series))
            if pts:
                head = pts[:5]
                tail = pts[-5:]
                print("First points:", ", ".join(fmt_point(x, y) for x, y in head))
                if len(pts) > 5:
                    print("Last points:", ", ".join(fmt_point(x, y) for x, y in tail))
            plot_xy(xs, series_by_col, is_time, title, xlabel, ylabel, savepath=(save or None), stats=stats)
        again = input("Plot another? [y/N]: ").strip().lower()
        if again != "y":
            break


def main(argv):
    if len(argv) >= 5:
        path = argv[1]
        x_idx = int(argv[2])
        y_indices = [int(v.strip()) for v in argv[3].split(",") if v.strip()]
        title = argv[4]
        savepath = None
        if "--save" in argv:
            si = argv.index("--save")
            if si + 1 < len(argv):
                savepath = argv[si + 1]
        xs, series_by_col, is_time, stats = load_multi_xy(path, x_idx, y_indices)
        if not xs:
            print("No valid data found for those columns.\nStats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
            return 1
        print("Stats: total=%(total)d used=%(used)d skipped_missing=%(skipped_missing)d skipped_parse=%(skipped_parse)d" % stats)
        # show point samples
        first_series = next((series_by_col[y] for y in y_indices if series_by_col.get(y)), [])
        pts = list(zip(xs, first_series))
        def fmt_point_main(xv, yv):
            if isinstance(xv, datetime):
                xs_s = xv.strftime("%H:%M:%S")
            else:
                xs_s = str(xv)
            y_s = str(yv)
            return f"({xs_s}, {y_s})"
        if pts:
            head = pts[:5]
            tail = pts[-5:]
            print("First points:", ", ".join(fmt_point_main(x, y) for x, y in head))
            if len(pts) > 5:
                print("Last points:", ", ".join(fmt_point_main(x, y) for x, y in tail))
        xlabel = f"Col {x_idx}"
        ylabel = " / ".join(f"Col {v}" for v in y_indices)
        plot_xy(xs, series_by_col, is_time, title, xlabel, ylabel, savepath=savepath, stats=stats)
        return 0
    else:
        prompt_loop()
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
