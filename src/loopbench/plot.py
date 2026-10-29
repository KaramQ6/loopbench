"""Lightweight ASCII visualization for step response in Markdown terminals."""

from __future__ import annotations

from typing import List


def render_ascii_plot(
    time_series: List[float],
    output_series: List[float],
    setpoint: float,
    width: int = 48,
    height: int = 10,
) -> str:
    """Generate a clean ASCII step response plot with setpoint guideline.

    Why ASCII: Returns instantaneous, zero-dependency visual confirmation in
    any chat context, eliminating external UI requirements or broken image links.
    """
    if not output_series or not time_series or len(output_series) < 2:
        return "[No data to plot]"

    min_y = min(min(output_series), 0.0, setpoint * 0.9)
    max_y = max(max(output_series), setpoint * 1.1)
    y_range = max_y - min_y
    if y_range < 1e-6:
        y_range = 1.0

    # Downsample time/output points into columns
    total_points = len(output_series)
    col_step = total_points / float(width)

    downsampled_y = []
    for col in range(width):
        start_idx = int(col * col_step)
        end_idx = min(total_points, int((col + 1) * col_step))
        if start_idx >= total_points:
            start_idx = total_points - 1
        sub_slice = output_series[start_idx:max(start_idx + 1, end_idx)]
        avg_val = sum(sub_slice) / len(sub_slice)
        downsampled_y.append(avg_val)

    # Grid initialization with spaces
    grid = [[" " for _ in range(width)] for _ in range(height)]

    # Draw setpoint target dashed line
    sp_row = int((max_y - setpoint) / y_range * (height - 1))
    sp_row = max(0, min(height - 1, sp_row))
    for col in range(width):
        grid[sp_row][col] = "-"

    # Plot response points
    for col, y_val in enumerate(downsampled_y):
        row = int((max_y - y_val) / y_range * (height - 1))
        row = max(0, min(height - 1, row))
        if grid[row][col] == "-":
            grid[row][col] = "+"
        else:
            grid[row][col] = "*"

    # Build ASCII output string with y-axis labels
    lines = []
    top_label = f"{max_y:6.2f} |"
    mid_label = f"{setpoint:6.2f} SP"
    bot_label = f"{min_y:6.2f} |"

    for r in range(height):
        row_str = "".join(grid[r])
        if r == 0:
            prefix = top_label
        elif r == sp_row:
            prefix = f"{mid_label:6s} |"
        elif r == height - 1:
            prefix = bot_label
        else:
            prefix = "       |"
        lines.append(f"{prefix}{row_str}")

    # X-axis timeline
    lines.append("       +" + "-" * width)
    t_start = f"{time_series[0]:.1f}s"
    t_end = f"{time_series[-1]:.1f}s"
    x_axis_label = f"        {t_start:<10}" + " " * (width - 20) + f"{t_end:>10}"
    lines.append(x_axis_label)

    return "\n".join(lines)
