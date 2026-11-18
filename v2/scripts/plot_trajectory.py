#!/usr/bin/env python3
"""Plot saved V2 trajectory values without deriving or changing data."""
import argparse
import csv
from datetime import date, datetime, timedelta, timezone
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FixedLocator, FormatStrFormatter

COORDS = ('world_car/x', 'world_car/y', 'world_car/theta',
          *(f'joint{i}' for i in range(1, 7)))
KINDS = ('q', 'v', 'a', 'jerk')
FIELDS = [f'{kind}_{coord}' for kind in KINDS for coord in COORDS]
OUTPUTS = Path(__file__).resolve().parents[1]/'outputs'
BEIJING = timezone(timedelta(hours=8))
UNITS = {'q': ('m', 'rad'), 'v': ('m/s', 'rad/s'),
         'a': ('m/s²', 'rad/s²'), 'jerk': ('m/s³', 'rad/s³')}
BASE_TITLES = {'q': 'Base Pose', 'v': 'Base Velocity',
               'a': 'Base Acceleration', 'jerk': 'Base Jerk'}
ARM_TITLES = {'q': 'Arm Joint Positions', 'v': 'Arm Velocity',
              'a': 'Arm Acceleration', 'jerk': 'Arm Jerk'}
BASE_LABELS = {'q': (r'$x$', r'$y$', r'$\theta$'),
               'v': (r'$v_x$', r'$v_y$', r'$\omega$'),
               'a': (r'$a_x$', r'$a_y$', r'$\alpha$'),
               'jerk': (r'$j_x$', r'$j_y$', r'$j_\theta$')}
ARM_LABELS = {
    'q': lambda i: rf'$q_{{{i}}}$',
    'v': lambda i: rf'$\dot{{q}}_{{{i}}}$',
    'a': lambda i: rf'$\ddot{{q}}_{{{i}}}$',
    'jerk': lambda i: rf'$\dddot{{q}}_{{{i}}}$',
}


def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as source:
        reader = csv.DictReader(source)
        required = ['time_s', *FIELDS]
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f'{path}: missing fields {missing}')
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError(f'{path}: fewer than two rows')
    data = {}
    for name in required:
        try:
            data[name] = np.asarray([float(row[name]) for row in rows])
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{path}: invalid number in {name}') from exc
        if not np.isfinite(data[name]).all():
            raise ValueError(f'{path}: nonfinite value in {name}')
    if not np.all(np.diff(data['time_s']) > 0):
        raise ValueError(f'{path}: time_s must be strictly increasing')
    return data


def plot_pair(base_ax, arm_ax, data, kind):
    t = data['time_s']
    # Keep the original x/y/theta overlay and upper-right legend, but separate
    # the angular scale and physical unit from the translational curves.
    angular_ax = base_ax.twinx()
    # A twinned Axes is drawn above its parent regardless of a legend artist's
    # zorder. Put the combined legend on that upper Axes instead.
    angular_ax.patch.set_visible(False)
    for coord, label in zip(('x', 'y'), BASE_LABELS[kind][:2]):
        base_ax.plot(t, data[f'{kind}_world_car/{coord}'], lw=1.2, label=label)
    angular_ax.plot(t, data[f'{kind}_world_car/theta'], lw=1.2,
                    color='tab:green', label=BASE_LABELS[kind][2])
    base_ax.set_title(BASE_TITLES[kind], fontweight='bold')
    base_ax.set_ylabel(f'{BASE_TITLES[kind].split()[-1]} ({UNITS[kind][0]})')
    angular_ax.set_ylabel(f'{BASE_TITLES[kind].split()[-1]} ({UNITS[kind][1]})')
    base_ax.grid(True)
    lines = base_ax.get_lines() + angular_ax.get_lines()
    angular_ax.legend(lines, [line.get_label() for line in lines],
                      loc='upper right', fontsize=9, frameon=True,
                      facecolor='white', framealpha=plt.rcParams['legend.framealpha'])

    for j in range(1, 7):
        arm_ax.plot(t, data[f'{kind}_joint{j}'], lw=1.0,
                    label=ARM_LABELS[kind](j))
    arm_ax.set_title(ARM_TITLES[kind], fontweight='bold')
    arm_ax.set_ylabel(f'{ARM_TITLES[kind].split()[-1]} ({UNITS[kind][1]})')
    arm_ax.grid(True)
    arm_ax.legend(loc='upper right', fontsize=8)
    return angular_ax


def align_base_ticks(base_ax, angular_ax):
    """Map visible left major tick heights into unchanged right-axis units."""
    left_min, left_max = base_ax.get_ylim()
    right_min, right_max = angular_ax.get_ylim()
    left_ticks = np.asarray(base_ax.get_yticks(), dtype=float)
    visible = left_ticks[(left_ticks >= left_min) & (left_ticks <= left_max)]
    mapped = right_min + (visible - left_min) * (right_max - right_min) / (left_max - left_min)
    angular_ax.yaxis.set_major_locator(FixedLocator(mapped))
    angular_ax.yaxis.set_major_formatter(FormatStrFormatter('%.3g'))
    return visible, mapped


def export_emf(fig, target):
    """Export vector paths via a temporary SVG; never leave SVG in output."""
    inkscape = shutil.which('inkscape')
    if inkscape is None:
        return False
    with tempfile.TemporaryDirectory(prefix='v2_plot_') as scratch:
        svg = Path(scratch)/'figure.svg'
        emf = Path(scratch)/'figure.emf'
        # Paths avoid font substitution in Word while retaining vector curves/text.
        with plt.rc_context({'svg.fonttype': 'path'}):
            fig.savefig(svg, bbox_inches='tight', facecolor='white')
        subprocess.run([inkscape, str(svg), '--export-type=emf',
                        f'--export-filename={emf}'], check=True,
                       capture_output=True, text=True)
        if not emf.is_file() or emf.stat().st_size < 88 or emf.read_bytes()[40:44] != b' EMF':
            raise ValueError(f'Inkscape output is not a valid EMF: {emf}')
        staged = target.with_suffix('.emf.tmp')
        try:
            shutil.copyfile(emf, staged)
            os.replace(staged, target)
        finally:
            staged.unlink(missing_ok=True)
    return True


def save_plots(data, folder):
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    pairs = []
    for row, kind in enumerate(('q', 'v', 'a')):
        pairs.append((axes[row, 0], plot_pair(axes[row, 0], axes[row, 1], data, kind)))
    for ax in axes[-1]:
        ax.set_xlabel('Time (s)')
    fig.subplots_adjust(left=.08, right=.92, top=.94, hspace=.35, wspace=.45)
    fig.canvas.draw()  # finish autoscale before mapping major ticks
    for left, right in pairs:
        align_base_ticks(left, right)
    fig.savefig(folder/'trajectory_6panel.png', dpi=220, bbox_inches='tight', facecolor='white')
    try:
        six_emf = export_emf(fig, folder/'trajectory_6panel.emf')
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        print(f'{folder}: EMF unavailable: {exc}')
        six_emf = False
    plt.close(fig)

    # Jerk is supplementary: retain the original q/v/a six-panel ordering.
    fig, axes = plt.subplots(1, 2, figsize=(16, 4.5))
    right = plot_pair(axes[0], axes[1], data, 'jerk')
    for ax in axes:
        ax.set_xlabel('Time (s)')
    fig.subplots_adjust(left=.08, right=.92, top=.88, wspace=.45)
    fig.canvas.draw()
    align_base_ticks(axes[0], right)
    fig.savefig(folder/'jerk.png', dpi=220, bbox_inches='tight', facecolor='white')
    try:
        jerk_emf = export_emf(fig, folder/'jerk.emf')
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        print(f'{folder}: EMF unavailable: {exc}')
        jerk_emf = False
    plt.close(fig)
    return six_emf and jerk_emf


def header_valid(path):
    with path.open(newline='', encoding='utf-8-sig') as source:
        header = next(csv.reader(source), [])
    return len(header) == len(set(header)) and all(name in header for name in ('time_s', *FIELDS))


def dated(path, root):
    for part in path.relative_to(root).parts:
        match = re.match(r'^V2-(\d{8})(?:-|$)', part)
        if match:
            try:
                return datetime.strptime(match.group(1), '%Y%m%d').date(), 'directory'
            except ValueError:
                break
    return datetime.fromtimestamp(path.stat().st_mtime, BEIJING).date(), 'mtime'


def scan(root, since=None):
    found = []
    if not root.is_dir():
        return found
    for path in root.rglob('trajectory.csv'):
        if any(part.startswith('.') for part in path.relative_to(root).parts):
            continue
        try:
            if not path.is_file() or not header_valid(path):
                continue
            day, source = dated(path, root)
            if since is None or day >= since:
                found.append((path, day, source))
        except (OSError, UnicodeError, csv.Error):
            continue
    return sorted(found, key=lambda item: (-item[1].toordinal(), str(item[0].relative_to(root))))


def destination(path, output):
    if output is None:
        return path.parent/'plots'
    try:
        return output/path.parent.relative_to(OUTPUTS)
    except ValueError:
        return output/path.parent.name


def draw_one(path, output=None):
    path = path.resolve(strict=True)
    data = read_csv(path)
    folder = destination(path, output)
    try:
        folder.mkdir(parents=True, exist_ok=True)
        has_emf = save_plots(data, folder)
        (folder/'source.txt').write_text(
            f'CSV: {path}\nRows: {len(data["time_s"])}\n'
            'Raw exported q/v/a/jerk; no filtering, resampling or differentiation.\n'
            f'EMF: {"exported" if has_emf else "unavailable"}\n', encoding='utf-8')
        print(f'{path} -> {folder}: trajectory_6panel.png jerk.png; '
              f'EMF={"PASS" if has_emf else "UNAVAILABLE"}')
    finally:
        plt.close('all')


def draw_many(paths, output=None):
    destinations = {}
    failures = 0
    for path in paths:
        try:
            target = destination(path.resolve(strict=True), output)
            previous = destinations.get(target)
            if previous is not None and previous != path.resolve():
                raise ValueError(f'output collision with {previous}; use separate output roots')
            destinations[target] = path.resolve()
            draw_one(path, output)
        except Exception as exc:
            print(f'PLOT_FAILED {path}: {exc}')
            failures += 1
    return failures


def show(items, root):
    print(f'Available trajectories: {len(items)}')
    for number, (path, day, source) in enumerate(items, 1):
        flag = ' [mtime]' if source == 'mtime' else ''
        print(f'{number}. {day}{flag} {path.relative_to(root)}')


def interactive(root, since, output):
    items = scan(root, since)
    show(items, root)
    while True:
        try:
            line = input('plot> ').strip().split()
        except EOFError:
            break
        if not line:
            continue
        command, *arguments = line
        if command == 'quit' and not arguments:
            break
        if command == 'list' and not arguments:
            show(items, root)
        elif command == 'refresh' and not arguments:
            items = scan(root, since)
            show(items, root)
        elif command == 'plot' and arguments:
            if arguments == ['all']:
                selected = [item[0] for item in items]
            elif all(arg.isdigit() and 1 <= int(arg) <= len(items) for arg in arguments):
                selected = [items[int(arg)-1][0] for arg in arguments]
            else:
                print('Choose existing numbers or: plot all')
                continue
            draw_many(selected, output)
        else:
            print('Commands: list | refresh | plot 1 [2 ...] | plot all | quit')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, nargs='*', help='one or more saved trajectory CSV files')
    parser.add_argument('--since', type=date.fromisoformat, metavar='YYYY-MM-DD',
                        help='scan mode: include this date and later')
    parser.add_argument('--output', type=Path,
                        help='optional output root; preserves each CSV path below v2/outputs')
    args = parser.parse_args()
    output = args.output.resolve() if args.output else None
    if args.csv:
        return 1 if draw_many(args.csv, output) else 0
    else:
        interactive(OUTPUTS, args.since, output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
