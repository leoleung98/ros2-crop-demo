#!/usr/bin/env python3
import csv
import math
import sys
from statistics import mean, pstdev

THRESH_SYM = 0.05
WINDOW_SEC = 3.0

def time_to_stable(rows):
    # rows: list of dict with keys t_sec sym status
    n = len(rows)
    j = 0
    for i in range(n):
        t_start = float(rows[i]['t_sec'])
        # advance j until window covers WINDOW_SEC
        while j < n and float(rows[j]['t_sec']) - t_start < WINDOW_SEC:
            j += 1
        if j <= i:
            continue
        ok = True
        for k in range(i, j):
            if abs(float(rows[k]['crop_symmetry'])) > THRESH_SYM: ok = False; break
            if int(rows[k]['status']) != 1: ok = False; break
        if ok:
            return t_start
    return math.nan

def load_rows(path):
    with open(path, 'r') as f:
        r = csv.DictReader(f)
        return list(r)

def main():
    if len(sys.argv) < 2:
        print('usage: analyze_metrics.py path_to_csv'); sys.exit(1)
    rows = load_rows(sys.argv[1])
    if not rows:
        print('empty csv'); sys.exit(2)

    t_stable = time_to_stable(rows)
    greens = [float(x['green_lane']) for x in rows]
    grounds = [float(x['ground_lane']) for x in rows]
    syms = [abs(float(x['crop_symmetry'])) for x in rows]
    heads = [float(x['heading']) for x in rows]

    print('time_to_stable_sec', f'{t_stable:.3f}' if not math.isnan(t_stable) else 'NaN')
    print('mean_green_lane', f'{mean(greens):.4f}')
    print('mean_ground_lane', f'{mean(grounds):.4f}')
    print('mean_abs_symmetry', f'{mean(syms):.4f}')
    print('std_heading', f'{pstdev(heads):.4f}')

if __name__ == '__main__':
    main()
