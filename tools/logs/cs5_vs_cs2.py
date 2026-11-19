"""Scores the robot's element count against the CS2 launch count, with no human labels (2026-11-18).

CS2 (the sensor at the top of the feed ramp) sees each element as a salvo launches it, so the number of CS2 peaks in a salvo is
the number of elements that LEFT. The count the robot used at that salvo (the CS5 entry count, capped at the capacity, as logged in
the SALVO started line: "LAUNCHED n (the entry count k ...") is what it BELIEVED was held. Where the two differ, the CS5 count was
wrong.

A salvo is left out of the score (listed as "skip") when the truth is not what entered since the last salvo:
  - a manual outtake happened since the previous salvo started (elements left through the intake),
  - the previous salvo launched fewer than the count said (an element may have stayed in the robot),
  - it is the first salvo of the run (a preload of unknown size),
  - the salvo launched nothing by CS2 (a repeat salvo).

Usage:  python tools/logs/cs5_vs_cs2.py [log files...]      (no arguments: every log under robot-logs/ that has a cs2f column; a Launch Grid Tuner log lists each salvo with its spot)
        --peaks HI PROM   the CS2 peak rule (rise above the empty level, prominence), default 8 and 6 (use 5 and 3 on logs recorded at the 25 ms period)
"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
CS2_EMPTY = 37          # the CS2 empty level in the logs of 2026-11-18 (proximity counts)
CS2_RATE = 80           # key resolution for merging the fast samples (per second)


def load(path):
    ev, rows, hdr = [], [], None
    with open(path, encoding='utf-8', errors='replace') as fh:
        for ln in fh:
            if ln.startswith('H,'):
                hdr = ln.strip().split(',')
            elif ln.startswith('E,'):
                p = ln.rstrip('\n').split(',', 2)
                try:
                    ev.append((float(p[1]), p[2] if len(p) > 2 else ''))
                except ValueError:
                    pass
            elif ln.startswith('S,'):
                rows.append(ln.rstrip('\n').split(','))
    return ev, rows, hdr


def stream(rows, hdr, col):
    if hdr is None or col not in hdr:
        return [], []
    ix = hdr.index(col)
    ser = {}
    for f in rows:
        if len(f) <= ix:
            continue
        try:
            t = float(f[1])
        except ValueError:
            continue
        for tok in f[ix].split():
            m = re.match(r'(-?\d+)@(-?[\d.]+)', tok)
            if m:
                ser[round((t + float(m.group(2)) / 1000.0) * CS2_RATE)] = int(m.group(1))
    ks = sorted(ser)
    return [k / CS2_RATE for k in ks], [ser[k] for k in ks]


def peaks(T, V, a, b, hi, prom):
    w = [(t, v) for t, v in zip(T, V) if a <= t <= b]
    s = [v for _, v in w]
    sm = [sum(s[max(0, i - 1):i + 2]) / len(s[max(0, i - 1):i + 2]) for i in range(len(s))]
    cand = [i for i in range(1, len(sm) - 1) if sm[i] >= sm[i - 1] and sm[i] > sm[i + 1] and sm[i] >= CS2_EMPTY + hi]
    counted = []
    for i in cand:
        if not counted:
            counted.append(i)
            continue
        j = counted[-1]
        if min(sm[j], sm[i]) - min(sm[j:i + 1]) >= prom:
            counted.append(i)
        elif sm[i] > sm[j]:
            counted[-1] = i
    return [w[i][0] for i in counted]


def score(path, hi, prom, quiet=False):
    ev, rows, hdr = load(path)
    T, V = stream(rows, hdr, 'cs2f')
    if not T:
        return None
    outtakes = [t for t, m in ev if m.startswith('COMMAND manual outtake held')]
    salvos = []
    fires = [(t, re.match(r'FIRE spot (\d+) salvo (\d+)', m)) for t, m in ev if m.startswith('FIRE spot')]
    for t, m in ev:
        if not m.startswith('SALVO started'):
            continue
        used = re.search(r'LAUNCHED (\d+) \(the entry count (\d+)', m)
        spot = ''
        for ft, fm in fires:
            if 0.0 <= t - ft <= 3.0 and fm:
                spot = 'spot %s #%s' % (fm.group(1), fm.group(2))
        salvos.append((t, int(used.group(1)) if used else None, int(used.group(2)) if used else None, spot))
    out = []
    for i, (t, used, entries, spot) in enumerate(salvos):
        n2 = len(peaks(T, V, t - 0.2, t + 2.4, hi, prom))
        prev_t = salvos[i - 1][0] if i else None
        why = None
        if i == 0:
            why = 'first salvo (preload unknown)'
        elif any(prev_t <= o <= t for o in outtakes):
            why = 'outtake'
        elif out[-1]['n2'] < (out[-1]['used'] or 0):
            why = 'after a short salvo'
        elif n2 == 0:
            why = 'CS2 saw no launch'
        out.append({'t': t, 'used': used, 'entries': entries, 'n2': n2, 'skip': why, 'spot': spot})
    if not quiet:
        print('==', os.path.relpath(path, ROOT))
        print('  salvo   time  used  CS2  diff   note')
        for i, r in enumerate(out, 1):
            diff = '' if r['used'] is None else '%+d' % (r['used'] - r['n2'])
            note = 'skip: ' + r['skip'] if r['skip'] else ('ok' if r['used'] == r['n2'] else 'WRONG')
            print('  %5d %6.1f  %4s  %3d  %4s   %s%s' % (i, r['t'], r['used'], r['n2'], diff, note, ('   ' + r['spot']) if r['spot'] else ''))
    return out


def main(argv):
    hi, prom = 8, 6
    if '--peaks' in argv:
        k = argv.index('--peaks')
        hi, prom = int(argv[k + 1]), int(argv[k + 2])
        del argv[k:k + 3]
    paths = argv or sorted(glob.glob(os.path.join(ROOT, 'robot-logs', '*', '*.log')))
    scored = ok = 0
    under = over = 0
    for p in paths:
        res = score(p, hi, prom)
        if not res:
            continue
        for r in res:
            if r['skip'] or r['used'] is None:
                continue
            scored += 1
            if r['used'] == r['n2']:
                ok += 1
            elif r['used'] < r['n2']:
                under += 1
            else:
                over += 1
    print()
    print('scored salvos %d: CS5 count right %d, low %d, high %d  (truth = CS2 launches; skipped salvos are not counted)' % (scored, ok, under, over))


if __name__ == '__main__':
    main(sys.argv[1:])
