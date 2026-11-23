"""Put the dashboard CSVs of several events into one set, so the cycle-analysis dashboard can show one event or all of them.

    python tools/video/combine_dash.py --events "Hartland Qualifier 2026" "Novi Qualifier 2026"

Reads robot-logs/events/<event>/video/dash/*.csv (written by export_dash.py) for each event, in the order given (earliest
first), and writes robot-logs/events/_combined/video/dash/*.csv. Match labels repeat between events (Q17, M3, M10 ...), so in
the combined files `match` is the event's short name and the label ("Hartland Q17"), and every row gains:
    event   the event's short name: "dash_name" in its video_config.json, else the first word of its folder
    label   the match label within the event ("Q17")
matches.csv also gains `name`, how the match is shown ("M10 (final)"): "match_names" in video_config.json, else the label.
"""
import argparse, csv, json, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
FILES = ("salvos", "tips", "cycles", "tip_counts", "alignment", "matches", "partner_landings", "hive_pollen", "pickups", "cycle_salvos")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--events", nargs="+", required=True); a = ap.parse_args()
    ev_dir = os.path.join(ROOT, "robot-logs", "events")
    out_dir = os.path.join(ev_dir, "_combined", "video", "dash")
    os.makedirs(out_dir, exist_ok=True)
    cfgs = {e: json.load(open(os.path.join(ev_dir, e, "video", "video_config.json"))) for e in a.events}
    short = {e: cfgs[e].get("dash_name") or e.split()[0] for e in a.events}
    assert len(set(short.values())) == len(a.events), f"two events share a short name: {short}"
    for k in FILES:
        header, rows = None, []
        for e in a.events:
            with open(os.path.join(ev_dir, e, "video", "dash", f"{k}.csv"), newline="") as f:
                r = csv.reader(f); h = next(r)
                assert h[0] == "match" and (header is None or h == header), f"{e}/{k}.csv: columns differ from the other events' ({h})"
                header = h
                names = cfgs[e].get("match_names", {})
                for row in r:
                    label = row[0]
                    rows.append([f"{short[e]} {label}"] + row[1:] + [short[e], label] + ([names.get(label, label)] if k == "matches" else []))
        with open(os.path.join(out_dir, f"{k}.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(header + ["event", "label"] + (["name"] if k == "matches" else [])); w.writerows(rows)
        print(f"{k}.csv: {len(rows)} rows")


if __name__ == "__main__":
    main()
