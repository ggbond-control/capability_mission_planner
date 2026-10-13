#!/usr/bin/env python3
"""Compare SIPP with/without one repair, audit schedules, save raw metrics."""
import argparse
import csv
import json
import math
import statistics
import subprocess
from pathlib import Path


SCENARIOS = [
    "zju2/01_single_generalist", "zju2/02_four_specialists",
    "zju2/03_overlapping_capabilities", "zju2/04_more_robots_than_tasks",
    "zju2/05_many_tasks_few_robots", "zju2/06_shared_sites", "myj1/myj1",
    "bdz1/03_homogeneous_shared_charger_benchmark",
    "bdz1/04_five_robot_types_50_points",
    "sxbdz1/01_six_robots_80_sites_100_tasks",
]


def route_frames(path):
    routes = {}
    last_ticks = {}
    with path.open() as file:
        for row in csv.DictReader(file):
            robot = row.pop("robot")
            tick = int(row.pop("tick"))
            assert tick == last_ticks.get(robot, -1) + 1, "missing schedule tick"
            last_ticks[robot] = tick
            frames = routes.setdefault(robot, [])
            if not frames or frames[-1] != row:
                assert int(row["frame"]) == len(frames), "lost mandatory route/service frame"
                frames.append(row)
    return routes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("scenario_root", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--order-variants", action="store_true")
    parser.add_argument("--stress-variants", action="store_true",
                        help="also vary service duration and speed in the 100-task case (requires PyYAML)")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [(scenario, 0, 1, 1) for scenario in SCENARIOS]
    if args.order_variants:
        cases += [(SCENARIOS[-1], rotation, 1, 1) for rotation in range(1, 6)]
        cases += [(SCENARIOS[-2], rotation, 1, 1) for rotation in range(1, 5)]
    if args.stress_variants:
        cases += [(SCENARIOS[-1], 0, service, speed) for service, speed in ((1.5, 1), (2, 1), (1, .9))]
    result = []
    for scenario, rotation, service, speed in cases:
        name = scenario.replace("/", "__") + f"__rotate{rotation}"
        config_path = args.scenario_root / "configs" / (scenario + ".yaml")
        if service != 1 or speed != 1:
            import yaml
            name += f"__service{service}__speed{speed}"
            config = yaml.safe_load(config_path.read_text())
            config["map"]["directory"] = str((config_path.parent / config["map"]["directory"]).resolve())
            config["output_directory"] = str(args.output.resolve())
            for task in config["tasks"]:
                task["service_seconds"] = math.ceil(task["service_seconds"] * service)
            for robot in config["robots"]:
                robot["nominal_speed_mps"] *= speed
            config_path = args.output / f"{name}.yaml"
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        record = {"scenario": scenario, "rotation": rotation, "service_scale": service, "speed_scale": speed}
        for mode in ("sipp", "repair"):
            csv_path = args.output / f"{name}__{mode}.csv"
            samples = []
            for repetition in range(args.repeats):
                command = [str(args.benchmark.resolve()),
                           str(config_path),
                           f"--rotate={rotation}"]
                if repetition == 0:
                    command.append(f"--csv={csv_path}")
                if mode == "sipp":
                    command.append("--no-repair")
                process = subprocess.run(command, capture_output=True, text=True, timeout=90)
                if process.returncode:
                    raise RuntimeError(f"{name}/{mode}: {process.stderr} {process.stdout}")
                metrics = json.loads(process.stdout)
                assert metrics["unsafe"] == 0, "unsafe schedule"
                samples.append(metrics)
            record[mode] = {"samples": samples,
                            "median_seconds": statistics.median(s["seconds"] for s in samples),
                            "median_rss_mb": statistics.median(s["rss_mb"] for s in samples)}
        before = record["sipp"]["samples"][0]
        for after in record["repair"]["samples"]:
            assert after["makespan"] <= before["makespan"] + 1e-6, "makespan regression"
            assert after["wait"] <= before["wait"] + 1e-6, "wait regression"
            assert after["total_finish"] <= before["total_finish"] + 1e-6, "completion sum regression"
        assert route_frames(args.output / f"{name}__sipp.csv") == route_frames(args.output / f"{name}__repair.csv"), "changed task/route/service sequence"
        result.append(record)
        (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
        print(f"{name}: {before['makespan']} -> {record['repair']['samples'][0]['makespan']} s; "
              f"coord {record['sipp']['median_seconds']:.4f} -> {record['repair']['median_seconds']:.4f} s", flush=True)


if __name__ == "__main__":
    main()
