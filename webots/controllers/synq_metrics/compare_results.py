"""Compare two measured Webots result JSON files; never synthesize a PASS."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "synq_controller"))
from benchmark_metrics import improvement_percent

def main():
    if len(sys.argv) != 3:
        raise SystemExit("Usage: python compare_results.py baseline.json synq.json")
    baseline = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    synq = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    if baseline.get("mode") != "baseline" or synq.get("mode") != "synq":
        raise SystemExit("Files must contain measured baseline and synq results, in that order")
    if not all(result.get("physically_complete") and result.get("peer_exchange_complete") for result in (baseline, synq)):
        raise SystemExit("Both runs need three physical goals and all-to-all peer intent exchange")
    if baseline.get("makespan") is None or synq.get("makespan") is None:
        raise SystemExit("Both experiments must complete before comparison")
    improvement = improvement_percent(baseline["makespan"], synq["makespan"])
    zero = baseline["collisions"] == synq["collisions"] == 0
    print(f"baseline_makespan: {baseline['makespan']:.3f}")
    print(f"synq_makespan: {synq['makespan']:.3f}")
    print(f"improvement_percent: {improvement:.2f}")
    print("SUCCESS CRITERIA")
    print("----------------")
    print(f"Zero collisions: {'PASS' if zero else 'FAIL'}")
    print(f">=20% task-time improvement: {'PASS' if improvement >= 20 else 'FAIL'}")

if __name__ == "__main__":
    main()
