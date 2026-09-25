"""Pure benchmark arithmetic; physical measurements come from Webots."""

def improvement_percent(baseline_makespan, synq_makespan):
    if baseline_makespan <= 0:
        raise ValueError("Baseline makespan must be positive")
    return (baseline_makespan - synq_makespan) / baseline_makespan * 100.0

def result_summary(mode, completion, wait, conflicts, yields, collisions, start):
    if set(completion) != {"R1", "R2", "R3"}:
        raise ValueError("All three robots must finish before a result is valid")
    return {"mode": mode, "completion": dict(completion),
            "makespan": max(completion.values()) - start,
            "total_waiting": sum(wait.values()), "conflicts": sum(conflicts.values()),
            "yields": sum(yields.values()), "collisions": collisions}
