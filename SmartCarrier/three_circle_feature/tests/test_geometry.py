import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "maixcam"))

from three_circle_geometry import (  # noqa: E402
    SOURCE_FUSED,
    SOURCE_INFERRED,
    solve_three_targets,
)


def observation(number, x, y, detect=90, label=90):
    return {
        "label": number,
        "label_confidence": label,
        "detection_confidence": detect,
        "x": x,
        "y": y,
    }


def target(solution, number):
    return next(item for item in solution["targets"] if item["number"] == number)


def test_adjacent_pair_infers_third():
    solution = solve_three_targets([
        observation(1, 20, 100),
        observation(2, 170, 100),
    ])
    assert solution["valid"]
    third = target(solution, 3)
    assert third["source"] == SOURCE_INFERRED
    assert abs(third["x"] - 310) < 0.01
    assert abs(third["y"] - 100) < 0.01


def test_outer_pair_infers_middle_using_300_of_580_ratio():
    solution = solve_three_targets([
        observation(1, 10, 30),
        observation(3, 300, 88),
    ])
    middle = target(solution, 2)
    assert middle["source"] == SOURCE_INFERRED
    assert abs(middle["x"] - 160) < 0.01
    assert abs(middle["y"] - 60) < 0.01


def test_weak_fragment_loses_to_geometric_prediction():
    fragment = observation(3, 318, 104, detect=25, label=20)
    solution = solve_three_targets([
        observation(1, 20, 100),
        observation(2, 170, 100),
        fragment,
    ])
    third = target(solution, 3)
    assert third["source"] == SOURCE_INFERRED
    assert abs(third["x"] - 310) < 0.01


def test_reliable_fragment_is_fused_when_consistent():
    solution = solve_three_targets([
        observation(1, 20, 100),
        observation(2, 170, 100),
        observation(3, 313, 102, detect=82, label=78),
    ])
    third = target(solution, 3)
    assert third["source"] == SOURCE_FUSED
    assert 310 < third["x"] < 313


if __name__ == "__main__":
    tests = [value for name, value in globals().items() if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
