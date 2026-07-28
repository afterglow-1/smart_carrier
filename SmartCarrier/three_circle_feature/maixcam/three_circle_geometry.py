"""Pure geometry model for the numbered three-circle board.

This module intentionally has no Maix imports so the inference rules can be
tested on a desktop. Board coordinates come from the supplied drawing:
number 1 = 0 mm, number 2 = 300 mm, number 3 = 580 mm.
"""

BOARD_POSITION_MM = {1: 0.0, 2: 300.0, 3: 580.0}

SOURCE_MISSING = 0
SOURCE_MEASURED = 1
SOURCE_INFERRED = 2
SOURCE_FUSED = 3

MIN_LABEL_CONFIDENCE = 45


def _distance(a, b):
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    return (dx * dx + dy * dy) ** 0.5


def _prediction(origin_x, origin_y, axis_x, axis_y, number):
    position = BOARD_POSITION_MM[number]
    return origin_x + axis_x * position, origin_y + axis_y * position


def _fit_pair(first, second):
    position_delta = (
        BOARD_POSITION_MM[second["label"]]
        - BOARD_POSITION_MM[first["label"]]
    )
    if abs(position_delta) < 1.0:
        return None
    axis_x = (second["x"] - first["x"]) / position_delta
    axis_y = (second["y"] - first["y"]) / position_delta
    origin_x = first["x"] - axis_x * BOARD_POSITION_MM[first["label"]]
    origin_y = first["y"] - axis_y * BOARD_POSITION_MM[first["label"]]
    return origin_x, origin_y, axis_x, axis_y


def _select_model(labeled):
    best = None
    best_score = -1000000.0
    for first_index in range(len(labeled)):
        for second_index in range(first_index + 1, len(labeled)):
            first = labeled[first_index]
            second = labeled[second_index]
            if first["label"] == second["label"]:
                continue
            model = _fit_pair(first, second)
            if model is None:
                continue
            origin_x, origin_y, axis_x, axis_y = model
            pixel_span = _distance(first, second)
            if pixel_span < 12.0:
                continue

            score = (
                first["label_confidence"]
                + second["label_confidence"]
                + first["detection_confidence"]
                + second["detection_confidence"]
            )
            # A wider numbered baseline gives a more accurate inferred point.
            number_span = abs(first["label"] - second["label"])
            score += number_span * 12.0
            residual_limit = max(8.0, pixel_span * 0.09)
            for observation in labeled:
                px, py = _prediction(
                    origin_x, origin_y, axis_x, axis_y,
                    observation["label"],
                )
                residual = ((observation["x"] - px) ** 2
                            + (observation["y"] - py) ** 2) ** 0.5
                score -= max(0.0, residual - residual_limit) * 5.0
            if score > best_score:
                best_score = score
                best = {
                    "origin_x": origin_x,
                    "origin_y": origin_y,
                    "axis_x": axis_x,
                    "axis_y": axis_y,
                    "anchors": (first, second),
                    "score": score,
                }
    return best


def _nearest_observation(observations, predicted_x, predicted_y, used):
    nearest = None
    nearest_distance = 1000000.0
    for observation in observations:
        if id(observation) in used:
            continue
        dx = observation["x"] - predicted_x
        dy = observation["y"] - predicted_y
        distance = (dx * dx + dy * dy) ** 0.5
        if distance < nearest_distance:
            nearest = observation
            nearest_distance = distance
    return nearest, nearest_distance


def solve_three_targets(observations):
    """Return all three numbered centres from two or three observations.

    Each observation needs x, y, detection_confidence, label and
    label_confidence. A weak third fragment is used only when it agrees with
    the two-circle model; otherwise the inferred point wins.
    """
    labeled_by_number = {}
    for observation in observations:
        number = int(observation.get("label", 0))
        label_confidence = int(observation.get("label_confidence", 0))
        if number not in BOARD_POSITION_MM or label_confidence < MIN_LABEL_CONFIDENCE:
            continue
        old = labeled_by_number.get(number)
        value = label_confidence + int(observation.get("detection_confidence", 0))
        if old is None or value > (
                old["label_confidence"] + old["detection_confidence"]):
            labeled_by_number[number] = observation

    labeled = list(labeled_by_number.values())
    if len(labeled) < 2:
        return {"valid": False, "quality": 0, "targets": []}

    model = _select_model(labeled)
    if model is None:
        return {"valid": False, "quality": 0, "targets": []}

    anchors = model["anchors"]
    used = {id(anchors[0]), id(anchors[1])}
    axis_length = (model["axis_x"] ** 2 + model["axis_y"] ** 2) ** 0.5
    adjacent_pixels = axis_length * 290.0
    agreement_gate = max(8.0, adjacent_pixels * 0.10)
    targets = []

    for number in (1, 2, 3):
        predicted_x, predicted_y = _prediction(
            model["origin_x"], model["origin_y"],
            model["axis_x"], model["axis_y"], number,
        )
        direct = labeled_by_number.get(number)
        if direct is None:
            direct, residual = _nearest_observation(
                observations, predicted_x, predicted_y, used,
            )
        else:
            residual = ((direct["x"] - predicted_x) ** 2
                        + (direct["y"] - predicted_y) ** 2) ** 0.5

        if direct is anchors[0] or direct is anchors[1]:
            source = SOURCE_MEASURED
            output_x = direct["x"]
            output_y = direct["y"]
            confidence = min(
                direct["detection_confidence"], direct["label_confidence"]
            )
            used.add(id(direct))
        elif direct is None or residual > agreement_gate:
            source = SOURCE_INFERRED
            output_x = predicted_x
            output_y = predicted_y
            confidence = min(
                anchors[0]["detection_confidence"],
                anchors[1]["detection_confidence"],
                anchors[0]["label_confidence"],
                anchors[1]["label_confidence"],
                82,
            )
        else:
            direct_quality = int(direct.get("detection_confidence", 0))
            label_quality = int(direct.get("label_confidence", 0))
            if direct.get("label", 0) != number:
                label_quality = 35
            effective_quality = min(direct_quality, max(1, label_quality))
            if effective_quality < 55:
                # A highly incomplete third ring is less accurate than the
                # model fitted from two reliable, numbered circles.
                source = SOURCE_INFERRED
                output_x = predicted_x
                output_y = predicted_y
                confidence = min(80, max(45, effective_quality + 15))
            else:
                source = SOURCE_FUSED
                direct_weight = min(0.75, max(0.35, effective_quality / 120.0))
                output_x = direct["x"] * direct_weight + predicted_x * (1.0 - direct_weight)
                output_y = direct["y"] * direct_weight + predicted_y * (1.0 - direct_weight)
                confidence = min(95, effective_quality + 8)
            used.add(id(direct))

        targets.append({
            "number": number,
            "x": output_x,
            "y": output_y,
            "confidence": int(confidence),
            "source": source,
        })

    anchor_quality = min(
        anchors[0]["detection_confidence"],
        anchors[1]["detection_confidence"],
        anchors[0]["label_confidence"],
        anchors[1]["label_confidence"],
    )
    span_bonus = 10 if abs(anchors[0]["label"] - anchors[1]["label"]) == 2 else 0
    return {
        "valid": True,
        "quality": min(100, int(anchor_quality + span_bonus)),
        "targets": targets,
        "pixels_per_mm": axis_length,
    }
