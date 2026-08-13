import csv
from pathlib import Path

import numpy as np

from trajectory_csv.compiler import compile_to_csv


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    return reader.fieldnames, rows


def _matrix(row, side):
    return np.array(
        [float(row[f"{side}_raw_matrix_{i}{j}"]) for i in range(4) for j in range(4)]
    ).reshape(4, 4)


def test_csv_has_33_columns_and_default_left_right_order(tmp_path):
    source = PACKAGE_ROOT / "trajectories" / "right_square_uniform_linear.yaml"
    output = tmp_path / "trajectory.csv"
    compile_to_csv(str(source), str(output))

    fieldnames, rows = _read_csv(output)
    assert len(fieldnames) == 33
    assert fieldnames[0] == "timestamp"
    assert fieldnames[1] == "left_raw_matrix_00"
    assert fieldnames[16] == "left_raw_matrix_33"
    assert fieldnames[17] == "right_raw_matrix_00"
    assert fieldnames[32] == "right_raw_matrix_33"
    assert float(rows[0]["timestamp"]) == 0.0
    assert np.all(np.diff([float(row["timestamp"]) for row in rows]) > 0.0)

    for side in ("left", "right"):
        for row in (rows[0], rows[len(rows) // 2], rows[-1]):
            matrix = _matrix(row, side)
            np.testing.assert_allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
            np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-12)
            np.testing.assert_allclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-12)


def test_reference_compatible_right_left_order(tmp_path):
    source = PACKAGE_ROOT / "trajectories" / "right_square_uniform_linear.yaml"
    output = tmp_path / "trajectory.csv"
    compile_to_csv(str(source), str(output), column_order="right-left")

    fieldnames, rows = _read_csv(output)
    assert fieldnames[1] == "right_raw_matrix_00"
    assert fieldnames[16] == "right_raw_matrix_33"
    assert fieldnames[17] == "left_raw_matrix_00"
    assert fieldnames[32] == "left_raw_matrix_33"
    assert _matrix(rows[0], "right").shape == (4, 4)
    assert _matrix(rows[0], "left").shape == (4, 4)
