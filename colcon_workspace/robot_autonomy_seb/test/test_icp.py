import numpy as np
import pytest
from robot_autonomy_seb.icp import ICP, rotation_to_angle


# TODO: use pytest fixture
@pytest.mark.skip(reason="Should be ignore (but not using this feature")
def test_template(points, angle, t, tolerance_angle=0.001, tolerance_t=1e-6):
    icp = ICP()

    # init icp with first point cloud
    icp.execute_icp(points)

    # execute icp in the transformed point cloud
    t = np.array([t]).T
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle), np.cos(angle)]])
    T = icp.execute_icp(np.dot(R, points) + t)

    diff_angle = abs(angle - rotation_to_angle(T[:2, :2])) * 180 / 3.14
    print(f"angle diff: {diff_angle:.08f} °")
    print(f"x, y diff: {(t - T[:2, 2:3]).T} meters")

    # check result
    assert diff_angle < tolerance_angle
    assert np.linalg.norm(t - T[:2, 2:3]) < tolerance_t


def test_no_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 0, [0, 0])


def test_x_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 0, [0.05, 0])


def test_y_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 0, [0, 0.05])


def test_rot1_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 2 / 180 * 3.14, [0, 0])


def test_rot2_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, -2 / 180 * 3.14, [0, 0])


def test_rot3_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 5 / 180 * 3.14, [0, 0])


def test_rot4_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, -5 / 180 * 3.14, [0, 0])


def test_both_displacement():
    points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]]).T
    test_template(points, 2 / 180 * 3.14, [0.05, 0.05])