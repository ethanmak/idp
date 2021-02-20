import numpy as np


def normalize(x: np.ndarray):
    """
    Returns the unit version of a vector

    :param x: Vector to be normalized
    :return: Normalized vector
    """
    return x / np.linalg.norm(x)


def clamp(x: float, low: float, high: float):
    """
    Clamps a value to bounds

    :param x: Value to clamp
    :param low: Lower bound
    :param high: Upper bound
    :return: Clamped value
    """
    return max(min(x, high), low)


def vector_degree(x: np.ndarray, flip_x: bool = False):
    """
    Turns a vector into a degree heading, where the positive x vector is 0 and angles increase clockwise

    :param x: Vector to process
    :param flip_x: Whether to flip x coordinate to account for left or right handedness
    :return: Degree heading of vector
    """
    index = 1 if len(x) == 2 else 2
    flip_x = -1 if flip_x else 1
    deg = np.degrees(np.arctan2(x[index], x[0] * flip_x))
    return deg if deg > 0 else deg + 360


def min_abs(*x):
    """
    Returns the value with the minimum absolute value

    :param x: Iterable of values to process, or values to process
    :return: Value with minimum absolute value
    """
    return min(x, key=abs)


def angle_subtract(x1: float, x2: float) -> float:
    """
    Returns the difference between two headings, taking into account the circular nature of headings and negative headings:
    ie. 0 deg - 359 deg = 1 deg

    :param x1: First degree heading
    :param x2: Second degree heading
    :return: Difference between headings
    """
    return min_abs(x1 - x2, x1 - x2 + 360, x1 - x2 - 360)


def degree_to_vector(degree: float) -> np.ndarray:
    """
    Turns a degree heading into a vector

    :param degree: Degree heading
    :return: Unit vector associated with degree heading
    """
    degree = np.radians(degree)
    return np.array([np.cos(degree), np.sin(degree)])


def distance_segment_point(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    """
    Determines the closest distance of a point to a line segment

    :param p1: Start coordinates of line segment
    :param p2: End coordinates of line segment
    :param p3: Coordinates of point
    :return: Calculated distance
    """
    if np.isclose(p1, p2).all():
        return np.linalg.norm(p3 - p2)
    l = max(0, min(1, np.dot(p3 - p1, p2 - p1) / np.linalg.norm(p2 - p1) ** 2))
    return np.linalg.norm(p3 - (p1 + l * (p2 - p1)))


def distance_segment_point_no_end(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
    """
    Determines the closest distance of a point to a line segment, excluding if the closest point on the line segment are the endpoints

    :param p1: Start coordinates of line segment
    :param p2: End coordinates of line segment
    :param p3: Coordinates of point
    :return: Calculated distance, 1000 if the closest point on line segment is an endpoint
    """
    dist = distance_segment_point(p1, p2, p3)
    if np.isclose(dist, np.linalg.norm(p3 - p2)) or np.isclose(dist, np.linalg.norm(p3 - p1)):
        return 1000
    return dist


def line_segments_intersect(p1, p2, p3, p4) -> bool:
    """
    Determines if two line segments intersect

    :param p1: Start coordinates of line segment 1
    :param p2: End coordinates of line segment 1
    :param p3: Start coordinates of line segment 2
    :param p4: End coordinates of line segment 2
    :return: True if line segments intersect, False otherwise
    """
    def perp(a):
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    da = p2 - p1
    db = p4 - p3
    dp = p1 - p3
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    point = (num / denom.astype(float)) * db + p3
    return point
