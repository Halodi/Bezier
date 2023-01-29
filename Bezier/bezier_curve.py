from bezier import Curve, PolyCurve
import numpy as np
from typing import List


class BezierControlPoints:
    def __init__(self, x_points: list, y_points: list):
        self.points = np.empty(shape=[0, 2])
        if len(x_points) != 0 and len(y_points) != 0:
            if len(x_points) != len(y_points):
                raise Exception("x and y don't have same dimension")
            self.set_points_raw(x_points, y_points)

    def set_points_raw(self, x_points: list, y_points: list):
        self.clear_points()
        for point in zip(x_points, y_points):
            self.add_point(point[0], point[1])

    def set_points(self, points: np.ndarray):
        if self.num_points_ext(points) > 4:
            raise Exception("Bezier curve can only have 4 points")
        self.points = points

    def add_point(self, x, y):
        if self.num_points() == 4:
            raise Exception("Bezier curve can only have 4 points")
        self.points = np.append(self.points, np.array([[x, y]]), axis=0)

    def clear_points(self):
        self.points = np.empty(shape=[0, 2])

    def num_points(self):
        return self.points.shape[0]

    @staticmethod
    def num_points_ext(points: np.ndarray):
        return points.shape[0]


class BezierCurve:

    def __init__(self, label: str, control_points: BezierControlPoints):
        self.bezier = None
        self.label = label
        self.control_points = control_points
        if control_points is None:
            self.control_points = BezierControlPoints([], [])
        self.bezier_line_object = None
        self.total_time = 0

    def get_label(self):
        return self.label

    def set_total_time(self, time: float):
        self.total_time = time

    def add_point(self, x, y):
        self.control_points.add_point(x, y)

    def clear_points(self):
        self.control_points.clear_points()

    def get_curve(self):
        return Curve(self.control_points.points)

    def set_control_points(self, control_points: BezierControlPoints):
        self.control_points = control_points

    def get_poly(self):
        if not self.control_points.num_points() == 4:
            raise Exception("Bezier curve can only have 4 points")
        else:
            b = Curve(self.control_points.points)
            interpolated = b.polyline(1.0001, 1.0)
            x = [x for x, y in interpolated]
            y = [y for x, y in interpolated]

            return x, y


class PolyBezierCurve:
    def __init__(self, label: str, curves: List[BezierCurve]):
        self.label: str = label
        self.curves: List[BezierCurve] = curves
        self.total_time = 0.0

    def get_num_segments(self):
        return len(self.curves)

    def get_label(self):
        return self.label

    def set_total_time(self, time: float):
        self.total_time = time

    def add_curve(self, curve: BezierCurve):
        self.curves.append(curve)

    def clear_curves(self):
        self.curves.clear()

    def set_curves(self, curves: List[BezierCurve]):
        self.curves = curves

    def pop(self):
        return self.curves.pop()

    def set_curve_from_label(self, curve: BezierCurve):
        for i in range(self.get_num_segments()):
            if self.curves[i].get_label() == curve.get_label():
                self.curves[i] = curve
                return
        raise Exception("Curve with this label not found in list of curves")

    def get_poly(self):
        if len(self.curves) == 0:
            raise Exception("Must have at least 1 bezier curve segment")
        else:
            p = PolyCurve()
            for c in self.curves:
                p.insertBack(c.get_curve())
            interpolated = p.polyline(1.0001, 1.0)
            x = [x for x, y in interpolated]
            y = [y for x, y in interpolated]

            return x, y
