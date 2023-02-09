from typing import List

import numpy as np
from bezier import Curve, PolyCurve, HermiteVelocityProfile, BezierSegment, BezierTrajectory2D
from geometry_msgs.msg import Vector3

from ocs2_ros2_msgs.msg import BezierSegment as SegmentMsg
from ocs2_ros2_msgs.msg import BezierTrajectory

NUM_BEZIER_POINTS: int = 4  # cubic


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
        if self.num_points_ext(points) > NUM_BEZIER_POINTS:
            raise Exception("Bezier curve can only have 4 points")
        self.points = points

    def add_point(self, x, y):
        if self.num_points() == NUM_BEZIER_POINTS:
            raise Exception("Bezier curve can only have " + str(NUM_BEZIER_POINTS) + " points")
        self.points = np.append(self.points, np.array([[x, y]]), axis=0)

    def set_point(self, idx, x, y):
        self.points[idx, 0] = x
        self.points[idx, 1] = y

    def clear_points(self):
        self.points = np.empty(shape=[0, 2])

    def num_points(self):
        return self.points.shape[0]

    @staticmethod
    def num_points_ext(points: np.ndarray):
        return points.shape[0]


class BezierCurve:

    def __init__(self, label: str, control_points: BezierControlPoints):
        self.bezier: Curve = None
        self.label = label
        self.control_points = control_points
        if control_points is None:
            self.control_points = BezierControlPoints([], [])
        if self.control_points.num_points() == NUM_BEZIER_POINTS:
            self.bezier = Curve(self.control_points.points)
        self.bezier_line_object = None
        self.total_time = 0.0
        self.start_velocity = 0.0
        self.end_velocity: float = 0.0

    def get_label(self):
        return self.label

    def set_end_time(self, time: float):
        self.total_time = time

    def add_point(self, x, y):
        self.control_points.add_point(x, y)
        if self.control_points.num_points() == NUM_BEZIER_POINTS:
            self.bezier = Curve(self.control_points.points)

    def clear_points(self):
        self.control_points.clear_points()
        self.bezier = None

    def get_curve(self):
        if self.bezier is not None:
            return self.bezier
        elif self.bezier is None and self.control_points.num_points() == NUM_BEZIER_POINTS:
            self.bezier = Curve(self.control_points.points)
            return self.bezier
        raise Exception("Cannot get curve when control points not established")

    def get_control_points(self):
        points = self.bezier.controlPoints()
        x = [x for x, y in points]
        y = [y for x, y in points]
        return x, y

    def make_continuous_with_other(self, other):
        self.get_curve().applyContinuity(other.get_curve())
        new_points = self.get_curve().controlPoints()
        x = [x for x, y in new_points]
        y = [y for x, y in new_points]
        self.set_control_points(BezierControlPoints(x, y))

    def set_control_points(self, control_points: BezierControlPoints):
        self.control_points = control_points
        self.bezier = Curve(self.control_points.points)

    def set_control_point(self, idx: int, x: float, y: float):
        self.control_points.set_point(idx, x, y)

    def update_curve(self):
        self.bezier = Curve(self.control_points.points)

    def get_poly(self):
        if not self.control_points.num_points() == NUM_BEZIER_POINTS:
            raise Exception("Bezier curve can only have " + str(NUM_BEZIER_POINTS) + " points")
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

    def add_curve_with_parameters(self, curve: BezierCurve, total_time, start_velocity, end_velocity):
        curve.total_time = total_time
        curve.start_velocity = start_velocity
        curve.end_velocity = end_velocity
        self.curves.append(curve)

    def clear_curves(self):
        self.curves.clear()

    def set_curves(self, curves: List[BezierCurve]):
        self.curves = curves

    def get_curves(self):
        return self.curves

    def pop(self):
        return self.curves.pop()

    def set_curve_from_label(self, curve: BezierCurve):
        for i in range(self.get_num_segments()):
            if self.curves[i].get_label() == curve.get_label():
                self.curves[i] = curve
                return
        raise Exception("Curve with this label not found in list of curves")

    def apply_continuity(self):
        for i in range(self.get_num_segments() - 1):
            self.curves[i].make_continuous_with_other(self.curves[i])

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

    def to_msg(self):
        time_array: List[float] = [0.]
        vel_array: List[float] = [0.]
        msg = BezierTrajectory()
        for idx, curve in enumerate(self.curves):
            segment = self.default_segment_msg()
            segment.start_time = time_array[idx]
            time_array.append(time_array[idx] + curve.total_time)
            segment.end_time = time_array[idx + 1]
            segment.start_velocity = vel_array[idx]
            vel_array.append(curve.end_velocity)
            print(type(vel_array[idx + 1]))
            segment.end_velocity = vel_array[idx + 1]
            x, y = curve.get_control_points()
            segment.control_points = [self.point_msg(x[i], y[i]) for i in range(len(x))]
            msg.trajectory.append(segment)
        return msg

    @staticmethod
    def default_segment_msg():
        msg = SegmentMsg()
        msg.dimension = 2
        msg.use_relative_position = True
        msg.use_relative_time = True
        return msg

    @staticmethod
    def point_msg(x, y):
        msg = Vector3()
        msg.x = x
        msg.y = y
        return msg


def generate_velocity_profile(poly: PolyBezierCurve, time_resolution: float, time_array: List[float],
                              velocity_array: List[float]):
    assert len(time_array) == len(velocity_array)
    assert len(time_array) == poly.get_num_segments()
    segment_list: List[BezierSegment] = [
        BezierSegment(poly.get_curves()[0].control_points.points,
                      HermiteVelocityProfile(0.0, 0.0, time_array[0], velocity_array[0]))]
    for idx, curve in enumerate(poly.get_curves()[1:]):
        segment_list.append(
            BezierSegment(curve.control_points.points,
                          HermiteVelocityProfile(time_array[idx], velocity_array[idx], time_array[idx + 1],
                                                 velocity_array[idx + 1])))

    trajectory = BezierTrajectory2D(segment_list)

    num_ticks: float = (time_array[-1] - time_array[0]) / time_resolution
    query_times = np.linspace(0, time_array[-1], int(num_ticks), True)

    abs_vel, headings, ang_vel = [], [], []
    for time in list(query_times):
        abs_vel.append(trajectory.getVelocityAbs(time))
        headings.append(trajectory.getHeading(time))
        ang_vel.append(trajectory.getAngularVelocity(time))

    return list(query_times), abs_vel, ang_vel, headings
