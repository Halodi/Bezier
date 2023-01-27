import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
from matplotlib.lines import Line2D
from bezier_curve import *
from typing import TypedDict, List


class LineCircleLabel(TypedDict):
    line: str
    circles: List[str]


class LineBezierLabel(TypedDict):
    line: str
    bezier: str


# drag and drop helpers
mousepress = None
circle_list_list: List[List[patches.Circle]] = [[]]
line_list: List[Line2D] = []
bezier_line_list: List[Line2D] = []
line_circle_label_list: List[LineCircleLabel] = []
line_bezier_label_list: List[LineBezierLabel] = []
bezier_list: List[BezierCurve] = []
currently_dragging: bool = False
current_artist = None
max_bz_points: int = 4
number_of_curves: int = 0
offset = []

# plots
fig, ax = plt.subplots()

ax.set_title(
    "Hello retard",
    loc="left",
)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_aspect("equal")


def remove_last_curve_definition(event):
    global number_of_curves
    if len(circle_list_list) == 0:
        return

        # remove circles
    for circle in (circle_list_list[-1]):
        circle.remove()
    circle_list_list.pop()

    # remove 2d line
    line_list[-1].remove()
    line_list.pop()

    # remove bezier line
    bezier_line_list[-1].remove()
    bezier_line_list.pop()

    # remove bezier curve
    bezier_list.pop()

    # remove label dictionaries
    line_circle_label_list.pop()
    line_bezier_label_list.pop()

    number_of_curves -= 1
    plt.draw()


def create_new_curve_definition(event):
    global number_of_curves
    number_of_curves += 1
    xdata, ydata = [], []

    # plot control points
    circle_list = []
    for i in range(4):
        x, y = number_of_curves * 2, i * 2
        xdata.append(x)
        ydata.append(y)
        label = "circle" + str(number_of_curves) + "_" + str(i)
        circle = patches.Circle(
            (x, y),
            0.5,
            color="r",
            fill=False,
            lw=2,
            alpha=0.8,
            transform=ax.transData,
            label=label,
        )

        circle.set_picker(5)
        ax.add_patch(circle)
        circle_list.append(circle)

    circle_list_list.append(circle_list)
    # plot line through control points
    line = ax.plot(xdata, ydata, alpha=0.5, c="r", lw=2, label="line" + str(number_of_curves), picker=True)
    line_list.append(line[0])
    curve = BezierCurve("bezier" + str(number_of_curves),
                        BezierControlPoints(line_list[-1].get_xdata(), line_list[-1].get_ydata()))
    line_circle_label_list.append({'line': line_list[-1].get_label(), 'circles': [c.get_label() for c in circle_list]})
    bezier_list.append(curve)
    line_bezier_label_list.append({'line': line_list[-1].get_label(), 'bezier': curve.get_label()})

    plot_bezier(curve)
    plt.draw()


def plot_bezier(bezier: BezierCurve):
    points = bezier.get_poly()

    line_exists = False
    for line in bezier_line_list:
        if line.get_label() == bezier.get_label():
            line.set_data(points[0], points[1])
            line_exists = True
            break
    if line_exists:
        pass
    else:
        bezier_line_list.append(ax.plot(
            points[0], points[1], alpha=0.5, c="b", label=bezier.get_label(), lw=2, picker=True
        )[0])
        plt.draw()


def get_line_from_bezier_label(bezier_label: str):
    for dict_entry in line_bezier_label_list:
        if dict_entry['bezier'] == bezier_label:
            for line in line_list:
                if line.get_label() == dict_entry['line']:
                    return line

    raise Exception("Could not find 2D line defining bezier control points")


def update_all_bezier_lines():
    for bezier in bezier_list:
        line = get_line_from_bezier_label(bezier.get_label())
        bezier.set_control_points(BezierControlPoints(line.get_xdata(), line.get_ydata()))
        plot_bezier(bezier)


def get_line_and_idx_containing_circle(circle_label: str):
    for label_dict in line_circle_label_list:
        idx = [i for i in range(len(label_dict['circles'])) if circle_label in label_dict['circles'][i]]
        if len(idx) == 1:
            for line in line_list:
                if line.get_label() == label_dict['line']:
                    return line, idx[0]
        elif len(idx) > 1:
            raise Exception("Non unique circle label in line:circle label list")
    raise Exception("Cannot find line containing this circle label")


def on_press(event):
    global currently_dragging
    global mousepress
    currently_dragging = True
    if event.button == 3:
        mousepress = "right"
    elif event.button == 1:
        mousepress = "left"


def on_release(event):
    global current_artist, currently_dragging
    current_artist = None
    currently_dragging = False
    update_all_bezier_lines()


def on_pick(event):
    global current_artist, offset
    if current_artist is None:
        current_artist = event.artist
        if isinstance(event.artist, patches.Circle):
            if not event.mouseevent.dblclick:
                x0, y0 = current_artist.center
                x1, y1 = event.mouseevent.xdata, event.mouseevent.ydata
                offset = [(x0 - x1), (y0 - y1)]
        elif isinstance(event.artist, Line2D):
            xdata = event.artist.get_xdata()
            ydata = event.artist.get_ydata()
            x1, y1 = event.mouseevent.xdata, event.mouseevent.ydata
            offset = xdata[0] - x1, ydata[0] - y1


def on_motion(event):
    global current_artist
    if not currently_dragging:
        return
    if current_artist is None:
        return
    if event.xdata is None:
        return
    dx, dy = offset
    if isinstance(current_artist, patches.Circle):
        cx, cy = event.xdata + dx, event.ydata + dy
        current_artist.center = cx, cy
        line_to_update, circle_idx = get_line_and_idx_containing_circle(circle_label=current_artist.get_label())
        xdata = list(line_to_update.get_xdata())
        ydata = list(line_to_update.get_ydata())

        xdata[circle_idx] = cx
        ydata[circle_idx] = cy

        line_to_update.set_data(xdata, ydata)
        update_all_bezier_lines()
    plt.draw()


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("pick_event", on_pick)
fig.canvas.mpl_connect("motion_notify_event", on_motion)

# buttons
create_new = fig.add_axes([0.81, 0.9, 0.2, 0.075])
create_new_bt = Button(create_new, 'Create')
create_new_bt.on_clicked(create_new_curve_definition)

remove_last = fig.add_axes([0.81, 0.7, 0.2, 0.075])
remove_last_bt = Button(remove_last, 'Remove')
remove_last_bt.on_clicked(remove_last_curve_definition)

ax.grid(b=True, which='major', color='k', linestyle='-')
plt.show()
