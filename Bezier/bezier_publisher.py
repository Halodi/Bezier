import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import TextBox
from matplotlib.lines import Line2D
from bezier import Curve
import numpy as np


class Bezier:
    def __init__(self):
        self.bezier = None
        self.points = np.empty(shape=[0, 2])
        self.bezier_line_object = None
        self.total_time = 0

    def set_total_time(self, time):
        self.total_time = time

    def add_point(self, x, y):
        new = np.array([[x, y]])
        self.points = np.append(self.points, new, axis=0)

    def clear_points(self):
        self.points = np.empty(shape=[0, 2])

    def draw(self):
        print(self.points)
        print(self.points.shape)
        if self.points.shape[0] >= 3:
            b = Curve(self.points)
            to_plot = b.polyline(1.0001, 1.0)
            x = [x for x, y in to_plot]
            y = [y for x, y in to_plot]

            if self.bezier_line_object is None:
                self.bezier_line_object = ax.plot(
                    x, y, alpha=0.5, c="b", lw=2, picker=True
                )
            else:
                self.bezier_line_object[0].set_data(x, y)
            plt.draw()


# ------------------------------------------------
listLabelPoints = []
point_alpha_default = 0.8
mousepress = None
currently_dragging = False
current_artist = None
offset = [0, 0]
n = 0
point_radius = 0.5
line_object = None
bezier_xy = np.array([])
bezier_ever_initialized = False
artist_fixed_control_point_dict = {}

bezier = Bezier()




# ------------------------------------------------
def on_press(event):
    global currently_dragging
    global mousepress
    currently_dragging = True
    if event.button == 3:
        mousepress = "right"
    elif event.button == 1:
        mousepress = "left"


# ------------------------------------------------
def on_release(event):
    global current_artist, currently_dragging, line_object, bezier
    current_artist = None
    currently_dragging = False
    if line_object is not None and len(list(line_object[0].get_xdata())) >= 3:
        bezier.clear_points()
        for i in range(len(list(line_object[0].get_xdata()))):
            bezier.add_point(
                list(line_object[0].get_xdata())[i], list(line_object[0].get_ydata())[i]
            )

        bezier.draw()

# ------------------------------------------------
def on_pick(event):
    global current_artist, offset, n
    global listLabelPoints
    if current_artist is None:
        current_artist = event.artist
        # print("pick ", current_artist)
        if isinstance(event.artist, patches.Circle):
            if event.mouseevent.dblclick:
                if mousepress == "right":
                    # print("double click right")
                    if len(ax.patches) > 2:
                        # print("\ndelete", event.artist.get_label())
                        event.artist.remove()
                        xdata = list(line_object[0].get_xdata())
                        ydata = list(line_object[0].get_ydata())
                        for i in range(0, len(xdata)):
                            if event.artist.get_label() == listLabelPoints[i]:
                                xdata.pop(i)
                                ydata.pop(i)
                                listLabelPoints.pop(i)
                                break
                        # print('--->', listLabelPoints)
                        line_object[0].set_data(xdata, ydata)
                        plt.draw()
                # elif mousepress == "left":
                #     event.artist.set_fill(True)
                #     new_text_box = TextBox(plt.axes([0.9, 0.5, 0.05, 0.075]), 'Time:', initial='0.5')
                #     plt.draw()
            else:
                x0, y0 = current_artist.center
                x1, y1 = event.mouseevent.xdata, event.mouseevent.ydata
                offset = [(x0 - x1), (y0 - y1)]
        elif isinstance(event.artist, Line2D):
            if event.mouseevent.dblclick:
                if mousepress == "left":
                    # print("double click left")
                    n = n + 1
                    x, y = event.mouseevent.xdata, event.mouseevent.ydata
                    newPointLabel = "point" + str(n)
                    point_object = patches.Circle(
                        [x, y],
                        point_radius,
                        color="r",
                        fill=False,
                        lw=2,
                        alpha=point_alpha_default,
                        transform=ax.transData,
                        label=newPointLabel,
                    )
                    point_object.set_picker(5)
                    ax.add_patch(point_object)
                    xdata = list(line_object[0].get_xdata())
                    ydata = list(line_object[0].get_ydata())
                    pointInserted = False
                    for i in range(0, len(xdata) - 1):
                        # print("--> testing inclusion %s in [%s-%s]"
                        #        %(newPointLabel, listLabelPoints[i], listLabelPoints[i+1]))
                        # print('----->', min(xdata[i],xdata[i+1]), '<', x, '<', max(xdata[i],xdata[i+1]))
                        # print('----->', min(ydata[i],ydata[i+1]), '<', y, '<', max(ydata[i],ydata[i+1]))
                        if (
                            x > min(xdata[i], xdata[i + 1])
                            and x < max(xdata[i], xdata[i + 1])
                            and y > min(ydata[i], ydata[i + 1])
                            and y < max(ydata[i], ydata[i + 1])
                        ):
                            xdata.insert(i + 1, x)
                            ydata.insert(i + 1, y)
                            listLabelPoints.insert(i + 1, newPointLabel)
                            pointInserted = True
                            # print("include", newPointLabel)
                            break
                    line_object[0].set_data(xdata, ydata)
                    # print('final', listLabelPoints)
                    plt.draw()
                    if not pointInserted:
                        print("Error: point not inserted")
            else:
                xdata = event.artist.get_xdata()
                ydata = event.artist.get_ydata()
                x1, y1 = event.mouseevent.xdata, event.mouseevent.ydata
                offset = xdata[0] - x1, ydata[0] - y1


# ------------------------------------------------
def on_motion(event):
    global current_artist
    if not currently_dragging:
        return
    if current_artist == None:
        return
    if event.xdata == None:
        return
    dx, dy = offset
    if isinstance(current_artist, patches.Circle):
        cx, cy = event.xdata + dx, event.ydata + dy
        current_artist.center = cx, cy
        # print("moving", current_artist.get_label())
        xdata = list(line_object[0].get_xdata())
        ydata = list(line_object[0].get_ydata())
        for i in range(0, len(xdata)):
            if listLabelPoints[i] == current_artist.get_label():
                xdata[i] = cx
                ydata[i] = cy
                break
        line_object[0].set_data(xdata, ydata)
    elif isinstance(current_artist, Line2D):
        xdata = list(line_object[0].get_xdata())
        ydata = list(line_object[0].get_ydata())
        xdata0 = xdata[0]
        ydata0 = ydata[0]
        for i in range(0, len(xdata)):
            xdata[i] = event.xdata + dx + xdata[i] - xdata0
            ydata[i] = event.ydata + dy + ydata[i] - ydata0
        line_object[0].set_data(xdata, ydata)
        for p in ax.patches:
            pointLabel = p.get_label()
            i = listLabelPoints.index(pointLabel)
            p.center = xdata[i], ydata[i]
    plt.draw()


# ------------------------------------------------
def on_click(event):
    global n, line_object
    if event and event.dblclick:
        if len(listLabelPoints) < 2:
            n = n + 1
            x, y = event.xdata, event.ydata
            newPointLabel = "point" + str(n)
            point_object = patches.Circle(
                [x, y],
                point_radius,
                color="r",
                fill=False,
                lw=2,
                alpha=point_alpha_default,
                transform=ax.transData,
                label=newPointLabel,
            )
            point_object.set_picker(5)
            ax.add_patch(point_object)
            listLabelPoints.append(newPointLabel)
            if len(listLabelPoints) == 2:
                xdata = []
                ydata = []
                for p in ax.patches:
                    cx, cy = p.center
                    xdata.append(cx)
                    ydata.append(cy)
                line_object = ax.plot(xdata, ydata, alpha=0.5, c="r", lw=2, picker=True)
                line_object[0].set_pickradius(5)
            plt.draw()


# ================================================
fig, ax = plt.subplots()

ax.set_title(
    "Double click left button to create draggable point\nDouble click right to remove a point",
    loc="left",
)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_aspect("equal")

axbox = plt.axes([0.9, 0.7, 0.05, 0.075])
text_box = TextBox(axbox, 'T Total:', initial='1')

def submit(text):
    global bezier
    bezier.set_total_time(float(text))

text_box.on_submit(submit)

fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("pick_event", on_pick)
fig.canvas.mpl_connect("motion_notify_event", on_motion)

ax.grid(b=True, which='major', color='k', linestyle='-')
plt.show()
