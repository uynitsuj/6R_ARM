'''
serialcomm = serial.Serial('COM6', 9600)
serialcomm.timeout = 1
'''
import tkinter as tk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from Visualize import IKvisualize as ik




class Application(tk.Frame):
    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.pack()
        self.create_widgets()
        self.IKplt()

    def create_widgets(self):
        self.RCtrlInter = tk.Label(self, text="Robot Control Interface")
        self.RCtrlInter.pack()
        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "Hello World\n(click me)"
        self.hi_there.pack(side="top")

        self.quit = tk.Button(self, text="QUIT", fg="red", command=self.master.destroy)
        self.quit.pack(side="bottom")

    def IKplt(self):
        fig = Figure(figsize=(5, 4), dpi=100)

        canvas = FigureCanvasTkAgg(fig, master=window)  # A tk.DrawingArea.
        canvas.draw()

        ax = fig.add_subplot(111, projection="3d")
        WINDOW_SIZE = 500
        ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        ax.set_zlim3d(0, WINDOW_SIZE)

        ax.set_xlabel('x (mm)')
        ax.set_ylabel('y (mm)')
        ax.set_zlabel('z (mm)')

        r11 = 8.59293961e-01
        r12 = -4.61824089e-01
        r13 = 2.19846310e-01
        x = float(-1e+02)
        r21 = 5.93911746e-02
        r22 = -3.36824089e-01
        r23 = -9.39692621e-01
        y = float(-1e+02)
        r31 = 5.08022222e-01
        r32 = 8.20529125e-01
        r33 = -2.62002630e-01
        z = float(1.5e+02)

        robot = ik.SixR(ax, H06=(r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z))
        robot.IK()
        try:
            robot.draw_limbs(tbn=1)
            robot.draw_limbs(tbn=2)
            robot.draw_limbs(tbn=3)
            robot.draw_limbs(tbn=4)
            robot.draw_limbs(tbn=5)
            robot.draw_limbs(tbn=6)
            robot.draw_limbs(tbn=7)
            robot.draw_limbs(tbn=8)
        except IndexError:
            print("Out of config space")

        toolbar = NavigationToolbar2Tk(canvas, window)
        toolbar.update()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)


window = tk.Tk()
app = Application(window)
window.wm_title("Robot Control Interface")
window.mainloop()
'''
class Application(tk.Frame):
    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        self.RCtrlInter = tk.Label(self, text="Robot Control Interface")
        self.RCtrlInter.pack()
        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "Hello World\n(click me)"
        self.hi_there["command"] = self.say_hi
        self.hi_there.pack(side="top")

        self.quit = tk.Button(self, text="QUIT", fg="red", command=self.master.destroy)
        self.quit.pack(side="bottom")

    def say_hi(self):
        print("hi there everyone!")

    def plot(self):
        x = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        v = np.array([16, 16.31925, 17.6394, 16.003, 17.2861, 17.3131, 19.1259, 18.9694, 22.0003, 22.81226])
        p = np.array([16.23697, 17.31653, 17.22094, 17.68631, 17.73641, 18.6368,
                      19.32125, 19.31756, 21.20247, 22.41444, 22.11718, 22.12453])

        fig = np.Figure(figsize=(6, 6))
        a = fig.add_subplot(111)
        a.scatter(v, x, color='red')
        a.plot(p, range(2 + max(x)), color='blue')
        a.invert_yaxis()

        a.set_title("Estimation Grid", fontsize=16)
        a.set_ylabel("Y", fontsize=14)
        a.set_xlabel("X", fontsize=14)

        canvas = FigureCanvasTkAgg(fig, master=self.window)
        canvas.get_tk_widget().pack()
        canvas.draw()


window = tk.Tk()
app = Application(window)
window.mainloop()
'''

'''
while True:
    i = input("input(on/off): ").strip()
    if i == 'done':
        print('finished program')
        break
    serialcomm.write((i+'\n').encode())
    time.sleep(0.2)
    print(serialcomm.readline().decode('ascii'))
serialcomm.close()
'''
