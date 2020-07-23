'''
serialcomm = serial.Serial('COM6', 9600)
serialcomm.timeout = 1
'''
import tkinter as tk
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from Visualize import IKvisualize as ik
from Visualize import FKvisualize as fk


def RXT(theta):
    rx = [[1, 0, 0],
          [0, np.cos(theta), -(np.sin(theta))],
          [0, np.sin(theta), np.cos(theta)]]
    return rx


def RYT(theta):
    ry = [[np.cos(theta), 0, np.sin(theta)],
          [0, 1, 0],
          [-(np.sin(theta)), 0, np.cos(theta)]]
    return ry


def RZT(theta):
    rz = [[np.cos(theta), -(np.sin(theta)), 0],
          [np.sin(theta), np.cos(theta), 0],
          [0, 0, 1]]
    return rz


# noinspection PyAttributeOutsideInit
class Application(tk.Frame):
    def __init__(self, window):
        super().__init__(window)
        self.fig = Figure(figsize=(6, 5), dpi=150)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.window = window
        self.pack(side="top")
        self.create_widgets()
        self.IKplt()

    def PreviousArmU(self, T1, T2, T3, T4, T5, T6):
        self.A1 = T1
        self.A2 = T2
        self.A3 = T3
        self.A4 = T4
        self.A5 = T5
        self.A6 = T6

    def PreviousArmD(self, ArmNum):
        if ArmNum == 1:
            return self.A1
        elif ArmNum == 2:
            return self.A2
        elif ArmNum == 3:
            return self.A3
        elif ArmNum == 4:
            return self.A4
        elif ArmNum == 5:
            return self.A5
        elif ArmNum == 6:
            return self.A6

    def IKupdate(self, r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z):
        self.ax.clear()
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')

        self.robot = ik.SixR(self.ax, H06=(r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z))
        self.robot.IK()
        try:
            self.robot.draw_limbs(tbn=1)
            self.robot.draw_limbs(tbn=2)
            self.robot.draw_limbs(tbn=3)
            self.robot.draw_limbs(tbn=4)
            self.robot.draw_limbs(tbn=5)
            self.robot.draw_limbs(tbn=6)
            self.robot.draw_limbs(tbn=7)
            self.robot.draw_limbs(tbn=8)
        except IndexError:
            print("Out of config space")
        self.toolbar.update()
        self.canvas.draw()

    def FKupdate(self, T1, T2, T3, T4, T5, T6):
        self.ax.clear()
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')
        robot = fk.SixR(self.ax, theta=(T1, T2, T3, T4, T5, T6))

        robot.draw_limbs()
        self.toolbar.update()
        self.canvas.draw()


    def create_widgets(self):
        frame1 = tk.Frame(master=window, height=50, borderwidth=5)
        frame1.pack(side="top", fill="x")

        frame2 = tk.Frame(master=window, height=50, borderwidth=5)
        frame2.pack(side="top", fill="x")

        frame3 = tk.Frame(master=window, height=50, borderwidth=5)
        frame3.pack(side="top", fill="x")

        frame4 = tk.Frame(master=window, height=50, borderwidth=5)
        frame4.pack(side="top", fill="x")

        frame5 = tk.Frame(master=window, height=50, borderwidth=5)
        frame5.pack(side="top", fill="x")

        frame6 = tk.Frame(master=window, height=50, borderwidth=5)
        frame6.pack(side="top", fill="x")

        frame7 = tk.Frame(master=window, height=50, borderwidth=5)
        frame7.pack(side="top", fill="x")

        frame8 = tk.Frame(master=window, height=50, borderwidth=5)
        frame8.pack(side="top", fill="x")

        RCtrlInter = tk.Label(self, text="Robot Control Interface")
        RCtrlInter.pack(side="top")

        Update = tk.Button(master=frame7, command=self.FKassign)
        Update["text"] = "Update FK"
        Update.pack(side="left")

        Update2 = tk.Button(master=frame7, command=self.IKassign)
        Update2["text"] = "Update IK"
        Update2.pack(side="right")

        Update3 = tk.Button(master=frame8)
        Update3["text"] = "Push FK"
        Update3.pack(side="left")

        blue = tk.Button(master=frame8, bg="blue", fg="white", command=self.blue)
        blue["text"] = "blue"
        blue.pack(side="right")

        orange = tk.Button(master=frame8, bg="orange", fg="white", command=self.orange)
        orange["text"] = "orange"
        orange.pack(side="right")

        green = tk.Button(master=frame8, bg="green", fg="white", command=self.green)
        green["text"] = "green"
        green.pack(side="right")

        red = tk.Button(master=frame8, bg="red", fg="white", command=self.red)
        red["text"] = "red"
        red.pack(side="right")

        purple = tk.Button(master=frame8, bg="purple", fg="white", command=self.purple)
        purple["text"] = "purple"
        purple.pack(side="right")

        brown = tk.Button(master=frame8, bg="brown", fg="white", command=self.brown)
        brown["text"] = "brown"
        brown.pack(side="right")

        pink = tk.Button(master=frame8, bg="pink", fg="white", command=self.pink)
        pink["text"] = "pink"
        pink.pack(side="right")

        grey = tk.Button(master=frame8, bg="grey", fg="white", command=self.grey)
        grey["text"] = "grey"
        grey.pack(side="right")

        posetransfer = tk.Label(master=frame8)
        posetransfer["text"] = "Transfer Pose"
        posetransfer.pack(side="right")

        T1L = tk.Label(master=frame1, text="Theta 1", height=1, width=10)
        T1L.pack(side="left")
        self.T1 = tk.Text(master=frame1, height=1, width=20)
        self.T1.insert(tk.END, "0")
        self.T1.pack(side="left")

        XL = tk.Label(master=frame1, text="X(mm)", height=1, width=10)
        XL.pack(side="right")
        self.X = tk.Text(master=frame1, height=1, width=20)
        self.X.insert(tk.END, "-100")
        self.X.pack(side="right")

        T2L = tk.Label(master=frame2, text="Theta 2", height=1, width=10)
        T2L.pack(side="left")
        self.T2 = tk.Text(master=frame2, height=1, width=20)
        self.T2.insert(tk.END, "0")
        self.T2.pack(side="left")

        YL = tk.Label(master=frame2, text="Y(mm)", height=1, width=10)
        YL.pack(side="right")
        self.Y = tk.Text(master=frame2, height=1, width=20)
        self.Y.insert(tk.END, "-100")
        self.Y.pack(side="right")

        T3L = tk.Label(master=frame3, text="Theta 3", height=1, width=10)
        T3L.pack(side="left")
        self.T3 = tk.Text(master=frame3, height=1, width=20)
        self.T3.insert(tk.END, "0")
        self.T3.pack(side="left")

        ZL = tk.Label(master=frame3, text="Z(mm)", height=1, width=10)
        ZL.pack(side="right")
        self.Z = tk.Text(master=frame3, height=1, width=20)
        self.Z.insert(tk.END, "100")
        self.Z.pack(side="right")

        T4L = tk.Label(master=frame4, text="Theta 4", height=1, width=10)
        T4L.pack(side="left")
        self.T4 = tk.Text(master=frame4, height=1, width=20)
        self.T4.insert(tk.END, "0")
        self.T4.pack(side="left")

        R1L = tk.Label(master=frame4, text="R1(theta)", height=1, width=10)
        R1L.pack(side="right")
        self.R1 = tk.Text(master=frame4, height=1, width=20)
        self.R1.insert(tk.END, "90")
        self.R1.pack(side="right")

        T5L = tk.Label(master=frame5, text="Theta 5", height=1, width=10)
        T5L.pack(side="left")
        self.T5 = tk.Text(master=frame5, height=1, width=20)
        self.T5.insert(tk.END, "0")
        self.T5.pack(side="left")

        R2L = tk.Label(master=frame5, text="R2(theta)", height=1, width=10)
        R2L.pack(side="right")
        self.R2 = tk.Text(master=frame5, height=1, width=20)
        self.R2.insert(tk.END, "0")
        self.R2.pack(side="right")

        T6L = tk.Label(master=frame6, text="Theta 6", height=1, width=10)
        T6L.pack(side="left")
        self.T6 = tk.Text(master=frame6, height=1, width=20)
        self.T6.insert(tk.END, "0")
        self.T6.pack(side="left")

        R3L = tk.Label(master=frame6, text="R3(theta)", height=1, width=10)
        R3L.pack(side="right")
        self.R3 = tk.Text(master=frame6, height=1, width=20)
        self.R3.insert(tk.END, "90")
        self.R3.pack(side="right")

    def FKinsert(self, a1, a2, a3, a4, a5, a6):
        self.T1.delete("1.0", tk.END)
        self.T2.delete("1.0", tk.END)
        self.T3.delete("1.0", tk.END)
        self.T4.delete("1.0", tk.END)
        self.T5.delete("1.0", tk.END)
        self.T6.delete("1.0", tk.END)
        self.T1.insert(tk.END, str(a1 * 180 / np.pi))
        self.T2.insert(tk.END, str(a2 * 180 / np.pi))
        self.T3.insert(tk.END, str(a3 * 180 / np.pi))
        self.T4.insert(tk.END, str(a4 * 180 / np.pi))
        self.T5.insert(tk.END, str(a5 * 180 / np.pi))
        self.T6.insert(tk.END, str(a6 * 180 / np.pi))
        self.FKupdate(a1 * 180 / np.pi, a2 * 180 / np.pi, a3 * 180 / np.pi, a4 * 180 / np.pi, a5 * 180 / np.pi, a6 * 180 / np.pi)

    def blue(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 0), self.robot.rtnposeang(3, 0),
                      self.robot.rtnposeang(4, 0), self.robot.rtnposeang(5, 0), self.robot.rtnposeang(6, 0))

    def orange(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 1), self.robot.rtnposeang(3, 1),
                      self.robot.rtnposeang(4, 1), self.robot.rtnposeang(5, 0), self.robot.rtnposeang(6, 0))

    def green(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 2), self.robot.rtnposeang(3, 2),
                      self.robot.rtnposeang(4, 2), self.robot.rtnposeang(5, 1), self.robot.rtnposeang(6, 1))

    def red(self):
        self.FKinsert(self.robot.rtnposeang(1, 0), self.robot.rtnposeang(2, 3), self.robot.rtnposeang(3, 3),
                      self.robot.rtnposeang(4, 3), self.robot.rtnposeang(5, 1), self.robot.rtnposeang(6, 1))

    def purple(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 4), self.robot.rtnposeang(3, 4),
                      self.robot.rtnposeang(4, 4), self.robot.rtnposeang(5, 2), self.robot.rtnposeang(6, 2))

    def brown(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 5), self.robot.rtnposeang(3, 5),
                      self.robot.rtnposeang(4, 5), self.robot.rtnposeang(5, 2), self.robot.rtnposeang(6, 2))

    def pink(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 6), self.robot.rtnposeang(3, 6),
                      self.robot.rtnposeang(4, 6), self.robot.rtnposeang(5, 3), self.robot.rtnposeang(6, 3))

    def grey(self):
        self.FKinsert(self.robot.rtnposeang(1, 1), self.robot.rtnposeang(2, 7), self.robot.rtnposeang(3, 7),
                      self.robot.rtnposeang(4, 7), self.robot.rtnposeang(5, 3), self.robot.rtnposeang(6, 3))

    def IKassign(self):
        rotation_TbTmatrix = np.dot(RXT(int(self.R1.get("1.0", tk.END)) * np.pi / 180),
                                    np.dot(RYT(int(self.R2.get("1.0", tk.END)) * np.pi / 180),
                                           RZT(int(self.R3.get("1.0", tk.END)) * np.pi / 180)))

        self.IKupdate(r11=rotation_TbTmatrix[0][0],
                      r12=rotation_TbTmatrix[0][1],
                      r13=rotation_TbTmatrix[0][2],
                      x=int(self.X.get("1.0", tk.END)),
                      r21=rotation_TbTmatrix[1][0],
                      r22=rotation_TbTmatrix[1][1],
                      r23=rotation_TbTmatrix[1][2],
                      y=int(self.Y.get("1.0", tk.END)),
                      r31=rotation_TbTmatrix[2][0],
                      r32=rotation_TbTmatrix[2][1],
                      r33=rotation_TbTmatrix[2][2],
                      z=int(self.Z.get("1.0", tk.END)))

    def FKassign(self):
        self.FKupdate(T1=float(self.T1.get("1.0", tk.END)), T2=float(self.T2.get("1.0", tk.END)),
                      T3=float(self.T3.get("1.0", tk.END)), T4=float(self.T4.get("1.0", tk.END)),
                      T5=float(self.T5.get("1.0", tk.END)), T6=float(self.T6.get("1.0", tk.END)))

    def IKplt(self):
        fig = Figure(figsize=(6, 5), dpi=150)

        self.canvas = FigureCanvasTkAgg(fig, master=window)  # A tk.DrawingArea.
        self.canvas.draw()

        self.ax = fig.add_subplot(111, projection="3d")
        WINDOW_SIZE = 500
        self.ax.set_xlim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_ylim3d(-WINDOW_SIZE / 2, WINDOW_SIZE / 2)
        self.ax.set_zlim3d(0, WINDOW_SIZE)

        self.ax.set_xlabel('x (mm)')
        self.ax.set_ylabel('y (mm)')
        self.ax.set_zlabel('z (mm)')

        r11 = 8.59293961e-01
        r12 = -4.61824089e-01
        r13 = 2.19846310e-01
        x = float(-.9e+02)
        r21 = 5.93911746e-02
        r22 = -3.36824089e-01
        r23 = -9.39692621e-01
        y = float(-1e+02)
        r31 = 5.08022222e-01
        r32 = 8.20529125e-01
        r33 = -2.62002630e-01
        z = float(1.5e+02)

        self.robot = ik.SixR(self.ax, H06=(r11, r12, r13, x, r21, r22, r23, y, r31, r32, r33, z))
        self.robot.IK()
        try:
            self.robot.draw_limbs(tbn=1)
            self.robot.draw_limbs(tbn=2)
            self.robot.draw_limbs(tbn=3)
            self.robot.draw_limbs(tbn=4)
            self.robot.draw_limbs(tbn=5)
            self.robot.draw_limbs(tbn=6)
            self.robot.draw_limbs(tbn=7)
            self.robot.draw_limbs(tbn=8)

        except IndexError:
            print("Out of config space")

        self.toolbar = NavigationToolbar2Tk(self.canvas, window)
        self.toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand="true")


window = tk.Tk()
app = Application(window)
window.wm_title("Robot Control Interface")
window.mainloop()

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
