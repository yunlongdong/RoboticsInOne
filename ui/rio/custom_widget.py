import sys, wx
import wx.lib.scrolledpanel
import wx.lib.agw.aui as aui
from wx import stc
import os.path as osp

import matplotlib

matplotlib.use("wxagg")
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wxagg import (
    NavigationToolbar2WxAgg as NavigationToolbar,
)


dir_abs_path = osp.dirname(osp.abspath(__file__))


class JointController(wx.lib.scrolledpanel.ScrolledPanel):
    def __init__(self, parent, joint_names):
        wx.ScrolledWindow.__init__(self, parent)
        # self.SetScrollbars(1, 1, 1, 1)
        self.SetupScrolling(scroll_x=False)
        self.bSizer = wx.BoxSizer(wx.VERTICAL)

        self.joint_sliders = []
        self.joint_texts = []
        self.create_sliders(joint_names)

        self.SetSizer(self.bSizer)
        self.Layout()

    def create_sliders(self, joint_names):
        for j_n in joint_names:
            self.add_slider(j_n)

    def add_slider(self, joint_name):
        """向面板添加一个新的滑动条"""
        bSizer_joint = wx.BoxSizer(wx.HORIZONTAL)
        text = wx.StaticText(
            self, wx.ID_ANY, joint_name, wx.DefaultPosition, wx.DefaultSize, 0
        )
        slider = wx.Slider(
            self,
            wx.ID_ANY,
            0,
            -100,
            100,
            wx.DefaultPosition,
            wx.DefaultSize,
            wx.SL_HORIZONTAL,
        )

        bSizer_joint.Add(text, 0, wx.ALL, 1)
        bSizer_joint.Add(slider, 1, wx.ALL | wx.EXPAND, 1)

        self.joint_sliders.append(slider)
        self.joint_texts.append(text)
        self.bSizer.Add(bSizer_joint, 0, wx.EXPAND | wx.ALL, 2)

        self.Layout()
        self.SetupScrolling(scroll_x=False)

    def update_sliders(self, joint_names):
        """更新滑动条列表以匹配新的关节名称列表"""
        self.clear_sliders()  # 首先清除现有的滑动条
        for j_n in joint_names:
            self.add_slider(j_n)

    def clear_sliders(self):
        """移除所有滑动条"""
        for slider in self.joint_sliders:
            slider.Destroy()
        for text in self.joint_texts:
            text.Destroy()
        self.joint_sliders = []
        self.joint_texts = []
        self.bSizer.Clear(True)  # 清除sizer并删除所有子窗口


class MyPlot(wx.Panel):
    def __init__(self, parent, id=-1, dpi=None, **kwargs):
        wx.Panel.__init__(self, parent, id=id, **kwargs)
        self.figure = Figure(figsize=(2, 2))
        self.canvas = FigureCanvas(self, -1, self.figure)
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas, 1, wx.EXPAND)
        sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(sizer)


class MyPlotNotebook(wx.Panel):
    def __init__(self, parent, id=-1):
        wx.Panel.__init__(self, parent, id=id)
        self.nb = aui.AuiNotebook(self)
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND)
        self.SetSizer(sizer)

    # -----------------------------------------------------------------------
    def Add(self, name="plot"):
        page = MyPlot(self.nb)
        self.nb.AddPage(page, name)
        return page.figure

    def Clear(self):
        if self.nb.GetPageCount():
            print("cur sel:", self.nb.GetSelection())
            for i in range(self.nb.GetPageCount()):
                print("remove", i)
                self.nb.RemovePage(i)
                self.nb.DeletePage(i)
                wx.Sleep(0.02)


class PlotFrame(wx.Frame):
    def __init__(self, parent, size=(300, 300)):
        dw, dh = wx.DisplaySize()
        wx.Frame.__init__(
            self,
            parent,
            -1,
            "visualization",
            size=size,
            pos=(dw // 2 - 150, dh // 2 - 150),
        )
        self.panel = MyPlotNotebook(self)
