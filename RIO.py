import wx
from core import main_frame


if __name__ == "__main__":
    app = wx.App()

    frame = main_frame.MainFrame(None)
    frame.Show()
    app.MainLoop()