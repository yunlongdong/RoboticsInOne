import wx
from ui.rio.main_frame import MainFrame


if __name__ == "__main__":
    app = wx.App()

    frame = MainFrame(None)
    frame.Show()
    app.MainLoop()
