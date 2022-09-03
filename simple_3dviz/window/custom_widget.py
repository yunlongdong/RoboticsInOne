import wx

class JointController(wx.Panel):
    def __init__(self, parent, joint_names):
        wx.Panel.__init__(self, parent)

        bSizer = wx.BoxSizer(wx.VERTICAL)

        self.joint_controller_sliders = []
        for j_n in joint_names:

            bSizer_joint = wx.BoxSizer(wx.HORIZONTAL)
            text = wx.StaticText(self, wx.ID_ANY, j_n, wx.DefaultPosition, wx.DefaultSize, 0)   
            slider = wx.Slider(self, wx.ID_ANY, 0, -100, 100, wx.DefaultPosition, wx.DefaultSize, wx.SL_HORIZONTAL) 

            bSizer_joint.Add(text, 0, wx.ALL, 1)
            bSizer_joint.Add(slider, 1, wx.ALL, 1)

            self.joint_controller_sliders.append(slider)
            bSizer.Add( bSizer_joint, 0, wx.EXPAND|wx.ALL, 2 )

        self.SetSizer( bSizer )
        self.Layout()
    


if __name__ == "__main__":
    App = wx.App()
    frame = wx.Frame(None, size=(512, 512))
    jc = JointController(frame, ['j1', 'j2', 'j3'])

    frame.Show()
    App.MainLoop()



