import sys, wx
import wx.lib.scrolledpanel
from wx import stc

from .code_stc import CodePad

sys.path.append('../../')
from core.kinematics.fk_codegen import fk_CODEGEN

class JointController(wx.lib.scrolledpanel.ScrolledPanel):
    def __init__(self, parent, joint_names):
        wx.ScrolledWindow.__init__(self, parent)
        # self.SetScrollbars(1, 1, 1, 1)
        self.SetupScrolling(scroll_x=False)
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



class KinematicsFrame(wx.Frame):
    def __init__(self, parent, robot, id=wx.ID_ANY, title="Kinematics", size=(512, 512)):
        wx.Frame.__init__(self, parent, size=size, title=title)
        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )
        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
        self.robot = robot
        self.codegen = fk_CODEGEN(robot)

        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1_1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"Forward", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText1.Wrap( -1 )

        bSizer1_1_1.Add( self.m_staticText1, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 2 )

        self.m_button_gen_fk = wx.Button( self, wx.ID_ANY, u"Copy", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1_1.Add( self.m_button_gen_fk, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 6 )

        self.m_button3 = wx.Button( self, wx.ID_ANY, u"MyButton", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1_1.Add( self.m_button3, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        bSizer1_1.Add( bSizer1_1_1, 0, wx.EXPAND, 5 )
        bSizer1.Add( bSizer1_1, 1, wx.EXPAND, 5 )

        bSizer1_2 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_2_1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_staticText2 = wx.StaticText( self, wx.ID_ANY, u"Jacobian ", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText2.Wrap( -1 )

        bSizer1_2_1.Add( self.m_staticText2, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 2 )

        self.m_button_gen_jacobian = wx.Button( self, wx.ID_ANY, u"Copy", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2_1.Add( self.m_button_gen_jacobian, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 6 )

        self.m_button31 = wx.Button( self, wx.ID_ANY, u"MyButton", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2_1.Add( self.m_button31, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        bSizer1_2.Add( bSizer1_2_1, 0, wx.EXPAND, 5 )

        self.m_fk_stc = CodePad(self)
        bSizer1_1.Add( self.m_fk_stc, 1, wx.EXPAND |wx.ALL, 5 )
        self.m_ik_stc = CodePad(self)
        bSizer1_2.Add( self.m_ik_stc, 1, wx.EXPAND |wx.ALL, 5 )

        bSizer1.Add( bSizer1_2, 1, wx.EXPAND, 5 )

        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

    def SetIK(self, ik_code):
        self.m_ik_stc.SetValue(self.codegen.jacobian_python_code_gen())

    def SetFK(self, fk_code):
        self.m_fk_stc.SetValue(self.codegen.fk_python_code_gen())

if __name__ == "__main__":
    App = wx.App()
    frame = KinematicsFrame(None, size=(512, 512))
    frame.m_fk_stc.SetValue("""import numpy as np

def fk(q):
    x = forward_kinematics(q)
    return x
""")
    frame.m_ik_stc.SetValue("""import numpy as np

def ik(x):
    q = forward_kinematics(x)
    return q
""")
    frame.Show()
    App.MainLoop()
