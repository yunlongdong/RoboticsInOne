import sys, wx
import wx.lib.scrolledpanel
from wx import stc

from .code_stc import CodePad

sys.path.append('../../')
from core.kinematics.fk_codegen import fk_CODEGEN
from core.dynamics.dyn_codegen import dyn_CODEGEN

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

    def __init__( self, parent, robot):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Kinematics Toolbox", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )

        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_staticText3 = wx.StaticText( self, wx.ID_ANY, u"Kinematics", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText3.Wrap( -1 )

        bSizer1_1.Add( self.m_staticText3, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        m_choice_kinematicsChoices = [ u"Forward Kinematics", u"Jacobian" ]
        self.m_choice_kinematics = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_choice_kinematicsChoices, 0 )
        self.m_choice_kinematics.SetSelection( 0 )
        bSizer1_1.Add( self.m_choice_kinematics, 0, wx.ALL, 5 )

        self.m_button_codegen = wx.Button( self, wx.ID_ANY, u"Code", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_codegen, 0, wx.ALL, 5 )

        self.m_button_kine_check = wx.Button( self, wx.ID_ANY, u"Check", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_kine_check, 0, wx.ALL, 5 )

        self.m_button_cpp = wx.Button( self, wx.ID_ANY, u"C++", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_cpp, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_1, 0, wx.EXPAND, 5 )

        self.python_codepad = CodePad(self)
        bSizer1.Add( self.python_codepad, 1, wx.EXPAND |wx.ALL, 5 )

        bSizer1_2 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_button_run = wx.Button( self, wx.ID_ANY, u"run", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2.Add( self.m_button_run, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_2, 0, wx.EXPAND, 5 )

        self.m_textCtrl_results = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE )
        bSizer1.Add( self.m_textCtrl_results, 1, wx.ALL|wx.EXPAND, 3 )


        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

        self.robot = robot
        self.codegen = fk_CODEGEN(robot)


        self.Bind(wx.EVT_BUTTON, self.OnCode, self.m_button_codegen)
        self.Bind(wx.EVT_BUTTON, self.OnRun, self.m_button_run)
    
    def OnCode(self, e):
        mode = self.m_choice_kinematics.GetCurrentSelection()
        robot = self.robot
        # choose FK
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.fk_code)
        else:
            self.python_codepad.SetValue(self.codegen.jacobian_code)

    def OnRun(self, e):
        print('run code')
        self.m_textCtrl_results.SetValue('1\n2\n')

class DynamicsFrame(wx.Frame):
    def __init__(self, parent, robot, id=wx.ID_ANY, title="Dynamics", size=(512, 512)):
        wx.Frame.__init__(self, parent, size=size, title=title)
        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )
        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
        self.robot = robot
        self.codegen = dyn_CODEGEN(robot)

        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1_1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"Mass Matrix", wx.DefaultPosition, wx.DefaultSize, 0 )
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

        self.m_staticText2 = wx.StaticText( self, wx.ID_ANY, u"Inverse Dynamics", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText2.Wrap( -1 )

        bSizer1_2_1.Add( self.m_staticText2, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 2 )

        self.m_button_gen_jacobian = wx.Button( self, wx.ID_ANY, u"Copy", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2_1.Add( self.m_button_gen_jacobian, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 6 )

        self.m_button31 = wx.Button( self, wx.ID_ANY, u"MyButton", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2_1.Add( self.m_button31, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        bSizer1_2.Add( bSizer1_2_1, 0, wx.EXPAND, 5 )

        self.m_mass_stc = CodePad(self)
        bSizer1_1.Add( self.m_mass_stc, 1, wx.EXPAND |wx.ALL, 5 )
        self.m_idyn_stc = CodePad(self)
        bSizer1_2.Add( self.m_idyn_stc, 1, wx.EXPAND |wx.ALL, 5 )

        bSizer1.Add( bSizer1_2, 1, wx.EXPAND, 5 )

        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

    def SetMass(self, M_code):
        self.m_mass_stc.SetValue(self.codegen.M_code)

    def SetIdyn(self, dyn_code):
        self.m_idyn_stc.SetValue(self.codegen.idm_code)

if __name__ == "__main__":
    App = wx.App()
    frame = KinematicsFrame(None, size=(512, 512))
    frame.m_fk_stc.SetValue("""import numpy as np""")
    frame.m_jacobian_stc.SetValue("""import numpy as np""")
    frame.Show()
    App.MainLoop()
