import wx, sys
import numpy as np
from numpy import sin, cos
import os.path as osp
from .custom_widget import MyPlotNotebook
sys.path.append('../../')
from core.kinematics.fk_codegen import fk_CODEGEN

dir_abs_path = osp.dirname(osp.abspath(__file__))

class TrajectoryFrame ( wx.Frame ):

    def __init__( self, parent, robot):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Trajectory Generation", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.robot = robot
        self.joint_nums = robot.num_robotjoints

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )

        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1 = wx.BoxSizer( wx.HORIZONTAL )

        m_choice_traj_spaceChoices = [ u"Joint Space", u"Task Space" ]
        self.m_choice_traj_space = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_choice_traj_spaceChoices, 0 )
        self.m_choice_traj_space.SetSelection( 0 )
        bSizer1_1.Add( self.m_choice_traj_space, 0, wx.ALL, 5 )

        m_choice_dofChoices = [ u"q{}".format(i+1) for i in range(self.joint_nums) ]

        self.joints_traj = [ u"0.2*sin(10*t) + 0.1" ] * self.joint_nums

        self.m_choice_dof = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_choice_dofChoices, 0 )
        self.m_choice_dof.SetSelection( 0 )
        bSizer1_1.Add( self.m_choice_dof, 0, wx.ALL, 5 )

        self.m_textCtrl_expression = wx.TextCtrl( self, wx.ID_ANY, u"0.2*sin(10*t) + 0.1", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_textCtrl_expression, 1, wx.ALL, 5 )

        self.m_button_plot = wx.Button( self, wx.ID_ANY, u"plot", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_plot, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_1, 0, wx.EXPAND, 5 )

        self.plot_panel = MyPlotNotebook(self)
        bSizer1.Add( self.plot_panel, 1, wx.EXPAND |wx.ALL, 5 )


        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

        icon = wx.Icon()
        icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/dyn.bmp"), wx.BITMAP_TYPE_ANY))
        self.SetIcon(icon)

        self.Bind(wx.EVT_BUTTON, self.OnPlot, self.m_button_plot)
        self.Bind(wx.EVT_TEXT, self.OnExpression, self.m_textCtrl_expression)
        self.Bind(wx.EVT_CHOICE, self.OnChoiceDof, self.m_choice_dof)

        self.codegen = fk_CODEGEN(robot)

    def OnChoiceDof(self, e):
        index = self.m_choice_dof.GetCurrentSelection()
        self.m_textCtrl_expression.SetValue(self.joints_traj[index])

    def OnExpression(self, e):
        index = self.m_choice_dof.GetCurrentSelection()
        self.joints_traj[index] = self.m_textCtrl_expression.GetValue()

    def OnPlot(self, e):
        urdf_abs_path = osp.dirname(self.robot.urdf_file)
        fp = open(osp.join(urdf_abs_path, 'generated_fk.py'), 'w')
        code = self.codegen.fk_code.replace('__name__ == "__main__"', 'True')
        fp.write(self.codegen.fk_code)
        fp.close()

        sys.path.append(urdf_abs_path)
        from generated_fk import FK_SYM


        self.plot_panel.Clear()
        print(self.joint_nums)
        for i in range(self.joint_nums):
            axes = self.plot_panel.Add('q{}'.format(i+1)).gca()
            t = np.linspace(0, 10, 1000)
            y = eval(self.joints_traj[i])
            axes.plot(t, y)
