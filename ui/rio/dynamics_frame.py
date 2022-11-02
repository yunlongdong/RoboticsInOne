import sys, wx
import wx.lib.scrolledpanel
from wx import stc
import os.path as osp
from .code_stc import CodePad

sys.path.append('../../')
from core.dynamics.dyn_codegen import dyn_CODEGEN

from io import StringIO
from contextlib import redirect_stdout
import threading

dir_abs_path = osp.dirname(osp.abspath(__file__))

class DynamicsFrame(wx.Frame):
    def __init__(self, parent, robot):
        wx.Frame.__init__(self, parent, id = wx.ID_ANY, title = u"Dynamics Toolbox", pos = wx.DefaultPosition, size = wx.Size(800, 600), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL)

        self.SetSizeHints(wx.DefaultSize, wx.DefaultSize)
        self.SetBackgroundColour(wx.Colour(255, 255, 255))

        bSizer1 = wx.BoxSizer(wx.VERTICAL)

        bSizer1_1 = wx.BoxSizer(wx.HORIZONTAL)

        self.m_staticText3 = wx.StaticText( self, wx.ID_ANY, u"Dynamics", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText3.Wrap( -1 )

        bSizer1_1.Add( self.m_staticText3, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        m_choice_dynamicsChoices = [u"Mass Matrix", u"Inverse Dynamics", u"Base Inertia Parameters", u"Inv Dynamics with Base Params"]
        self.m_choice_dynamics = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_choice_dynamicsChoices, 0 )
        self.m_choice_dynamics.SetSelection( 0 )
        bSizer1_1.Add(self.m_choice_dynamics, 0, wx.ALL, 5)

        self.m_button_codegen = wx.Button( self, wx.ID_ANY, u"Code", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add(self.m_button_codegen, 0, wx.ALL, 5)

        self.m_button_check = wx.Button( self, wx.ID_ANY, u"Check", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add(self.m_button_check, 0, wx.ALL, 5)

        self.m_button_cpp = wx.Button( self, wx.ID_ANY, u"C++", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_cpp, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_1, 0, wx.EXPAND, 5 )

        self.python_codepad = CodePad(self)
        bSizer1.Add( self.python_codepad, 1, wx.EXPAND |wx.ALL, 5 )

        bSizer1_2 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_button_run = wx.Button( self, wx.ID_ANY, u"Run", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2.Add( self.m_button_run, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_2, 0, wx.EXPAND, 5 )

        self.m_textCtrl_results = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE )
        bSizer1.Add( self.m_textCtrl_results, 1, wx.ALL|wx.EXPAND, 3 )


        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

        icon = wx.Icon()
        icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/dyn.bmp"), wx.BITMAP_TYPE_ANY))
        self.SetIcon(icon)

        self.robot = robot
        self.codegen = dyn_CODEGEN(robot)


        self.Bind(wx.EVT_BUTTON, self.OnCode, self.m_button_codegen)
        self.Bind(wx.EVT_BUTTON, self.OnCheck, self.m_button_check)
        self.Bind(wx.EVT_BUTTON, self.OnCpp, self.m_button_cpp)
        self.Bind(wx.EVT_BUTTON, self.OnRun, self.m_button_run)
        self.code_type = "code"

        self.running_state = 0
        self.result = ""

    def OnCode(self, e):
        self.code_type = "code"
        mode = self.m_choice_dynamics.GetCurrentSelection()
        # choose mass matrix
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.M_code)
        # choose idm
        elif mode == 1:
            self.python_codepad.SetValue(self.codegen.idm_code)
        # choose systemID
        elif mode == 2:
            self.python_codepad.SetValue(self.codegen.systemID_code)
        elif mode == 3:
            self.python_codepad.SetValue(self.codegen.invdyn_baseparams_code)

    def OnCheck(self, e):
        self.code_type = "check"
        mode = self.m_choice_dynamics.GetCurrentSelection()
        # choose mass matrix
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.check_M_code)
        # choose idm
        elif mode == 1:
            self.python_codepad.SetValue(self.codegen.check_idm_code)
        # choose systemID
        elif mode == 2:
            self.python_codepad.SetValue(self.codegen.check_systemID_code)
        elif mode == 3:
            self.python_codepad.SetValue(self.codegen.check_invdyn_baseparams_code)

    def OnCpp(self, e):
        self.code_type = "cpp"
        self.python_codepad.SetValue("# Coming soon...")

    def OnRun(self, e):
        self.thread_run = threading.Thread(target=self.run)
        if self.running_state == 0:
            self.m_button_run.Disable()
            self.thread_run.start()
    
    def run(self):
        self.running_state = 1
        old_str = "__main__"
        new_str = "ui.rio.dynamics_frame"
        #keep a named handle on the prior stdout 
        old_stdout = sys.stdout 
        #keep a named handle on io.StringIO() buffer 
        new_stdout = StringIO() 
        #Redirect python stdout into the builtin io.StringIO() buffer 
        sys.stdout = new_stdout

        mode = self.m_choice_dynamics.GetCurrentSelection()
        try:
            # choose mass matrix
            if mode == 0:
                if self.code_type == "code":
                    exec(self.codegen.M_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_M_code.replace(old_str, new_str), globals())
                elif self.code_type == "cpp":
                    exec("print('Running cpp code is not supported now...')")
            # choose idm
            elif mode == 1:
                if self.code_type == "code":
                    exec(self.codegen.idm_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_idm_code.replace(old_str, new_str), globals())
                elif self.code_type == "cpp":
                    exec("print('Running cpp code is not supported now...')")
            elif mode == 2:
                if self.code_type == "code":
                    exec(self.codegen.systemID_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_systemID_code.replace(old_str, new_str), globals())
                elif self.code_type == "cpp":
                    exec("print('Running cpp code is not supported now...')")
            elif mode == 3:
                if self.code_type == "code":
                    exec(self.codegen.invdyn_baseparams_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_invdyn_baseparams_code.replace(old_str, new_str), globals())
                elif self.code_type == "cpp":
                    exec("print('Running cpp code is not supported now...')")
            self.result = str(sys.stdout.getvalue().strip())
        except:
            self.result = "error..."
        sys.stdout = old_stdout
        self.running_state = 0
        wx.CallAfter(self.afterRun)
    
    def afterRun(self):
        self.m_textCtrl_results.SetValue(self.result)
        self.m_button_run.Enable()