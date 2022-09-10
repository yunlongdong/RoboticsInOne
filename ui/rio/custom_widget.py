import sys, wx
import wx.lib.scrolledpanel
from wx import stc

from .code_stc import CodePad

sys.path.append('../../')
from core.kinematics.fk_codegen import fk_CODEGEN
from core.dynamics.dyn_codegen import dyn_CODEGEN

from io import StringIO
from contextlib import redirect_stdout

import ctypes
import inspect
import threading

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)

    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        # raise ValueError("invalid thread id")
        print("stop invalid thread id:", tid)
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        # raise SystemError("PyThreadState_SetAsyncExc failed")
        print("PyThreadState_SetAsyncExc failed")
 
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

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

        self.m_button_run = wx.Button( self, wx.ID_ANY, u"Run", wx.DefaultPosition, wx.DefaultSize, 0 )
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
        self.Bind(wx.EVT_BUTTON, self.OnCheck, self.m_button_kine_check)
        self.Bind(wx.EVT_BUTTON, self.OnCpp, self.m_button_cpp)
        self.Bind(wx.EVT_BUTTON, self.OnRun, self.m_button_run)
        self.code_type = "code"

        
        self.running_state = 0

    
    def OnCode(self, e):
        self.code_type = "code"
        mode = self.m_choice_kinematics.GetCurrentSelection()
        # choose FK
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.fk_code)
        else:
            self.python_codepad.SetValue(self.codegen.jacobian_code)

    def OnCheck(self, e):
        self.code_type = "check"
        mode = self.m_choice_kinematics.GetCurrentSelection()
        # choose FK
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.check_fk_code)
        else:
            self.python_codepad.SetValue(self.codegen.check_jacobian_code)

    def OnCpp(self, e):
        self.python_codepad.SetValue("# Coming soon...")

    def OnRun(self, e):
        self.thread_run = threading.Thread(target=self.run)
        if self.running_state == 0:
            self.thread_run.start()

    def run(self):
        self.m_button_run.Disable()
        self.running_state = 1
        old_str = "__main__"
        new_str = "ui.rio.custom_widget"
        #keep a named handle on the prior stdout 
        old_stdout = sys.stdout 
        #keep a named handle on io.StringIO() buffer 
        new_stdout = StringIO() 
        #Redirect python stdout into the builtin io.StringIO() buffer 
        sys.stdout = new_stdout

        mode = self.m_choice_kinematics.GetCurrentSelection()
        try:
            # choose FK
            if mode == 0:
                if self.code_type == "code":
                    exec(self.codegen.fk_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_fk_code.replace(old_str, new_str), globals())
            # jacobian
            else:
                if self.code_type == "code":
                    exec(self.codegen.jacobian_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_jacobian_code.replace(old_str, new_str), globals())
            result = str(sys.stdout.getvalue().strip())
        except:
            result = "error..."
        sys.stdout = old_stdout
        self.m_textCtrl_results.SetValue(result)
        self.running_state = 0
        self.m_button_run.Enable()

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

        m_choice_dynamicsChoices = [ u"Mass Matrix", u"Inverse Dynamics" ]
        self.m_choice_dynamics = wx.Choice( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, m_choice_dynamicsChoices, 0 )
        self.m_choice_dynamics.SetSelection( 0 )
        bSizer1_1.Add(self.m_choice_dynamics, 0, wx.ALL, 5)

        self.m_button_codegen = wx.Button( self, wx.ID_ANY, u"Code", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add(self.m_button_codegen, 0, wx.ALL, 5)

        self.m_button_kine_check = wx.Button( self, wx.ID_ANY, u"Check", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add(self.m_button_kine_check, 0, wx.ALL, 5)

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

        self.robot = robot
        self.codegen = dyn_CODEGEN(robot)


        self.Bind(wx.EVT_BUTTON, self.OnCode, self.m_button_codegen)
        self.Bind(wx.EVT_BUTTON, self.OnCheck, self.m_button_kine_check)
        self.Bind(wx.EVT_BUTTON, self.OnCpp, self.m_button_cpp)
        self.Bind(wx.EVT_BUTTON, self.OnRun, self.m_button_run)
        self.code_type = "code"

    def OnCode(self, e):
        self.code_type = "code"
        mode = self.m_choice_dynamics.GetCurrentSelection()
        # choose mass matrix
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.M_code)
        # choose idm
        else:
            self.python_codepad.SetValue(self.codegen.idm_code)

    def OnCheck(self, e):
        self.code_type = "check"
        mode = self.m_choice_dynamics.GetCurrentSelection()
        # choose mass matrix
        if mode == 0:
            self.python_codepad.SetValue(self.codegen.check_M_code)
        # choose idm
        else:
            self.python_codepad.SetValue(self.codegen.check_idm_code)

    def OnCpp(self, e):
        self.python_codepad.SetValue("# Coming soon...")

    def OnRun(self, e):
        old_str = "__main__"
        new_str = "ui.rio.custom_widget"
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
            # choose idm
            else:
                if self.code_type == "code":
                    exec(self.codegen.idm_code.replace(old_str, new_str), globals())
                elif self.code_type == "check":
                    exec(self.codegen.check_idm_code.replace(old_str, new_str), globals())
            result = str(sys.stdout.getvalue().strip())
        except:
            result = "error..."
        sys.stdout = old_stdout
        self.m_textCtrl_results.SetValue(result)


if __name__ == "__main__":
    App = wx.App()
    frame = KinematicsFrame(None, size=(512, 512))
    frame.m_fk_stc.SetValue("""import numpy as np""")
    frame.m_jacobian_stc.SetValue("""import numpy as np""")
    frame.Show()
    App.MainLoop()
