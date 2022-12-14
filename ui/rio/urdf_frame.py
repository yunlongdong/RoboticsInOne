import moderngl
import numpy as np
import wx
import wx.glcanvas
from wx import html, stc
from copy import copy
import os, shutil
import os.path as osp
import sys
sys.path.append('../../')
from core.urdf_parser.utils import inv_tf

from ..s3d.behaviours import Behaviour
from ..s3d.scenes import Scene
from ..s3d.window.base import BaseWindow
from ..s3d.renderables import Mesh, Lines, Spherecloud

from .sysid_frame import SystemIDFrame
from .dynamics_frame import DynamicsFrame
from .kinematics_frame import KinematicsFrame
from .trajectory_frame import TrajectoryFrame
from .custom_widget import JointController

dir_abs_path = osp.dirname(osp.abspath(__file__))

class Window(BaseWindow):
    _FRAME_STYLE = wx.DEFAULT_FRAME_STYLE & ~(
        wx.RESIZE_BORDER | wx.MAXIMIZE_BOX
    )

    class _Frame(wx.Frame):
        """A simple frame for our wxWidgets app."""
        def __init__(self, window, size, title):
            super(Window._Frame, self).__init__(
                None,
                style=Window._FRAME_STYLE
            )
            self._window = window
            self.SetTitle(title)
            self.SetClientSize(size)
            self.Center()
            self.view = Window._Canvas(self._window, self)
            self.Bind(wx.EVT_CLOSE, self._on_close)

            icon = wx.Icon()
            icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/ico.bmp"), wx.BITMAP_TYPE_ANY))
            # icon.LoadFile("icons/sm.ico", wx.BITMAP_TYPE_ANY)
            self.SetIcon(icon)
            self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )

            bSizer1 = wx.BoxSizer( wx.VERTICAL )
            bSizer2 = wx.BoxSizer( wx.HORIZONTAL )
            bSizer2_1 = wx.BoxSizer( wx.VERTICAL )
            bSizer3 = wx.BoxSizer( wx.HORIZONTAL )

            self.robot = self._window.robot or []
            self.m_checklist_link = wx.CheckListBox( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, list(self.robot.robotlinks.keys()), 0 )
            text = wx.StaticText( self, wx.ID_ANY, 'Links Information', wx.DefaultPosition, wx.DefaultSize, 0 )
            bSizer2_1.Add( text, -1, wx.ALL, 0)
            bSizer2_1.Add( self.m_checklist_link , 1,  wx.EXPAND|wx.ALL, 0)

            joint_names = [ i for i in list(self.robot.robotjoints.keys())]
            self.joint_names = joint_names
            self.joint_inv_prev = {j:0 for j in self.joint_names}
            self.m_checklist_invert_j = wx.CheckListBox( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, joint_names, 0 )
            text1 = wx.StaticText( self, wx.ID_ANY, 'Invert Joint', wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_button_save = wx.Button( self, wx.ID_ANY, 'Save URDF')
            self.m_button_mdh = wx.Button(self, wx.ID_ANY, "Save MDH")
            self.m_button_kine = wx.Button( self, wx.ID_ANY, 'Kinematics')
            self.m_button_dyn = wx.Button( self, wx.ID_ANY, 'Dynamics')
            self.m_button_sysid = wx.Button( self, wx.ID_ANY, 'Identification')
            self.m_button_traj_gen = wx.Button( self, wx.ID_ANY, 'Trajectory')
            # add menu bar
            menuBar = wx.MenuBar()

            menu= wx.Menu()
            self.save_urdf_item = wx.MenuItem(menu, wx.ID_ANY, u"Save URDF", wx.EmptyString)
            self.save_MDH_item = wx.MenuItem(menu, wx.ID_ANY, u"Save MDH", wx.EmptyString)
            menu.Append(self.save_urdf_item)
            menu.Append(self.save_MDH_item)
            menuBar.Append(menu,"Save")

            menu= wx.Menu()
            self.menu_kin = wx.MenuItem(menu, wx.ID_ANY, u"Kinematics", wx.EmptyString)
            self.menu_dyn = wx.MenuItem(menu, wx.ID_ANY, u"Dynamics", wx.EmptyString)
            self.menu_sysid = wx.MenuItem(menu, wx.ID_ANY, u"Identification", wx.EmptyString)
            self.menu_trajgen = wx.MenuItem(menu, wx.ID_ANY, u"Trajectory", wx.EmptyString)
            menu.Append(self.menu_kin)
            menu.Append(self.menu_dyn)
            menu.Append(self.menu_sysid)
            menu.Append(self.menu_trajgen)
            menuBar.Append(menu, "Codegen")
            self.SetMenuBar(menuBar)  # Adding the MenuBar to the Frame content


            bSizer2_1.Add( text1, -1, wx.ALL, 0)
            bSizer2_1.Add( self.m_checklist_invert_j , 1,  wx.EXPAND|wx.ALL, 0)


            
            self.joint_control = JointController(self, joint_names)

            text = wx.StaticText( self, wx.ID_ANY, 'Joints Control', wx.DefaultPosition, wx.DefaultSize, 0 )
            bSizer2_1.Add( text, -1, wx.ALL, 0)
            bSizer2_1.Add (self.joint_control, 5, wx.EXPAND|wx.ALL, 0)

            bSizer2.Add( self.view, 5, wx.EXPAND |wx.ALL, 0 )
            bSizer2.Add( bSizer2_1, -1, wx.EXPAND|wx.ALL, 0 )
            

            self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"Color", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_staticText1.Wrap( -1 )
            bSizer3.Add( self.m_staticText1, 0, wx.ALIGN_CENTER|wx.ALL, 5 )
            self.m_colourPicker2 = wx.ColourPickerCtrl( self, wx.ID_ANY, '#708090', wx.DefaultPosition, wx.DefaultSize, wx.CLRP_DEFAULT_STYLE )
            self.m_colourPicker2.SetColour('#708090')
            bSizer3.Add( self.m_colourPicker2, 0, wx.ALL, 5 )

            self.m_staticText2 = wx.StaticText( self, wx.ID_ANY, u"Alpha", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_staticText2.Wrap( -1 )

            bSizer3.Add( self.m_staticText2, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

            self.m_slider2 = wx.Slider( self, wx.ID_ANY, 20, 0, 100, wx.DefaultPosition, wx.DefaultSize, wx.SL_HORIZONTAL )
            bSizer3.Add( self.m_slider2, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

            self.m_checkBoxCoM = wx.CheckBox( self, wx.ID_ANY, u"CoM", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_checkBoxCoM.SetValue(True)
            bSizer3.Add( self.m_checkBoxCoM, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

            self.m_checkBoxAxis = wx.CheckBox( self, wx.ID_ANY, u"Axis", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_checkBoxAxis.SetValue(True)
            bSizer3.Add( self.m_checkBoxAxis, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

            self.m_checkBoxAxis_MDH = wx.CheckBox( self, wx.ID_ANY, u"MDH Axis", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_checkBoxAxis_MDH.SetValue(False)
            bSizer3.Add( self.m_checkBoxAxis_MDH, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

            self.m_checkBoxAxis_World = wx.CheckBox( self, wx.ID_ANY, u"World", wx.DefaultPosition, wx.DefaultSize, 0 )
            self.m_checkBoxAxis_World.SetValue(True)
            bSizer3.Add( self.m_checkBoxAxis_World, 0, wx.ALIGN_CENTER|wx.ALL, 5 )


            bSizer1.Add( bSizer2, 6, wx.ALL|wx.EXPAND, 0 )
            bSizer1.Add( bSizer3, 0, wx.ALL|wx.EXPAND, 0 )


            self.SetSizer( bSizer1 )
            self.Layout()

            self.Centre( wx.BOTH )

            self.Bind(wx.EVT_COLOURPICKER_CHANGED, self.OnColor, self.m_colourPicker2)
            self.Bind(wx.EVT_SCROLL, self.OnSlider, self.m_slider2)
            self.Bind(wx.EVT_CHECKBOX, self.OnCheckerCoM, self.m_checkBoxCoM)
            self.Bind(wx.EVT_CHECKBOX, self.OnCheckerAxis, self.m_checkBoxAxis)
            self.Bind(wx.EVT_CHECKBOX, self.OnCheckerAxis_MDH, self.m_checkBoxAxis_MDH)
            self.Bind(wx.EVT_CHECKBOX, self.OnCheckerAxis_World, self.m_checkBoxAxis_World)
            self.Bind(wx.EVT_CHECKLISTBOX, self.OnCheckerLink, self.m_checklist_link)
            self.Bind(wx.EVT_CHECKLISTBOX, self.OnCheckerInvJ, self.m_checklist_invert_j)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonSave, self.m_button_save)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonSaveMDH, self.m_button_mdh)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonKin, self.m_button_kine)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonDyn, self.m_button_dyn)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonSysid, self.m_button_sysid)
            # self.Bind(wx.EVT_BUTTON, self.OnButtonTrajGen, self.m_button_traj_gen)
            #  menu event
            self.Bind(wx.EVT_MENU, self.OnButtonSave, self.save_urdf_item)
            self.Bind(wx.EVT_MENU, self.OnButtonSaveMDH, self.save_MDH_item)
            self.Bind(wx.EVT_MENU, self.OnButtonKin, self.menu_kin)
            self.Bind(wx.EVT_MENU, self.OnButtonDyn, self.menu_dyn)
            self.Bind(wx.EVT_MENU, self.OnButtonSysid, self.menu_sysid)
            self.Bind(wx.EVT_MENU, self.OnButtonTrajGen, self.menu_trajgen)

            for i in self.joint_control.joint_controller_sliders:
                self.Bind(wx.EVT_COMMAND_SCROLL_THUMBTRACK, self.OnSliderControl, i)

            self.show_all_link()

        def show_all_link(self):
            for i in range(len(self.robot.robotlinks.keys())):
                self.m_checklist_link.Check(i)
        
        def OnButtonSave(self, e):
            dlg = wx.MessageDialog(self, 'The generated URDF will be saved besides the original URDF file.', caption='Save URDF?', style=wx.YES_NO)
            if dlg.ShowModal() == wx.ID_YES:
                # do something here
                robot = self._window.robot
                robot.export_to_urdf()

        def OnButtonSaveMDH(self, e):
            dlg = wx.MessageDialog(self, 'The generated MDH file will be saved besides the original URDF file.', caption='Save MDH?', style=wx.YES_NO)
            if dlg.ShowModal() == wx.ID_YES:
                # do something here
                robot = self._window.robot
                robot.export_to_MDH()

        def OnButtonKin(self, e):
            frame = KinematicsFrame(self, self.robot)
            frame.Show()
            return
        
        def OnButtonDyn(self, e):
            frame = DynamicsFrame(self, self.robot)
            frame.Show()
            return 
        
        def OnButtonTrajGen(self, e):
            frame = TrajectoryFrame(self, self.robot)
            frame.Show()
            return 

        def OnButtonSysid(self, e):
            frame = SystemIDFrame(self, self.robot)
            frame.Show()
            return

        def OnColor(self, e):
            self.show_all_link()
            color = self.m_colourPicker2.GetColour()[:3]
            color = [ c/255.0 for c in color]
            for render in self._window._scene._renderables:
                if isinstance(render, Mesh):
                    render.colors = color + [self.m_slider2.GetValue()/100.0]
            
            self.view._on_paint(None)
            return 

        def OnSlider(self, e):
            self.show_all_link()
            for render in self._window._scene._renderables:
                if isinstance(render, Mesh):
                    colors = render.colors                 
                    colors[:, -1] = self.m_slider2.GetValue()/100.0 
                    render.colors = colors
            self.view._on_paint(None)
            return 

        def OnSliderControl(self, e):
            q = [ i.GetValue()/20.0 for i in self.joint_control.joint_controller_sliders]
            robot = self._window.robot
            robot.set_joint_angle(q)
            self.updateAllRenders()          
            self.view._on_paint(None)
            return 

        def OnCheckerCoM(self, e):
            if self.m_checkBoxCoM.IsChecked():
                for render in self._window._scene._renderables:
                    if isinstance(render, Spherecloud):
                        render.scale(1000)
            else:
                for render in self._window._scene._renderables:
                    if isinstance(render, Spherecloud):
                        render.scale(0.001) 
            self.view._on_paint(None)
            return

        def OnCheckerAxis(self, e):
            if self.m_checkBoxAxis.IsChecked():
                for render in self._window._scene._renderables:
                    if "MDH_" in render.name or "World_" in render.name:
                        continue
                    elif isinstance(render, Lines):
                        render.scale(1000)
            else:
                for render in self._window._scene._renderables:
                    if "MDH_" in render.name or "World_" in render.name:
                        continue
                    elif isinstance(render, Lines):
                        render.scale(0.001)
            self.view._on_paint(None)
            return

        def OnCheckerAxis_MDH(self, e):
            if self.m_checkBoxAxis_MDH.IsChecked():
                for render in self._window._scene._renderables:
                    if isinstance(render, Lines) and "MDH_" in render.name:
                        render.scale(1000)
            else:
                for render in self._window._scene._renderables:
                    if isinstance(render, Lines) and "MDH_" in render.name:
                        render.scale(0.001)
            self.view._on_paint(None)
            return
        
        def OnCheckerAxis_World(self, e):
            if self.m_checkBoxAxis_World.IsChecked():
                for render in self._window._scene._renderables:
                    if isinstance(render, Lines) and "World_" in render.name:
                        render.scale(1000)
            else:
                for render in self._window._scene._renderables:
                    if isinstance(render, Lines) and "World_" in render.name:
                        render.scale(0.001)
            self.view._on_paint(None)
            return

        def OnCheckerLink(self, e):
            for render in self._window._scene._renderables:
                if isinstance(render, Mesh):
                    name = render.name
                    if not name in self.m_checklist_link.GetCheckedStrings():
                        colors = render.colors                   
                        colors[:, -1] = 0.0
                        render.colors = colors
                    else:
                        colors = render.colors                   
                        colors[:, -1] = self.m_slider2.GetValue()/100.0
                        render.colors = colors
            self.view._on_paint(None)
            return

        def OnCheckerInvJ(self, e):
            robot = self._window.robot
            # reset robot
            robot.set_joint_angle(np.zeros(robot.num_robotjoints))
            # reset slider
            for i in self.joint_control.joint_controller_sliders:
                i.SetValue(0) 

            self.joint_inv_now = {j:0 for j in self.joint_names}

            for inv_j in self.m_checklist_invert_j.GetCheckedStrings():
                self.joint_inv_now[inv_j] = 1
            # print('prev:', self.joint_inv_prev)
            # print('now:', self.joint_inv_now)

            for j_n in self.joint_names:
                if self.joint_inv_now[j_n] != self.joint_inv_prev[j_n]:
                    robot.invert_joint_z(j_n)

            self.joint_inv_prev = copy(self.joint_inv_now)

            self.updateAllRenders()         
            self.view._on_paint(None)
            return
        
        def clean(self):
            urdf_abs_path = osp.dirname(self.robot.urdf_file)
            for root, dirs, files in os.walk(urdf_abs_path):
                for file in files:
                    if file.startswith('generated'):
                        os.remove(osp.join(root, file))
                for dir in dirs:
                    if dir == "__pycache__":
                        shutil.rmtree(osp.join(root, dir))
                

        def _on_close(self, event):
            # If close was called before then close
            if self._window._closing:
                self.clean()
                self.view._on_close(event)
                self.Destroy()

            # otherwise just set the window to closing
            self._window._closing = True
        
        # The followings are utility funtions
        def updateAllRenders(self):
            robot = self._window.robot
            for render in self._window._scene._renderables:
                if isinstance(render, Mesh):
                    robotlink = robot.robotlinks[render.name]
                    abs_tf_visual = robotlink.abs_tf_visual
                    m = np.eye(4)
                    m[:3, :3] = render.R.T
                    m[:3, 3] = render.t
                    m_inv = inv_tf(m)
                    render.affine_transform_no_update(R=m_inv[:3, :3].T, t=m_inv[:3, 3])
                    render.affine_transform(R=abs_tf_visual[:3, :3].T, t=abs_tf_visual[:3, 3])
                elif isinstance(render, Lines):
                    linkname = None
                    abs_tf_link = None
                    if "World_" in render.name:
                        linkname = render.name.replace("World_", "")
                        robotlink = robot.robotlinks[linkname]
                        abs_tf_link = robotlink.abs_tf_link
                    elif "MDH_" in render.name:
                        linkname = render.name.replace("MDH_", "")
                        robotlink = robot.robotlinks[linkname]
                        abs_tf_link = robotlink.abs_tf_link_MDH
                    else:
                        linkname = render.name
                        robotlink = robot.robotlinks[linkname]
                        abs_tf_link = robotlink.abs_tf_link
                    m = np.eye(4)
                    m[:3, :3] = render.R.T
                    m[:3, 3] = render.t
                    m_inv = inv_tf(m)
                    render.affine_transform_no_update(R=m_inv[:3, :3].T, t=m_inv[:3, 3])
                    render.affine_transform(R=abs_tf_link[:3, :3].T, t=abs_tf_link[:3, 3])
                elif isinstance(render, Spherecloud):
                    robotlink = robot.robotlinks[render.name]
                    render.updatePos([robotlink.abs_com])


    class _Canvas(wx.glcanvas.GLCanvas):
        def __init__(self, window, parent):
            super(Window._Canvas, self).__init__(
                parent,
                attribList=[
                    wx.glcanvas.WX_GL_CORE_PROFILE,
                    wx.glcanvas.WX_GL_RGBA,
                    wx.glcanvas.WX_GL_DOUBLEBUFFER,
                    wx.glcanvas.WX_GL_DEPTH_SIZE,
                    24
                ]
            )
            self._window = window
            self._window._get_frame = self._get_frame
            self._context = wx.glcanvas.GLContext(self)
            self._mgl_context = None
            self._ticker = wx.Timer(self)

            self.Bind(wx.EVT_PAINT, self._on_paint)
            self.Bind(wx.EVT_TIMER, self._on_tick, self._ticker)
            self.Bind(wx.EVT_MOUSE_EVENTS, self._on_mouse)
            self.Bind(wx.EVT_KEY_DOWN, self._on_keyboard)
            self.Bind(wx.EVT_KEY_UP, self._on_keyboard)

        def _get_frame(self):
            framebuffer = self._mgl_context.detect_framebuffer()
            return np.frombuffer(
                framebuffer.read(components=4),
                dtype=np.uint8
            ).reshape(*(framebuffer.size + (4,)))[::-1]

        def _on_close(self, event):
            self._ticker.Stop()
            self.Unbind(wx.EVT_TIMER)

        def _on_paint(self, event):
            
            self.SetCurrent(self._context)
            if self._window._scene is None:
                self._mgl_context = moderngl.create_context()
                self._mgl_context.enable(moderngl.BLEND)
                self._mgl_context.blend_func = (
                    # moderngl.SRC_ALPHA,
                    # moderngl.ONE_MINUS_SRC_ALPHA
                    moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA,
                    # moderngl.FUNC_ADD, moderngl.FUNC_ADD
                    # moderngl.ONE, moderngl.ONE
                )
                self._window._scene = Scene(
                    size=(self.Size.width, self.Size.height),
                    background=(0,)*4,
                    ctx=self._mgl_context
                )
                self._ticker.Start(16)

            self._window._draw()
            self.SwapBuffers()

        def _on_tick(self, event):
            if self._window._behave(event):
                self.Refresh()
            if self._window._closing:
                self.GetParent()._on_close(None)
            self._window._mouse.wheel_rotation = 0
            self._window._keyboard.keys_up.clear()

        def _on_mouse(self, event):
            state = wx.GetMouseState()
            self._window._mouse.location = (state.GetX(), state.GetY())
            self._window._mouse.left_pressed = state.LeftIsDown()
            self._window._mouse.middle_pressed = state.MiddleIsDown()
            if abs(event.GetWheelRotation()) > 0:
                self._window._mouse.wheel_rotation += (
                    event.GetWheelRotation()/event.GetWheelDelta()
                )

        def _on_keyboard(self, event):
            down = event.GetEventType() == wx.EVT_KEY_DOWN.typeId
            key = chr(event.GetUnicodeKey())
            alt = event.AltDown() == down
            ctrl = event.ControlDown() == down
            cmd = event.CmdDown() == down
            meta = event.MetaDown() == down
            keys = set(
                [key] if key != "\00" else [] +
                (["<alt>"] if alt else []) +
                (["<ctrl>"] if ctrl else []) +
                (["<cmd>"] if cmd else []) +
                (["<meta>"] if meta else [])
            )

            if down:
                self._window._keyboard.keys_down.update(keys)
            else:
                self._window._keyboard.keys_up.update(
                    keys | self._window._keyboard.keys_down
                )
                self._window._keyboard.keys_down.difference_update(keys)

    def __init__(self, info, robot, size=(512, 512)):
        super(Window, self).__init__(size)
        self.info = info
        self.robot = robot
        self._scene = None
        self._mouse = Behaviour.Mouse(None, None, None, None)
        self._keyboard = Behaviour.Keyboard([], [])
        self._closing = False

    def _behave(self, event):
        # Make the behaviour parameters
        params = Behaviour.Params(
            self,
            self._scene,
            self._get_frame,
            self._mouse,
            self._keyboard,
            self._closing
        )

        # Run the behaviours
        remove = []
        for i, b in enumerate(self._behaviours):
            b.behave(params)
            if params.done:
                remove.append(i)
                params.done = False
            if params.stop_propagation:
                break

        # Remove the ones that asked to be removed
        for i in reversed(remove):
            self._behaviours.pop(i)

        # If we are closing, remove all behaviours
        if self._closing:
            self._behaviours.clear()

        # Return whether we should paint again
        return params.refresh

    def _draw(self):
        self._scene.render()

    def show(self, title="Scene"):
        # app = wx.App(False)
        frame = self._Frame(self, self.size, title)
        frame.Show()
        # app.MainLoop()
