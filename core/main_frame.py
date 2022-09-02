import wx
from wx import html, stc
import numpy as np
import os.path as osp

from .urdf_render import urdf_show

dir_abs_path = osp.dirname(osp.abspath(__file__))

class MainFrame ( wx.Frame ):
    def __init__(self, parent):
        
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Robotics In One", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )

        bSizer1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_html_start_doc = html.HtmlWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.html.HW_SCROLLBAR_AUTO )
        bSizer1.Add( self.m_html_start_doc, 2, wx.EXPAND|wx.ALL, 2 )

        self.m_text_show = stc.StyledTextCtrl( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
        self.m_text_show.SetLexer(stc.STC_LEX_XML)
        bSizer1.Add( self.m_text_show, 4, wx.EXPAND |wx.ALL, 5 )

        # self.m_button1 = wx.Button( self, wx.ID_ANY, u"MyButton", wx.DefaultPosition, wx.DefaultSize, 0 )
        # bSizer1.Add( self.m_button1, 0, wx.ALL, 5 )

        self.SetSizer( bSizer1 )
        self.Layout()
        self.MenuMain = wx.MenuBar( 0 )
        self.MenuFile = wx.Menu()
        self.MenuFileOpen = wx.Menu()
        self.m_menuItem_open_urdf = wx.MenuItem( self.MenuFileOpen, wx.ID_ANY, u"URDF\tCtrl+U", wx.EmptyString, wx.ITEM_NORMAL )
        self.MenuFileOpen.Append( self.m_menuItem_open_urdf )

        self.m_menuItem_open_MDH = wx.MenuItem( self.MenuFileOpen, wx.ID_ANY, u"Modified D-H", wx.EmptyString, wx.ITEM_NORMAL )
        self.MenuFileOpen.Append( self.m_menuItem_open_MDH )

        self.MenuFile.AppendSubMenu( self.MenuFileOpen, u"Open" )

        self.MenuMain.Append( self.MenuFile, u"File" )

        self.MenuKinematics = wx.Menu()
        self.m_menuItem_kin_FK = wx.MenuItem( self.MenuKinematics, wx.ID_ANY, u"Forward Kinematics", wx.EmptyString, wx.ITEM_NORMAL )
        self.MenuKinematics.Append( self.m_menuItem_kin_FK )

        self.m_menuItem_kin_IK = wx.MenuItem( self.MenuKinematics, wx.ID_ANY, u"Inverse Kinematics", wx.EmptyString, wx.ITEM_NORMAL )
        self.MenuKinematics.Append( self.m_menuItem_kin_IK )

        self.MenuMain.Append( self.MenuKinematics, u"Kinematics" )

        self.MenuDynamics = wx.Menu()
        self.MenuMain.Append( self.MenuDynamics, u"Dynamics" )

        self.MenuVis = wx.Menu()
        self.MenuMain.Append( self.MenuVis, u"Visulization" )

        self.SetMenuBar( self.MenuMain )

        self.m_statusBar = self.CreateStatusBar( 1, wx.STB_SIZEGRIP, wx.ID_ANY )

        self.Centre( wx.BOTH )

        self.load()
 
    def load(self):
        icon = wx.Icon()
        icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/ico.bmp"), wx.BITMAP_TYPE_ANY))
        # icon.LoadFile("icons/sm.ico", wx.BITMAP_TYPE_ANY)
        self.SetIcon(icon)

        # start doc
        self.m_html_start_doc.LoadFile(osp.join(dir_abs_path, "../docs/start.html"))

        # bind event
        self.bind_all()

    def __del__(self):
        pass

    def bind_all(self):
        self.Bind(wx.EVT_MENU, self.OnOpenURDF, self.m_menuItem_open_urdf)

    def OnOpenURDF(self, evt):
        print('Open URDF...')
        with wx.FileDialog(self, "Open URDF file", wildcard="URDF files (*.urdf)|*.urdf",
                       style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

            if fileDialog.ShowModal() == wx.ID_CANCEL:
                return    

            # Proceed loading the file chosen by the user
            pathname = fileDialog.GetPath()
            
            self.m_text_show.ChangeValue(open(pathname, 'r').read())
            self.m_statusBar.SetStatusText(pathname)
            print(pathname)
            self.pop_3d_viewer(pathname)

    def pop_3d_viewer(self, urdf_path):
        urdf_show(urdf_path)
 

