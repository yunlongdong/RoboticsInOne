import wx
from wx import html, richtext
import os.path as osp


class MainFrame ( wx.Frame ):

    def __init__( self, parent ):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Robotics In One", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )

        bSizer1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_html_start_doc = html.HtmlWindow( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.html.HW_SCROLLBAR_AUTO )
        bSizer1.Add( self.m_html_start_doc, 2, wx.EXPAND|wx.ALL, 2 )

        self.m_panel_start = wx.Panel( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
        bSizer1.Add( self.m_panel_start, 4, wx.EXPAND |wx.ALL, 5 )


        self.SetSizer( bSizer1 )
        self.Layout()
        self.MenuMain = wx.MenuBar( 0 )
        self.MenuFile = wx.Menu()
        self.MenuFileOpen = wx.Menu()
        self.m_menuItem_open_urdf = wx.MenuItem( self.MenuFileOpen, wx.ID_ANY, u"URDF", wx.EmptyString, wx.ITEM_NORMAL )
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
        icon.CopyFromBitmap(wx.Bitmap("icons/ico.bmp", wx.BITMAP_TYPE_ANY))
        # icon.LoadFile("icons/sm.ico", wx.BITMAP_TYPE_ANY)
        self.SetIcon(icon)

    def __del__( self ):
        pass

if __name__ == "__main__":
    app = wx.App()

    frame = MainFrame(None)
    frame.m_html_start_doc.LoadFile("./doc/start.html")
    frame.Show()

    app.MainLoop()