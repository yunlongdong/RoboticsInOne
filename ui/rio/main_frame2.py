import wx
from wx import html, stc
import numpy as np
import os.path as osp
import os



from .urdf_show import urdf_show
from .mdpad import MDFrame

dir_abs_path = osp.dirname(osp.abspath(__file__))


class BmpPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.start_bmp_path = osp.join(dir_abs_path, '../icons/start.png')
        bitmap =wx.Bitmap(self.start_bmp_path)
        self.bitmap_ctrl = wx.StaticBitmap(self, bitmap=bitmap)
        # 布局
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.bitmap_ctrl, 1, wx.EXPAND|wx.ALL, 0)
        self.SetSizer(sizer)

        self.Bind(wx.EVT_SIZE, self.OnSize)

    def OnSize(self, event):
        # 获取面板尺寸
        panel_size = self.GetSize()

        # 调整位图尺寸
        image = wx.Bitmap(self.start_bmp_path).ConvertToImage()
        image.Rescale(panel_size.width, panel_size.height, wx.IMAGE_QUALITY_HIGH)
        self.bitmap = wx.Bitmap(image)
        self.bitmap_ctrl.SetBitmap(self.bitmap)

class MainFrame ( wx.Frame ):
    def __init__(self, parent):
        
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Robotics In One", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize)

        self.bitmap_ctrl = BmpPanel(self)

        hsizer1 = wx.BoxSizer(wx.HORIZONTAL)
        tutorial_btn = wx.Button(self, label="README")
        open_btn = wx.Button(self, label="Open URDF")
        contact_btn = wx.Button(self, label="Contact")

        self.tutorial_btn = tutorial_btn
        self.open_btn = open_btn
        self.contact_btn = contact_btn

        hsizer1.AddStretchSpacer(1)
        hsizer1.Add(tutorial_btn, 0, wx.ALIGN_CENTER, 2)
        hsizer1.AddSpacer(10)
        hsizer1.Add(open_btn, 0, wx.ALIGN_CENTER, 2)
        hsizer1.AddSpacer(10)
        hsizer1.Add(contact_btn, 0, wx.ALIGN_CENTER, 2)
        hsizer1.AddStretchSpacer(1)

        Vsizer = wx.BoxSizer(wx.VERTICAL)
        Vsizer.Add(self.bitmap_ctrl, 1, wx.ALL|wx.EXPAND, 2)
        Vsizer.Add(hsizer1, 0, wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, 2)
        self.SetSizer(Vsizer)


        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )

        self.m_statusBar = self.CreateStatusBar( 1, wx.STB_SIZEGRIP, wx.ID_ANY )

        self.Centre( wx.BOTH )

        self.load()
 
    def load(self):
        icon = wx.Icon()
        icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/ico.bmp"), wx.BITMAP_TYPE_ANY))
        # icon.LoadFile("icons/sm.ico", wx.BITMAP_TYPE_ANY)
        self.SetIcon(icon)
        # bind event
        self.bind_all()

    def __del__(self):
        pass

    def bind_all(self):
        self.Bind(wx.EVT_BUTTON, self.OnOpenURDF, self.open_btn)
        self.Bind(wx.EVT_BUTTON, self.OnTutorial, self.tutorial_btn)
        self.Bind(wx.EVT_BUTTON, self.OnContact, self.contact_btn)
    
    def OnTutorial(self, e):   
        path = osp.join(dir_abs_path, "../../docs/start.md")
        with open(path, encoding='utf-8') as f:
            cont = f.read()
            frame = MDFrame(self, title='README', cont=cont, home=osp.dirname(osp.abspath(path)))
            frame.Show()

    def OnContact(self, e):
        path = osp.join(dir_abs_path, "../../docs/contact_us.md")
        with open(path, encoding='utf-8') as f:
            cont = f.read()
            frame = MDFrame(self, title='Contact Us', cont=cont, home=osp.dirname(osp.abspath(path)))
            frame.Show()

    def OnOpenURDF(self, evt):
        print('Open URDF...')
        print(osp.dirname(osp.abspath(__file__)))
        with wx.FileDialog(self, "Open URDF file", defaultDir=osp.join(dir_abs_path, '../../urdf_examples/kuka iiwa') ,wildcard="URDF files (*.urdf)|*.urdf",
                       style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

            if fileDialog.ShowModal() == wx.ID_CANCEL:
                return    

            # Proceed loading the file chosen by the user
            pathname = fileDialog.GetPath()
            
            
            self.m_statusBar.SetStatusText(pathname)
            print(pathname)
            self.pop_3d_viewer(pathname)

    def pop_3d_viewer(self, urdf_path):
        urdf_show(urdf_path)
 
   