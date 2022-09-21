import wx
from wx import grid
import os.path as osp
import numpy as np
import matplotlib
matplotlib.use('wxagg')
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wxagg import NavigationToolbar2WxAgg as NavigationToolbar2Wx
import threading

class PlotPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.CreateCtrls()
        self.DoLayout()

    def CreateCtrls(self):
        
        self.figure = Figure()
        self.axes = self.figure.add_subplot(111)
        self.canvas = FigureCanvas(self, -1, self.figure)

        self.toolbar = NavigationToolbar2Wx(self.canvas)
        self.toolbar.Realize()

    def DoLayout(self):
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas, 1, wx.LEFT | wx.TOP | wx.GROW)
        sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(sizer)
        self.Fit()

class PlotFrame(wx.Frame):
    def __init__(self, parent):
        dw, dh = wx.DisplaySize()
        wx.Frame.__init__(self, parent, -1, 'visualization', size=(300, 300), pos = (dw//2-150, dh//2-150))
        self.panel = PlotPanel(self)


class SystemIDFrame ( wx.Frame ):

    def __init__( self, parent , robot):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = "System Identification", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
        self.robot = robot
        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
        self.SetBackgroundColour( wx.Colour( 255, 255, 255 ) )
        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        bSizer1_1 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_button_open = wx.Button( self, wx.ID_ANY, u"Open", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_open, 0, wx.ALL, 5 )

        self.m_button_plot = wx.Button( self, wx.ID_ANY, u"Plot", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_plot, 0, wx.ALL, 5 )

        self.m_button_start = wx.Button( self, wx.ID_ANY, u"Start Identification", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_1.Add( self.m_button_start, 0, wx.ALL, 5 )


        bSizer1.Add( bSizer1_1, 0, wx.EXPAND, 2 )

        self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"*.txt file should contain q1 q2 dq1 dq2 ddq1 ddq2 tau1 tau2 (space separated) in each row.\nChoose the columns and click plot to visualize the curve.", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText1.Wrap( -1 )

        bSizer1.Add( self.m_staticText1, 0, wx.ALL, 5 )

        self.m_textCtrl_results = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE )
        bSizer1.Add( self.m_textCtrl_results, 1, wx.ALL|wx.EXPAND, 3 )
        
        self.m_grid1 = grid.Grid( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1.Add( self.m_grid1, 1, wx.ALL|wx.EXPAND, 5 )

        
        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

        self.Bind(wx.EVT_BUTTON, self.OnOpen, self.m_button_open)
        self.Bind(wx.EVT_BUTTON, self.OnPlot, self.m_button_plot)
        self.Bind(wx.EVT_BUTTON, self.OnStart, self.m_button_start)
    
    def OnOpen(self, e):
        with wx.FileDialog(self, "Open URDF file", wildcard="txt files (*.txt)|*.txt",
                       style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

            if fileDialog.ShowModal() == wx.ID_CANCEL:
                return    

            # Proceed loading the file chosen by the user
            pathname = fileDialog.GetPath()
            data = np.loadtxt(pathname)
            self.data = data
            self.m_grid1.CreateGrid( data.shape[0], data.shape[1] )
            # Grid
            self.m_grid1.EnableEditing( False )
            self.m_grid1.EnableGridLines( True )
            self.m_grid1.EnableDragGridSize( False )
            self.m_grid1.SetMargins( 0, 0 )

            # Columns
            # self.m_grid1.SetColSize( 0, 85 )
            # self.m_grid1.SetColSize( 1, 85 )
            # self.m_grid1.SetColSize( 2, 80 )
            # self.m_grid1.SetColSize( 3, 80 )
            self.m_grid1.EnableDragColMove( False )
            self.m_grid1.EnableDragColSize( True )

            col_labels = ['q', 'dq', 'ddq', 'tau']

            dof = data.shape[1]//4
            self.dof = dof
          
            
            for j in range(4):
                for i in range(dof):
                    self.m_grid1.SetColLabelValue( i + j*dof, u"{}{}".format(col_labels[j] ,i+1) )
            


            self.m_grid1.SetColLabelSize( wx.grid.GRID_AUTOSIZE )
            self.m_grid1.SetColLabelAlignment( wx.ALIGN_CENTER, wx.ALIGN_CENTER )

            # Rows
            self.m_grid1.EnableDragRowSize( True )
            self.m_grid1.SetRowLabelAlignment( wx.ALIGN_CENTER, wx.ALIGN_CENTER )

            # Label Appearance

            # Cell Defaults
            self.m_grid1.SetDefaultCellAlignment( wx.ALIGN_LEFT, wx.ALIGN_TOP )

            for i in range(data.shape[0]):
                for j in range(data.shape[1]):
                    self.m_grid1.SetCellValue(i, j, str(data[i, j]))

    def OnPlot(self, e):
        cols = self.m_grid1.GetSelectedCols()
        if len(cols):
            plot_frame = PlotFrame(self)
            for col in cols:
                plot_frame.panel.axes.plot(self.data[:, col], label=self.m_grid1.GetColLabelValue(col))
            plot_frame.panel.axes.legend() 
            plot_frame.Show()
    
    def OnStart(self, e):
        self.thread_run = threading.Thread(target=self.run)
        
        self.thread_run.start()

    def run(self):
        self.m_button_start.Disable()
        dof = self.dof
        q = self.data[:, :dof]
        qd = self.data[:, dof:2*dof]
        qdd = self.data[:, 2*dof:3*dof]
        tau = self.data[:, 3*dof:4*dof]

        for i in range(100):
            self.m_textCtrl_results.write('{}\n'.format(i))
        self.m_button_start.Enable()