import wx
from wx import grid
import wx.lib.agw.aui as aui
import os.path as osp
import numpy as np
import matplotlib
matplotlib.use('wxagg')
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wxagg import NavigationToolbar2WxAgg as NavigationToolbar
import threading
import sys
sys.path.append('../../')
from core.dynamics.dyn_codegen import dyn_CODEGEN


dir_abs_path = osp.dirname(osp.abspath(__file__))

class MyPlot(wx.Panel):
    def __init__(self, parent, id=-1, dpi=None, **kwargs):
        wx.Panel.__init__(self, parent, id=id, **kwargs)
        self.figure = Figure(figsize=(2, 2))
        self.canvas = FigureCanvas(self, -1, self.figure)
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas, 1, wx.EXPAND)
        sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(sizer)


class MyPlotNotebook(wx.Panel):
    def __init__(self, parent, id=-1):
        wx.Panel.__init__(self, parent, id=id)
        self.nb = aui.AuiNotebook(self)
        sizer = wx.BoxSizer()
        sizer.Add(self.nb, 1, wx.EXPAND)
        self.SetSizer(sizer)

    #-----------------------------------------------------------------------
    def Add(self, name="plot"):    
        page = MyPlot(self.nb)
        self.nb.AddPage(page, name)
        return page.figure

class PlotFrame(wx.Frame):
    def __init__(self, parent, size=(300, 300)):
        dw, dh = wx.DisplaySize()
        wx.Frame.__init__(self, parent, -1, 'visualization', size=size, pos = (dw//2-150, dh//2-150))
        self.panel = MyPlotNotebook(self)

class SystemIDFrame ( wx.Frame ):

    def __init__( self, parent , robot):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = "System Identification", pos = wx.DefaultPosition, size = wx.Size( 800,600 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
        self.robot = robot
        self.codegen = dyn_CODEGEN(robot)
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

        bSizer1_2 = wx.BoxSizer( wx.HORIZONTAL )

        self.m_grid1 = grid.Grid( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer1_2.Add( self.m_grid1, 5, wx.ALL|wx.EXPAND, 3 )


        self.m_textCtrl_results = wx.TextCtrl( self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_MULTILINE )
        bSizer1_2.Add( self.m_textCtrl_results, 1, wx.ALL|wx.EXPAND, 3 )
        
        bSizer1.Add( bSizer1_2, 1, wx.EXPAND, 5 )
        
        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )
        icon = wx.Icon()
        icon.CopyFromBitmap(wx.Bitmap(osp.join(dir_abs_path, "../icons/sysid.bmp"), wx.BITMAP_TYPE_ANY))
        self.SetIcon(icon)

        self.Bind(wx.EVT_BUTTON, self.OnOpen, self.m_button_open)
        self.Bind(wx.EVT_BUTTON, self.OnPlot, self.m_button_plot)
        self.Bind(wx.EVT_BUTTON, self.OnStart, self.m_button_start)

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.show_sysid)
        self.timer.Start(10)  
        self.id_done = 0
    
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
            axes = plot_frame.panel.Add('Visualization').gca()
            for col in cols:
                axes.plot(self.data[:, col], label=self.m_grid1.GetColLabelValue(col))
            axes.legend() 
            plot_frame.Show()
    
    def OnStart(self, e):
        self.thread_run = threading.Thread(target=self.run) 
        self.thread_run.start()
        self.plot_frame = PlotFrame(self, size=(400,300))

    def show_sysid(self, e):
        # for i in range(6):
        #     axes = self.plot_frame.panel.Add('tau{}'.format(i+1)).gca()
        #     axes.plot(self.pred_tau[:, i], 'r', label='pred')
        #     axes.plot(self.true_tau[:, i], 'b-.', label='true')
        #     axes.legend()
        # axes = self.plot_frame.panel.Add('Visualization').gca()
        # axes.plot(self.data[:, 0])
        # self.plot_frame.Show()
        if self.id_done:
            self.id_done = 0
            for i in range(self.dof):
                axes = self.plot_frame.panel.Add('tau{}'.format(i+1)).gca()
                axes.plot(self.pred_tau[:, i], 'r', label='pred')
                axes.plot(self.true_tau[:, i], 'b-.', label='true')
                axes.legend()
            self.plot_frame.Show()


    def run(self):
        self.m_button_start.Disable()
        fp = open(osp.join(dir_abs_path, 'returnA.py'), 'w')
        fp.write(self.codegen.systemID_code)
        fp.close()
        
        from .returnA import returnA

        dof = self.dof
        q = self.data[:, :dof]
        qd = self.data[:, dof:2*dof]
        qdd = self.data[:, 2*dof:3*dof]
        tau = self.data[:, 3*dof:4*dof]

        A_set = []
        tau_set = []

        self.m_textCtrl_results.write('Start...\n')
        for i in range(len(qdd)):      
            A = returnA(q[i, :], qd[i, :], qdd[i, :])

            # joint friction
            # friction = np.ones((A.shape[0], 1))
            # damping = qd[0, :][:, None]
            # A = np.concatenate([A, friction, damping], axis=-1)
            A_set.append(A)  
            tau_set.append(tau[i, :])

            if i%100 == 1:
                self.m_textCtrl_results.write('>>>{}/{}\n'.format(i, len(qdd)))

        A_serial = np.concatenate(A_set, axis=0)
        tau_serial = np.concatenate(tau_set)
        theta = np.linalg.pinv(A_serial).dot(tau_serial)
        pred_tau = []
        true_tau = []
        for i in range(len(qdd)):
            # A = my_systemID.returnA(j_pos[i, :], j_vel[i, :], j_acc[i, :])
            A = A_set[i]
            pred_tau.append(np.matmul(A, theta))
            true_tau.append(tau[i, :])

        self.pred_tau = np.asarray(pred_tau)
        self.true_tau = np.asarray(true_tau)

        
        self.m_button_start.Enable()
        self.id_done = 1
        
        