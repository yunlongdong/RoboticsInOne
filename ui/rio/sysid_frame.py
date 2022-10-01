import wx
from wx import grid
import os.path as osp
import numpy as np

import threading
import sys
sys.path.append('../../')

from core.dynamics.dyn_codegen import dyn_CODEGEN
from .custom_widget import PlotFrame


dir_abs_path = osp.dirname(osp.abspath(__file__))




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

        self.m_grid1 = grid.Grid( self, wx.ID_ANY, wx.DefaultPosition, wx.Size(1,-1), 0 )
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
        self.timer.Start(15)  
        self.id_done = 0
    
    def OnOpen(self, e):
        with wx.FileDialog(self, "Open URDF file", wildcard="txt files (*.txt)|*.txt", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:

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
        if self.id_done:
            self.id_done = 0
            for i in range(self.dof):
                axes = self.plot_frame.panel.Add('tau{}'.format(i+1)).gca()
                axes.plot(self.pred_tau[:, i], 'r', label='pred')
                axes.plot(self.true_tau[:, i], 'b-.', label='true')
                axes.legend()
            # 辨识代价函数变化曲线
            axes = self.plot_frame.panel.Add('J').gca()
            axes.plot(self.J, 'r')
            axes.set_title("loss v.s. time")
            self.plot_frame.Show()


    def run(self):
        self.m_button_start.Disable()
        urdf_abs_path = osp.dirname(self.robot.urdf_file)
        fp = open(osp.join(urdf_abs_path, 'generated_returnA.py'), 'w')
        fp.write(self.codegen.systemID_code)
        fp.close()
        
        sys.path.append(urdf_abs_path)
        from generated_returnA import returnA, RLS

        try:
            dof = self.dof
            q = self.data[:, :dof]
            qd = self.data[:, dof:2*dof]
            qdd = self.data[:, 2*dof:3*dof]
            tau = self.data[:, 3*dof:4*dof]
        except:
            self.m_textCtrl_results.write('No available data...\n')
            return

        self.m_textCtrl_results.write('Starting...\n')
        pred_tau = []
        true_tau = []
        my_rls = RLS()
        for i in range(len(qdd)):
            A = returnA(q[i, :], qd[i, :], qdd[i, :])
            my_rls.add_obs(A, tau[i, :])
            true_tau.append(tau[i, :])

            if i%100 == 1:
                self.m_textCtrl_results.write('>>>{}/{}\n'.format(i, len(qdd)))

        self.m_textCtrl_results.write('Plotting...\n')
        
        for i in range(len(qdd)):
            A = returnA(q[i, :], qd[i, :], qdd[i, :])
            pred_tau.append(np.matmul(A, my_rls.theta))
            
        self.pred_tau = np.asarray(pred_tau)
        self.true_tau = np.asarray(true_tau)

        self.J = my_rls.meanJ_list
        
        self.m_button_start.Enable()
        self.id_done = 1
        
        