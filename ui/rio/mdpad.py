import wx, os, time, wx.html2 as webview
import os.path as osp
import wx.lib.agw.aui as aui
from markdown import markdown
from .mdutil import md2html
import sys
import os.path as osp



dir_abs_path = osp.dirname(osp.abspath(__file__))

sys.path.append(osp.join(dir_abs_path, '../../'))


class MDPad(wx.Panel):
    def __init__(self, parent, cont='', url='', home='.', title='markdown pad'):
        wx.Panel.__init__(self, parent)
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.wv = webview.WebView.New(self)
        self.Bind(webview.EVT_WEBVIEW_TITLE_CHANGED, self.OnWebViewTitleChanged, self.wv)
        sizer.Add(self.wv, 1, wx.EXPAND)
        self.SetSizer(sizer)
        self.num = 0
        self.home = home
        if url != '': self.load_url(url)
        elif cont!='': self.set_cont(cont)
        self.title = title

    def set_cont(self, value):
        value = value.replace('](./', '](%s/'%self.home)
        return self.wv.SetPage(md2html(value), '')
        # I do not know why use SetPage the js would not run, So I write a file here
        here = osp.split(osp.abspath(__file__))
        for n in range(1,10):
            path = osp.join(here[0], 'index%s.htm'%n)
            if not osp.exists(path):break
        self.num = n
        with open(path, 'w') as f:
            f.write(md2html(value))
        self.load_url(path)

    def load_url(self, url):
        self.wv.LoadURL(url)

    def OnWebViewTitleChanged(self, evt):
        if evt.GetString() == 'about:blank': return
        if evt.GetString() == 'http:///': return
        here = osp.split(osp.abspath(__file__))
        path = osp.join(here[0], 'index%s.htm'%self.num)
        if osp.exists(path): os.remove(path)

class MDFrame(wx.Frame):
    def __init__(self, parent, title='报告结果', cont='', url='', home='.'):
        dw, dh = wx.DisplaySize()
        wx.Frame.__init__ (self, parent, id = wx.ID_ANY, title = title, size = wx.Size(600,800), pos = (dw//2-300, 30))
        cont = '\n'.join([i.strip() for i in cont.split('\n')])
        self.mdpad = MDPad(self, cont, url, home=home, title=title)
        self.set_cont, self.load_url = self.mdpad.set_cont, self.mdpad.load_url
        icon = wx.Icon()
        icon.CopyFromBitmap(wx.ArtProvider.GetBitmap(wx.ART_HELP_BOOK, wx.ART_OTHER, (16, 16)))
        self.SetIcon(icon)

    

class MDNoteBook(wx.lib.agw.aui.AuiNotebook):
    def __init__(self, parent):
        wx.lib.agw.aui.AuiNotebook.__init__( self, parent, wx.ID_ANY, 
            wx.DefaultPosition, wx.DefaultSize, wx.lib.agw.aui.AUI_NB_DEFAULT_STYLE )
        self.Bind( wx.lib.agw.aui.EVT_AUINOTEBOOK_PAGE_CHANGED, self.on_valid) 
        self.Bind( wx.lib.agw.aui.EVT_AUINOTEBOOK_PAGE_CLOSE, self.on_close)
        self.SetArtProvider(aui.AuiSimpleTabArt())
        self.Bind( wx.EVT_IDLE, self.on_idle)

    def on_idle(self, event):
        for i in range(self.GetPageCount()):
            title = self.GetPage(i).title
            if self.GetPageText(i) != title:
                self.SetPageText(i, title)

    def page(self, i=None):
        if not i is None: return self.GetPage(i)
        else: return self.GetCurrentPage()
        


    def add_page(self, mdpanel=None):
        if mdpanel is None: mdpanel = MDPad(self)
        self.AddPage(mdpanel, 'markdown', True, wx.NullBitmap )
        return mdpanel

    def set_title(self, panel, title):
        self.SetPageText(self.GetPageIndex(panel), title)

    def on_valid(self, event): pass

    def on_close(self, event): pass

class MDNoteFrame(wx.Frame):
    def __init__(self, parent, title='MarkDownNoteFrame'):
        wx.Frame.__init__ ( self, parent, id = wx.ID_ANY,
                            title = title,
                            pos = wx.DefaultPosition,
                            size = wx.Size( 500,500),
                            style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.notebook = MDNoteBook(self)
        self.page = self.notebook.page
        sizer.Add( self.notebook, 1, wx.EXPAND |wx.ALL, 0 )
        self.SetSizer(sizer)
        self.add_page = self.notebook.add_page
        self.Layout()
        
if __name__ == '__main__':
    app = wx.App()
    frame = wx.Frame(None)

    with open('./test.md', encoding='utf-8') as f:
        cont = f.read()
    mpd = MDPad(frame, home=osp.dirname(osp.abspath(__file__)))
    mpd.set_cont(cont)
    frame.Show()
    app.MainLoop()

    '''
    app = wx.App()
    mnf = MDNoteFrame(None)
    mdpanel1 = mnf.add_page('markdown1')
    mdpanel1.set_cont('abc')
    mdpanel2 = mnf.add_page('markdown2')
    mdpanel2.set_cont('def')
    mnf.Show()
    app.MainLoop()
    '''