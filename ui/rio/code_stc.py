import wx, wx.stc as stc
import json
import keyword

style_config = json.loads(
'''{
    "STYLE_DEFAULT":["#000000", "#ffffff", "Courier New", 10, false, false],
    "STYLE_LINENUMBER":[null, "#C0C0C0", null, 12, null, null],
    "STYLE_CONTROLCHAR":[null, null, null, null, null, null],
    "STYLE_BRACELIGHT":["#0000FF", "#FFFF88", null, null, null, null],
    "STYLE_BRACEBAD":["#FF0000", "#FFFF88", null, null, null, null],
    "P_DEFAULT":[null, null, null, null, null, null],
    "P_COMMENTLINE":["#007F00", null, null, null, null, null],
    "P_NUMBER":[null, null, null, null, null, null],
    "P_STRING":["#7F007F", null, null, null, null, null],
    "P_CHARACTER":["#7F007F", null, null, null, null, null],
    "P_WORD":["#00007F", null, null, null, true, null],
    "P_TRIPLE":["#7F0000", null, null, null, null, null],
    "P_TRIPLEDOUBLE":["#000033", "#FFFFE8", null, null, null, null],
    "P_CLASSNAME":["#0000FF", null, null, null, true, null],
    "P_DEFNAME":["#007F7F", null, null, null, true, null],
    "P_OPERATOR":[null, null, null, null, null, null],
    "P_IDENTIFIER":[null, null, null, null, null, null],
    "P_COMMENTBLOCK":["#7F7F7F", null, null, null, null, null],
    "P_STRINGEOL":["#000000", "#E0C0E0", null, null, null, null]
}''')


class CodePad(stc.StyledTextCtrl):
    """EditWindow based on StyledTextCtrl."""    
    def __init__(self, parent):
        """Create EditWindow instance."""
        stc.StyledTextCtrl.__init__(self, parent, id=-1, pos=wx.DefaultPosition,
                 size=wx.DefaultSize, style=wx.CLIP_CHILDREN | wx.SUNKEN_BORDER)
        self.__config()


    def __config(self):
        
        self.SetLexer(stc.STC_LEX_PYTHON)
        self.SetKeyWords(0, ' '.join(keyword.kwlist))

        self.setStyles()
        self.SetViewWhiteSpace(False)
        self.SetTabWidth(4)
        self.SetUseTabs(False)
        # self.SetEditable(False)
        # Do we want to automatically pop up command completion options?
        self.autoComplete = True
        self.autoCompleteIncludeMagic = True
        self.autoCompleteIncludeSingle = True
        self.autoCompleteIncludeDouble = True
        self.autoCompleteCaseInsensitive = False
        self.AutoCompSetIgnoreCase(self.autoCompleteCaseInsensitive)
        self.autoCompleteAutoHide = True
        self.AutoCompSetAutoHide(self.autoCompleteAutoHide)
        #self.AutoCompStops(' .,;:([)]}\'"\\<>%^&+-=*/|`')
        # Do we want to automatically pop up command argument help?
        self.autoCallTip = True
        self.callTipInsert = True
        #self.CallTipSetBackground(FACES['calltipbg'])
        #self.CallTipSetForeground(FACES['calltipfg'])
        self.SetWrapMode(False)
        try:
            self.SetEndAtLastLine(False)
        except AttributeError:
            pass
        self.breakpoints = []
        self.SetupMargin()
        
        self.SetBackSpaceUnIndents(True)
        self.SetWrapIndentMode(stc.STC_WRAPINDENT_FIXED)

    def SetupMargin(self):
        self.SetMarginType(0, stc.STC_MARGIN_NUMBER)
        self.SetMarginWidth(0, 0)
        self.SetMarginType(1, stc.STC_MARGIN_SYMBOL)
        self.SetMarginWidth(1, 22)
        self.SetMarginSensitive(1, True)
        # self.MarkerDefine(0, stc.STC_MARK_ROUNDRECT, "#CCFF00", "RED")
        self.MarkerSetBackground(0, "red")
        self.MarkerDefine(1, stc.STC_MARK_ARROW, "#00FF00", "#00FF00")

    def setStyles(self):
        def dump(cont):
            title = ['fore','back','face','size','bold','italic']
            idx = [i for i in range(6) if not (cont[i] is None or cont[i] is False)]
            return ','.join(['%s:%s'%(title[i], cont[i]) for i in idx])
        self.StyleSetSpec(stc.STC_STYLE_DEFAULT, dump(style_config['STYLE_DEFAULT']))
        self.StyleClearAll()
        self.SetSelForeground(True, wx.SystemSettings.GetColour(wx.SYS_COLOUR_HIGHLIGHTTEXT))
        self.SetSelBackground(True, wx.SystemSettings.GetColour(wx.SYS_COLOUR_HIGHLIGHT))
        for i in style_config: self.StyleSetSpec(eval('stc.STC_'+i), dump(style_config[i]))
        
   

if __name__ == '__main__':
    app = wx.App()
    frame = wx.Frame(None)
    pad = CodePad(frame)
    pad.SetValue("""import numpy as np
a = 1
b = 3
m = n
a
b
b
b
""")
    frame.Show()
    app.MainLoop()