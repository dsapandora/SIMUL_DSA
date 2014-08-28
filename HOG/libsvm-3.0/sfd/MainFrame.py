#Boa:Frame:MainFrame

from wxPython.wx import *
import MyCanvas

def create(parent):
    return MainFrame(parent)

[wxID_MAINFRAME, wxID_MAINFRAMESPLITTER, 
] = map(lambda _init_ctrls: wxNewId(), range(2))

[wxID_MAINFRAMEMENU_RUNITEMS1, wxID_MAINFRAMEMENU_RUNITEMS2, 
 wxID_MAINFRAMEMENU_RUNITEM_RUN_ALL, 
] = map(lambda _init_coll_menu_run_Items: wxNewId(), range(3))

[wxID_MAINFRAMEMENU_FILEITEMS1, wxID_MAINFRAMEMENU_FILEITEM_EXIT, 
] = map(lambda _init_coll_menu_file_Items: wxNewId(), range(2))

class MainFrame(wxFrame):
    
    def _init_coll_menu_run_Items(self, parent):
        # generated method, don't edit

        parent.Append(helpString='', id=wxID_MAINFRAMEMENU_RUNITEM_RUN_ALL,
              item='Run All', kind=wxITEM_NORMAL)
        parent.Append(helpString='', id=wxID_MAINFRAMEMENU_RUNITEMS2,
              item='Reset all', kind=wxITEM_NORMAL)
        parent.Append(helpString='', id=wxID_MAINFRAMEMENU_RUNITEMS1,
              item='Clear log window', kind=wxITEM_NORMAL)
        EVT_MENU(self, wxID_MAINFRAMEMENU_RUNITEM_RUN_ALL, self.OnMenu_RunAll)
        EVT_MENU(self, wxID_MAINFRAMEMENU_RUNITEMS1, self.OnMenu_ClearLogWindow)
        EVT_MENU(self, wxID_MAINFRAMEMENU_RUNITEMS2, self.OnMenu_ResetAll)

    def _init_coll_menu_file_Items(self, parent):
        # generated method, don't edit

        parent.Append(helpString='', id=wxID_MAINFRAMEMENU_FILEITEMS1,
              item='Reload all plugins', kind=wxITEM_NORMAL)
        parent.Append(helpString='', id=wxID_MAINFRAMEMENU_FILEITEM_EXIT,
              item='Quit', kind=wxITEM_NORMAL)
        EVT_MENU(self, wxID_MAINFRAMEMENU_FILEITEMS1, self.OnMenu_ReloadAll)
        EVT_MENU(self, wxID_MAINFRAMEMENU_FILEITEM_EXIT, self.OnMenu_Quit)

    def _init_coll_menuBar1_Menus(self, parent):
        # generated method, don't edit

        parent.Append(menu=self.menu_file, title='File')
        parent.Append(menu=self.menu_run, title='Run')

    def _init_utils(self):
        # generated method, don't edit
        self.menuBar1 = wxMenuBar()

        self.menu_file = wxMenu(title='')
        self._init_coll_menu_file_Items(self.menu_file)

        self.menu_run = wxMenu(title='')
        self._init_coll_menu_run_Items(self.menu_run)

        self._init_coll_menuBar1_Menus(self.menuBar1)

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wxFrame.__init__(self, id=wxID_MAINFRAME, name='MainFrame', parent=prnt,
              pos=wxPoint(100, 100), size=wxSize(600, 400),
              style=wxDEFAULT_FRAME_STYLE, title='SVM for Dummies')
        self._init_utils()
        self.SetClientSize(wxSize(600, 400))
        self.SetMenuBar(self.menuBar1)
        EVT_CLOSE(self, self.OnWxframe1Close)

        self.splitter = wxSplitterWindow(id=wxID_MAINFRAMESPLITTER,
              name='splitter', parent=self, point=wxPoint(0, 0),
              size=wxSize(600, 280), style=wxNO_3D)
        self.splitter.SetToolTipString('')

    def __init__(self, parent):
        self._init_ctrls(parent)
        self.init_log_window()                
        self.init_canvas()        
        self.splitter.SplitHorizontally(self.canvas, self.log_window, 330)            
        self.splitter.SetMinimumPaneSize(10)                
        
    def init_canvas(self):
        self.canvas = MyCanvas.create(self.splitter, self.log_window)

    def init_log_window(self):
        self.log_window = wxTextCtrl(id = -1, name='LogWindow',
              parent=self.splitter, pos=wxPoint(0,0), size=wxSize(600,100), 
              style=wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH2 | 
                    wxHSCROLL | wxVSCROLL | wxSUNKEN_BORDER,
              value='')        
                
    def OnWxframe1Close(self, event):
        self.canvas.CleanUp()    
        event.Skip()

    def OnMenu_RunAll(self, event):
        self.canvas.RunAll()

    def OnMenu_Quit(self, event):               
        self.Close()
        
    def OnMenu_ClearLogWindow(self, event):
        self.log_window.Clear()

    def OnMenu_ReloadAll(self, event):
        self.canvas.ReloadAllPlugins()

    def OnMenu_ResetAll(self, event):
        self.canvas.ResetAll()
