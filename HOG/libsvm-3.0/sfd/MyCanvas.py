from wxPython.wx import *
from wxPython.ogl import *
import util
from util import curry, MsgBox
import ComponentManager

wxOGLInitialize()

EVT_COMPONENT_MANAGER_ID = wxNewId()   # callback from ComponentManager

# types for ComponentManagerEvent
CM_START, CM_FINISH, CM_UNFINISH, CM_LOG, CM_RUN = range(5)

def EVT_COMPONENT_MANAGER(win, func):
    win.Connect(-1, -1, EVT_COMPONENT_MANAGER_ID, func)

class ComponentManagerEvent(wxPyEvent):
    def __init__(self, type, data):
        wxPyEvent.__init__(self)
        self.SetEventType(EVT_COMPONENT_MANAGER_ID)
        self.type = type
        self.data = data

def create(parent,log):
    return MyCanvas(parent,log)

class MyCanvas(wxShapeCanvas):

    def __init__(self,parent,log):
        wxShapeCanvas.__init__(self,parent,style=wxSUNKEN_BORDER)
        self.SetBackgroundColour(wxWHITE)
        self.parent = parent
        self.log = log
        self.diagram = wxDiagram()
        self.SetDiagram(self.diagram)
        self.diagram.SetCanvas(self)
        EVT_RIGHT_UP(self,curry(self.OnMouseClick,button='RIGHT'))
        EVT_LEFT_DCLICK(self,curry(self.OnMouseClick,button='LEFT'))
        EVT_COMPONENT_MANAGER(self,self.OnComponentManagerEvent)
        self.shape2component = {}
        self.component2shape = {}
        self.line2connection = {}
        self.connection2line = {}
        self.cm = ComponentManager.ComponentManager(self,
                    self.ComponentManagerCallback(self))
        self.CreatePluginMenu()

    def AddMenuItem(self,menu,text,func):
        id = wxNewId()
        menu.Append(id,text)
        EVT_MENU(self, id, func)

    def CreatePluginMenu(self):
        plugins = self.cm.get_plugins()
        categories = dict([(p.category,1) for p in plugins]).keys()
        self.plugin_menu = wxMenu("-- Create Components --")
        for c in categories:
            m = wxMenu()
            for p in [p for p in plugins if p.category == c]:
                self.AddMenuItem(m, p.name, curry(self.CreateComponent,plugin=p))
            self.plugin_menu.AppendMenu(wxNewId(),c,m)

    def CreateComponent(self,event,plugin):
        try:
            c = self.cm.create_component(plugin)
            if c is None: return    # cancel
        except:
            MsgBox("create_component() failed.")
            self.LogException()
            return

        s = wxRectangleShape(100,50)
        s.AddText(c.name)
        s.SetX(self.current_x)
        s.SetY(self.current_y)
        s.SetPen(wxBLACK_PEN)
        s.SetBrush(wxBrush(wxColour(219,219,255),wxSOLID))
        s.SetCanvas(self)
        self.AddShape(s)
        s.Show(True)
        self.shape2component[s] = c
        self.component2shape[c] = s

    def CreateConnection(self, event,
                      from_component, from_port, to_component, to_port):
        try:
            conn = self.cm.create_connection(from_port, to_port)
        except:
            MsgBox("create_connection() failed.")
            self.LogException()
            return
        from_shape = self.component2shape[from_component]
        to_shape = self.component2shape[to_component]

        line = wxLineShape()
        line.SetCanvas(self)
        line.SetPen(wxBLACK_PEN)
        line.SetBrush(wxBLACK_BRUSH)
        line.AddArrow(ARROW_ARROW)
        line.MakeLineControlPoints(2)
        from_shape.AddLine(line, to_shape)
        self.diagram.AddShape(line)
        line.Show(true)
        self.line2connection[line] = conn
        self.connection2line[conn] = line

        dc = wxClientDC(self)
        self.PrepareDC(dc)
        # for some reason, the shapes have to be moved for the line to show up...
        from_shape.Move(dc, from_shape.GetX(), from_shape.GetY())

    def OnLeftDClickComponent(self,event,component):
        self.cm.configure_component(component)

    def OnRightClickComponent(self,event,c):
        m = wxMenu()
        for port in self.cm.get_component_output_ports(c):
            m2 = wxMenu()
            for input_c, input_p in self.cm.get_available_input(port):
                text = '[ ' + input_c.name + ':' + input_p.name + ' ]'
                func = curry(self.CreateConnection,
                             from_component = c, to_component = input_c,
                             from_port = port, to_port = input_p)
                self.AddMenuItem(m2,text,func)
            m.AppendMenu(wxNewId(),
                "Connect [ %s ] to..." % port.name, m2)
        for port in self.cm.get_component_output_ports(c):
            self.AddMenuItem(m, "View [ %s ]" % port.name,
                        curry(self.ViewPort,port=port))
        m.AppendSeparator()
        self.AddMenuItem(m, "Configure", curry(self.OnLeftDClickComponent,component=c))
        self.AddMenuItem(m, "Rename", curry(self.RenameComponent,component=c))
        self.AddMenuItem(m, "Run", curry(self.RunOne,component=c))
        self.AddMenuItem(m, "Reset",  curry(self.ResetComponent,component=c))
        self.AddMenuItem(m, "Remove", curry(self.RemoveComponent,component=c))
        self.PopupMenu(m, event.GetPosition())
        m.Destroy()

    def RemoveComponent(self,event,component):
        cm = self.cm
        for c in cm.get_component_connections(component):
            self.RemoveConnection(None,c)
        cm.remove_isolated_component(component)
        shape = self.component2shape[component]
        del self.shape2component[shape]
        del self.component2shape[component]
        self.RemoveShape(shape)

        # for some reason...
        self.Hide()
        self.Show()

    def RenameComponent(self,event,component):
        self.cm.rename_component(component)
        s = self.component2shape[component]
        s.ClearText()
        s.AddText(component.name)
        self.Refresh()

    def ResetComponent(self,event,component):
        self.cm.reset_component(component)

    def ResetAll(self):
        for c in self.component2shape.keys():
            self.cm.reset_component(c)

    def ViewPort(self,event,port):
        parent = self.parent
        title = self.cm.get_port_description(port)
        obj = self.cm.get_port_object(port)
        if hasattr(obj,'view'):
            obj.view(parent, title)
        else:
            MsgBox(str(obj))

    def RemoveConnection(self,event,connection):
        try:
            self.cm.remove_connection(connection)
            pass
        except:
            MsgBox("remove_connection() failed.")
            self.LogException()
            return

        line = self.connection2line[connection]
        self.RemoveShape(line)
        line.Unlink()
        del self.connection2line[connection]
        del self.line2connection[line]

        # for some reason...
        self.Hide()
        self.Show()

    def OnRightClickConnection(self,event,c):
        m = wxMenu()
        port = self.cm.get_connection_port(c)[0]
        self.AddMenuItem(m,"View",curry(self.ViewPort,port=port))
        self.AddMenuItem(m,"Remove",curry(self.RemoveConnection,connection=c))
        self.PopupMenu(m, event.GetPosition())

    def OnLeftDClickConnection(self,event,c):
        pass

    def OnMouseClick(self,event, button):
        fs = self.FindShape(*event.GetPosition())
        if fs:
            shape = fs[0]
            if shape in self.shape2component:
                if button == 'RIGHT':
                    self.OnRightClickComponent(event,self.shape2component[shape])
                else:
                    self.OnLeftDClickComponent(event,self.shape2component[shape])
            elif shape in self.line2connection:
                if button == 'RIGHT':
                    self.OnRightClickConnection(event,self.line2connection[shape])
                else:
                    self.OnLeftDClickConnection(event,self.line2connection[shape])
            else:
                raise Exception, "Cannot find corresponding shape?"
        else:
            self.current_x, self.current_y  = pos = event.GetPosition()
            self.PopupMenu(self.plugin_menu, pos)
            self.Refresh()

    def Refresh(self):
        dc = wxClientDC(self)
        self.PrepareDC(dc)
        self.Redraw(dc)

    def RunAll(self):
        try:
            self.cm.run_all()
        except:
            MsgBox("run_all() failed.")
            self.LogException()

    def RunOne(self,event,component):
        try:
            self.cm.reset_component(component)
            self.cm.run_one(component)
        except:
            MsgBox("run_one() failed.")
            self.LogException()

    class ComponentManagerCallback:
        def __init__(self, evt_mgr):
            self.evt_mgr = evt_mgr
        def start(self,obj):
            wxPostEvent(self.evt_mgr,ComponentManagerEvent(CM_START,obj))
        def finish(self,obj):
            wxPostEvent(self.evt_mgr,ComponentManagerEvent(CM_FINISH,obj))
        def unfinish(self,obj):
            wxPostEvent(self.evt_mgr,ComponentManagerEvent(CM_UNFINISH,obj))
        def run(self,obj):
            wxPostEvent(self.evt_mgr,ComponentManagerEvent(CM_RUN,obj))
        def log(self,msg):
            wxPostEvent(self.evt_mgr,ComponentManagerEvent(CM_LOG,msg))

    def OnComponentManagerEvent(self,event):
        if event.type == CM_START:
            s = self.component2shape[event.data]
            s.SetPen(wxPen("ORANGE"))
            self.Refresh()
        elif event.type == CM_FINISH:
            s = self.component2shape[event.data]
            s.SetPen(wxPen(wxBLUE))
            self.Refresh()
        elif event.type == CM_UNFINISH:
            s = self.component2shape[event.data]
            s.SetPen(wxPen(wxBLACK))
            self.Refresh()
        elif event.type == CM_LOG:
            self.WriteLog(str(event.data))
        elif event.type == CM_RUN:
            try:
                event.data()
            except:
                self.LogException()

    def LogException(self):
        self.WriteLog(util.get_traceback())

    def WriteLog(self,msg):
        self.log.AppendText(msg)
        self.log.ShowPosition(self.log.GetLastPosition())

    def UnFinishComponent(self,c):
        s = self.component2shape[c]
        s.SetPen(wxBLACK_PEN)
        self.Refresh()

    def CleanUp(self):
        self.cm.clean_up()

    def ReloadAllPlugins(self):
        self.cm.reload_all_plugins()
