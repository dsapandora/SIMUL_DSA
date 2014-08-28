import util
from util import MsgBox
from thread import start_new_thread

class plugin:
    # only .category and .name are public outside this module
    def __init__(self,category,module):
        self.category = category
        self.name = module.plugin_name
        self.mod = module

class component:
    # only .name is public outside this module
    def __init__(self,component_object,name):
        self.name = name
        self.obj = component_object
        self.input = [port(name,interface,self)
                      for (name,interface) in self.obj.input_description]
        self.output = [port(name,interface,self)
                      for (name,interface) in self.obj.output_description]
        self.done = False

class port:
    # only .name is public outside this module
    def __init__(self, name, interface, component):
        self.name = name
        self.interface = interface
        self.component = component
        self.output_obj = None
        self.from_port = None

class connection:
    def __init__(self,from_port,to_port):
        self.from_port = from_port
        self.to_port = to_port

class ComponentManager:

    def __init__(self,canvas,callback):
        self.canvas = canvas
        self.callback = callback
        self.plugins = []
        self.components = []
        self.connections = []
        self.load_plugins()
        
    def reload_all_plugins(self):
        for p in self.plugins:
            reload(p.mod)

    def load_plugins(self):
        import sys
        import glob
        import os.path
        dirs = glob.glob(os.path.join('plugin','*'))
        dirs.sort()
        for dir in dirs:
            if not os.path.isdir(dir): continue
            sys.path.append(dir)
            files = glob.glob(os.path.join(dir,'*_plugin.py'))
            files.sort()
            category = os.path.split(dir)[1]
            for f in files:
                try:
                    p = plugin(category,__import__(os.path.split(f)[-1][:-3]))
                    if (p.category,p.name) in [(q.category,q.name) for q in self.plugins]:
                        raise Exception, "Duplicate plugin name: %s/%s" % (p.category,p.name)
                    self.plugins.append(p)
                except:
                    MsgBox("Cannot load %s -- \n%s" % (f,util.get_traceback()))

    def create_component(self, plugin):
        """create a component using a plugin, 
           return None if user cancelled the operation"""
        
        used_names = [c.name for c in self.components]
        while 1:
            name = util.GetInput(caption = "Create Component",
                                 question = "Name of the component:",
                                 default = plugin.name)
            if not name: return None
            if name in used_names:
                MsgBox("The name is already used.")
            else:
                break
        obj = plugin.mod.create(self.callback)
        c = component(obj, name)
        self.components.append(c)
        return c

    def create_connection(self, from_port, to_port):
        self.reset_component(to_port.component)
        c = connection(from_port, to_port)
        self.connections.append(c)
        to_port.from_port = from_port
        return c
    
    def _run_one(self, component):
        # collect input
        args = {}
        for p in component.input:
            if p.from_port:
                args[p.name] = p.from_port.output_obj
            else:
                args[p.name] = None
        
        # fire
        result = component.obj.run(**args)

        for p in component.output:
            p.output_obj = result[p.name]

    def _run_all(self,schedule):
        try:
            for c in schedule:
                if not c.done:
                    self.callback.start(c)
                    self._run_one(c)
                    self.callback.finish(c)                    
                    c.done = True
        except:
            self.callback.unfinish(c)
            self.callback.log(util.get_traceback())

    def run_all(self):
        schedule = self.calculate_run_schedule()
        start_new_thread(self._run_all,(schedule,))

    def run_one(self,c):
        start_new_thread(self._run_all,([c],))

    def calculate_run_schedule(self):
        # topological sort for components
        edges = [(c.from_port.component, c.to_port.component) 
                 for c in self.connections]
        edge_dict = dict([(c,[]) for c in self.components])
        for x, y in edges:
            edge_dict[x].append(y)
        
        visited = []
        for node in self.components:
            if node not in visited:
                self.dfs(node,[],edge_dict,visited)
        visited.reverse()
        return visited

    def dfs(self,node,path,edges,visited):
        if node in path:
            raise Exception,"Cyclic dependency on " + node.name
        new_path = path[:]
        new_path.append(node)
        for node2 in edges[node]:
            if node2 not in visited:
                self.dfs(node2, new_path, edges, visited)
        visited.append(node)

    def get_plugins(self):
        return self.plugins

    def get_component_output_ports(self,c):
        return c.output

    def get_connection_port(self,c):
        return c.from_port, c.to_port

    def get_port_object(self,p):        
        return p.output_obj

    def get_port_description(self,port):
        if port.interface is None:  # see comments of get_available_input()
            s = "None"
        else:
            s = port.interface.__name__
        return "%s:%s (%s)" % (port.component.name,port.name,s)
                
    # What we have here is a fundamental issue of static typing systems...
    # The easy way out: if the output interface is None, let it go.
    def get_available_input(self,output_port):
        # reject stupid choices, but cyclic dependency are not checked here
        ret = []
        for c in self.components:
            if c is output_port.component: continue
            for p in c.input:
                if p.from_port is not None: continue
                if output_port.interface is None or \
                   issubclass(output_port.interface, p.interface):
                    ret.append((c,p))
        return ret

    def rename_component(self,c):
        name = util.GetInput(question="Name of the component:",default=c.name)
        if name: c.name = name

    def configure_component(self,c):
        if hasattr(c.obj, 'configure'):
            if c.obj.configure():
                self.reset_component(c)

    def reset_component(self,c):
        if c.done:
            c.done = False
            for port in c.output:
                port.output_obj = None
            self.canvas.UnFinishComponent(c)
            for conn in self.connections:
                if conn.from_port.component is c:
                    self.reset_component(conn.to_port.component)
    
    def remove_connection(self,c):
        self.reset_component(c.to_port.component)
        self.connections.remove(c)
        c.to_port.from_port = None

    def get_component_connections(self,component):
        return [c for c in self.connections \
                if component in (c.from_port.component, c.to_port.component)]

    def remove_isolated_component(self,c):
        if self.get_component_connections(c):
            raise Exception, "Trying to remove non-isolated component"
        self.components.remove(c)

    def clean_up(self):
        # reset all components to remove reference to output_obj,
        # so output_obj.__del__ can be called
        for c in self.components:
            self.reset_component(c)
