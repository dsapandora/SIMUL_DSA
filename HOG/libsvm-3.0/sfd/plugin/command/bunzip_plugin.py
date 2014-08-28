from piper_plugin import Piper

plugin_name  = "bunzip2"

def create(callback):
    return Piper("bunzip2 -c",callback)
