from piper_plugin import Piper

plugin_name  = "bzip2"

def create(callback):
    return Piper("bzip2 -c",callback)
