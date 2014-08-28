from piper_plugin import Piper

plugin_name  = "gunzip"

def create(callback):
    return Piper("gunzip -c",callback)
