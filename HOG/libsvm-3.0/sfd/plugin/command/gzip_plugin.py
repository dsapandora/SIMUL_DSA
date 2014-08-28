from piper_plugin import Piper

plugin_name  = "gzip"

def create(callback):
    return Piper("gzip -c",callback)
