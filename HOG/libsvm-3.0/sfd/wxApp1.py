#!/usr/bin/env python
#Boa:App:BoaApp

from wxPython.wx import *

import MainFrame

modules ={'ComponentManager': [0, '', 'ComponentManager.py'],
 'MainFrame': [1, 'Main frame of Application', 'MainFrame.py'],
 'Makefile': [0, '', 'Makefile'],
 'MyCanvas': [0, '', 'MyCanvas.py'],
 'ViewImageFrame': [0, '', 'ViewImageFrame.py'],
 'ViewTextFrame': [0, '', 'ViewTextFrame.py'],
 'common_interface': [0, '', 'common_interface.py'],
 'plugin': [0, '', 'plugin.txt'],
 'util': [0, '', 'util.py']}

class BoaApp(wxApp):
    def OnInit(self):
        wxInitAllImageHandlers()
        self.main = MainFrame.create(None)
        # needed when running from Boa under Windows 9X
        self.main.Show();self.main.Hide();self.main.Show()
        self.SetTopWindow(self.main)
        return true

def main():
    application = BoaApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
