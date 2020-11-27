import sys
import wx
import requests

class Layout1(wx.Frame):
    def __init__(self, *args, **kwargs):
        super (Layout1, self).__init__ (*args, **kwargs)
        self.createMenu()
        self.createPanel()
        self.SetSize((600, 600))
        self.SetTitle('ACME Film Database')
        self.Centre()
   
    def createMenu(self):
        menub = wx.MenuBar()
    
    def createPanel(self):
        panel = wx.Panel(self)

        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)


class Api():
    def __init__(self):
        userInput = 'titanic'
        searchurl = 'http://www.omdbapi.com/?apikey=91d6b606&s=' + userInput
        req = requests.get(searchurl)
        txt = req.text
        self.search(txt, "Title")
        
    def search(self, txt, searchterm):
        start = 0
        count = 0
        Results = str(txt.count(searchterm)) + " Results Found"
        first = txt.find(searchterm)
        last = txt.rfind(searchterm)
        while start <= last:
            count = txt.find(searchterm, start) #Find title in string
            start = count + 1 #reset search parameter to find next film
            TitleEnd = txt.find('"', start)
            QuoteStart = txt.find('"', TitleEnd + 1)
            QuoteEnd = txt.find('"', QuoteStart + 1)
            print(txt[QuoteStart + 1 : QuoteEnd])

def main ():
    Api()
    app = wx.App() 
    frm = Layout1(None, title="API interaction") 
    frm.Show()
    app.MainLoop() 
    
if __name__ == '__main__':
    main()
