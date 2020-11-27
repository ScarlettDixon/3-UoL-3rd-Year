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
        
        filemenu = wx.Menu()
        New = filemenu.Append(wx.ID_NEW, '&New Search')
        Op_Wish = filemenu.Append(wx.ID_OPEN, '&Open Wishlist')
        Sa_Wish = filemenu.Append(wx.ID_SAVE, '&Save Wishlist')
        filemenu.AppendSeparator()
        Qu = filemenu.Append(wx.ID_EXIT, '&Quit')
        
        datamenu = wx.Menu()
        datamenu.Append(wx.ID_ANY, 'API 1')
        datamenu.Append(wx.ID_ANY, 'API 2')
        
        menub.Append(filemenu, 'File')
        menub.Append(datamenu, 'Database')
        
        
        self.SetMenuBar(menub)
    
    def createPanel(self):
        panel = wx.Panel(self)

        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        
        fgs = wx.FlexGridSizer(5, 2, 9, 25)
        fgs2 = wx.FlexGridSizer(1, 3, 9, 25)

        title = wx.StaticText(panel, label="Title")
        year = wx.StaticText(panel, label="Year of Release")
        imdbid = wx.StaticText(panel, label="imdbID")
        media_Type = wx.StaticText(panel, label="Type of Media")
        poster_Link = wx.StaticText(panel, label="Poster Link")

        tc1 = wx.TextCtrl(panel)
        tc2 = wx.TextCtrl(panel)
        tc3 = wx.TextCtrl(panel)
        tc4 = wx.TextCtrl(panel)
        tc5 = wx.TextCtrl(panel, style=wx.TE_MULTILINE)


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
