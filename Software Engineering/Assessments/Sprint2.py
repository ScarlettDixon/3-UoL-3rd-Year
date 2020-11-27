import sys
import wx
import requests
from selenium import webdriver
import time
from bs4 import BeautifulSoup as bs

#Needed for this project:
# A User Interface
# Options for user to press
# Options for user input
# Displaying of data from api to based on user interaction


class Api():
    def __init__(self):
        searchurl = 'http://www.omdbapi.com/?apikey=91d6b606&s=titanic'
        test = requests.get(searchurl)
        print(test);
        self.search(test, "Year")
    def search(self, txt, searchterm):
        start = 0
        End = 0
        Count = 0
        
    
def main ():
    Api()
    
    
if __name__ == '__main__':
    main()
