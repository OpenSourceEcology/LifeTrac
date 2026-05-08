import urllib.request
import urllib.parse
from html.parser import HTMLParser

class DDGParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.in_a = False
        self.results = []
        self.current_url = ""
    def handle_starttag(self, tag, attrs):
        if tag == 'a':
            attrs_dict = dict(attrs)
            if 'class' in attrs_dict and 'result__url' in attrs_dict['class']:
                self.in_a = True
                self.current_url = attrs_dict.get('href', "")
    def handle_data(self, data):
        if self.in_a:
            self.results.append((data.strip(), self.current_url))
    def handle_endtag(self, tag):
        if tag == 'a':
            self.in_a = False

q = urllib.parse.quote('Arduino "Portenta Max Carrier" (J-Link OR VCOM OR UART)')
url = f'https://html.duckduckgo.com/html/?q={q}'
req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'})
try:
    html = urllib.request.urlopen(req).read().decode('utf-8')
    parser = DDGParser()
    parser.feed(html)
    for title, link in parser.results[:10]:
        if title:
            print(f"{title}: {link}")
except Exception as e:
    print('Error:', e)
