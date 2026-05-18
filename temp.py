
import urllib.request, re
url = 'https://html.duckduckgo.com/html/?q=HPH2-A-10+pitch'
req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'})
html = urllib.request.urlopen(req).read().decode('utf-8')
for m in re.finditer(r'class=.result__snippet[^>]*>(.*?)</a', html, re.DOTALL):
    print(re.sub('<[^>]*>', '', m.group(1)).strip())

