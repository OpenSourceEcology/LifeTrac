import urllib.request
import json
import urllib.parse
q = urllib.parse.quote('"Max Carrier" UART')
url = f'https://api.github.com/search/issues?q={q}'
req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
try:
    response = urllib.request.urlopen(req)
    data = json.loads(response.read().decode('utf-8'))
    for item in data.get('items', [])[:10]:
        print(f"Title: {item['title']}\nURL: {item['html_url']}\n---")
except Exception as e:
    pass

q = urllib.parse.quote('"Portenta" "Max Carrier" JLink')
url = f'https://api.github.com/search/issues?q={q}'
req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
try:
    response = urllib.request.urlopen(req)
    data = json.loads(response.read().decode('utf-8'))
    for item in data.get('items', [])[:10]:
        print(f"Title: {item['title']}\nURL: {item['html_url']}\n---")
except Exception as e:
    pass

q = urllib.parse.quote('"Max Carrier" debug')
url = f'https://api.github.com/search/issues?q={q}'
req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
try:
    response = urllib.request.urlopen(req)
    data = json.loads(response.read().decode('utf-8'))
    for item in data.get('items', [])[:10]:
        print(f"Title: {item['title']}\nURL: {item['html_url']}\n---")
except Exception as e:
    pass
