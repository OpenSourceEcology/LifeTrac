import urllib.request
import json
import urllib.parse

def search(q):
    url = 'https://forum.arduino.cc/search/query.json?q=' + urllib.parse.quote(q)
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    print(f"--- QUERY: {q} ---")
    try:
        response = urllib.request.urlopen(req)
        data = json.loads(response.read().decode('utf-8'))
        for topic in data.get('topics', [])[:5]:
            print(f"Topic: {topic['title']}")
            print(f"URL: https://forum.arduino.cc/t/{topic['slug']}/{topic['id']}")
            print("---")
    except Exception as e:
        print('Failed:', e)

search('Max Carrier JLink')
search('Max Carrier J-Link')
search('Max Carrier UART')
search('Max Carrier serial')
search('Portenta Max Carrier VCOM')
