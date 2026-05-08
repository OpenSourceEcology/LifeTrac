import urllib.request
url = 'https://raw.githubusercontent.com/arduino/docs-content/main/content/hardware/04.pro/carriers/portenta-max-carrier/tutorials/user-manual/content.md'
content = urllib.request.urlopen(url).read().decode('utf-8')
import re
match = re.search(r'\| \*\*Ethernet DIP Switch Designation(.*?)(\n\n|$)', content, re.DOTALL)
if match:
    print(match.group(0))
