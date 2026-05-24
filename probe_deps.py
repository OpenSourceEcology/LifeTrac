missing = []
for m in ["fastapi", "uvicorn", "paho.mqtt.client", "cryptography", "jinja2"]:
    try:
        __import__(m)
    except ImportError:
        missing.append(m)
print("MISSING:", missing if missing else "none")
