try:
    import amqtt
    print("amqtt ok")
except Exception as e:
    print("amqtt FAIL:", e)
