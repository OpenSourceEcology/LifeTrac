import paho.mqtt.publish as p
p.single('lifetrac/v25/cmd/req_keyframe', b'trigger', hostname='tractor-mosquitto-v2')
print("Keyframe request published.")
