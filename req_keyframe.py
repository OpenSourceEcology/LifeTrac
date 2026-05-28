import paho.mqtt.publish as p
print("Publishing keyframe request to tractor-mosquitto-v2 on lifetrac/v25/cmd/req_keyframe...")
p.single('lifetrac/v25/cmd/req_keyframe', b'trigger', hostname='tractor-mosquitto-v2')
print("Published!")
