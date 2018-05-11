import paho.mqtt.client as mqtt
import json


class MQTTClient:
    def __init__(self, host, event, callback, port=1883, keepalive=60):
        assert isinstance(host, str), host
        assert isinstance(port, int), port
        assert isinstance(keepalive, int), keepalive
        
        self.client = mqtt.Client()
        self.client.connect(host, port, keepalive)
        self.stop_event = event
        self.callback = callback
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe("KOALA/curvecontrol")
        self.client.publish("controller", '{ "cmd":"reg", "id":2, "topic":"KOALA" }')
    
    def on_message(self, client, userdata, msg):
        self.stop_event.set()
        print(msg.topic+" "+str(msg.payload))
        payload = json.loads(msg.payload)
        r = float(payload.get("R"))
        l = float(payload.get("L"))
        self.callback(r, l)
        
    def start(self):
        self.client.loop_forever()
