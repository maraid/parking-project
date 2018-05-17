import paho.mqtt.client as mqtt
import json


class MQTTClient:
    def __init__(self, host, new_message_event, stop_flag, callback, port=1883, keepalive=60):
        assert isinstance(host, str), host
        assert isinstance(port, int), port
        assert isinstance(keepalive, int), keepalive
        
        self.client = mqtt.Client()
        self.client.connect(host, port, keepalive)
        self.new_message_event = new_message_event
        self.stop_flag = stop_flag
        self.callback = callback
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        self.client.subscribe("KOALA/curvecontrol")
        self.client.publish("controller", '{ "cmd":"reg", "id":2, "topic":"KOALA" }')
    
    def on_message(self, client, userdata, msg):
        self.new_message_event.set()
        self.stop_flag.wait()
        self.new_message_event.clear()
        self.stop_flag.clear()
        print(msg.topic+" "+str(msg.payload))
        payload = json.loads(msg.payload)
        radius = float(payload.get("R"))
        length = float(payload.get("L"))
        self.callback(radius, length)
        
    def start(self):
        self.client.loop_forever()
