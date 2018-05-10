import paho.mqtt.client as mqtt
import json


class MQTTClient:
    def __init__(self, host, on_message, port=1883, keepalive=60):
        assert isinstance(host, str), host
        assert isinstance(port, int), port
        assert isinstance(keepalive, int), keepalive
        
        self.client = mqtt.Client()
        self.client.connect(host, port, keepalive)
        self.loop_forever = self.client.loop_forever
        self.client.on_connect = self.on_connect
        self.client.on_message = on_message
        self.r = None
        self.l = None
        
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.client.subscribe("KOALA/curvecontrol")

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))
        paylaod = json.loads(msg.payload)
        self.r = float(paylaod.get("R"))
        slef.l = float(payload.get("L"))