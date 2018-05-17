import ArchingKoala
import MQTTClient
import threading


class KoalaController:
    def __init__(self, host, serial):
        self.new_message_flag = threading.Event()
        self.stop_flag = threading.Event()

        self.koala = ArchingKoala.ArchingKoala(serial, self.new_message_flag, self.stop_flag)

        self.mqtt = MQTTClient.MQTTClient(host, self.new_message_flag, self.stop_flag, self.koala.callback)
        self.mqtt_thread = threading.Thread(target=self.mqtt.start)


