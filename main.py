from fsm import FSM
import paho.mqtt.client as mqtt


def onConnected(connectedClient, userdata, flags, rc):
    print("Connection status: ", rc)
    connectedClient.subscribe("Put tha ting")


def onMessageReceived(client, userdata, msg):
    print("Message Received:\n", msg.topic, " ", str(msg.payload))


def main():
    fsm = FSM()

    client = mqtt.Client()
    client.on_connect = onConnected
    client.on_message = onMessageReceived

    # Need proper endpoint and stuff here...
    client.connect("mqtt.eclipseprojects.io", 1883, 60)

    client.loop_start()

    # Control loop
    while (True):
        fsm.doCurrentState()
        fsm.moveState()

    client.loop_stop()


if __name__ == "__main__":
    main()
