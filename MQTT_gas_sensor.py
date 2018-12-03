#!/usr/bin/env python3

import RPi.GPIO as GPIO

import socket
import time
import os.path
import argparse

import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
try:
    import urllib.request as request
except:
    import urllib2 as request
"""Setup the Pi. pin 19 is the connected upstream of R2 in design. Pin 26 is the base of Q1. The detector
 circuit is enabled with pin 26 set high."""
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.IN)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, True)

"""Setups the MQTT service. """
DEFAULT_MQTT_BROKER_HOST = "192.168.86.101"
DEFAULT_MQTT_BROKER_PORT = 1883
HOSTNAME = socket.gethostname()
COMMAND = "gas_sensor"
TIME_TO_INACTIVE = 120.0  # sec
DELTA_TIME_PLAYING_STATUS = 0.1
DEFAULT_ROOT_TOPIC = "/gas_sensor/state/" + HOSTNAME

"""This sets up using the script as a command line utility with switches. Not strictly necessary. """
parser = argparse.ArgumentParser(
    description='MQTT service for monitoring a gas sensor')
parser.add_argument('--host', '-H', type=str, nargs=1, default=DEFAULT_MQTT_BROKER_HOST,
                    help='The host for the mqtt broker, default {0}'.format(DEFAULT_MQTT_BROKER_HOST))
parser.add_argument('--port', '-p', type=int, nargs=1, default=DEFAULT_MQTT_BROKER_PORT,
                    help='The port for the mqtt broker, default {0}'.format(DEFAULT_MQTT_BROKER_PORT))
parser.add_argument('--topic', '-t', type=str, nargs=1, default=DEFAULT_ROOT_TOPIC,
                    help='The topic to use, default {0}'.format(DEFAULT_ROOT_TOPIC))
parser.add_argument('--send', '-s', type=str, nargs=1, default=None,
                    help='Only send this payload and exit')


def main():
    args = parser.parse_args()
    mqtt_broker_host = args.host
    mqtt_broker_port = args.port
    root_topic = args.topic
    read_topic = os.path.join(root_topic, "in")
    write_topic = os.path.join(root_topic, "out")
    inactive_timesteps = 0

    if args.send is not None:
        publish.single(write_topic,
                       payload=" ".join(args.send),
                       hostname=mqtt_broker_host,
                       port=mqtt_broker_port)
        exit(0)
    """These functions are used with the command line functions"""
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        print("MQTT Topic: {}, Connected to {}".format(
            root_topic, mqtt_broker_host))
        client.subscribe(read_topic)

    def on_disconnect(client, userdata, rc):
        connect(client, mqtt_broker_host, mqtt_broker_port)

    def on_message(client, userdata, msg):
        try:
            cmd = str(msg.payload.decode("utf-8"))
            if len(cmd.split()) >= 1:
                print("received {} from mqtt broker".format(cmd))
                action, *args = cmd.split()
                {
                    "reset": sensorReset()
                }[action](*args)
            else:
                print("unknown command {} received from mqtt broker".format(cmd))
        except:
            print('An error occured')

    def on_publish(client, userdata, mid):
        print('published')

    client = mqtt.Client()
    """Tells the server what to do when disconnect occurs"""
    client.will_set("/gas_sensor/availability/raspberrypi",
                    payload="offline", retain=True)
    connect(client, mqtt_broker_host, mqtt_broker_port)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.on_publish = on_publish
    client.loop_start()
    """this is the loop that always runs. Still contains some trouble shooting stuff"""
    while True:
        sensorReset()
        payloadString = sensorStatus()
        client.publish("/gas_sensor/availability/raspberrypi",
                       payload="online")
        client.publish(write_topic,
                       payload=payloadString)
        print(payloadString)
        time.sleep(5)
        pass

# Connects the client. If fails due to host being down or localhost network is down, retry


def connect(client, host, port):
    reconnected = False
    while not reconnected:
        try:
            client.connect(host, port, 60)
            reconnected = True
        except:
            print("Unable to connect, trying again...")
        time.sleep(1)


def sensorReset():
    """Resets D2."""
    GPIO.output(26, False)
    time.sleep(.5)
    GPIO.output(26, True)


def sensorStatus():
    """Reads the input from the detection circuit."""
    if (GPIO.input(19) == True):
        return "ON"
    else:
        return "OFF"


if __name__ == '__main__':
"""The try-except is to facilitate command line testing and exiting with ^C.  The
finally statement sends a death message to the MQTT server and releases the GPIO pins. """
   try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        client.publish("/gas_sensor/availability/raspberrypi", payload= "offline", retain = True)
        GPIO.cleanup()
