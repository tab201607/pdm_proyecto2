# Import standard python modules.
import sys
import time
import random
import serial


# This example uses the MQTTClient instead of the REST client
from Adafruit_IO import MQTTClient

# holds the count for the feed
global run_count

# colocar user y pass

# Set to the ID of the feed to subscribe to for updates.
#feedContador = 'contador'
feedBase = 'Base'
feedCodo = 'Codo'
feedHombro = 'Hombro'
feedPinza = 'Pinza'

mensaje = ''
submensaje = ''

# Define callback functions which will be called when certain events happen.
def connected(client):
    """Connected function will be called when the client is connected to
    Adafruit IO.This is a good place to subscribe to feed changes.  The client
    parameter passed to this function is the Adafruit IO MQTT client so you
    can make calls against it easily.
    """
    # Subscribe to changes on a feed named Counter.
    #print('Subscribing to Feed {0} and {1}'.format(feedLed, feedContador))
    client.subscribe(feedBase)
    client.subscribe(feedCodo)
    client.subscribe(feedHombro)
    client.subscribe(feedPinza)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    """Message function will be called when a subscribed feed has a new value.
    The feed_id parameter identifies the feed, and the payload parameter has
    the new value.
    """
    #print('Feed {0} received new value: {1}'.format(feed_id, payload))
    
    #if(feed_id == feedBase):
        #if(payload == '0'):
            #print('led1: OFF')
            #arduino.write(bytes('0\n', 'utf-8'))
        #if(payload == '1'):
            #print('led1: ON')
            #arduino.write(bytes('1\n', 'utf-8'))


    
    


try:
    client = MQTTClient(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY)

    # Setup the callback functions defined above.
    client.on_connect = connected
    client.on_disconnect = disconnected
    client.on_message = message

    # Connect to the Adafruit IO server.
    client.connect()
    client.loop_background()
              
    arduino = serial.Serial(port='COM5', baudrate =9600, timeout = 0.1)

    while True:    
        mensaje = arduino.readline().decode('utf-8')
        if(mensaje == 'BASE\n'):
            print('Nuevo dato: ')
            print('BASE\n')
            submensaje = arduino.readline().decode('utf-8')
            print(submensaje)
            client.publish(feedBase, submensaje)
        elif(mensaje == 'CODO\n'):
            print('Nuevo dato: ')
            print('CODO\n')
            submensaje = arduino.readline().decode('utf-8')
            print(submensaje)
            client.publish(feedCodo, submensaje)
        elif(mensaje == 'HOMBRO\n'):
            print('Nuevo dato: ')
            print('HOMBRO\n')
            submensaje = arduino.readline().decode('utf-8')
            print(submensaje)
            client.publish(feedHombro, submensaje)
        elif(mensaje == 'PINZA\n'):
            print('Nuevo dato: ')
            print('PINZA\n')
            submensaje = arduino.readline().decode('utf-8')
            print(submensaje)
            client.publish(feedPinza, submensaje)
        
        time.sleep(1)
        
        
except KeyboardInterrupt:
    print("Salimos del programa")
    if arduino.is_open:
        arduino.close()
    sys.exit(1)