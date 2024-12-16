"""
Project: LoRa Node for the Greenhouse
Author: Tilen Tinta
Date: October 2024
"""

import json
import paho.mqtt.client as mqtt # MQTT TTN
import psycopg2                 # json
import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime   # timestamp
import pytz
import logging                  # Logging actions
import socket                   # computer name


############ Load json files ############
# Load json file used for database connection config
def load_db_config(file_path):
    try:
        with open(file_path, 'r') as file:
            config = json.load(file)

        logging.info("Database json open success!"); 
        return config
    
    except Exception as error:
        logging.info(f"Database json file error: {error}")
        return -1


# Load json file used for mqtt connection config
def load_mqtt_config(file_path):
    try:
        with open(file_path, 'r') as file:
            config = json.load(file)

        logging.info("TTN json open success!")
        return config
    
    except Exception as error:
        logging.info(f"TTN json file error: {error}")
        return -1
    


############ DATABASE ############
def save_to_db(config, data):

    try:
        
        # InfluxDB connection
        client = influxdb_client.InfluxDBClient(config["url"], config["token"], config["org"])
        write_api = client.write_api(write_options=SYNCHRONOUS)

        time_zone = pytz.timezone(config["timezone"])
        utc_time = datetime.utcnow()
        my_time = pytz.utc.localize(utc_time).astimezone(time_zone)

        point = (
            Point(config["bucket"],)
            .tag("node_id", str(node_id))  # Tag to differentiate nodes
            .field("temperature", temperature)
            .field("humidity", humidity)
            .field("air_pressure", air_pressure)
            .field("earth_humidity", earth_humidity)
            .field("battery_voltage", battery_voltage)
            .time(my_time)  # Automatically uses current timestamp of preset location
        )

        write_api.write(config["bucket"], config["org"], record=point)


    except Exception as error:
        print(f"Error while writing to database: {error}")
        logging.info(f"Error while writing to database: {error}") # Write error log

    finally:

        # DB write OK
        logging.info("Database write success!"); 
        client.close() # Close client connection



############ MQTT ############
# Callback function when a message is received
def on_message(client, userdata, message, config_db):

    # Decode the message payload from bytes to string
    payload_str = message.payload.decode()
    
    # Parse the JSON payload
    payload = json.loads(payload_str)

    #TODO: Detect test/boot message
    
    # Example: Access some fields from the payload
    device_id = payload['end_device_ids']['device_id']
    received_time = payload['received_at']
    temperature = payload['uplink_message']['decoded_payload'].get('temperature', None)
    humidity = payload['uplink_message']['decoded_payload'].get('humidity', None)
        
    # Print the full message for debugging
    print("Full message payload:")
    print(json.dumps(payload, indent=4))

    save_to_db(config_db, payload) # TO TEST


# Callback function when the client connects
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to TTN MQTT broker")
        client.subscribe(config_ttn["topic"])
        logging.info("Successfully connected to TTN MQTT broker")
    else:
        print(f"Failed to connect, return code {rc}")



######################################################
######################## MAIN ########################
######################################################

if __name__ == "__main__":

    #-- Files --#
    db_json = 'InfluxDB_config.json'
    ttn_json = 'ttn_config.json'
    logFile = 'MQTT-SQL_DataManipulator.log'

    # Check if the log file exists, else create
    if not os.path.exists(logFile):
        open(logFile, 'w').close()

    # Configure the logging settings
    logging.basicConfig(
        filename= logFile,            
        filemode='a', # Append mode
        format='%(asctime)s - %(levelname)s - %(message)s', # Log format
        level=logging.INFO # Log level 
    )

    mc_name = socket.gethostname() # get computer name
    logging.info(f"Program started on: {mc_name}")

    #-- JSON files import --#
    config_db = load_db_config(db_json)
    config_ttn = load_mqtt_config(ttn_json)

    if config_db == -1 or config_ttn == -1:
        print("JSON file error!")
        exit()

    #-- MQTT conection --#
    # Initialize the MQTT client
    client = mqtt.Client()
    client.username_pw_set(config_ttn["app_id"], config_ttn["access_key"])
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the TTN MQTT broker
    client.connect(config_ttn["mqtt_broker"], config_ttn["mqtt_port"], 60)
    save_to_db(config_db, 1)
    # Start the MQTT client
    client.loop_forever()

    

    

    
