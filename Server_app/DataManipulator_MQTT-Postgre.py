"""
Project: LoRa Node for the Greenhouse
Author: Tilen Tinta
Date: October 2024
Status: UNFINISHED / NOT USED !!!!!!
"""

import json
import os
import paho.mqtt.client as mqtt # MQTT TTN
import psycopg2                 # json
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
    
    # Connect to database
    connection = None

    try:
        # json file data
        connection = psycopg2.connect(
            host=config["host"],
            database=config["database"],
            user=config["user"],
            password=config["password"],
            port=config["port"]
        )

        # Create a cursor object to interact with the database
        cursor = connection.cursor()

        query = 'SELECT * FROM "Nodes";' # change

        cursor.execute(query)
        rows = cursor.fetchall()

        for row in rows:
            print(row)

    except Exception as error:
        print(f"Error while connecting to database: {error}")
        logging.info(f"Error while connecting to database: {error}") # Write error log

    finally:

        # DB write OK
        logging.info("Database write success!"); 

        # Close the cursor and connection to free up resources
        if connection:
            cursor.close()
            connection.close()
            #print("Database connection closed")



############ MQTT ############
# Callback function when a message is received
def on_message(client, userdata, message, config_db):

    # Decode the message payload from bytes to string
    payload_str = message.payload.decode()
    
    # Parse the JSON payload
    payload = json.loads(payload_str)
    
    # Example: Access some fields from the payload
    device_id = payload['end_device_ids']['device_id']
    received_time = payload['received_at']
    temperature = payload['uplink_message']['decoded_payload'].get('temperature', None)
    humidity = payload['uplink_message']['decoded_payload'].get('humidity', None)
    
    print(f"Message received from device: {device_id}")
    print(f"Received at: {received_time}")
    
    # If decoded payload contains temperature and humidity
    if temperature is not None and humidity is not None:
        print(f"Temperature: {temperature}°C, Humidity: {humidity}%")
    else:
        print("Decoded payload is missing temperature or humidity data.")
    
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
    db_json = 'PostgreDB_config.json'
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

    # Start the MQTT client
    client.loop_forever()

    
