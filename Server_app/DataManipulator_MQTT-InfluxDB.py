"""
Project: LoRa Node for the Greenhouse
Author: Tilen Tinta
Date: December 2024
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
import base64


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
        utc_time = datetime.now(tz=pytz.utc)
        my_time = utc_time.astimezone(time_zone)

        point = (
            Point(config["bucket"])
            .tag("device_id", str(data['device_id']))  # Tag to differentiate devices
            .field("error_flag", data['error_flag'])
            .field("error_send_count", data['error_send_count'])
            .field("battery_voltage", data['battery_voltage'])
            .field("air_temperature", data['air_temperature'])
            .field("air_humidity", data['air_humidity'])
            .field("air_pressure", data['air_pressure'])
            .field("earth_humidity", data['earth_humidity'])
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
def on_message(client, userdata, message):

    config_db = userdata.get("config_db")  # Retrieve config_db from userdata

    try:
        # Parse the JSON payload
        payload = json.loads(message.payload.decode())

        # Decode the LoRa payload
        lora_payload_base64  = payload['uplink_message']['frm_payload']  # Assuming it's a hex string

        try:
            lora_payload_bytes = base64.b64decode(lora_payload_base64)
        except Exception as e:
            print(f"Error decoding Base64 payload: {e}")
            return

        # Detect message type
        message_type = detect_message_type(lora_payload_bytes)      

        # Decode the LoRa payload if it's not a test-boot message
        if message_type == 0:
            decoded_payload = decode_lora_payload(lora_payload_bytes.hex())

            # Print the decoded payload for debugging
            print("Decoded LoRa payload:")
            print(json.dumps(decoded_payload, indent=4))
            logging.info("Decoded LoRa payload...") # Write log
            
            # Save to the database
            save_to_db(config_db, decoded_payload)

        elif message_type == -1:
            print("Test-Boot message received, not decoding, not saving.")
            logging.info("Test-Boot message received, not decoding, not saving.") # Write log


        elif message_type == -2:
            print("Blank message received, ignoring (Boot).")
            logging.info("Blank message received, ignoring (Boot).") # Write log

    except Exception as e:
        print(f"Error processing message: {e}")
    


############ LoRa decoder ############
# Detect if the packet is boot-test message or real
def detect_message_type(data_packet):
    if not data_packet: # Empty packet (program start)
        return -2  
    if data_packet[:4] == bytes([0x01, 0x02, 0x03, 0x04]): # Boot-test packet
        return -1  
    return 0 # OK packet


def decode_lora_payload(data_packet_hex):
    # Convert hex string to bytes
    data_packet = bytes.fromhex(data_packet_hex)

    decoded_data = {
        'device_id': data_packet[0],
        'error_flag': data_packet[1],
        'error_send_count': data_packet[2],
        'battery_voltage': data_packet[3] / 10.0, # V
        'air_temperature': int.from_bytes(data_packet[4:8], byteorder='little', signed=True) / 100, # C
        'air_humidity': data_packet[8], # %
        'air_pressure': int.from_bytes(data_packet[9:13], byteorder='little', signed=False) / 100, # hPa
        'earth_humidity': data_packet[13], # %
    }

    return decoded_data


# Callback function when the client connects
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        topic = str(config_ttn["topic"])
        client.subscribe(topic)
        print(f"Subscribed to topic: {config_ttn['topic']}")
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
    client = mqtt.Client(userdata={"config_db": config_db})
    client.on_connect = on_connect
    client.on_message = on_message
    app_id = str(config_ttn["app_id"])+"@eu1"
    acc_key = str(config_ttn["access_key"])
    client.username_pw_set(app_id, acc_key)
    #client.tls_set()  # Enables SSL/TLS


    try:
        broker = str(config_ttn["mqtt_broker"])
        port = int(config_ttn["mqtt_port"])
        client.connect(broker, port, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("Disconnecting from MQTT broker...")
        client.disconnect()

    

    

    
