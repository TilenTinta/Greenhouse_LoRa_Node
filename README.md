# Greenhouse LoRa Node

**Greenhouse LoRa Node** is an open-source monitoring system designed for use in greenhouses, orchards or other agricultural environments. This project includes all the necessary components to build the system, including:

- A custom PCB for the node (supports LoRa or SIM card communication),
- Firmware for the PCB,
- PCB casing,
- Server terminal application,
- GUI template using Grafana.

![Greenhouse LoRa Node](https://github.com/TilenTinta/Greenhouse_LoRa_Node/blob/main/Images/main_page_img.jpg)

---

## PCB

The PCB is a two-layer board that integrates essential hardware components. It is versatile and supports different communication modules. Users can choose between a LoRa module or a SIM card module (both cannot be used simultaneously). 

### Features
- Two LDO voltage regulators (3.3V and 4V, with 4V dedicated to the SIM module),
- Battery voltage monitoring,
- Battery cell holder for 18650 cells,
- Three connectors:
  - Earth humidity probe
  - AUX1 (I2C interface)
  - AUX2 (SPI interface),
- Support for LoRa module (RFM95W / SX1276) or SIM module (SIM800L),
- u.FL antenna connector,
- BME280 sensor (measures air temperature, humidity, and pressure),
- MCU: STM32F103C8T6 with RTC,
- 10-pin JTAG programming connector.

---

## Firmware

The firmware is written entirely in C using STM CubeIDE. It is designed to be easily updated or modified. The MCU supports multiple sleep modes for power optimization, which can be configured directly in the code.

The project does not include the `keys.h` file, as it contains user-specific information. The structure of the `keys.h` file is as follows:

```c
#ifndef KEYS_H_
#define KEYS_H_

#define DEVICE_ID            1       // ID number of the node

#define DEVICE_ADDRESS       {0x01, 0x23, 0x45, 0x56}
#define APPSKEY              {0x01, 0x23, 0x45, 0x67, 0x89, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20}
#define NWKSKEY              {0x01, 0x23, 0x45, 0x67, 0x89, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20}

#endif
```

The library for the RFM95W LoRa module is based on the repository linked below, with slight modifications made for this project: [https://github.com/henriheimann/stm32-hal-rfm95](https://github.com/henriheimann/stm32-hal-rfm95)

> [!CAUTION]
> When using the **RFM95W LoRa module** and the graphical peripheral editor in **CubeIDE**, additional code modifications are required. After saving changes in the IDE, the auto-generated code will be overwritten, re-enabling all IRQs by default. 
> The **RFM95W module** generates a 1 MHz digital signal on one of its IRQ pins at boot. If interrupts are not properly initialized, this can trigger errors in the MCU. To prevent this issue, **all interrupts must be disabled until the module is fully initialized**.

---

## Case

The PCB case is 3D printed using PETG filament on a standard FDM 3D printer. When deploying the node outdoors, ensure the filament can withstand UV exposure. Materials like ASA or ABS might be more suitable for such environments.

### Design Features:
- The case has a dedicated section with perforations to allow airflow near the sensor, enabling accurate air parameter measurements while protecting the rest of the PCB from external conditions.
- A groove runs along the perimeter of the lower part of the case, designed to fit a 3D-printed seal made from flexible materials like TPU or TPE. This ensures a better seal to protect the internal components.


### Notes:
- The design is optimized for easy replication using commonly available 3D printing materials and techniques.
- Consider using higher-temperature and UV-resistant materials for long-term outdoor use.

---

## Server Application

The server application is entirely written in Python and is terminal-based. Every operation performed by the application is logged into a log file for future inspection in case of issues. 

### Features
- Connects to your LoRaWAN devices on TTN via MQTT.
- Configurable `ttn_config.json` for easy integration.
- Configurable `InfluxDB_config.json` for easy integration.
- Supports multiple devices under a single application.

### Configuration Files

Upon boot, the application requires two JSON files to load all the configurations for the database and MQTT connection to the TTN broker:

1. **`InfluxDB_config.json`**  
   Configuration file for connecting to the InfluxDB database:
   ```json
   {
       "_comment": "JSON config file for database connection",
       "url": "http://192.168.X.X:8086",
       "org": "database",
       "token": "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
       "bucket": "bucket_name",
       "timezone": "Europe/Ljubljana"
   }
    ```

2. **`ttn_config.json`** 
    ```json
    {
        "_comment": "JSON config file for mqttTTN connection",
        "app_id": "lora-home-node",
        "username": "lora-home-node@ttn",
        "access_key": "XXXXXXXXXXXXXXXXXXXXXXXXXXX.XXXXXXXXXXXXXXXXXXXXXXXXX",
        "mqtt_broker": "eu1.cloud.thethings.network",
        "mqtt_port": 1883,
        "topic": "v3/lora-home-node@ttn/devices/+/up" 
    }
    ```

### Prerequirements

Before running the application, ensure you have the required dependencies installed. The following packages are necessary:

    ```bash
        pip install json paho-mqtt psycopg2 influxdb-client pytz logging socket base64
    ```

---

## Database

This project utilizes **InfluxDB** as its primary database solution. InfluxDB was selected due to its high efficiency in handling time-series data and its seamless integration with **Grafana**, which simplifies data visualization and analysis. By leveraging InfluxDB's ability to store data in a **time-series format**, this project ensures scalability for managing and analyzing temporal data.

### Notes:
- Ensure that InfluxDB is properly configured to handle the expected volume of data from the monitoring nodes.


---

## Grafana

This repository contains pre-configured Grafana UI files designed to read and present data from sensors in an intuitive and visually appealing way. These files provide a basic setup to help you quickly get started with monitoring and analyzing your sensor data using Grafana's powerful dashboard capabilities.

### Requirements:
- **Grafana** and **InfluxDB** should be installed on either a Raspberry Pi or a home server (depending on your setup).
- Ensure the terminal application is run on the same machine where the database and Grafana are installed.


