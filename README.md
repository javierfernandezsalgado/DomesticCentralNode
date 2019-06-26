# DomesticCentralNode
The Domestic central Node is in charge of recollect the data of a different set of sensors
## Introduction
The DomesticCentralNode is a part of a Domotic project. It is in charge of the sensor management. 
Its role is collect the information of the sensor subscribed to the central node and develop a complete Sensors API. Some of functionalities are described below:
- Buffering sensor information
- Getting the sensor status
- Defining complete sensor APIS. Currenlty It is provided an API for Temperature sensor and the Rele.

## Hardware

The hardware is a LOLIN board. It is based on ESP8266. Its interface are:
 - Integrated Wifi device
 - Ble device: It has been added.
 
 ## Domotic Architecture Element
 
 The DomesticCentralNode is connected to the sensors throught the BLE device. The central node provided its interface throught the Wifi interface.

