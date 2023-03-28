# esp32_controller
Generic application for an IOT device based on ESP32 controller
## Introduction
The purpose of this application is to provide a small framework to implement different IOT controller devices.<br>
The application provided here run on [Esspressif ESP32 module](https://www.espressif.com) and the development environment is using [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)<br>
It is built in Eclipse CDT 2022-09 with ESP-IDF plug-in, based on esp-idf-v4.4.3<br>
The fwk itself is putting together several modules provided in esp-idf (there are plenty of them) and adapted them to my purpose.<br> All these modules are part of the [esp32_common]( https://github.com/ves011/esp32_common) repository.<br>
Beside the fwk you need to implement control logic for specific sensors, or actuators, or... you need to monitor and control.
<br><br>
To control the application you need a MQTT boroker (in my case Mosquitto)<br>
The fwk connects to the broker and subcribes to predefined topics:
- [...]/ctrl -- any message recieved on this topic goes to cmds modules
- [...]/cmd -- any message received on this topic goes to the control logic
- [network name]/query -- used by external clients to identify which IOT devices are alive in the network<br>
<br>
In response to a received message send a response by publishing the result on <br>

- [...]/state - for messages received on [...]/cmd
- [network name]/response - for messages received on [network name]/query
- [...]/monitor topic is used by the control logic in case it has to publish unsolicited (URC) device events<br>

[network name] is a generic name for IOT network hosting the devices<br>
[...] is the name of each individual dvice. IT HAS TO BE UNIQUE in the network!<br>
