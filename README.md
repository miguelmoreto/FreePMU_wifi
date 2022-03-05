
# Dummy FreePMU

This project is part of the FreePMU. It consistis in a ESP32 Module (ESP32 DevKitC) that connects to a WiFi network and implements the PMU communications as defined in IEEE Standard [C37.118.2-2011](//https://standards.ieee.org/ieee/C37.118.1/4902/).

The Dummy FreePMU send randon phasor data at a rate of 30 phasors per second. The phasors sent are the same of the FreePMU:

* 3 fundamental frquency voltage phasors (electrical phases R, S and T)
* 10 harmonic phasors for each electrical phase.

This Dummy FreePMU can be used to test communications infrastructure and the Phasor Data Concentrator (PDC).

The complementary objective is to use this module as a WiFi bridge for the microcontroler where the sychronized phasor estimation takes place.

## Data Frame Organization

The data frame send by FreePMU actualy comprises four "child" PMUs:

* `FreePMU <ID> RST`: contains the magnitude and phase of the 3 fundamental voltage phasors (6 channels).
* `FreePMU <ID+1> H2-5 `: contains the magnitude and phase of the 2nd, 3rd, 4th and 5th harmonic components for each electrical phase (12 channels)
* `FreePMU <ID+2> H6-8 `: contains the magnitude and phase of the 6th, 7th and 8th harmonic components for each electrical phase (9 channels)
* `FreePMU <ID+3> H9-11`: contains the magnitude and phase of the 6th, 7th and 8th harmonic components for each electrical phase (9 channels)

In the fields denoted by `< >` ID is the number of the IDCODE of the data stream source (the FreePMU device ID). Each "child" PMU should have a unique IDCODE, thus the sequential numbering base on the device ID.


_ _ _


In order to create TCP client that communicates with TCP server example, choose one of the following options.

There are many host-side tools which can be used to interact with the UDP/TCP server/client. 
One command line tool is [netcat](http://netcat.sourceforge.net) which can send and receive many kinds of packets. 
Note: please replace `192.168.0.167 3333` with desired IPV4/IPV6 address (displayed in monitor console) and port number in the following command.

In addition to those tools, simple Python scripts can be found under sockets/scripts directory. Every script is designed to interact with one of the examples.

### TCP client using netcat
```
nc 192.168.0.167 3333
```

### Python scripts
Script example_test.py could be used as a counter part to the tcp-server application,
IP address and the message to be send to the server shall be stated as arguments. Example:

```
python example_test.py 192.168.0.167 Message
```
Note that this script is used in automated tests, as well, so the IDF test framework packages need to be imported;
please add `$IDF_PATH/tools/ci/python_packages` to `PYTHONPATH`.

## Hardware Required

This example can be run on any commonly available ESP32 development board.

## Configure the project

```
idf.py menuconfig
```

Set following parameters under Example Configuration Options:

* Set `IP version` of the example to be IPV4 or IPV6.

* Set `Port` number of the socket, that server example will create.

* Set `TCP keep-alive idle time(s)` value of TCP keep alive idle time. This time is the time between the last data transmission.

* Set `TCP keep-alive interval time(s)` value of TCP keep alive interval time. This time is the interval time of keepalive probe packets.

* Set `TCP keep-alive packet retry send counts` value of TCP keep alive packet retry send counts. This is the number of retries of the keepalive probe packet.

Configure Wi-Fi or Ethernet under "Example Connection Configuration" menu. See "Establishing Wi-Fi or Ethernet Connection" section in [examples/protocols/README.md](../../README.md) for more details.

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


## Troubleshooting

Start server first, to receive data sent from the client (application).
