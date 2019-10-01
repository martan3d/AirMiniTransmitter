# AirMiniTransmitter
The project's overall purpose is to provide a low-cost DCC-compatible transmitter (and receiver) using open-source software that operates on [Airwire(TM)](http://www.cvpusa.com/airwire_system.php) channels 0-16 in the 902-928 MHz "ISM" band. 

The project's hardware target is the the ["ProMini Air"](http://blueridgeengineering.net/index.php/wiki/building-the-promini-air/) transmitter/receiver hardware developed by [Blueridge Engineering](http://blueridgeengineering.net). The hardware uses an Arduino Pro Mini controller and a PCB that hosts the Texas Instruments CC1101 transceiver board. Interfacing to an "I2C" LCD for diagnostics is provided.

The ProMini Air transmitter interfaces to any standard DCC throttle/controller through a simple opto-isolator available at Blueridge Engineering and transmits DCC waveforms that can be received by any DCC-compatible receiver on Airwire(TM) channels 0-16. Compatible receivers include the Airwire(TM) CONVRTR; Tam Valley Depot DRS1, Mark III and IV; QSI Solutions GWire; and the ProMini Air receiver. The Airwire(TM) CONVRTR's special requirements for numerous "keep-alive" idle DCC packets are provided by the ProMini Air transmitter's ability to insert these packets into the "garden-variety" DCC packets produced by typical DCC throttles/controllers.

The ProMini transmitter can be configured like any mobile decoder by accessing its DCC address (default: 9000) in "OPS" mode and changing Configuration Variable (CV) values described in the documentation.

The ProMini Air *receiver* shares exactly the same hardware as the ProMini Air transmitter, and changing only the #defines in config.h (in library/AirMiniTransmitter) will create a receiver version of the software. As described at the Blueridge Engineering site, an inexpensive, but powerful [amplifier](http://blueridgeengineering.net/index.php/wiki/cheap-airwire-dead-rail-dcc/) is available to convert the ProMini Air receiver's 5V "logic-level" DCC to "bi-polar" DCC for the onboard DCC decoder.

