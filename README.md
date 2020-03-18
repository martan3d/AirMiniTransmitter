# AirMiniTransmitter
The project's overall purpose is to provide a low-cost DCC-compatible transmitter and receiver using open-source software that operates on [Airwire(TM)](http://www.cvpusa.com/airwire_system.php) channels 0-16 in the 902-928 MHz "ISM" band. The code has now been modified to also operate in the European "868MHz" ISM band @869.85MHz, designated Channel 17. We have verified the compatibility of the ProMini Air transmitter/receiver with the Tam Valley Depot DRS 1 European receiver/transmitter operating at this frequency.

The project's hardware target is the the ["ProMini Air"](http://blueridgeengineering.net/index.php/wiki/assembling-the-promini-air-receiver-transmitter/) transmitter/receiver hardware developed by [Blueridge Engineering](http://blueridgeengineering.net). The hardware uses an [Arduino Pro Mini controller](https://smile.amazon.com/gp/product/B015MGHLNA/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) and a [PCB that hosts the Texas Instruments CC1101 transceiver](https://smile.amazon.com/coolxan-Wireless-Transceiver-RF1100SE-Antenna/dp/B00MNI4792/ref=pd_rhf_dp_p_img_9?_encoding=UTF8&psc=1&refRID=2KY8W2G0TDXEASJPFMG6) that are widely available. Interfacing to an ["I2C" LCD](https://smile.amazon.com/gp/product/B071XP6PPT/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) for diagnostics is provided.

The ProMini Air *transmitter* connects to any standard DCC throttle/controller through a very simple dcc converter circuit (2 diodes and one resitor) described in the [Users Manual](https://github.com/martan3d/AirMiniTransmitter/blob/master/doc/AirMini_Users_Manual.pdf) and transmits DCC waveforms that can be received by any DCC-compatible receiver on Airwire(TM) channels 0-16. Compatible receivers include the Airwire(TM) CONVRTR; Tam Valley Depot DRS1, Mark III and IV; QSI Solutions GWire; and the ProMini Air receiver. The ProMini Air transmitter satisfies the special requirements of the Airwire(TM) CONVRTR for numerous "keep-alive" idle DCC packets by "interleaving" these packets into the "garden-variety" DCC packets produced by typical DCC throttles/controllers.

The ProMini Air *receiver* shares exactly the same PCB as the ProMini Air transmitter (with jumper-only changes for how power is provided), and changing only the #defines in [libraries/config/config.h](https://github.com/martan3d/AirMiniTransmitter/blob/master/libraries/config/config.h) will create a receiver version of the hardware. On start-up, the ProMini Air receiver will search for a valid DCC signal on the default channel (stored in EEPROM and is 0 by default). If valid DCC is not found on this channel, then the ProMini Air will search for valid DCC on Airwire channels in the following order: 0, 17(EU), 16, 1, 2, 3, ..., 15. This search will help ensure that if a Tam Valley transmitter is used, that the ProMini Air receiver will "find" it quickly on channel 17(EU) or 16. Depending on the default frequency specified in config.h, if the ProMini Air receiver does not find a valid channel, then it will wait on this this default frequency for a valid transmission. This default channel should usually be 0 (for North America) or 17 (for Europe). 

As described at the Blueridge Engineering site, an inexpensive, but powerful [amplifier](http://blueridgeengineering.net/index.php/wiki/cheap-airwire-dead-rail-dcc/) (the [Cytron MD13S](https://smile.amazon.com/gp/product/B07CW3GRL6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)) is available to convert the ProMini Air receiver's 5V "logic-level" DCC to "bipolar" DCC for the onboard DCC decoder. 

The ProMini transmitter/receiver can be configured like any mobile decoder by accessing its DCC address (default: 9000/9001) in "OPS" mode and changing Configuration Variable (CV) values described in the documentation.

The project has been updated to operate with CC1101 (or CC110L) transceiver boards that operate at 27MHz (such as the FCC/IC/ETSI-approved Anaren boards) or 26 MHz. Selection is performed by editing [libraries/config/config.h](https://github.com/martan3d/AirMiniTransmitter/blob/master/libraries/config/config.h).

The LiquidCrystal_I2C library in this project has been MODIFIED with a change to a parameterless LiquidCrystal_I2C declaration, so that it can be global in scope, and modificaiton of the init method with address and sizing parameters to allow using EEPROM data, which is by necessity inside a function call. THIS MODIFIED LIBRARY MUST BE USED WITH THIS PROJECT - THE "VANILLA" LiquidCrystal_I2C LIBRARY WILL NOT WORK! Our thanks to the LiquidCrystal_I2C library authors!
