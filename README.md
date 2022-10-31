# AirMiniTransmitter
The project's overall purpose is to provide a low-cost DCC-compatible transmitter and receiver using open-source software that operates on [Airwire(TM)](http://www.cvpusa.com/airwire_system.php) channels 0-16 in the 902-928 MHz "ISM" band. A special S-Cab compatible channel 17 has been added to operate in this band. The older Tam Valley Depot DRS, Mk III receiver/amplifier and the NCE D13DRJ wireless decoder will also operate on either this channel or channel 16. Also, the code has now been modified to operate in the European "868MHz" ISM band @869.85MHz, designated Channel 18. We have verified the compatibility of the ProMini Air transmitter/receiver with the Tam Valley Depot DRS 1 European receiver/transmitter operating at this frequency.

The code has been updated to use transceivers that operate in the European 434MHz ISM band to improve range performance using a "repeater." The base station _transmits_ wireless DCC at 434MHz, and a fixed-location repeater, consisting of a 434MHz _receiver_ whose 5V DCC output is directly connected to a 869.85MHz _transmitter_, in turn provides wireless DCC transmissions to mobile 869.85MHz _receivers_ onboard a locomotive. The 434MHz ISM band is not practical as a "stand-along" transmitter-receiver solution because the antennas are too long for mobile receivers. 

Transceiver setting have now been devised and tested in the 2.4GHz ISM band for the TI CC2500 transceiver operating at a base frequency of 2.433GHz. This ISM band has worldwide regulatory approval and provides numerous channels. Currently, the channels are separated by 310kHz and are designated 0-7.

The project's hardware target is the the ["ProMini Air"](https://oscaledeadrail.com/building-the-promini-air-wireless-dcc-transmitter-receiver/) transmitter/receiver hardware developed by [OScaleDeadRail](https://oscaledeadrail.com). The original MCU hardware was an [Arduino Pro Mini controller](https://smile.amazon.com/gp/product/B015MGHLNA/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) and a [PCB that hosts the Texas Instruments CC1101 transceiver](https://smile.amazon.com/coolxan-Wireless-Transceiver-RF1100SE-Antenna/dp/B00MNI4792/ref=pd_rhf_dp_p_img_9?_encoding=UTF8&psc=1&refRID=2KY8W2G0TDXEASJPFMG6) that are widely available. Interfacing to an ["I2C" LCD](https://smile.amazon.com/gp/product/B071XP6PPT/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1) for diagnostics is provided. 

The latest hardware uses an Atmega328P mounted on the PCB with the transceiver. See the [pcb](https://github.com/martan3d/AirMiniTransmitter/tree/master/pcb/EasyEda) directory for the latest [EasyEda](https://easyeda.com/) PCB designs.

The ProMini Air *transmitter* connects to any standard DCC throttle/controller through a very simple dcc converter circuit (2 diodes and one resitor) described in the [Users Manual](https://github.com/martan3d/AirMiniTransmitter/blob/master/doc/AirMini_Users_Manual.pdf) and transmits DCC waveforms that can be received by any DCC-compatible receiver on Airwire(TM) channels 0-16 and the "European" channel 17 (@869.85MHz). Compatible receivers include the Airwire(TM) CONVRTR (Airwire channels 0-16); Tam Valley Depot DRS1, Mark III and IV (Airwire channels 0-16, S-Cab channel 17, and European channel 18 @ 869.85MHz); QSI Solutions GWire (Airwire channels 0-8); and the ProMini Air receiver (Airwire channels 0-16, S-Cab channel 17, and European channel 18 @ 869.85MHz). The ProMini Air transmitter satisfies the special requirements of the Airwire(TM) CONVRTR for numerous "keep-alive" idle DCC packets by "interleaving" these packets into the "garden-variety" DCC packets produced by typical DCC throttles/controllers.

The ProMini Air *transmitter* is easily and inexpensively integrated with a WiFi-equipped [EX-CommandStation](https://dcc-ex.com/ex-commandstation/index.html) developed by the very active [DCC-EX](https://dcc-ex.com/) group as a *completely stand-alone* transmitter solution using a smart phone [throttle app](https://dcc-ex.com/throttles/software/index.html) to connect to the ProMini Air transmtter for dead-rail control. See [this link](https://oscaledeadrail.com/2022/08/28/a-low-cost-wifi-equipped-dcc-base-station-for-the-promini-air-transmitter/) for details.

The ProMini Air *receiver* shares exactly the same PCB as the ProMini Air transmitter (with jumper-only changes for how power is provided), and changing only the #defines in [libraries/config/config.h](https://github.com/martan3d/AirMiniTransmitter/blob/master/libraries/config/config.h) will create a receiver version of the hardware. On start-up, the ProMini Air receiver will search for a valid DCC signal on the default channel (stored in EEPROM). If valid DCC is not found on this channel, then the ProMini Air will search for valid DCC on Airwire channels in the following order: 0, 18(EU), 17, 1, 2, 3, ..., 15, 16. This search will help ensure that if a Tam Valley transmitter is used, that the ProMini Air receiver will "find" it quickly on channel 18(EU) or 17. Depending on the default frequency specified in config.h, if the ProMini Air receiver does not find a valid channel, then it will wait at this this default frequency for a valid transmission. This default channel should usually be 0 (for North America) or 18 (for Europe). 

As described at the Blueridge Engineering site, an inexpensive, but powerful amplifier, the [Cytron MD13S](https://smile.amazon.com/gp/product/B07CW3GRL6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1), is available to convert the ProMini Air receiver's 5V "logic-level" DCC to "bipolar" DCC for the onboard DCC decoder. 

The ProMini transmitter/receiver can be configured like any mobile decoder by accessing its DCC address (default: 9900(Transmitter)/9901(Receiver)) in "OPS" mode and changing Configuration Variable (CV) values described in the documentation. Example settings that can be changed via DCC OPS mode include default transmit/recieve channel (CV255=0-17), transmission power (CV254=0-10), "factory" reset (CV8=8), use short or long address (CV29=0 or 32), short address (CV3), long address (CV17 and CV18). The I2C display, if connected, will be found automatically regardless of its address, which is usually 39 (0x27) or 63 (0x3F).

The project has been updated to operate with CC1101 (or CC110L) transceiver boards that operate at 27MHz (such as the FCC/IC/ETSI-approved Anaren boards) or 26 MHz. Selection is performed by editing [libraries/config/config.h](https://github.com/martan3d/AirMiniTransmitter/blob/master/libraries/config/config.h).

The LiquidCrystal_I2C library in this project has been MODIFIED: a) the LiquidCrystal_I2C declaration now has no arguments, so that it can be declared global in scope without requiring arguments, and b) adding address and sizing arguments to the init method to access EEPROM data, which is by necessity inside a function call. THIS MODIFIED LIBRARY MUST BE USED WITH THIS PROJECT - THE "VANILLA" LiquidCrystal_I2C LIBRARY WILL NOT WORK! Our thanks to the LiquidCrystal_I2C library developers!
