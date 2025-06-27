# Small NMEA2000 Library.

This is Smaller NMEA2000 library for MCP2515 driven by AVR (328p) targetting smaller devices. It requries just under 1K of ram and 12K flash to operate. It is only really intended for small NMEA2000 sensor devices that send readings onto the bus and reply to standard ISO requests. For all other use cases, probably better to use a chip with more ram and flash and use the NMEA2000 library as is.

It supports the standard ISO Request messages for address claim and negotiation as well as Product Info, Product Configuration and PGN Lists. It has support for fast and sandard 8 bit packets, but no other support.

It is intended to send directly onto the can bus from PROGMEM or by calls. ProductInfo, Product COnfiguration and PGN Lists should all be defined in PROGMEM to save RAM space.

To use for a specific purpose extend the SNMEA2000 class and add the send message methods required. See the EngineManagement class for an example.

Much of the code was inspired from the NMEA2000 library (https://github.com/ttlappalainen/NMEA2000)  including what needed to be supported. Without that guidance this library would have taken many more hours to create.

Also a lot of guidance from https://canboat.github.io/canboat/canboat.html

For a fully fleged NMEA2000 library, use https://github.com/ttlappalainen/NMEA2000. This library is only intended for MCUs with < 3K RAM eg Attiny3224.

eg A NMEA2000 Battery Shunt which needs about 400 bytes of RAM to operate.

RAM:   [=         ]  12.6% (used 388 bytes from 3072 bytes)
Flash: [=======   ]  70.5% (used 23100 bytes from 32768 bytes)


# Address claim problems discovered

The code was incorrectly emitting address claims in response to all address claim messages rather than just where the address was claimed by 2 devices. This caused a race condition and with more devices on the bus using this library the race condition got worse. In addition the decoding of the Can device name (uint64_t) had a bug that caused the remote name to always have a higher precidence resulting in these devices always assuming their address was claimed by another, hence these devices. Other defices would see it the other way round and hence these devices would sweep up all the addresses on the bus.

For some devices on the bus this would cause a reset resulting in the first message on the new address having incomplete data. Eg Em-Track B921 devices report no GPS Fix for the first message on a new address. This caused MFDs to report lost fix and in some cases lost AIS data.

The fixes. 

* Only emit claim responses when the addresses are claimed by 2 devices.
* parse the address claim message completely. Its a 8 byte little endian uint_64 and can be set by pointing to the message buffer on a little endian CPU.
* Where the names are identical, due to incorrect configuration if the device instance field, ramdomly select a new device instance field in the CAN device name to recolve the conflict. Since the Can Name device instance field is not generally used
to indicate the physical measurement instance (eg Battery Instance), and cant be set (no support for setting it over Can Group Functions), this doesnt matter.  


# Testing

Using a CandelLite USB-CAN bus adapter with socket can on a linux box. see testscripts/

# references

Details of ISO Address Claim https://copperhilltech.com/blog/sae-j1939-address-management-messages-request-for-address-claimed-and-address-claimed/

# Changes

## 20250627

Was found that the mast and filters in the chip were non functional blocking all traffic after address claim. Since in most cases with NMEA2000 the range of packets recieved is too great to be accomidated in the filters, filtering has been moved into code. This may not be fast enough with full busload and needs to be tested. Dropped packets in the chip may not matter so much. In addition the rxPGNList and txPGNList must now be in ram not PROGMEM to allow rapid checks. The length of these must also be supplied. This will mean some code changes after this change.


# ToDO

* [x] Fix address claim race conditions
* [x] Fix incorrect name encoding and decoding
* [x] Deal with name clashes on address claims
* [x] Test triggering address claim 
* [x] Fix some errors in unsigned and signed messages, firmware updates not required, see previous commit.
* [x] Drop non functional register level filters and replace with recieved list checks. Note, this may not be fast enough.


