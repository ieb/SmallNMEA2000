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


