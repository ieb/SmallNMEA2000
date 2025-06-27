#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "SmallNMEA2000.h"


unsigned long getPgnId(unsigned long ID) {
    // CAN ID is 29 bits long
    // 111 1 1 11111111 11111111 11111111
    //                           -------- Source Addresss 8 bits @0
    //                  -------- PDU specific 8 bits @ 8  this is canIdPS
    //         -------- PDU Format 8 bits @ 16 this is canIdPF
    //       - Data Page 1 but @24 this is canIdDP
    //     - reserved 1 bit @25
    // --- priority (3 bits starting @26)

    // PDU Format < 240 is transmitted with a destination address, which is in 8 bits 8
    // PDU Format 240 > 255 is a broadcast with the Group extension in 8 bits @8.
    unsigned char canIdPF = (unsigned char) (ID >> 16); // PDU Format
    unsigned char canIdPS = (unsigned char) (ID >> 8);  // PDU Specific
    unsigned char canIdDP = (unsigned char) (ID >> 24) & 1; // Data Page

    if (canIdPF < 240) {
    /* PDU1 format, the PS contains the destination address */
        return (((unsigned long)canIdDP) << 16) | (((unsigned long)canIdPF) << 8);
    } else {
    /* PDU2 format, the destination is implied global and the PGN is extended */
        return (((unsigned long)canIdDP) << 16) | (((unsigned long)canIdPF) << 8) | (unsigned long)canIdPS;
    }        
};
MessageHeader::MessageHeader(unsigned long ID, unsigned long PGN) {
    pgn = PGN;
    id = ID;
    source = (unsigned char) id & 0xff;
    priority = (unsigned char) ((id >> 26) & 0x7);
    unsigned char canIdPF = (unsigned char) (id >> 16);
    if (canIdPF < 240) {
        destination = (unsigned char) (id >> 8);
    } else {
        destination = 0xff;
    }
};

MessageHeader::MessageHeader(unsigned long PGN, unsigned char Priority, unsigned char Source, unsigned char Destination) {
   // from NMEA2000 code base.
   pgn = PGN;
   priority = Priority;
   source = Source;
   destination = Destination;
   unsigned char canIdPF = (unsigned char) (PGN >> 8);

    if (canIdPF < 240) {  // PDU1 format
        if ( (PGN & 0xff) != 0 ) {
            id = 0;
        } else {
            id = ( ((unsigned long)(priority & 0x7))<<26 | pgn<<8 | ((unsigned long)Destination)<<8 | (unsigned long)Source);
        }
    } else { // PDU2 format
        id = ( ((unsigned long)(priority & 0x7))<<26 | pgn<<8 | (unsigned long)Source);
    }
};

// |-| Priority

// PDU1 format    DDSS
void MessageHeader::print(Print *console, byte *buf, int len) {
     console->print(":");
     console->print(pgn);
     console->print(",");
     console->print(source);
     console->print(",");
     console->print(destination);
     console->print(",");
     console->print(priority);
     console->print(",");
     console->print(len);
     console->print(",");
     console->print(id,HEX);
     for (int i = 0; i < len; i++) {
         console->print(",");
         console->print(buf[i],HEX);
     }
     console->println(";");
}



bool SNMEA2000::open(byte clockSet) {
    if ( canIsOpen ) {
        return true;
    }
    uint8_t res =  CAN.begin(CAN_250KBPS, clockSet);
    if (res == CAN_OK ) {
        // in most cases setting filters on the chip  with NMEA2000 doesnt work
        // where the range of PGNs is enough to set all bits.
        // also the filters are persistant.
        if ( CAN.init_Mask(0,1,0x0)  != MCP2515_OK ) {
            return false;
        }
        if ( CAN.init_Mask(1,1,0x0)  != MCP2515_OK ) {
            return false;
        }

        canIsOpen = true;
        delay(200);
        claimAddress();
        return true;
    } else {
        console->print(F("CAN Fail code:"));
        console->println(res);
    }
    return false;
}




void SNMEA2000::processMessages() {
    if ( ! canIsOpen ) {
        return;
    }
    unsigned char len = 0;
    int frames = 0;
    unsigned char buf[8];
    hasClaimedAddress();
    while(frames < 20 && CAN_MSGAVAIL == CAN.checkReceive()){
        frames++;
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        unsigned long canId = CAN.getCanId();
        unsigned long pgn = getPgnId(canId);
        uint8_t i = 0;
        for (; i < rxListLen; i++) {
            if (rxPGNList[i] == pgn) {
                break;
            }
        }
        if (i == rxListLen) {
            messagesDropped++;
            continue;
        }
        messagesRecieved++;
        MessageHeader messageHeader(canId, pgn);
        if ( diagnostics ) {
            console->print(F("can:"));
            switch(pgn) {
                case 59392L: console->print(F("<a")); break;
                case 59904L: console->print(F("<r")); break;
                case 60928L: console->print(F("<c")); break;
                default: console->print(F("<o")); break;
            }
            messageHeader.print(console, buf, len);
        }
        switch (pgn) {
          case 59904L: /*ISO Request*/
            handleISORequest(&messageHeader, buf, len);
            break;
          case 60928L: /*ISO Address Claim*/
            handleISOAddressClaim(&messageHeader, buf, len);
            break;

          default:
            if ( messageHandler != NULL) {
                messageHandler(&messageHeader, buf, len);
            }
        }
    }
}

//void SNMEA2000::print_uint64_t(uint64_t num) {
// RAM:   [===       ]  32.7% (used 669 bytes from 2048 bytes)
// Flash: [========= ]  92.7% (used 28474 bytes from 30720 bytes)
//  char rev[128]; 
//  char *p = rev+1;
//
//  while (num > 0) {
//    *p++ = '0' + ( num % 10);
//    num/= 10;
//  }
//  p--;
//  /*Print the number which is now in reverse*/
//  while (p > rev) {
//    console->print(*p--);
//  }
//}


void SNMEA2000::handleISOAddressClaim(MessageHeader *messageHeader, byte * buffer, int len) {
    if ( messageHeader->source == 254 ) return; // annother node cannot claim an address, ignore this.
    tUnionDeviceInformation * remoteDeviceInfo = (tUnionDeviceInformation *)(&buffer[0]);
    if ( messageHeader->source == deviceAddress ) {
        // annother device is claiming this address 

        //console->print(F("can: 2 node claiming this address. local:"));
        //print_uint64_t(devInfo->getName());
        //console->print(F(" remote:"));
        //print_uint64_t(remoteDeviceInfo->name);
        //console->print(F(" msg:"));
        //messageHeader->print(console, buffer, len);
        if (devInfo->getName() == remoteDeviceInfo->name ) { 
            // should not happen. This means that we have a collision of 2 of the same devices on the same bus 
            // with the same device instance number in the name.
            // rather than go into an infinite loop both should randomly choose a new instance number.
            // and try again. For some devices, the instance number is used in the messages eg BatteryInstance but for
            // most of devices based on this library thats not the case, and its not currently possible to 
            // set the instance number using NMEA2000 group functions.
            //console->println(F("can: Claim from remote is identical, duplicate device names "));
            uint8_t newInstance = 0xfe & millis();
            devInfo->setDeviceInstanceNumber(newInstance);
            //console->print(F("can: Device Instances set to  "));
            //console->println(newInstance, DEC);
        } else if (devInfo->getName() > remoteDeviceInfo->name ) { 
            // but our name is > callers so we must increment address and claim
            // 
            //console->println(F("can: Claim from remote has higher precidence"));
            deviceAddress++;
        } else {
            //console->println(F("can: Claim from remote has lower precidence"));
        }
        claimAddress();
    }
}

void SNMEA2000::claimAddress() {
    sendIsoAddressClaim();
    addressClaimStarted = millis();
    //console->print(F("Send Address claim for:"));
    //console->print(deviceAddress);
    //console->print(F(" at "));
    //console->println(addressClaimStarted);
}

bool SNMEA2000::hasClaimedAddress() {
    if ( (addressClaimStarted != 0) && 
         (millis()-addressClaimStarted > 250)  ) {
        //console->print(F("Address claimed as "));
        //console->print(deviceAddress);
        //console->print(F(" at "));
        //console->println(millis());
        addressClaimStarted = 0;
    }
    return (addressClaimStarted==0);
}



void SNMEA2000::handleISORequest(MessageHeader *messageHeader, byte * buffer, int len) {
    if ( len < 3 || len > 8) {
        return; // ISO requests are expected to be between 3 and 8 bytes.
    }
    if (messageHeader->destination != 0xff && messageHeader->destination != deviceAddress ) {
        return;
    }
    if (!hasClaimedAddress()) {
        return; // dont respond to queries until address is claimed.
    }
    unsigned long requestedPGN = (((unsigned long )buffer[2])<<16)|(((unsigned long )buffer[1])<<8)|(buffer[0]);
    switch(requestedPGN) {
      case 60928L: /*ISO Address Claim*/  // Someone is asking others to claim their addresses
        sendIsoAddressClaim();
        break;
      case 126464L:
        sendPGNLists(messageHeader);
        break;
      case 126996L: /* Product information */
        sendProductInformation(messageHeader);
        break;
      case 126998L: /* Configuration information */
        sendConfigurationInformation(messageHeader);
        break;
      default:
        if ( isoRequestHandler == NULL || !isoRequestHandler(requestedPGN, messageHeader, buffer, len) ) {
            //console->print(F("Not Known"));
            //console->println(requestedPGN);
            sendIsoAcknowlegement(messageHeader, 1, 0xff);
        }
    }

}




// the PDN lists are 
void SNMEA2000::sendPGNLists(MessageHeader *requestMessageHeader) {
    MessageHeader messageHeader(126464L, 6, deviceAddress, requestMessageHeader->source);
    // 126464L structure is a fast packet sequence with the
    // total length is 1+npgns*3
    sendPGNList(&messageHeader, 0, txPGNList, rxListLen);
    sendPGNList(&messageHeader, 1, rxPGNList, txListLen);
}



void SNMEA2000::sendPGNList(MessageHeader *messageHeader, int listType, const unsigned long *pgnList, uint8_t len ) {
    startFastPacket(messageHeader, 1+len*3);
    outputByte(listType); // RX PGN List
    for(int i = 0; i < len; i++) {
        output3ByteInt(pgnList[i]);
    }
    finishFastPacket();
}




void SNMEA2000::sendIsoAddressClaim() {
  MessageHeader messageHeader(60928L, 6, deviceAddress, 0xff);
  // CAN and AVR are little endian, so this is ok.
  sendMessage(&messageHeader,devInfo->getDeviceNameBuffer(), 8);
}



void SNMEA2000::sendProductInformation(MessageHeader *requestMessageHeader) {
    MessageHeader messageHeader(126996L, 6, deviceAddress, requestMessageHeader->source);

    // this is a fast packet, has to be constructed from the struc on the fly.
    startFastPacket(&messageHeader, 4+32*4+2);
    output2ByteUInt(pgm_read_word(&productInfo->nk2version));
    output2ByteUInt(pgm_read_word(&productInfo->productCode));
    outputFixedString(pgm_read_ptr(&productInfo->modelID), 32, 0xff);
    outputFixedString(pgm_read_ptr(&productInfo->softwareVersion), 32, 0xff);
    outputFixedString(pgm_read_ptr(&productInfo->modelVersion), 32, 0xff);
    outputFixedString(pgm_read_ptr(&productInfo->serialNumber), 32, 0xff);
    outputByte(pgm_read_byte(&productInfo->certificationLevel));
    outputByte(pgm_read_byte(&productInfo->loadEquivalency));
    finishFastPacket();
}


void SNMEA2000::sendConfigurationInformation(MessageHeader *requestMessageHeader) {
    MessageHeader messageHeader(126998L, 6, deviceAddress, requestMessageHeader->source);
    // this is a fast packet.
    char * manufacturerInfo = pgm_read_ptr(&configInfo->manufacturerInfo);
    char * installDesc1 = pgm_read_ptr(&configInfo->installDesc1);
    char * installDesc2 = pgm_read_ptr(&configInfo->installDesc2);
    
    int manufacturerInfoLen = strnlen(manufacturerInfo,70);
    int installDesc1Len = strnlen(installDesc1,70);
    int installDesc2Len = strnlen(installDesc2,70);
    startFastPacket(&messageHeader, manufacturerInfoLen+installDesc1Len+installDesc2Len+6 );
    outputVarString(manufacturerInfo, manufacturerInfoLen);
    outputVarString(installDesc1, installDesc1Len);
    outputVarString(installDesc2, installDesc2Len);
    finishFastPacket();
}

void SNMEA2000::outputVarString(const char * str,  uint8_t strLen) {
    outputByte(strLen+2);
    outputByte(0x01);
    for(int i = 0; i < strLen; i++) {
        outputByte(str[i]);
    }
}

void SNMEA2000::outputFixedString(const char * str, int maxLen, byte padding) {
    int ib = 0;
    while( ib < maxLen ) {
        byte bs = str[ib++];
        outputByte(bs);
        if (bs == '\0') break;
    }
    while (ib < maxLen) {
        outputByte(padding);
        ib++;
    }
}

void SNMEA2000::output2ByteUInt(uint16_t i) {
    outputByte(i&0xff);
    outputByte((i>>8)&0xff);
}
void SNMEA2000::output2ByteInt(uint16_t i) {
    outputByte(i&0xff);
    outputByte((i>>8)&0xff);
}
void SNMEA2000::output3ByteInt(int32_t i) {
    outputByte(i&0xff);
    outputByte((i>>8)&0xff);
    outputByte((i>>16)&0xff);
}
void SNMEA2000::output3ByteUInt(uint32_t i) {
    outputByte(i&0xff);
    outputByte((i>>8)&0xff);
    outputByte((i>>16)&0xff);
}
void SNMEA2000::output2ByteDouble(double value, double precision) {
    if (value == SNMEA2000::n2kDoubleNA ) {
        // udefined is 32767 = 0x7FFF
        outputByte(0xff);
        outputByte(0x7f);
    } else {
        double vd=value/precision;
        vd = round(vd);
        int16_t i = (vd>=-32768 && vd<0x7fee)?(int16_t)vd:0x7fee;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
    }
}
void SNMEA2000::output2ByteUDouble(double value, double precision) {
    // indef is 65535U = 0xFFFF
    if (value == SNMEA2000::n2kDoubleNA) {
        outputByte(0xff);
        outputByte(0xff);
    } else {
        double vd=value/precision;
        vd = round(vd);
        uint16_t i = (vd>=0 && vd<0xfffe)?(int16_t)vd:0xfffe;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
    }

}

void SNMEA2000::output3ByteDouble(double value, double precision) {
    if (value == SNMEA2000::n2kDoubleNA ) {
        // undef is 2147483647 = 7FFFFF
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0x7f);
    } else {
        double vd=value/precision;
        vd = round(vd);
        int32_t i = (vd>=-8388608L && vd<8388606L)?(int32_t)vd:8388606L;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
        outputByte((i>>16)&0xff);
    }
}
void SNMEA2000::output3ByteUDouble(double value, double precision) {
    if (value == SNMEA2000::n2kDoubleNA ) {
        // undef is 2147483647 = FFFFFF
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0xff);
    } else {
        double vd=value/precision;
        vd = round(vd);
        int32_t i = (vd>=0 && vd<16777214L)?(int32_t)vd:16777214L;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
        outputByte((i>>16)&0xff);
    }
}

void SNMEA2000::output4ByteDouble(double value, double precision) {
    if (value == SNMEA2000::n2kDoubleNA ) {
        // undef is 2147483647 = 7FFFFFFF
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0x7f);
    } else {
        double vd=value/precision;
        vd = round(vd);
        int32_t i = (vd>=-2147483648L && vd<2147483647L)?(int32_t)vd:2147483647L;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
        outputByte((i>>16)&0xff);
        outputByte((i>>24)&0xff);
    }
}






void SNMEA2000::output4ByteUDouble(double value, double precision) {
    if (value == SNMEA2000::n2kDoubleNA ) {
        // undef is 4294967295U = FFFFFFFF
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0xff);
        outputByte(0xff);
    } else {
        double vd=value/precision;
        vd = round(vd);
        uint32_t i = (vd>=0 && vd<0xfffffffe)?(uint32_t)vd:0xfffffffe;
        outputByte(i&0xff);
        outputByte((i>>8)&0xff);
        outputByte((i>>16)&0xff);
        outputByte((i>>24)&0xff);
    }
}








void SNMEA2000::startPacket(MessageHeader *messageHeader) {
    packetMessageHeader = messageHeader;
    ob = 0;
    fastPacket = false;
} 
void SNMEA2000::finishPacket() {
    if ( !fastPacket ) {
        if ( ob > 0) {
            sendMessage(packetMessageHeader, buffer, ob);
        }
    } else {
        packetErrors++;
        console->println(F("Error: Not a Single Packet"));
    }
}


void SNMEA2000::startFastPacket(MessageHeader *messageHeader, int length) {
    packetMessageHeader = messageHeader;
    fastPacket = true;
    fastPacketSequence++;
    if (fastPacketSequence > 7 ) {
        fastPacketSequence = 0;
    }
    frame = 0;
    ob = 0;
    buffer[ob++] =  (fastPacketSequence << 5) | frame++;
    buffer[ob++] = length;
    fastPacketLength = length;
    fastPacketSent = 0;
} 

void SNMEA2000::checkFastPacket() {
    if ( fastPacket ) {
        if ( fastPacketLength > 223 ) {
            console->print(F("Error: fastpacket too long:"));
            console->println(fastPacketLength);
        }
        if ( fastPacketSent != fastPacketLength ) {
            console->print(F("Error: FastPacket length wrong wanted:"));
            console->print(fastPacketLength);
            console->print(F(" got:"));
            console->println(fastPacketSent);
        } 
    } else {
        console->println(F("not fast packet"));
    }
}


void SNMEA2000::finishFastPacket() {
    if (fastPacket ) {
        if (ob > 1) {
            // send remaining frame
            sendMessage(packetMessageHeader, buffer, ob);
        }
    } else {
        packetErrors++;
        console->println(F("Error: Not a FastPacket"));
    }
}

void SNMEA2000::outputByte(byte opb) {
    if ( ob < 8 ) {
        buffer[ob++] = opb;
        fastPacketLength--;
        if( fastPacket && ob == 8) {
            sendMessage(packetMessageHeader, &buffer[0], 8);
            ob = 0;
            buffer[ob++] = (fastPacketSequence << 5) | frame++;
        }
    } else {
        frameErrors++;
        console->println(F("Error: Frame > 8 bytes"));
    }
}

/*
void SNMEA2000::sendFastPacket(MessageHeader *messageHeader, byte *message, int len, bool progmem) {
    startFastPacket(messageHeader, len);
    int ib = 0;
    while(ib < len) {
        if ( progmem ) {
            outputByte(pgm_read_byte(&message[ib++]));
        } else {
            outputByte(message[ib++]);
        }
    }
    finishFastPacket();
}
*/

void SNMEA2000::sendIsoAcknowlegement(MessageHeader *requestMessageHeader, unsigned char control, unsigned char groupFunction) {
    MessageHeader messageHeader(59392L, 6, deviceAddress, requestMessageHeader->source);
    startPacket(&messageHeader);
    outputByte(control);
    outputByte(groupFunction);
    outputByte(0xff);
    outputByte(0xff);
    outputByte(0xff);
    output3ByteInt(requestMessageHeader->pgn);
    finishPacket();
}

void SNMEA2000::sendMessage(MessageHeader *messageHeader, byte * message, int length) {
    if ( ! canIsOpen ) {
        return;
    }
    uint8_t res = CAN.sendMsgBuf(messageHeader->id, 1, length, message);
    if ( res != CAN_OK ) {
        console->print(F("can: err"));
        console->println(res);
    }
    //if ( diagnostics ) {
    //    console->print(F("can: out>"));
    //    messageHeader->print(console, message,length);
    //}
    messagesSent++;

}



void EngineMonitor::sendRapidEngineDataMessage(
    byte engineInstance, 
    double engineSpeed, 
    double engineBoostPressure, 
    byte engineTiltTrim) {
    MessageHeader messageHeader(127488L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(engineInstance);
    output2ByteUDouble(engineSpeed,0.25);
    output2ByteUDouble(engineBoostPressure,100);
    outputByte(engineTiltTrim);
    outputByte(0xff);
    outputByte(0xff);
    finishPacket();
}

void EngineMonitor::sendEngineDynamicParamMessage(
    byte engineInstance,
    double engineHours,
    double engineCoolantTemperature,
    double alternatorVoltage,
    uint16_t status1,
    uint16_t status2,
    double engineOilPressure,
    double engineOilTemperature,
    double fuelRate,
    double engineCoolantPressure,
    double engineFuelPressure,
    byte engineLoad,
    byte engineTorque
    
) {
    MessageHeader messageHeader(127489L, 2, getAddress(), SNMEA2000::broadcastAddress);
    startFastPacket(&messageHeader, 26);
    outputByte(engineInstance);
    output2ByteUDouble(engineOilPressure,100);
    output2ByteUDouble(engineOilTemperature,0.1);
    output2ByteUDouble(engineCoolantTemperature,0.01);
    output2ByteDouble(alternatorVoltage,0.01);
    output2ByteDouble(fuelRate,0.1);
    output4ByteUDouble(engineHours,1);
    output2ByteUDouble(engineCoolantPressure,100);
    output2ByteUDouble(engineFuelPressure,1000);
    outputByte(0xff); // reserved
    output2ByteUInt(status1);
    output2ByteUInt(status2);
    outputByte(engineLoad);
    outputByte(engineTorque);
    finishFastPacket();

}
void EngineMonitor::sendDCBatterStatusMessage(
    byte batteryInstance, 
    byte sid,
    double batteryVoltage,
    double batteryTemperature,
    double batteryCurrent
    ) {
    MessageHeader messageHeader(127508L, 6, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(batteryInstance);  
    output2ByteDouble(batteryVoltage,0.01);
    output2ByteDouble(batteryCurrent,0.1);
    output2ByteUDouble(batteryTemperature,0.01);
    outputByte(sid);
    finishPacket();
}
void EngineMonitor::sendFluidLevelMessage(
    byte type,
    byte instance,
    double level, 
    double capacity) {
    MessageHeader messageHeader(127505L, 6, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    byte b = ((type<<4)&0xff)|(instance&0xff);
    outputByte(b);
    output2ByteDouble(level,0.004);
    output4ByteUDouble(capacity,0.1);
    outputByte(0xff);
    finishPacket();
}
void EngineMonitor::sendTemperatureMessage(
    byte sid, 
    byte instance,
    byte source,
    double actual,
    double requested) {
    MessageHeader messageHeader(130312L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    outputByte(instance);
    outputByte(source);
    output2ByteUDouble(actual,0.01);
    output2ByteUDouble(requested,0.01);
    outputByte(0xff);
    finishPacket();
}



void PressureMonitor::sendOutsideEnvironmentParameters(
        byte sid, 
        double waterTemperature,
        double outsideAirTemperature,
        double atmospheicPressure
        ) {
    MessageHeader messageHeader(130310L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    output2ByteUDouble(waterTemperature,0.01);
    output2ByteUDouble(outsideAirTemperature,0.01);
    output2ByteUDouble(atmospheicPressure,100);
    outputByte(0xff);
    finishPacket();
}

void PressureMonitor::sendEnvironmentParameters(
        byte sid,
        double atmosphericPressure, // Atomspheric Pressure 
        byte tempSource,
        double temperature, // temperature 
        byte humiditySource,
        double humidity // humidity
        ) {
    MessageHeader messageHeader(130311L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    outputByte(((humiditySource) & 0x03)<<6 | (tempSource & 0x3f));
    output2ByteUDouble(temperature,0.01);
    output2ByteDouble(humidity,0.004);
    output2ByteUDouble(atmosphericPressure,100);
    finishPacket();
}


void PressureMonitor::sendHumidity(byte sid, byte humiditySource, byte humidityInstance, double humidity) {
    MessageHeader messageHeader(130313L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    outputByte(humidityInstance);
    outputByte(humiditySource);
    output2ByteDouble(humidity,0.004);
    output2ByteDouble(SNMEA2000::n2kDoubleNA,0.004);
    outputByte(0xff);
    finishPacket();
}

void PressureMonitor::sendPressure(byte sid, byte pressureSource, byte pressureInstance, double pressure ) {
    MessageHeader messageHeader(130314L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    outputByte(pressureInstance);
    outputByte(pressureSource);
    output4ByteDouble(pressure,0.1);
    outputByte(0xff);
    finishPacket();
}

void PressureMonitor::sendTemperature(byte sid, byte temperatureSource,  byte temperatureInstance, double tremperature ) {
    MessageHeader messageHeader(130316L, 5, getAddress(), SNMEA2000::broadcastAddress);
    startPacket(&messageHeader);
    outputByte(sid);
    outputByte(temperatureInstance);
    outputByte(temperatureSource);
    output3ByteUDouble(tremperature,0.001);
    output2ByteUDouble(SNMEA2000::n2kDoubleNA,0.1);
    finishPacket();
}

