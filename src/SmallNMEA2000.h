#ifndef SmallNMEA2000_H
#define SmallNMEA2000_H

#include <mcp_can.h>


#define CToKelvin(x) (x+273.15)


typedef struct MsgHeader {
    unsigned char Priority;
    unsigned long PGN;
    unsigned char Source;
    unsigned char Destination;

} MsgHeader;

/**
  Example of the product Information buffer and configuration information buffer.
  This byte[] is sent as is in fastPacket messages. 
byte productInformationBuffer[] PROGMEM = {
    N2K_VERSION_2BYTEINT,
    PRODUCT_CODE_2BYTEINT,
    MODEL_ID_BYTEFF, 0xff,
    SWCODE_BYTEFF, 0xff,
    MODEL_VERSION_BYTEFF, 0xff,
    SERIAL_CODE_BYTEFF, 0xff,
    CERTIFICATION_LEVEL_BYTE,
    LOAD_EQUIV_BYTE
};

byte configurationInformation[] PROGMEM = {
    INSTALLER_DESC1_LEN_P2_BYTE,
    0x01,
    INSTALLER_DESC1_BYTES,
    0x0ff,
    INSTALLER_DESC2_LEN_P2_BYTE
    0x01,
    INSTALLER_DESC2_BYTES,
    0x0ff,
    MAN_INFO_LEN_P2_BYTE
    0x01,
    MAN_INFO_BYTES,
    0x0ff
};
*/

/**
 *  Appled these PGNs to your TX and RX lists when defining in PROGMEM,
 *  They are RX and TX by the SNMEA2000 class but we need to get them into 
 *  a contiguous memory space in program memory.
 */ 
#define SNMEA200_DEFAULT_TX_PGN 126464L,60928L,126996L,126998L,59392L,0L
#define SNMEA200_DEFAULT_RX_PGN 59392L,59904L,60928L,0L

// from NMEA2000 library, makes it much easier creating the name.



// From https://github.com/ttlappalainen/NMEA2000/blob/ad30dced133cf7063b97aaa9ea05e434912e9100/src/NMEA2000.h#L155
class SNMEA2000DeviceInfo {
  public:
    /**
     * @brief Construct a new Device Information object
     * 
     * @param uniqueNumber  unique mumber 
     * @param deviceFunction http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
     * @param deviceClass http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
     * @param manufacturersCode http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
     * @param industryGroup 4 is marine
     * @param deviceInstance 
     * @param industryGroup 
     */
     SNMEA2000DeviceInfo(
        uint32_t uniqueNumber, 
        unsigned char deviceFunction, 
        unsigned char deviceClass,
        uint16_t manufacturersCode = 2048, 
        unsigned char industryGroup = 4,
        unsigned char deviceInstance = 0,
        unsigned char systemInstance = 0
        ) {
            deviceInformation.unicNumberAndManCode = (uniqueNumber&0x1fffff) | (((unsigned long)(manufacturersCode&0x7ff))<<21); 
            deviceInformation.deviceInstance = deviceInstance;
            deviceInformation.deviceFunction = deviceFunction;
            deviceInformation.deviceClass = (deviceClass&0x7f)<<1;
            deviceInformation.industryGroupAndSystemInstance = (industryGroup<<4) | (systemInstance&0x0f);
        };
        void setSerialNumber(uint32_t uniqueNumber) {
            Serial.print("Serial Number set to ");
            Serial.println(uniqueNumber);
            deviceInformation.unicNumberAndManCode=(deviceInformation.unicNumberAndManCode&0xffe00000) | (uniqueNumber&0x1fffff);
        }
        uint64_t getName() {
            return deviceInformation.name;
        };
        byte * getDeviceNameBuffer() {
            return (byte *) &(deviceInformation);
        }
    
  protected:
    typedef union {
      uint64_t name;
      struct {
        uint32_t unicNumberAndManCode; // ManufacturerCode 11 bits , UniqueNumber 21 bits
        unsigned char deviceInstance;
        unsigned char deviceFunction;
        unsigned char deviceClass;
      // I found document: http://www.novatel.com/assets/Documents/Bulletins/apn050.pdf it says about next fields:
      // The System Instance Field can be utilized to facilitate multiple NMEA 2000 networks on these larger marine platforms.
      // NMEA 2000 devices behind a bridge, router, gateway, or as part of some network segment could all indicate this by use
      // and application of the System Instance Field.
      // DeviceInstance and SystemInstance fields can be now changed by function SetDeviceInformationInstances or
      // by NMEA 2000 group function. Group function handling is build in the library.
        unsigned char industryGroupAndSystemInstance; // 4 bits each
      };
    } tUnionDeviceInformation;

    tUnionDeviceInformation deviceInformation;
};




typedef struct SNMEA2000ProductInfo {
    uint16_t nk2version;
    uint16_t productCode;
    const char * modelID;  // 32 bytes max
    const char * softwareVersion; // 32 bytes max padded
     const char * modelVersion; // 32 bytes max padded
    const char * serialNumber; // 32 bytes max padded
    byte certificationLevel;
    byte loadEquivalency;
} SNMEA2000ProductInfo;

typedef struct SNMEA2000ConfigInfo {
    const char * manufacturerInfo; // max 71 var string
    const char * installDesc1; // max 71 var string
    const char * installDesc2; // max 71 var string
} SNMEA2000ConfigInfo;

class MessageHeader {
    public:
       MessageHeader(unsigned long ID) {
            // CAN ID is 29 bits long
            // 111 1 1 11111111 11111111 11111111
            //                           -------- Source Addresss 8 bits @0
            //                  -------- PDU specific 8 bits @ 8
            //         -------- PDU Format 8 bits @ 16
            //       - Data Page 1 but @24
            //     - reserved 1 bit @25
            // --- priority (3 bits starting @26)

            // PDU Format < 240 is transmitted with a destination address, which is in 8 bits 8
            // PDU Format 240 > 255 is a broadcast with the Group extension in 8 bits @8.
            id = ID;
            unsigned char canIdPF = (unsigned char) (id >> 16);
            unsigned char canIdPS = (unsigned char) (id >> 8);
            unsigned char canIdDP = (unsigned char) (id >> 24) & 1;

            source = (unsigned char) id >> 0;
            priority = (unsigned char) ((id >> 26) & 0x7);

            if (canIdPF < 240) {
            /* PDU1 format, the PS contains the destination address */
                destination = canIdPS;
                pgn = (((unsigned long)canIdDP) << 16) | (((unsigned long)canIdPF) << 8);
            } else {
            /* PDU2 format, the destination is implied global and the PGN is extended */
                destination = 0xff;
                pgn = (((unsigned long)canIdDP) << 16) | (((unsigned long)canIdPF) << 8) | (unsigned long)canIdPS;
            }
       };
       MessageHeader(unsigned long PGN, unsigned char Priority, unsigned char Source, unsigned char Destination) {
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
       void print(const char * label, byte *buf, int len) {
             Serial.print(label);
             Serial.print(":");
             Serial.print(pgn);
             Serial.print(",");
             Serial.print(source);
             Serial.print(",");
             Serial.print(destination);
             Serial.print(",");
             Serial.print(priority);
             Serial.print(",");
             Serial.print(len);
             Serial.print(",");
             Serial.print(id,HEX);
             for (int i = 0; i < len; i++) {
                 Serial.print(",");
                 Serial.print(buf[i],HEX);
             }
             Serial.println(";");
       }

       unsigned long id;
       unsigned long pgn;
       unsigned char priority;
       unsigned char source;
       unsigned char destination;
};



class SNMEA2000 {
    public:
        SNMEA2000(byte addr,
        SNMEA2000DeviceInfo * devInfo, 
        const SNMEA2000ProductInfo * pinfo, 
        const SNMEA2000ConfigInfo * cinfo,
        const unsigned long *tx,
        const unsigned long *rx,
        const uint8_t csPin
        ): 
        deviceAddress{addr},
        devInfo{devInfo},
        productInfo{pinfo},
        configInfo{cinfo},
        txPGNList{tx},
        rxPGNList{rx} ,
        CAN{csPin}
        {
        };

        bool open();
        void processMessages();
        void dumpStatus() {
            Serial.print(F("NMEA2000 Status sent="));
            Serial.print(messagesSent);
            Serial.print(F(" recieved="));
            Serial.print(messagesRecieved);
            Serial.print(F(" packet errors="));
            Serial.print(packetErrors);
            Serial.print(F(" frame errors="));
            Serial.println(frameErrors);
        };
        void setDiagnostics(bool enabled) {
            diagnostics = enabled;
        };
        void sendMessage(MessageHeader *messageHeader, byte *message, int len);
        //void sendFastPacket(MessageHeader *messageHeader, byte *message, int len, bool progmem=false);    
        void setSerialNumber(uint32_t _serialNumber) { 
            devInfo->setSerialNumber(_serialNumber); 
        };
        void setDeviceAddress(unsigned char _deviceAddress) { 
            deviceAddress = _deviceAddress; 
            Serial.print("Device Address set to ");
            Serial.println(deviceAddress);
        };
        unsigned char getAddress() { return deviceAddress; };
        void startPacket(MessageHeader *messageHeader);
        void finishPacket();
        void startFastPacket(MessageHeader *messageHeader, int length);
        void finishFastPacket();
        void outputByte(byte opb);
        void output3ByteInt(int32_t i);
        void output2ByteUInt(uint16_t i);
        void outputFixedString(const char * str, int maxLen, byte padding);
        void outputVarString(const char * str,  uint8_t strLen);

        void output2ByteInt(uint16_t i);
        void output2ByteDouble(double v, double p);
        void output2ByteUDouble(double v, double p);
        void output3ByteDouble(double v, double p);
        void output4ByteDouble(double v, double p);
        void output4ByteUDouble(double v, double p);

        
        static const byte broadcastAddress=0xff;
        static const int16_t undefined2ByteDouble=0x7ffe;

        static constexpr double n2kDoubleNA=-1000000000.0;
        static const uint8_t n2kInt8NA=127;

    private:
        void handleISOAddressClaim(MessageHeader *messageHeader, byte * buffer, int len);
        void claimAddress();
        bool hasClaimedAddress();
        void handleISORequest(MessageHeader *messageHeader, byte * buffer, int len);
        void sendPGNLists(MessageHeader *requestMessageHeader);
        void sendPGNList(MessageHeader *messageHeader, int listType, const unsigned long *pgnList);
        void sendIsoAddressClaim();
        void sendProductInformation(MessageHeader *requestMessageHeader);
        void sendConfigurationInformation(MessageHeader *requestMessageHeader);
        void sendIsoAcknowlegement(MessageHeader *requestMessageHeader, byte control, byte groupFunction);
        bool clearRXFilter();
        bool setupRXFilter();
        int getPgmSize(const char *str, int maxLen);



        unsigned char deviceAddress;
        SNMEA2000DeviceInfo * devInfo;
        const SNMEA2000ProductInfo * productInfo;
        const SNMEA2000ConfigInfo * configInfo;
        const unsigned long *txPGNList;
        const unsigned long *rxPGNList;
        MCP_CAN CAN;
        unsigned long addressClaimStarted=0;
        //output buffer and frames
        MessageHeader *packetMessageHeader;
        bool fastPacket;
        bool rxFiltersSet = false;
        bool diagnostics = false;
        uint8_t fastPacketSequence;
        byte buffer[8];
        uint8_t frame;
        uint8_t ob;
        uint16_t messagesRecieved = 0;
        uint16_t messagesSent = 0;
        uint16_t packetErrors = 0;
        uint16_t frameErrors = 0;



};

class PressureMonitor : public SNMEA2000 {
    public:
      PressureMonitor(byte addr,
        const SNMEA2000DeviceInfo * devInfo, 
        const SNMEA2000ProductInfo * pinfo, 
        const SNMEA2000ConfigInfo * cinfo,
        const unsigned long *tx,
        const unsigned long *rx,
        const uint8_t csPin
        ): SNMEA2000{addr, devInfo, pinfo, cinfo, tx, rx, csPin} {};

    /**
     * @brief PGN 130310 
     * 
     * @param sid sequence ID
     * @param waterTemperature in K
     * @param outsideAirTemperature in K 
     * @param atmospheicPressure in Pascals 
     */
    void sendOutsideEnvironmentParameters(
        byte sid=0, 
        double waterTemperature = SNMEA2000::n2kDoubleNA,
        double outsideAirTemperature = SNMEA2000::n2kDoubleNA,
        double atmospheicPressure=SNMEA2000::n2kDoubleNA
        );


    
    /**
     * @brief  PGN 130311L
     * 
     * @param sid  sequence ID
     * @param atmosphericPressure  in Pascals
     * @param tempSource 
     * @param temperature in K 
     * @param humiditySource 
     * @param humidity in % RH
     */
    void sendEnvironmentParameters(
        byte sid=0,
        double atmosphericPressure=SNMEA2000::n2kDoubleNA, // Atomspheric Pressure 
        byte tempSource=0,
        double temperature=SNMEA2000::n2kDoubleNA, // temperature 
        byte humiditySource=0,
        double humidity=SNMEA2000::n2kDoubleNA // humidity
        ); 

    /**
     * @brief PGN 130313L
     * @param sid
     * @param humiditySource
     * @param humidityInstance
     * @param humidity in percent
     */
    void sendHumidity(byte sid, byte humiditySource, byte humidityInstance, double humidity);



    /**
     * @brief PGN 130314L
     * @param sid
     * @param pressureSource
     * @param pressureInstance
     * @param pressure in Pascal
     */
    void sendPressure(byte sid, byte pressureSource, byte pressureInstance, double pressure );

    /**
     * @brief PGN 130316L
     * @param sid
     * @param temperatureSource
     * @param temperatureInstance
     * @param temperature in K
     */
    void sendTemperature(byte sid, byte temperatureSource,  byte temperatureInstance, double temperature );

};


class EngineMonitor : public SNMEA2000 {
    public:
      EngineMonitor(byte addr,
        const SNMEA2000DeviceInfo * devInfo, 
        const SNMEA2000ProductInfo * pinfo, 
        const SNMEA2000ConfigInfo * cinfo,
        const unsigned long *tx,
        const unsigned long *rx,
        const uint8_t csPin
        ): SNMEA2000{addr, devInfo, pinfo, cinfo, tx, rx, csPin} {};
    /**
     * RapidEngine Data - PGN 127488, standard packet
     * enginInstance starting a 0
     * engineSpeed in RPM
     * engineBoostPressure in Pascal
     * engineTrim in %
     */ 
    void sendRapidEngineDataMessage(
        byte engineInstance=0, 
        double engineSpeed=SNMEA2000::n2kDoubleNA, // RPM 
        double engineBoostPressure=SNMEA2000::n2kDoubleNA, // Pa
        byte engineTiltTrim=SNMEA2000::n2kInt8NA // %
        );
    /**
     * EngineDynamicParams - PGN 127489, fast packet
     * engineInstance starting at 0
     * engineHours in seconds
     * engingCoolantTemperature in K
     * alternator Volage in V
     * status1 bitmap
     * status2 bitmap
     * engineOilPressure in Pascal
     * engineOilPTemperature in K
     * fuelRate in l/h
     * engineCoolantPressure in Pascal
     * engineFuelPressure in Pascal
     * engineLoad in %
     * engineTorque in %
     */ 
    void sendEngineDynamicParamMessage(
        byte engineInstance = 0, 
        double engineHours = SNMEA2000::n2kDoubleNA, // s
        double engineCoolantTemperature = SNMEA2000::n2kDoubleNA, // K
        double alternatorVoltage = SNMEA2000::n2kDoubleNA, // V
        uint16_t status1 = 0,
        uint16_t status2 = 0,
        double engineOilPressure = SNMEA2000::n2kDoubleNA, // Pa
        double engineOilPTemperature = SNMEA2000::n2kDoubleNA, // K
        double fuelRate = SNMEA2000::n2kDoubleNA, // l/h
        double engineCoolantPressure = SNMEA2000::n2kDoubleNA, // K
        double engineFuelPressure = SNMEA2000::n2kDoubleNA, // Pa
        byte engineLoad = SNMEA2000::n2kInt8NA, // %
        byte engineTorque = SNMEA2000::n2kInt8NA // %
        );
    /**
     * DC Battery Status PGN 127508
     * batteryInstance starting a 0
     * SID sequence id 
     * batterVoltage in volts
     * batterTemperature in K
     * batteryCurrent in A
     */ 
    void sendDCBatterStatusMessage(
            byte batteryInstance=0, 
            byte sid=0,
            double batteryVoltage = SNMEA2000::n2kDoubleNA, // V
            double batteryTemperature = SNMEA2000::n2kDoubleNA, // K
            double batteryCurrent = SNMEA2000::n2kDoubleNA // A
        );
    /**
     * Fluid Level PGN 127505
     * type fuel=0, 
     *       water=1, 
     *       greywater=2, 
     *       levelwell=3,  
     *        oil=4,  
     *        blackwater=5,
     *       petrol=6
     *       error=14
     *       unavailable=15
     * instance unique tank instance 
     * level in %
     * capacity in l
     */ 
    void sendFluidLevelMessage(
        byte type = 0,
        byte instance = 0,
        double level = SNMEA2000::n2kDoubleNA, // % 
        double capacity = SNMEA2000::n2kDoubleNA // l
        );
    /*
     * Temperature PGN 130312
     * sid sequence id
     * intance instance starting at 0
     * source
     *    sea=0,
     *    outside=1
     *    inside=2
     *    engineroom=3
     *    maincabin=4
     *    livewell=5
     *    baitwell=6
     *    fridge=7
     *    heating=8
     *    dewpoint=9
     *    apparent wind chill=10
     *    theoretical wind chill = 11
     *    heatindextemperature=12
     *    freezer=13
     *    exhaustgas=14
     * actual in K
     * requested in K
     */
    void sendTemperatureMessage(
        byte sid = 0, 
        byte instance = 0,
        byte source = 0,
        double actual = SNMEA2000::n2kDoubleNA, // K
        double requested = SNMEA2000::n2kDoubleNA // K
        );
};



// message type structures.


/*
typedef union EngineDynamicParamMessage {
  byte buffer[26];
  struct {
    byte engineInstance = 0; // 
    uint16_t engineOilPressure = SNMEA2000::undefined2ByteUDouble; // in 1 = 100  
    uint16_t engineOilPTemperature = SNMEA2000::undefined2ByteUDouble; // in 1 = 0.1 
    uint16_t engineCoolantTemperature = SNMEA2000::undefined2ByteUDouble; // in 1 = 0.01 
    int16_t alternatorVoltage = SNMEA2000::undefined2ByteDouble; // in 1 = 0.01
    int16_t fuelRate = SNMEA2000::undefined2ByteDouble; // in 1 = 0.1
    uint32_t engineHours = SNMEA2000::undefined4ByteUDouble; // in 1 = 1
    uint16_t engineCoolantPressure = SNMEA2000::undefined2ByteUDouble; // in 1 = 100
    uint16_t engineFuelPressure = SNMEA2000::undefined2ByteUDouble; // in 1 = 1000
    byte reserved1 = 0xff;
    uint16_t status1;
    uint16_t status2;
    byte engineLoad;
    byte engineTorque;
  };
} EngineDynamicParamMessage;


typedef union DCBatStatusMessage {
    byte buffer[8];
    struct {
        byte batteryInstance = 0;
        uint16_t batteryVoltage = SNMEA2000::undefined2ByteDouble; // in 1 = 0.01
        uint16_t batteryCurrent = SNMEA2000::undefined2ByteDouble; // in 1 = 0.1
        uint16_t batteryTemperature = SNMEA2000::undefined2ByteDouble; // 1 = 0.01
        byte SID = 1;
    };
} DCBatStatusMessage;

typedef union FluidLevelMessage {
    byte buffer[8];
    struct {
        byte typeInstance = 0; // hi nibble is type, low nibble is instance.
        int16_t level = SNMEA2000::undefined2ByteDouble; // in 1 = 0.004
        uint32_t capacity = SNMEA2000::undefined4ByteUDouble; // in 1 = 0.1
        byte reserved = 0xff;
    };
} FluidLevelMessage;

typedef union TemperatureMessage {
    byte buffer[8];
    struct {
        byte SID = 0;
        byte instance = 0;
        byte source = 0;
        uint16_t actual = SNMEA2000::undefined2ByteUDouble; // in 1 = 0.01
        uint16_t requested = SNMEA2000::undefined2ByteUDouble; // in 1 = 0.01
        byte reserved = 0xff;
    };
} TemperatureMessage;
*/

#endif