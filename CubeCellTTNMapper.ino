#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h"

#define POI_BUTTON (GPIO7)

uint8_t devEui[] = { 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0xD0, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint32_t devAddr = ( uint32_t)0x26000000;
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

extern bool IsLoRaMacNetworkJoined;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000 }; // Subchannel 2 for AU915

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/*LoraWan Class*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;
/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;
/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;
/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;
/*LoraWan REGION*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false; //LORAWAN_UPLINKMODE;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/* Application port */
uint8_t appPort = 2;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 1000 * 20;

uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
bool poi; // point of interest
bool DRfast; // slow or fast DR. Flip-flip between them.

#define DR_SLOW DR_2 // SF10
#define DR_FAST DR_5 // SF7

void poiButtonPress()
{
  poi = true;
}

// Set the Datarate (interrelated to SF. DR_0 = SF12, mostly. DR_2 = SF10 for AU915, DR_5 is SF7)
void setDR(int8_t datarate) {
  MibRequestConfirm_t mibSet;
  
  mibSet.Type = MIB_CHANNELS_DATARATE;
  mibSet.Param.ChannelsDatarate = datarate;
  LoRaMacMibSetRequestConfirm( &mibSet );
}

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  appDataSize = 0;//AppDataSize max value is 64

  Serial.printf("GPS data is %ds old - %d sats\n", Air530.location.age()/1000, Air530.satellites.value());
  
  if (Air530.location.age() < 1000) {
    
    appDataSize = 10;

    LatitudeBinary = ((Air530.location.lat() + 90) / 180) * 16777215;
    LongitudeBinary = ((Air530.location.lng() + 180) / 360) * 16777215;

    appData[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    appData[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    appData[2] = LatitudeBinary & 0xFF;

    appData[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    appData[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    appData[5] = LongitudeBinary & 0xFF;

    altitudeGps = Air530.altitude.meters();
    appData[6] = ( altitudeGps >> 8 ) & 0xFF;
    appData[7] = altitudeGps & 0xFF;

    hdopGps = Air530.hdop.hdop() * 10;
    appData[8] = hdopGps & 0xFF;

    appData[9] = Air530.satellites.value(); // < 128 sats. Could probably steal another bit.
    appData[9] &= ~(1 << 7);
    if (poi) {
      appData[9] |= 1 << 7;
      poi = false;
      Serial.println("POI");
    }
  } else {
    Serial.print("No GPS data found:");
    Serial.printf("%04d-%02d-%02dT%02d:%02d:%02d\n", Air530.date.year(), Air530.date.month(), Air530.date.day(),
                  Air530.time.hour(), Air530.time.minute(), Air530.time.second());

  }
}

void setup() {
  pinMode(POI_BUTTON,INPUT_PULLUP);
  attachInterrupt(POI_BUTTON, poiButtonPress, FALLING);
  
  boardInitMcu();

  Serial.begin(115200);
#if(AT_SUPPORT)
  enableAt();
#endif
  //LoRaWAN.displayMcuInit();
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  Air530.setmode(MODE_GPS_GLONASS_BEIDOU);
  Air530.setNMEA(NMEA_RMC);
  Air530.begin();
  poi = false;
}

void get_gps() {
  if (Air530.available() > 0)
  {
    Air530.encode(Air530.read());
  }
}

void loop()
{
  get_gps(); // get something useful
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(AT_SUPPORT)
        getDevParam();
#endif
        printDevParam();
        Serial.printf("LoRaWan Class%X  start! \r\n", loraWanClass + 10);
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        //setTTNBand();
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame( appPort );
        if (appDataSize > 0) {
          if(DRfast) {
            Serial.println("Setting DRfast");
            setDR(DR_FAST);
          } else {
            Serial.println("Setting DRslow");
            setDR(DR_SLOW);
          }
          DRfast = !DRfast;
          LoRaWAN.send();
        }
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        //LoRaWAN.sleep();
        Radio.IrqProcess( );
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
