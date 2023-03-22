//bibliotecas -----------------------------------------------------------------------------------------------------------
#include <lmic.h>             //lmic
#include <hal/hal.h>          //lmic
#include <SPI.h>              //lmic
#include <TinyGPS++.h>        //gps
#include "axp20x.h"           //axp
//bibliotecas -----------------------------------------------------------------------------------------------------------

//configs ---------------------------------------------------------------------------------------------------------------
//chaves de autenticação OTAA
#define APPEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                          //lsb
#define DEVEUI_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00                                                          //lsb
#define APPKEY_KEY 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00          //msb

//constante de intervalo de envio
#define INTERVALO_ENVIO 30 //em segundos

//mapa de pinos
#define NSS_PIN 18
#define RST_PIN 14
#define DIO0_PIN 26
#define DIO1_PIN 33
#define DIO2_PIN 32

//velocidade serial
#define SERIAL_BAUND 9600

//informação do pacote
//#define USE_SPEED
#define USE_ALTITUDE
//configs ---------------------------------------------------------------------------------------------------------------

//axp -------------------------------------------------------------------------------------------------------------------
AXP20X_Class axp;

void configureAXP()
{
  Wire.begin(21, 22); // configurado a comunicação com o axp
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
  {
      Serial.println("AXP192 Begin PASS");
  }
  else
  {
      Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
}
//axp -------------------------------------------------------------------------------------------------------------------

//gps -------------------------------------------------------------------------------------------------------------------
TinyGPSPlus gps;
HardwareSerial GPS(1);

void configureGPS()
{
  GPS.begin(9600, SERIAL_8N1, 34, 12);
}

void loopGPS()
{
  while (GPS.available())
    gps.encode(GPS.read());
}
//gps -------------------------------------------------------------------------------------------------------------------

//packet ----------------------------------------------------------------------------------------------------------------
//Construção de pacote com os dados do gps para envio
void buildPacket(uint8_t packet[9])
{
  uint32_t LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  packet[0] = (LatitudeBinary >> 16) & 0xFF;
  packet[1] = (LatitudeBinary >> 8) & 0xFF;
  packet[2] = LatitudeBinary & 0xFF;

  uint32_t LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  packet[3] = (LongitudeBinary >> 16) & 0xFF;
  packet[4] = (LongitudeBinary >> 8) & 0xFF;
  packet[5] = LongitudeBinary & 0xFF;

  #ifdef USE_SPEED
  uint16_t Speed = gps.speed.kmh();
  packet[6] = (Speed >> 8) & 0xFF;
  packet[7] = Speed & 0xFF;
  #endif

  #ifdef USE_ALTITUDE
  uint16_t Altitude = gps.altitude.meters() * 10;
  packet[6] = (Altitude >> 8) & 0xFF;
  packet[7] = Altitude & 0xFF;
  #endif

  uint8_t Hdops = gps.hdop.value() / 10;
  packet[8] = Hdops & 0xFF;
}

void printPacket()
{
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat());
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng());
  Serial.print("Altitude: ");
  Serial.println(gps.altitude.meters());
  Serial.print("Hdop: ");
  Serial.println(gps.hdop.value());
}
//packet ----------------------------------------------------------------------------------------------------------------

//LMIC ------------------------------------------------------------------------------------------------------------------
//chaves OTAA LMIC
static const u1_t PROGMEM APPEUI[8] = { APPEUI_KEY }; //lsb format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8]  = { DEVEUI_KEY }; // lsb format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = { APPKEY_KEY }; //msb format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

//sendjob LMIC
static osjob_t sendjob;

//pinmap TBEAM LMIC - settings.h
const lmic_pinmap lmic_pins = {
    .nss = NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RST_PIN,
    .dio = {DIO0_PIN, DIO1_PIN, DIO2_PIN},
};

const unsigned TX_INTERVAL = INTERVALO_ENVIO;

//payload
uint8_t payload[9];

//função de envio LMIC
void do_send(osjob_t *j);

//LMIC - eventos de comunicação
//os únicos eventos estanciados são EV_JOINING e EV_TXCOMPLETE
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        //? Se já deu Join uma vez, desativa a verificação para as próximas comunicações
        LMIC_setLinkCheckMode(0);
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
          Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
          Serial.println(F("Received "));
          Serial.println(LMIC.dataLen);
          Serial.println(F(" bytes of payload"));
        }
        //!Agenda as proximas transmissões em TX_INTERVAL segundos
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        break;
      default:
        Serial.println(F("Unknown event"));
        break;
  }
}

//função de envio LMIC
void do_send(osjob_t *j)
{
  //verifica se já está ocorrendo uma tranmissão
  if(LMIC.opmode & OP_TXRXPEND)
    Serial.println("OP_TXRXPEND, not sending");
  else
  {
    buildPacket(payload); //*contrução do pacote para envio - packet.h
    printPacket();
    LMIC_setTxData2(1, payload, sizeof(payload), 0); //função de envio LMIC
    Serial.println("Packet queued");
    Serial.print("Sending packet on frequency: ");
    Serial.println(LMIC.freq); 
  }
}
//LMIC ------------------------------------------------------------------------------------------------------------------


void setup() {
  Serial.begin(SERIAL_BAUND);

  delay(3000);

  Serial.println("Starting");

  //iniciando módulos
  configureAXP();  //axp.h
  Serial.println("Axp OK");
  delay(300);
  configureGPS();
  Serial.println("GPS OK");
  delay(300);

  //iniciando lmic
  os_init();
  Serial.println("LMIC OK");
  delay(1000);
  LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.
  delay(1000);
  do_send(&sendjob); //Start
}

void loop() {
  loopGPS(); //leitura GPS
  os_runloop_once();
}
