//Programa: Leitura gps utilizando FreeRTOS

/*
Task             Core  Prio     Descrição
-------------------------------------------------------------------------------------------------------
vTaskGPS           1     2     faz a leitura do gps
vTaskMQTT          1     3     Publica valor do gps em tópico MQTT

Timer RTOS
--------------------------------------------------------------------------------------------------------
    xTimerLED  Pisca led se conexão mqtt funcionar
*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;
#define LEDPIN 2 //LED QUE INDICA A LEITURA

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

const char* ssid = "BambinoCima2.4";
const char* password =  "biofa40413";
const char* mqttServer = "dnsmassa.duckdns.org";
const int mqttPort = 1883;
const char* mqttUser = "admin"; //COLOCAR USUÁRIO SE TIVER, EXEMPLO: const char* mqttUser = "SEU_USUÁRIO_MQTT";
const char* mqttPassword = "jujuba455"; //COLOCAR SENHA SE TIVER, EXEMPLO: const char* mqttUser = "SUA_SENHA_MQTT";

WiFiClient espClient;
PubSubClient client(espClient);
int analog_value = 0;
int contador = 1;
char mensagem[50] = "";
char mensagem1[50] = "";
char mensagem2[50] = "";

// Timer rtos
xTimerHandle xTimerLED, xTimerSensor; //Armazena o handle do timer
QueueHandle_t xFila; //cria objeto fila

void callBackTimerLED(TimerHandle_t pxTimerLED );     //Timer LED

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*******";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("**** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("**** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void reconectabroker()
{
  //Conexao ao broker MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    Serial.println("Conectando ao broker MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword ))
    {
      Serial.println("Conectado ao broker!");
    }
    else
    {
      Serial.print("Falha na conexao ao broker - Estado: ");
      Serial.print(client.state());
      delay(1000);
    }
  }
  Serial.println("Conectado ao broker!");
}


void vTaskGPS(void *p)
{
  while(1){
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
      printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      printInt(gps.location.age(), gps.location.isValid(), 5);
      printDateTime(gps.date, gps.time);
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
      printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
      printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
      printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "* ", 6);
    
      unsigned long distanceKmToLondon =
        (unsigned long)TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON) / 1000;
      printInt(distanceKmToLondon, gps.location.isValid(), 9);
    
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
    
      printFloat(courseToLondon, gps.location.isValid(), 7, 2);
    
      const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);
    
      printStr(gps.location.isValid() ? cardinalToLondon : "* ", 6);
    
      printInt(gps.charsProcessed(), true, 6);
      printInt(gps.sentencesWithFix(), true, 10);
      printInt(gps.failedChecksum(), true, 9);
      Serial.println();
    }

}

void vTaskMQTT (void *p)
{
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
        
    double latitude = (gps.location.lat());
    double longitude = (gps.location.lng());
    reconectabroker();

  snprintf(mensagem1, 50, "%lf", latitude);
  //Envia a mensagem ao broker
  client.publish("gps_ju_lat", mensagem1);

  snprintf(mensagem2, 50, "%lf", longitude);
  //Envia a mensagem ao broker
  client.publish("gps_ju_long", mensagem2);
    
  //Incrementa o contador
  contador++;
  //Aguarda 10 segundos para enviar uma nova mensagem
  smartDelay(1000);  
}

//Timers
void callBackTimerLED(TimerHandle_t pxTimerLED ){
  if(!client.connected())
  {
    digitalWrite(LEDPIN,HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Iniciando conexao com a rede WiFi..");
  }
  Serial.println("Conectado na rede WiFi!");
  
  ss.begin(GPSBaud);

  xFila = xQueueCreate(1, sizeof(int));
   if (xFila == NULL)
  {
     Serial.println("Erro: nao e possivel criar a fila");
     while(1); /* Sem a fila o funcionamento esta comprometido. Nada mais deve ser feito. */
  }

  xTimerLED = xTimerCreate("TIMER",pdMS_TO_TICKS(2000),pdTRUE, 0, callBackTimerLED); //Reaiza a rotina do timer do LED a cada dois segundos 

  xTaskCreate(vTaskGPS, "GPS GY-NEO6MV2", 5000, NULL, 2, NULL);
  xTaskCreate(vTaskMQTT, "Transporte MQTT", 5000, NULL, 5, NULL);

  xTimerStart(xTimerLED,0); //Starta o timer do LED
  
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop() {
  reconectabroker();
  vTaskDelay(pdMS_TO_TICKS(1000));
}
