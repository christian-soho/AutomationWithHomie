#include <Homie.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "ACS712.h"

const int LIGHT_RELAY = 5;
const int AC_RELAY = 4;
const int LUM_SENSOR = 2;
//const int TEMP_HUM_SENS = 14;

// Definições para o sensor de humidade e temperatura
#define DHTPIN D5
#define DHTTYPE DHT22
const int TEMPERATURE_INTERVAL = 30;
unsigned long last_temperature_sent = 0;
const int HUMIDITY_INTERVAL = 30;
unsigned long last_humidity_sent = 0;

// Variáveis para medição de corrente
//const int CUR_CAPTURE_INTERVAL = 1500; //ms
//const int CUR_PUBLISH_INTERVAL = 10000; //ms
float accumulated_current = 0;
float average_current = 0;
unsigned long cur_t0 = 0;
unsigned long cur_tx = 0;
unsigned long timestamp;
float currentRead;
unsigned long capture_interval;
unsigned long publish_interval;
int currentCounter = 0;

// Declaração dos Nodes do Homie. Cada Node representa um topico específico de MQTT.
// O primeiro argumento representa o subtopico que será criado após o ID do dispositivo que é configurado no processo de instalação inicial
// O segundo argumento representa o tipo do nodo e tem finalidade meramente informativa, sem impacto na estrutura de tópico MQTT
HomieNode lightNode("light", "relay");
HomieNode acNode("ac", "relay");
HomieNode luminosityNode("luminosity", "LDR");
HomieNode temperatureNode("temperature", "AM2302");
HomieNode humidityNode("humidity", "AM2302");
HomieNode currentNode("current", "ACS712");

// Inicialização do sensor de umidade e temperatura
DHT_Unified dht(DHTPIN, DHTTYPE);

// Inicialização do sensor de corrente
ACS712 currentSensor(ACS712_30A, A0);

// Função que obtém a temperatura do sensor, imprime na saida serial e log e publica no tópico MQTT
void temperatureHandler() {
  // Verifica se já está na hora de capturar a informação do sensor
  if (millis() - last_temperature_sent >= TEMPERATURE_INTERVAL * 1000UL || last_temperature_sent == 0) {
    // a biblioteca DHT da Adafruit sugere a utilização de uma variável evento para receber informações do sensor
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    //verifica se o sensor o ESP consegue receber informações do sensor
    if (isnan(event.temperature)) {
      Serial.println("Falha na comunicação com o sensor!");
      return;
    }
    //variável que recebe a temperatura do evento DHT
    float temperature = event.temperature;
    //publica a temperatura no log do Homie
    Homie.getLogger() << "Temperatura: " << temperature << " °C" << endl;
    //publica a temperatura no MQTT
    temperatureNode.setProperty("celsius").send(String(temperature));
    //atualiza a variável com o último tempo de aquisição do registro
    last_temperature_sent = millis();
  }
}

void humidityHandler() {
  // Verifica se já está na hora de capturar a informação do sensor
  if (millis() - last_humidity_sent >= HUMIDITY_INTERVAL * 1000UL || last_humidity_sent == 0) {
    // a biblioteca DHT da Adafruit sugere a utilização de uma variável evento para receber informações do sensor
    sensors_event_t event;
    dht.humidity().getEvent(&event);
    //verifica se o sensor o ESP consegue receber informações do sensor
    if (isnan(event.relative_humidity)) {
      Serial.println("Falha na comunicação com o sensor!");
      return;
    }
    //variável que recebe a umidade do evento DHT
    float humidity = event.relative_humidity;
    //publica a temperatura no log do Homie
    Homie.getLogger() << "Umidade: " << humidity << " %" << endl;
    //publica a umidade no MQTT
    humidityNode.setProperty("percentage").send(String(humidity));
    //atualiza a variável com o último tempo de aquisição do registro
    last_humidity_sent = millis();
  }
}

bool lightNodeHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(LIGHT_RELAY, on ? HIGH : LOW);
  lightNode.setProperty("on").send(value);
  Homie.getLogger() << "Luz " << (on ? "acendeu" : "apagou") << endl;

  return true;
}

bool acNodeHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(AC_RELAY, on ? HIGH : LOW);
  acNode.setProperty("on").send(value);
  Homie.getLogger() << "A/C " << (on ? "ligou" : "desligou") << endl;

  return true;
}

// Função que faz leituras da corrente elétrica, imprime na saida serial e log e publica no tópico MQTT
void currentNodeHandler() {
  timestamp = millis();
  capture_interval = timestamp - cur_tx;
  publish_interval = timestamp - cur_t0;

  // Verifica se janela de leitura já iniciou
  if (accumulated_current == 0){
    cur_t0 = cur_tx = timestamp;
    currentRead = currentSensor.getCurrentAC(60);
    accumulated_current = currentRead;
    currentCounter++;
  }
  else{
    if (capture_interval >= 1500UL){
      currentRead = currentSensor.getCurrentAC(60);
      accumulated_current = accumulated_current + currentRead;
      Homie.getLogger() << "Corrente atual: " << currentRead << " Ah" << endl;
      cur_tx = timestamp;
      currentCounter++;
    }
    if (publish_interval >= 10000UL){
      average_current = accumulated_current / currentCounter;
      Homie.getLogger() << "Corrente média: " << average_current << " Ah" << endl;
      currentNode.setProperty("Ah").send(String(average_current));
      accumulated_current = 0;
      currentCounter = 0;
    }
  }
}

void acs712Handler(){
  timestamp = millis();
  capture_interval = timestamp - cur_tx;
  if (capture_interval >= 1500UL){
    Homie.getLogger() << "Corrente atual: " << currentSensor.getCurrentAC(60) << " A" << endl;
    cur_tx = timestamp;
  }
}

void setupHandler() {
  temperatureNode.setProperty("unit").send("c");
  Homie.getLogger() << "Offset de calibração do sensor de corrente: " << currentSensor.calibrate() << endl;

}

void loopHandler(){
  //temperatureHandler();
  //humidityHandler();
  //currentNodeHandler();
  acs712Handler();
}

void setup() {
  //inicializa a variavel DHT para coleta de temperatura e umidade
  dht.begin();

  //altera o branding do Homie para soho
  Homie_setBrand("soho");

  //inicialização da serial
  Serial.begin(115200);
  Serial << endl << endl;

  //inicialização dos pinos do ESP8266
  pinMode(LIGHT_RELAY, OUTPUT);
  pinMode(AC_RELAY, OUTPUT);
  pinMode(LUM_SENSOR, INPUT);
  digitalWrite(LIGHT_RELAY, LOW);
  digitalWrite(AC_RELAY, LOW);

  //Versão do firmware que é exibida na configuração inicial e utilizada para atualizações OTA
  Homie_setFirmware("soho-automation", "0.0.1");

  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);


  //inicialização das publicações MQTT
  lightNode.advertise("on").settable(lightNodeHandler);
  acNode.advertise("on").settable(acNodeHandler);
  humidityNode.advertise("percentage");
  temperatureNode.advertise("celsius");
  currentNode.advertise("current");

  Homie.setup();
}

void loop() {
  Homie.loop();
}
