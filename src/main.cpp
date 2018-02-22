#include <Homie.h>
#include <string.h>

HomieNode flowNode("flow", "sensor");
HomieNode fcalibrationNode("flow", "calibration");
HomieNode volumeNode("volume", "sensor");
HomieNode condsensorNode("conductivity", "sensor");
HomieNode relay1Node("relay1", "relay");
HomieNode relay2Node("relay2", "relay");
HomieNode relay3Node("relay3", "relay");


// Pinos do sensor de vazão
byte statusLed    = 3;
byte sensorInterrupt = 14;  // 0 = digital pin 2
byte sensorPin       = 14;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 6.5;
volatile byte pulseCount;
float flowRate;
float flowLitres;
float totalLitres;
unsigned long oldTime;
unsigned long mqttTime;
int flowWindow = 0;
int flowInterval = 10000; //ms


// Sensor de condutividade
unsigned long int time_off_cond;                                          // tempo em ms

/*-----------------------Medidas----------------------*/
const float V_esp = 3.3;
float Vm;   //Variável que irá armazenar as leituras em INT do ADC no ponto médio do divisor de tensão
float Gx; //Condutância da solução

const int RELAY1PIN = 13; //D2
const int RELAY2PIN = 5; //D1
const int RELAY3PIN = 4; //D2

void conductivity_read(){
  if((millis() - time_off_cond) > 15000){
    digitalWrite(D6, 1);
    delay(200);
    Vm = analogRead(A0);
    digitalWrite(D6, 0);
    // Calculo da condutividade
    if (Vm <=182 ){
      Gx = 2000;
    } else if (Vm <= 209) {
      Gx = ((Vm-241.827567)/(-0.030691964));
    } else if (Vm <=568.5) {
      Gx = ((Vm-597.926)/(-0.374085684));
    } else if (Vm <= 958){
      Gx = ((Vm-962.930379)/(-4.930379));
    } else if (Vm >= 958) {
      Gx = 0;
    }
    /* Mensagem para o publish */
    String messageout;
    messageout += round(Gx);
    Homie.getLogger() << "Condutividade: " << messageout << "uS" << endl;
    condsensorNode.setProperty("us").send(messageout);
    time_off_cond = millis();
  }
}

void pulseCounter()
{
  pulseCount++;
}

void flowLoop(){
  flowWindow = (millis() - oldTime);
  if(flowWindow > 1000)
  {
     detachInterrupt(sensorInterrupt);

     flowRate = flowWindow / 1000 * pulseCount / calibrationFactor;

     oldTime = millis();

     flowLitres = flowRate / 60;

     totalLitres += flowLitres;

     unsigned int frac;

     // Print the flow rate for this second in litres / minute
     if (millis() - mqttTime > flowInterval){
       Homie.getLogger() << "Vazão média: " << flowRate << "L/min" << endl;
       flowNode.setProperty("lm").send(String(flowRate));

       Homie.getLogger() << "Volume acumulado: " << totalLitres << "L" << endl;
       volumeNode.setProperty("litros").send(String(totalLitres));

       mqttTime = millis();
     }

     // Reset the pulse counter so we can start incrementing again
     pulseCount = 0;

     // Enable the interrupt again now that we've finished sending output
     attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
   }
}

bool relay1NodeHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(RELAY1PIN, on ? 1 : 0);
  relay1Node.setProperty("on").send(value);
  Homie.getLogger() << "Relay 1 " << (on ? "ligou" : "desligou") << endl;

  return true;
}

bool relay2NodeHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(RELAY2PIN, on ? 1 : 0);
  relay2Node.setProperty("on").send(value);
  Homie.getLogger() << "Relay 2 " << (on ? "ligou" : "desligou") << endl;

  return true;
}

bool relay3NodeHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(RELAY3PIN, on ? 1 : 0);
  relay3Node.setProperty("on").send(value);
  Homie.getLogger() << "Relay 3 " << (on ? "ligou" : "desligou") << endl;

  return true;
}

bool fcalibrationNodeHandler(const HomieRange& range, const String& value) {
  if (value == "") return false;
  calibrationFactor = atof(value.c_str());
  fcalibrationNode.setProperty("factor").send(value);
  Homie.getLogger() << "Fator de calibracao alterado para: " << calibrationFactor << endl;

  return true;
}

void setupHandler() {
    // Set up the status LED line as an output
    pinMode(statusLed, OUTPUT);
    digitalWrite(statusLed, HIGH);  // We have an active-low LED attached

    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);

    pulseCount        = 0;
    flowRate          = 0.0;
    flowLitres   = 0;
    totalLitres  = 0;
    oldTime           = 0;

    // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
    // Configured to trigger on a FALLING state change (transition from HIGH
    // state to LOW state)
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    //inicialização dos pinos
    pinMode(A0, INPUT);
    pinMode(RELAY1PIN, OUTPUT);
    pinMode(RELAY2PIN, OUTPUT);
    pinMode(RELAY3PIN, OUTPUT);
    pinMode(D6, OUTPUT);
    digitalWrite(RELAY1PIN, HIGH);
    digitalWrite(RELAY2PIN, HIGH);
    digitalWrite(RELAY3PIN, HIGH);
    digitalWrite(D6, LOW);
}

void loopHandler(){
  flowLoop();
  conductivity_read();
}

void setup() {
  //altera o branding do Homie para soho
  Homie_setBrand("soho-labs");

  //inicialização da serial
  Serial.begin(115200);
  Serial << endl << endl;

  //Versão do firmware que é exibida na configuração inicial e utilizada para atualizações OTA
  Homie_setFirmware("soholabs-waterAnalisys", "0.1");

  //inicialização das publicações MQTT
  flowNode.advertise("lm");
  volumeNode.advertise("litros");
  condsensorNode.advertise("us");
  relay1Node.advertise("on").settable(relay1NodeHandler);
  relay2Node.advertise("on").settable(relay2NodeHandler);
  relay3Node.advertise("on").settable(relay3NodeHandler);
  fcalibrationNode.advertise("factor").settable(fcalibrationNodeHandler);

  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  Homie.setup();
}

void loop() {
  Homie.loop();
}
