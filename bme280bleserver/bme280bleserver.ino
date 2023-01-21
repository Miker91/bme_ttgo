#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Default Temperature is in Celsius
//Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

//BLE server name
#define bleServerName "BME280_ESP32"

Adafruit_BME280 bme; // I2C

float temp;
float press;
float hum;

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

bool deviceConnected = false;
const char *idUla = "Ul_1";

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic and Descriptor
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2901));

// Humidity Characteristic and Descriptor
BLECharacteristic bmeHumidityCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeHumidityDescriptor(BLEUUID((uint16_t)0x2902));

// Pressure Characteristic and Descriptor
BLECharacteristic bmePressureCharacteristics("80d92805-e1b3-49c3-aa46-acd5aef61606", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmePressureDescriptor(BLEUUID((uint16_t)0x2903));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void initBME(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void setup() {
  // Start serial communication 
  // Serial.begin(115200);
  Serial.println(F("BME280 test"));
  Serial.begin(9600);

  // Init BME Sensor
  initBME();

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // ID
  // bmeService->addCharacteristic(&idUlaCharacteristic);
  // idUlaDescriptor.setValue("BME temperature Celsius");
  // idUlaCharacteristic.addDescriptor(&idUlaDescriptor);

  // Temperature
  bmeService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
  bmeTemperatureCelsiusDescriptor.setValue("BME temperature Celsius");
  bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);

  // Humidity
  bmeService->addCharacteristic(&bmeHumidityCharacteristics);
  bmeHumidityDescriptor.setValue("BME humidity");
  bmeHumidityCharacteristics.addDescriptor(&bmeHumidityDescriptor);

  // Pressure
  bmeService->addCharacteristic(&bmePressureCharacteristics);
  bmePressureDescriptor.setValue("BME pressure");
  bmePressureCharacteristics.addDescriptor(&bmePressureDescriptor);
  
  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      // Read temperature as Celsius (the default)
      temp = bme.readTemperature();
      // Fahrenheit
      press = bme.readPressure();
      // Read humidity
      hum = bme.readHumidity();
  
      //Notify temperature reading from BME sensor
      // #ifdef temperatureCelsius
      static char temperatureCTemp[6];
      dtostrf(temp, 6, 2, temperatureCTemp);
      //Set temperature Characteristic value and notify connected client
      bmeTemperatureCelsiusCharacteristics.setValue(temperatureCTemp);
      bmeTemperatureCelsiusCharacteristics.notify();
      Serial.print("Temperature Celsius: ");
      Serial.print(temp);
      Serial.print(" *C");

      //Notify humidity reading from BME
      static char humidityTemp[6];
      dtostrf(hum, 6, 2, humidityTemp);
      //Set humidity Characteristic value and notify connected client
      bmeHumidityCharacteristics.setValue(humidityTemp);
      bmeHumidityCharacteristics.notify();   
      Serial.print(" - Humidity: ");
      Serial.print(hum);
      Serial.println(" %");

      //Notify pressure reading from BME
      static char pressure[6];
      dtostrf((press / 100.0F), 6, 2, pressure);
      //Set humidity Characteristic value and notify connected client
      bmePressureCharacteristics.setValue(pressure);
      bmePressureCharacteristics.notify();   
      Serial.print(" - Pressure: ");
      Serial.print(press / 100.0F);
      Serial.println(" hPa");

      lastTime = millis();
    }
  }
}