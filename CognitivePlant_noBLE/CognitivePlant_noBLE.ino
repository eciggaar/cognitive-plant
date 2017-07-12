
/***************************************************************************
   External Libraries
 **************************************************************************/
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "DHT.h"

/***************************************************************************
   Internet Connectivity Setup - Variables & Functions
 **************************************************************************/
char ssid[] = "<your-network>"; // your network SSID (name)
char pass[] = "<your-passwd>";  // your network password
int status = WL_IDLE_STATUS;    // the Wifi radio's status

// Initialize the WiFi client library
WiFiClient wifiClient;

/***************************************************************************
   MQTT Initialization
 **************************************************************************/
#define MQTT_SERVER "realtime.ngi.ibm.com"

// Initialize pubsubclient to publish sensor data
PubSubClient client;

/***************************************************************************
   Grove board sensor initialization
 **************************************************************************/
#define MOISTURE_SENSOR A0    // input pin for moisture sensor
#define DHT_SENSOR A1         // input pin for temp & humidity sensor
#define DHTTYPE DHT11         // type of DHT sensor (in this case DHT11)
#define LIGHT_SENSOR A2       // input pin for light sensor

int moistureValue = 0;        // variable to store data from moisture sensor A0
int lightSensorValue = 0;     // variable to store data from lightsensor A2

float temperature;            // variable used to store temperature
float humidity;               // variable used to store humidity

StaticJsonBuffer<200> jsonBuffer;
char sensCharArray[200];
JsonObject& sensorJSON = jsonBuffer.createObject();

DHT dht(DHT_SENSOR, DHTTYPE); // Initializing dht client

/***************************************************************************
   Board setup
 **************************************************************************/
void setup()
{
  // init serial link for debugging
  Serial.begin(9600);

  // Initialize humidity and temp sensor
  dht.begin();

  // Connect to WiFi network
  setupWiFi();
  
  // Setup client to MQTT server via port 1883 over WiFi. 
  client = PubSubClient(MQTT_SERVER, 1883, wifiClient);

  if (!client.connected()) {
    connectToMQTT();
  }
}

/***************************************************************************
   Default loop method
 **************************************************************************/
void loop()
{  
  // Read sensor data from Grove board
  refreshMeasurements();

  /***************************************************************************
    // Publish sensor data & wait for next measurement
  ***************************************************************************/
  sensorJSON.printTo(sensCharArray, sizeof(sensCharArray));
  client.publish("iot-2/cmd/sensordata/fmt/string", sensCharArray);
  
  delay(10000); // Wait a few seconds between measurements

  client.loop();
}

/***************************************************************************
   Setup WiFi connection
 **************************************************************************/
void setupWiFi() {
  // Check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("[ERROR] WiFi shield not present");
    // don't continue:
    while (true);
  }

  // Print WiFi shield firmware version
  Serial.print("[INFO] WiFi firmware version: ");
  Serial.println(WiFi.firmwareVersion());

  // Print WiFi MAC Address
  Serial.print("[INFO] WiFi shield's MAC address: ");
  
  // Print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("[INFO] Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // Wait 10 seconds for connection:
    delay(10000);
  }

  // Print IP address received from DHCP server
  Serial.print("[INFO] Connected using IP address: ");
  Serial.println(WiFi.localIP());
}

/***************************************************************************
  Input pin A0: Read moisture sensor
***************************************************************************/
void readMoistureSensor() {
  moistureValue = analogRead(MOISTURE_SENSOR);
  sensorJSON["moisture"] = moistureValue;

}

/***************************************************************************
  Connect to predefined MQTT broker
***************************************************************************/
void connectToMQTT() {
  // clientID
  if (client.connect("d:xelcdl:ARDUINO:iot-arduino-001")) {
    Serial.println();
    Serial.print("[INFO] Connected to MQTT broker ");
    Serial.println(MQTT_SERVER);
  }
}

/***************************************************************************
  Read sensor data from sensors connected to Grove board
***************************************************************************/
void refreshMeasurements() {
  // If lost connection to MQTT broker, attempt to reconnect...
  if (!client.connected()) {
    Serial.println("[WARN] Oops....lost connection to MQTT!! Trying to reconnect");
    connectToMQTT();
  } else {
    readMoistureSensor();

    /***************************************************************************
      // Input pin A1: Read temp & humidity sensor
    ***************************************************************************/
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidity    = dht.readHumidity();
    temperature = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("[ERROR] Failed to read from DHT");
    }
    else
    {
      sensorJSON["humidity"] = humidity;
      sensorJSON["temperature"] = temperature;
    }

    /***************************************************************************
      // Input pin A2: Read light sensor
    ***************************************************************************/
    lightSensorValue = analogRead(LIGHT_SENSOR);
    sensorJSON["light"] = lightSensorValue;
  }
}
