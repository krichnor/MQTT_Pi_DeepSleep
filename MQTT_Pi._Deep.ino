#include "SSD1306.h" 
#include <DHT.h>      // Library DHT-sensor-library-master.zip

////////// Setup watch Dog ////////



//////////  Sensor NAME  //////////
const char* SensorName     = "krich";
//////////  Initialize the OLED display using Wire library
SSD1306  display(0x3c, 5, 4);

//////////  DHT Setting // for Temperature sensor type DHT11
#define DHTTYPE DHT11 // Library DHT-sensor-library-master.zip
#define DHTPIN 2     // Library DHT-sensor-library-master.zip
DHT dht(DHTPIN, DHTTYPE);

// Temporary variables // for Temperature sensor type DHT11
static char celsiusTemp[7];
static char humidityTemp[7];
static char HIndTemp[7];
char charSensorDisplay[30];   //Buffer to display message

void TempSensorReadDisplay();

////////// Greeting init variables //////////
int Device_state = -2;   //1st WifiConnect= 0, reconWIFI >1
int Device_state_prev = 0;
void Greeting();

////////// WIFI & MQTT Header //////////
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid     = "true_home2G_751";
const char* password = "17a53751";
String Statuses[] =  { "WL_IDLE_STATUS(0)", "WL_NO_SSID_AVAIL(1)", "WL_SCAN_COMPLETED(2)", "WL_CONNECTED(3)", "WL_CONNECT_FAILED(4)", "WL_CONNECTION_LOST(5)", "WL_DISCONNECTED(6)"};

WiFiClient espClient;
void WIFIconnect();

////////// MQTT Setting //////////
const char* mqttServer = "192.168.1.40";
const int mqttPort = 1883;
const char* mqttUser = "ESP32";
const char* mqttPassword = "khunkrich";
PubSubClient client(espClient);
String getMessage;
char MQtopic[50];

void MQTTconnect();
void callback(char* topic, byte* payload, unsigned int length) {
  getMessage = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    getMessage = getMessage + (char)payload[i];
  }
  Serial.println(getMessage);
}

////////// NTP Setting //////////
#include <TimeLib.h>  // need PaulStoffregen library and Time-master.zip
#include <WiFiUdp.h>

static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 7;     // Central European Time (Bangkok GMT +7)
unsigned int localPort = 8888;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
WiFiUDP ntpUDP;
time_t prevDisplay = 0; // when the digital clock was displayed
void sendNTPpacket(IPAddress &address);
char charDateTime[25];
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets


void setup() {
  dht.begin();
  Serial.begin(115200);
  Greeting();

  ////////// WIFI chacking and setting up ////////////
  if (WiFi.status() != WL_CONNECTED) {
    WIFIconnect();
  }
  if (WiFi.status() == WL_CONNECTED) {
    MQTTconnect();    //MQTT setup
    ////////// NTP chacking and setting up ////////////
    if (timeStatus() != timeSet) {
      Serial.print("Start NTP Setup Process");
      setSyncProvider(getNtpTime);
      setSyncInterval(300);
      if (timeStatus() == timeSet) { //update the display only if time has changed
        Serial.print("NTP Time is set -> ");
        sprintf (charDateTime, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
        Serial.println(charDateTime);
      }
    }
  }
}

void loop() {

  // WiFi connect checking status //
  if (WiFi.status() != WL_CONNECTED) {
    setup();
  }
  TempSensorReadDisplay();

  Serial.print("Device_state = ");
  Serial.println(Device_state);

  Serial.print("Device_state_prev = ");
  Serial.println(Device_state_prev);

  ////////// Show NTS if not set ////////////
  if (timeStatus() == timeNotSet) {
    Serial.println("NTP Time is not set");
  }
  /*  ////////// Show Date -  ////////////
    if (timeStatus() != timeNotSet) {
      if (now() != prevDisplay) { //update the display only if time has changed
        prevDisplay = now();
        Serial.print("Time stamp from NTP -> ");
        digitalClockDisplay();
        Serial.println();
      }
    }   */

  delay(10000); // 10 sec interval loop
}
/*---------------------------- SUB FUNCTION ----------------------------*/

///////// KBTG Greeting message /////////
void Greeting()
{
  if (Device_state < -1) {
    ///////// Initial OLED /////////
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 10, "IoT Project");
    display.setFont(ArialMT_Plain_24);
    display.drawString(64, 30, "Temp & Hue");
    display.display();
    Device_state = -1;
    delay(2000);    // Greeting meessage will be display only one time.
  }
}

void WIFIconnect()
{
  int mytimeout = millis() / 1000;
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "KBTG IoT:");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(128, 20, "Temperature Sensor Proj.");
    display.drawString(128, 35, "Connecting to WiFi...");
    display.drawString(128, 50, "SSID: " + String(ssid));
    display.display();
    if ((millis() / 1000) > mytimeout + 4) { // try for less than 4 seconds to connect to WiFi router
      break;
    }
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("SSID: ");
    Serial.print(String(ssid));
    Serial.println(" Cannot be connected.");
    Serial.print("  WiFi.status = ");
    Serial.println(Statuses[WiFi.status()]);  //Show WIFI connection status error
  } else {
    Serial.println("Connected to the WiFi network");
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "KBTG IoT:");
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(128, 20, "Temperature Sensor Proj.");
    display.drawString(128, 35, "Connected to WiFi: Done");
    display.drawString(128, 50, "SSID: " + String(ssid));
    display.display();
    Device_state ++;
  }
}

void MQTTconnect() {
  ///////// MQTT setup /////////
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println(" Connected to MQTT: Done");
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "KBTG IoT:");
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.setFont(ArialMT_Plain_10);
      display.drawString(128, 20, "Temperature Sensor Proj.");
      display.drawString(128, 35, "Connected MQTT: Done");
      display.drawString(128, 50, "SSID: " + String(ssid));
      display.display();
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "IoT Project");
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.setFont(ArialMT_Plain_10);
      display.drawString(128, 20, "Temperature Sensor Proj.");
      display.drawString(128, 35, "Connecting to Pi MQTT");
      display.drawString(128, 50, "SSID: " + String(ssid));
      display.display();
      delay(1000);  //Re-connecting in a minute
    }
  }
  client.disconnect();
}

void TempSensorReadDisplay()
{
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  Serial.println();
  Serial.println("==========================");
  Serial.println("Temp value on pin 25 is ");
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    // Display Sensor Value to OLED display //
    display.clear();
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Project IoT: Temp.");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 20, "DHT sensor cannot be read");
    display.drawString(0, 35, "Please checking connector");
    display.drawString(0, 50, "and sensor.");
    display.display();
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  // Display Sensor Value to COM port //
  sprintf (charSensorDisplay, "   Temperature: %2.2f *C ", t);
  Serial.println(charSensorDisplay);
  sprintf (charSensorDisplay, "      Humidity: %2.2f  %% ", h);
  Serial.println(charSensorDisplay);
  sprintf (charSensorDisplay, "   Temperature: %2.2f *C ", hic);
  Serial.println(charSensorDisplay);
  Serial.println("==========================");

  // Display Sensor Value to OLED display //
  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "IoT Project: Temp.");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(120, 20, "Temperature: " + String(t) + " *C");
  display.drawString(120, 35, "Humidity: " + String(h) + "  %");
  display.drawString(120, 50, "Heat index: " + String(hic) + " *C");

  display.display();

  client.loop();
  dtostrf(t, 6, 2, celsiusTemp);
  dtostrf(h, 6, 2, humidityTemp);
  dtostrf(hic, 6, 2, HIndTemp);

  ////////// Publish MQTT data ////////////
  if (WiFi.status() == WL_CONNECTED) {
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      sprintf (charDateTime, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
      sprintf (MQtopic, "%s/Temp", SensorName);
      client.publish(MQtopic, celsiusTemp);
      sprintf (MQtopic, "%s/Hue", SensorName);
      client.publish(MQtopic, humidityTemp);
      sprintf (MQtopic, "%s/HInd", SensorName);
      client.publish(MQtopic, HIndTemp);
      if (Device_state < 1) {
        sprintf (MQtopic, "%s/TimeBoot", SensorName);
        client.publish(MQtopic, charDateTime);
        Device_state ++;
        Device_state_prev = Device_state;
        Serial.print("After stamp boot time, change Device_state to ");
        Serial.println(Device_state);
      } else if (Device_state != Device_state_prev) {
        sprintf (MQtopic, "%s/TimeReconWIFI", SensorName);
        client.publish(MQtopic, charDateTime);
        sprintf (charDateTime, "%i", Device_state);
        sprintf (MQtopic, "%s/TimeReconTime", SensorName);
        client.publish(MQtopic, charDateTime);
        Device_state_prev = Device_state;
      } else {
        sprintf (MQtopic, "%s/TimeStamp", SensorName);
        client.publish(MQtopic, charDateTime);
        client.disconnect();
      }
    } else {
        Serial.println("Connecting to MQTT");
        Serial.print("failed with state ");
        Serial.print(client.state());
    }
  }  
}


/*-------- NTP code ----------*/
time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (ntpUDP.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = ntpUDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      ntpUDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUDP.beginPacket(address, 123); //NTP requests are to port 123
  ntpUDP.write(packetBuffer, NTP_PACKET_SIZE);
  ntpUDP.endPacket();
}
