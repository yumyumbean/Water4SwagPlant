/* 
 * Project TheLazyBum Plant Waterer
 * Author: Ethan Walper
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "IoTClassroom_CNM.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Credentials.h"
#include "Air_Quality_Sensor.h"
#include "Bitmaps.h"

const int OLED_RESET=-1;
const int bMe = 0x76;
const int oLed = 0x3C;
float tempF;
float pressPA;
float humidRH;
bool status;
int soilPin = D13;
int pump = D16;
int READING;
int currentTime;
int lastSecond;
int timer30S = 30000;
int timer1S = 1500;
int timer5S = 2000;
int timer10S = 5000;
int timer15S = 7000;
int timer20S = 9000;

IoTTimer timerOne;
IoTTimer timerTwo;
IoTTimer timerThree;
IoTTimer timerFour;
IoTTimer timerFive;
IoTTimer timerSix;
AirQualitySensor sensor(A1);

float pubValue, subValue;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe subFeedB = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonOnOff"); //Button from Adafruit
Adafruit_MQTT_Publish pubFeedAQ = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality"); //Air Quality
Adafruit_MQTT_Publish pubFeedSM = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/SoilMoisture"); //Soil Moisture
Adafruit_MQTT_Publish pubFeedAH = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity"); //Air Humidity
Adafruit_MQTT_Publish pubFeedAP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure"); //Air Pressure
Adafruit_MQTT_Publish pubFeedT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temp"); //Tempurature


SYSTEM_MODE(AUTOMATIC);

void MQTT_connect();
bool MQTT_ping();

// setup() runs once, when the device is first turned on
void setup(void) {
  pinMode(D13, INPUT); 
  pinMode(D16, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display(); /// display splash screen
  delay(2000);

  status = bme.begin(bMe); //turn on BME
  delay(1000);
    // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  // Serial.printf("\n\n");
  //   Serial.printf("\n\n");
  //   while (!Serial);

  //   Serial.println("INITIALIZING SENSOR");

    if (sensor.init()) {
        //Serial.println("SENSOR ON.");
    } else {
        //Serial.println("SENSOR ERROR!");
    }

  // Setup MQTT subscription
  mqtt.subscribe(&subFeedB);
  timerOne.startTimer(timer30S);
  timerThree.startTimer(timer10S);
  timerFour.startTimer(timer15S);
  timerFive.startTimer(timer20S);
  timerSix.startTimer(timer20S);
}

void loop(void) {
  MQTT_connect();
  MQTT_ping();
  READING = analogRead(soilPin);

  tempF = (bme.readTemperature()*1.8) +32; // deg C
  pressPA = (bme.readPressure()*0.00029529971444518); // pascals
  humidRH = bme.readHumidity(); //%RH
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
     if (subscription == &subFeedB) {
       subValue = atof((char *)subFeedB.lastread);
       if (subValue == 1){
     digitalWrite(pump, HIGH); //when the Adafruit button is clicked, turn pump on
     digitalWrite(D7, HIGH);
   timerTwo.startTimer(timer1S);
  }
}
}    
   
if (timerTwo.isTimerReady()){
  digitalWrite(pump, LOW);
  digitalWrite(D7, LOW);
}
    

  if (timerOne.isTimerReady()){
    pubFeedSM.publish(READING);    
    pubFeedAH.publish(humidRH); 
    pubFeedAP.publish(pressPA);  
    pubFeedT.publish(tempF);
    timerOne.startTimer(timer30S);
  }
    if (READING >= 2500){
      digitalWrite(pump, HIGH); //when the soil is dry, turn pump on
      digitalWrite(D7, HIGH);
      timerTwo.startTimer(timer1S);
    }
    
      if (timerSix.isTimerReady()){
        int quality = sensor.slope();
        if (quality == AirQualitySensor::FORCE_SIGNAL) {
        //Serial.println("EXTREME POLLUTION! SAVE YOUR PLANT!");
            pubFeedAQ.publish("EXTREME POLLUTION! SAVE YOUR PLANT!");
            display.clearDisplay();
            display.drawBitmap(0, 1,  bitmap_VDIRTYPLANT, 128, 64, WHITE);
            display.display();
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
           pubFeedAQ.publish("Your air is very dirty!");
        // Serial.println("Your air is very dirty!");
            display.clearDisplay();
            display.drawBitmap(0, 1,  bitmap_DIRTYPLANT, 128, 64, WHITE);
            display.display();
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
           pubFeedAQ.publish("Your air is a lil dirty!");
        // Serial.println("Your air is a lil dirty!");
            display.clearDisplay();
            display.drawBitmap(0, 1,  bitmap_MILDPLANT, 128, 64, WHITE);
            display.display();
    } else if (quality == AirQualitySensor::FRESH_AIR) {
           pubFeedAQ.publish("Fresh as heck.");
        // Serial.println("Fresh as heck.");
            display.clearDisplay();
            display.drawBitmap(0, 1,  bitmap_HAPPYPLANT, 128, 64, WHITE);
            display.display();
          }
            timerSix.startTimer(timer20S);
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds and try again
  }
    Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
    Serial.printf("Disconnecting \n");
    mqtt.disconnect();
  }
    last = millis();
  }
    return pingStatus;
}
