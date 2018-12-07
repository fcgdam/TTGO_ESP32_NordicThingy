/*
 *  BLE Client -sketch from examples, modified slightly for demonstration video by Hugatry's HackVlog
 */
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "BLEDevice.h"

// TTGO OLED Support. Check your board pinouts.
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET    16
#define OLED_SDA      4
#define OLED_SCL      15

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

//UUID's of the service, characteristic that we want to read and characteristic that we want to write.
static BLEUUID serviceUUID    ("ef680100-9b35-4933-9b10-52ffa9740042"); // We search the Nordic device by the advertised service
static BLEUUID environmentUUID("ef680200-9b35-4933-9b10-52ffa9740042");
static BLEUUID temperatureUUID("ef680201-9b35-4933-9b10-52ffa9740042");
static BLEUUID pressureUUID   ("ef680202-9b35-4933-9b10-52ffa9740042");
static BLEUUID humidityUUID   ("ef680203-9b35-4933-9b10-52ffa9740042");
static BLEUUID gasUUID        ("ef680204-9b35-4933-9b10-52ffa9740042");

const uint8_t bothOff[]        = {0x0, 0x0};
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t indicationOn[]   = {0x2, 0x0};
const uint8_t bothOn[]         = {0x3, 0x0};

uint8_t buffer[10];
static BLERemoteDescriptor *pRD;

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning... Hopefully.
static BLEAddress *pServerAddress;

//Characteristic that we want to read and characteristic that we want to write.
static BLERemoteCharacteristic* temperatureCharacteristic;
static BLERemoteCharacteristic* pressureCharacteristic;
static BLERemoteCharacteristic* humidityCharacteristic;
static BLERemoteCharacteristic* gasCharacteristic;
//static BLERemoteCharacteristic* buttonCharacteristic;

// Sensor data:
float  Temperature = 0;
float  Pressure    = 0;
int    Humidity    = 0;
float  CO2         = 0;
float  TVOC        = 0;


/*
   Callback function that gets called, when another device's advertisement has been received
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("Device found: ");
      //Serial.println(advertisedDevice.getName().toString());

      // Check if the device found has the Nordic Thingy52 service available.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
        // Scan can be stopped, we have found what we are looking for
        advertisedDevice.getScan()->stop();

        // We need the device address to connect to it.
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        //Serial.println(" " + pServerAddress->toString());

        // Set indicator, stating that we are ready to connect
        // This will trigger on the loop() function the server connection phase.
        doConnect = true;
        Serial.println("");

        Serial.println("Nordic Thingy52 found!");
        // We will connect at the loop() function.

        // Do not call display functions here, otherwise, the ESP might crash
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Starting Arduino BLE Client application...");

  // reset the OLED
  pinMode(OLED_RESET,OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  display.init ();
  display.flipScreenVertically ();
  display.setFont (ArialMT_Plain_10);

  display.setTextAlignment (TEXT_ALIGN_LEFT);

  // Initialize the BLE sub system
  BLEDevice::init("");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);

  display.drawString(0,0,"BLE scan...");
  display.display();

} // End of setup.

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

  BLEUUID bleuuid = pBLERemoteCharacteristic->getUUID();

  if ( bleuuid.toString() == temperatureUUID.toString() ) {
    Temperature = (float)pData[0] + ((float)pData[1])/100;
  }

  if ( bleuuid.toString() == pressureUUID.toString() ) {
    Pressure = (float)pData[0] + (float)pData[1]*256 + ((float)pData[4])/100;
  }

  if ( bleuuid.toString() == humidityUUID.toString() ) {
    Humidity = (int)pData[0];
  }

  if ( bleuuid.toString() == gasUUID.toString() ) {
    CO2  = pData[0] + pData[1]*256;
    TVOC = pData[2] + pData[3]*256;
  }
/*
  for(int i = 0; i < length; i++)
  {
    Serial.print( pData[i] , HEX);
    Serial.print(" ");
  }
  Serial.println();
*/
}

bool enableCharNotification( BLERemoteService* pRemoteService, BLERemoteCharacteristic* characteristic, BLEUUID characteristicUUID) {
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  characteristic = pRemoteService->getCharacteristic(characteristicUUID);
  //buttonCharacteristic = pRemoteService->getCharacteristic(charUUID2);
  if (characteristic == nullptr ) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");

  // Enable notifications
  Serial.println("Enabling notifications");
  pRD = characteristic->getDescriptor(BLEUUID((uint16_t)0x2902));

  if ( pRD != nullptr) {
    pRD->writeValue((uint8_t*)bothOn, 2, true);
    Serial.println("Enable notify!");
    characteristic->registerForNotify(notifyCallback);
  } else {
    Serial.println("Failed to set notification");
    return false;
  }

  return true;
}

bool connectToServer(BLEAddress pAddress) {

  BLEClient* pClient = BLEDevice::createClient();

  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(environmentUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return (false);
  }

  Serial.println("Found our service.");

  if ( enableCharNotification( pRemoteService, temperatureCharacteristic , temperatureUUID) ) {
    Serial.println("Notification for temperature set ok!");
  }

  if ( enableCharNotification( pRemoteService, pressureCharacteristic , pressureUUID) ) {
    Serial.println("Notification for pressure set ok!");
  }

  if ( enableCharNotification( pRemoteService, humidityCharacteristic , humidityUUID) ) {
    Serial.println("Notification for humidity set ok!");
  }

  if ( enableCharNotification( pRemoteService, gasCharacteristic , gasUUID) ) {
    Serial.println("Notification for TVOC gases set ok!");
  }

  return true;
}

// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;
      display.clear();
      display.drawString(0,0,"Nordic found!");
      display.display();
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    // Reset the flag, so now the loop() doesn't do anything anymore.
    doConnect = false;
  }

  // If connected, we can now show the sensor data
  if ( connected ) {
    // Show the sensed temperature value:
    display.clear();
    // On display
    display.drawString(0,0, "Temp: ");
    display.drawString(48,0 , String(Temperature) +"º");
    // On Serial
    Serial.println("The temperature is: " + String(Temperature) +"C");


    display.drawString(0,10, "Hum: ");
    display.drawString(48,10 , String(Humidity) + "%");
    Serial.println("The Humidity is: " + String(Humidity));


    display.drawString(0, 20, "Press: ");
    display.drawString(48,20, String(Pressure) +"hPa");
    Serial.println("The Pressure is: " + String(Pressure) + "hPa");


    display.drawString(0, 30, "CO2: ");
    display.drawString(48,30, String(CO2) + "ppm");
    Serial.println("The CO2 is: " + String(CO2));
    

    display.drawString(0, 40, "TVOC: ");
    display.drawString(48,40, String(TVOC) + "ppb");
    Serial.println("The TVOC is: " + String(TVOC));
    display.display();

  }

  delay(1000); // Delay a second between loops.
} // End of loop
