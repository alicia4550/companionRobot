/*
  AUTHORS: Rahul "Ray" A. Sundarrajan, Vivian Li
*/

#include <Arduino.h>
#include <bits/stdc++.h>

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEBeacon.h>
#include <BLEUUID.h>

#include <cmath>

#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0x30, 0x83, 0x98, 0xEE, 0x4D, 0xEC};
 //30:83:98:EE:4D:EC robot
esp_now_peer_info_t peerInfo;
// Variable to store if sending data was successful
String success;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

//#define ENDIAN_CHANGE_U16(x) ((((x) &0xFF00) >> 8) + (((x) &0xFF) << 8))

// The beacons' names will all begin with this prefix.
const char* PREFIX = "ESP32";

/* These strings are used to specify the Version 4 UUIDs of the beacons used
   in trilateration. They were randomly generated, as per the V4 spec. They
   must match the beacons of interest, such that they can be identified over
   the network. */
std::string REF_BEACON_A_STRING = "7a0247e7-8e88-409b-a959-ab5092ddb03e";
std::string REF_BEACON_B_STRING = "1b906e62-46c2-4a7a-b4f8-525313caf89f";
std::string REF_BEACON_C_STRING = "466e341e-5777-4675-bd17-404e4d2324b5";
std::string REF_BEACON_P_STRING = "507e77a2-035b-45f2-a135-f714087f4809";


// Instantiating a scan, as defined by Kolban's BLEScan class.
BLEScan *pBLEScan;

/* DESC: Taken from Giampietro Nigro's Master's Thesis, n represents the FREE
         SPACE FACTOR, which is an environmental factor describing the perceived
         signal strength. It must be experimentally determined for each deployment
         environment, but can be thought of as being related to free-space path
         loss.
   VALS: It ranges from a value of 2 (weak connection) to 4 (strong connection).
*/
float n = 2.95;

/* DESC: Flag used to indicate whether to run troubleshooting code segments that can be observed
         in the serial monitor.
   VALS: Boolean, i.e., true (troubleshooting), false (not troubleshooting).
*/
bool troubleshooting = true;

const int NUM_SAMPLES = 8;
const int NUM_MEDIANS = 4;
const int NUM_BEACONS = 4;

/* DESC: Used to store raw beacon distance estimates. The rows are mapped as follows:
          0 – BEACON A: Reference Beacon A
          1 – BEACON B: Reference Beacon B
          2 – BEACON C: Reference Beacon C
          3 – BEACON P: Reference Beacon P
   VALS: Positive real numbers. */
float distanceMeasurements[NUM_BEACONS][NUM_SAMPLES];

/* DESC: Used to store the number of the distance estimates taken, where the indices are:
          0 – BEACON A: Reference Beacon A
          1 – BEACON B: Reference Beacon B
          2 – BEACON C: Reference Beacon C
          3 – BEACON P: Reference Beacon P
   VALS: Natural Numbers. */
int distanceMeasurementRecords[NUM_BEACONS];

/* DESC: Used to store the median of the distance estimates. The rows are mapped as follows:
          0 – BEACON A: Reference Beacon A
          1 – BEACON B: Reference Beacon B
          2 – BEACON C: Reference Beacon C
          3 – BEACON P: Reference Beacon P
   VALS: Positive real numbers. */
float distanceMedians[NUM_BEACONS][NUM_MEDIANS];

int distanceMedianRecords[NUM_BEACONS];
float distanceFinals[NUM_BEACONS];
float coordinates[2];
int beaconNumber = -1;


int fsrPin = 37;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      if (advertisedDevice.haveName())
      {
        //if(troubleshooting) { Serial.println("Found device..."); }

        // Reporting when an ESP32 has been scanned to the serial monitor for troubleshooting.
        if (strncmp(advertisedDevice.getName().c_str(), PREFIX, strlen(PREFIX)) == 0)
        {
          //Serial.print("Found an ESP32!\n");
          //Serial.print("Device name: ");
          //Serial.println(advertisedDevice.getName().c_str());
          //Serial.println("");

          BLEUUID devUUID;

          // Checking and conditionally reporting whether the ESP32 has a UUID.
          //Serial.print(advertisedDevice.haveServiceUUID());
          if (advertisedDevice.haveServiceUUID())
          {
            devUUID = advertisedDevice.getServiceUUID();

            if (troubleshooting)
            {
              //Serial.print("Found ServiceUUID: ");
              //Serial.println(devUUID.toString().c_str());
              //Serial.println("");
            }

            // Checking whether the ESP32 with a UUID has valid manufacturer data.
            if (advertisedDevice.haveManufacturerData() == true)
            {
              std::string strManufacturerData = advertisedDevice.getManufacturerData();

              uint8_t cManufacturerData[100];
              strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
             
              if (strcmp(devUUID.toString().c_str(), REF_BEACON_A_STRING.c_str()) == 0) { beaconNumber = 0; }
              else if (strcmp(devUUID.toString().c_str(), REF_BEACON_B_STRING.c_str()) == 0) { beaconNumber = 1; }
              else if (strcmp(devUUID.toString().c_str(), REF_BEACON_C_STRING.c_str()) == 0) { beaconNumber = 2; }
              else if (strcmp(devUUID.toString().c_str(), REF_BEACON_P_STRING.c_str()) == 0) { beaconNumber = 3; }
              else { beaconNumber = -2; }
              // Serial.printf("Beacon Number: %d\n", beaconNumber);

              if (strManufacturerData.length() == 25 &&
                  cManufacturerData[0] == 0x4C && // The 0x004C manufacturer code is specific to iBeacons.
                  cManufacturerData[1] == 0x00 &&
                  beaconNumber >= 0
                )
              {
                //if (troubleshooting) { Serial.println("Found a Beacon!"); }

                BLEBeacon oBeacon = BLEBeacon();
                oBeacon.setData(strManufacturerData);
                //Serial.printf("iBeacon Frame\n");
                //Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d RSSI: %d TX Power: %d\n",
                              // oBeacon.getManufacturerId(),
                              // ENDIAN_CHANGE_U16(oBeacon.getMajor()),
                              // ENDIAN_CHANGE_U16(oBeacon.getMinor()),
                              // oBeacon.getProximityUUID().toString().c_str(),
                              // oBeacon.getSignalPower(),
                              // advertisedDevice.getRSSI(),
                              // advertisedDevice.getTXPower());

                float distance = pow(10,((advertisedDevice.getTXPower() - advertisedDevice.getRSSI()) / (10 * n)));

                distanceMeasurements[beaconNumber][distanceMeasurementRecords[beaconNumber]] = distance;
                distanceMeasurementRecords[beaconNumber] += 1;

                if (distanceMeasurementRecords[beaconNumber] > NUM_SAMPLES)
                {
                  int temp[NUM_SAMPLES];
                  for (int i = 0; i < NUM_SAMPLES; i++)
                  {
                      temp[i] = distanceMeasurements[beaconNumber][i];
                  }

                  std::sort(temp, temp + NUM_SAMPLES);

                  if (NUM_SAMPLES % 2 != 0)
                  {
                    distanceMedians[beaconNumber][distanceMedianRecords[beaconNumber]] = temp[NUM_SAMPLES/2];
                  }
                  else
                  {
                    distanceMedians[beaconNumber][distanceMedianRecords[beaconNumber]] = (temp[(NUM_SAMPLES-1)/2] + temp[NUM_SAMPLES/2])/2;
                  }

                  distanceMeasurementRecords[beaconNumber] = 0;
                  distanceMedianRecords[beaconNumber] += 1;

                  if (distanceMedianRecords[beaconNumber] > NUM_MEDIANS)
                  {
                    int temp_med[NUM_MEDIANS];
                    for (int j = 0; j < NUM_MEDIANS; j++)
                    {
                        temp_med[j] = distanceMedians[beaconNumber][j];
                    }

                    std::sort(temp_med, temp_med + NUM_MEDIANS);

                    if (NUM_MEDIANS % 2 != 0)
                    {
                      distanceFinals[beaconNumber] = temp_med[NUM_MEDIANS/2];
                    }
                    else
                    {
                      distanceFinals[beaconNumber] = (temp_med[(NUM_MEDIANS-1)/2] + temp_med[NUM_MEDIANS/2])/2;
                    }

                    distanceMedianRecords[beaconNumber] = 0;
                  }
                }

                Serial.printf("Distance to Beacon: %f\n", distanceFinals[beaconNumber]);
               
                coordinates[0] = (-2 * pow(distanceFinals[0]/100,2) - 12.96 + pow(distanceFinals[1]/100,2) + pow(distanceFinals[2]/100,2))/7.2;
                coordinates[1] = (- pow(distanceFinals[1]/100,2) + pow(distanceFinals[2]/100,2))/7.2;
               

              }
            }
          }
        }
      }
    }
};

typedef struct struct_message {
  String data;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message myData;

void setup() {
  // Serial communication set at 115200 baud.
  Serial.begin(115200);
  Serial.println("Initiating Scan...");

  // Initializing the Beacon Scanner as a BLE Device.
  BLEDevice::init("");

  // Creating a new scan instance.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

  // Opting for ACTIVE BLE scanning to maximize the number of detected responses.
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
 
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
 
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


 
}

void loop() {
  // put your main code here, to run repeatedly:
  // The amount of time spent on a single scan, specified in seconds.
  int scanTime = 5;

  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

  fsrReading = analogRead(fsrPin);
  Serial.print("Analog reading = ");
  Serial.print(fsrReading);     // print the raw analog reading
  // fsrReading = 300;
  if (fsrReading > 200) {
    Serial.println(" - Light touch");
    String data = String(coordinates[0]) + "," + String(coordinates[1]);
    myData.data = data;
    Serial.println(data);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    // Serial.println(coordinates[0]);
    // Serial.println(coordinates[1]);
 
  }
  else {
    Serial.println(" - No touch");
    String data = "0";
    myData.data = data;
    Serial.println(data);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  }
 
  delay(1000);
}
 
 