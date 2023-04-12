#define TX 1
#define RX 3
#define FSR 37
#include <SPI.h>
#include <Ethernet.h>

const int serverPort = 4080;
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
// EthernetServer TCPserver(serverPort);

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200, SERIAL_8N1, RX, TX);

  // Serial1.begin(9600);

  // // Initialize Ethernet Shield:
  // if (Ethernet.begin(mac) == 0)
  //   Serial1.println("Failed to configure Ethernet using DHCP");

  // // Print your local IP address:
  // Serial.print("TCP Server IP address: ");
  // Serial.println(Ethernet.localIP());
  // Serial.println("-> Please update the serverAddress in Arduino #1 code");

  // // Listening for a TCP client (from Arduino #1)
  // TCPserver.begin();

}

void loop() {
  //EthernetClient client = TCPserver.available();

  //if (client) {
    // Read the command from the TCP client:
    //char command = client.read();
    //Serial.print("- Received command: ");
    //Serial.println(command);

    //if (command == '1')
      //digitalWrite(LED_PIN, HIGH); // Turn LED on
    //else if (command == '0')
      //digitalWrite(LED_PIN, LOW);  // Turn LED off

    //Ethernet.maintain();
    
  //Serial.write("-1,0");
  //delay(5000);
  Serial.write('0');
  delay(1000);

// Wait for a TCP client from Arduino #1:
  
  }

