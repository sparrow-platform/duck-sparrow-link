// MamaWiFiSparrow
// This file is a part of Sparrow project and helps connect smartphone devices to a TTGO LoRa hardware via WiFi
//
// A HTTP server is served over the esp32 hardware, the messeges recieved on this interface are forwarded to LoRa and the messeges
// recieved on the LoRa interface are sent back as a response asynchronously
#include <SPI.h>              // include libraries
#include <LoRa.h>

#include "FS.h"
#include "SPIFFS.h"

#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>

#define FORMAT_SPIFFS_IF_FAILED true
#define NONE 0

// Set these to your desired credentials.
const char *ssid = "Sparrow";

const int blueLED = LED_BUILTIN; 
byte localAddress = 0xCC;     // address of this device
byte msgCount = 0;            // count of outgoing messages
byte destination = 0xBB;      // destination to send to


String buffer[16];
byte bufferIndex=0;
byte bufferFlag[16]={NONE};

WebServer server(80);

void sendToLora(){
  sendMessege(server.arg("data"));
}

void receivedLora(){
  int i=0;
  String response = "[";
  for(i=0;i<16;i++){
        if(bufferFlag[i] == 0)
          continue;
        response += buffer[i];
        response += ",";
  }
        response += "]";
          server.send(200, "application/json",response);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    
  Serial.println();
  Serial.println("Configuring access point...");


  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);


  Serial.println("WiFi Server started");
  
  server.on("/send", sendToLora);
  server.on("/receive",receivedLora);
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/gauge.min.js", SPIFFS, "/gauge.min.js");



  server.begin();
  Serial.println("LoRa Sender");
  setupLora();

}

void loop() {
  // put your main code here, to run repeatedly:  
    LoRa.receive();
}

void sendMessege(String outgoing) {
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount++);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  /*
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
*/
  buffer[bufferIndex]=incoming;
  bufferFlag[bufferIndex++] = 1 << 7;
  if(bufferIndex >= 16){
    bufferIndex = bufferIndex % 16;
    bufferFlag[bufferIndex] = 0;
  }
}

void setupLora() {
    // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  pinMode(blueLED, OUTPUT); // For LED feedback

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.onReceive(onReceive);
  LoRa.receive();
  
  Serial.println("LoRa init succeeded.");
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(12); // ranges from 6-12, default 7 see API docs

  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(14, PA_OUTPUT_RFO_PIN);
}

