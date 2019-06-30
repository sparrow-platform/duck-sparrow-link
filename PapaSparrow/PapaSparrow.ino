// PapaSparrow
// Copyright Ameya Apte 2019
// MIT License
//
// This file is a part of Sparrow project and helps connect internet with the Sparrow Mesh
//
// This code recieves messeges on LoRa from slave devices parses them and publishesh them to mqtt server for routing to IBM watson backend
// The response is recieved on another mqtt topic and a strucutred messege is generated and sent back on the LoRa interface

#include <SPI.h>              // include libraries
#include <LoRa.h>

#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "18.221.210.97";

String topicsToSubscribe[16];
uint32_t subscribeIndex = 0;
byte subscribeFlag[16]={0};

WiFiClient espClient;
PubSubClient client(espClient);

StaticJsonBuffer<400> jsonBuffer;

long lastMsg = 0;
char msg[50];

uint32_t value = 0;

String subscribeTopics[16];
const int blueLED = LED_BUILTIN; 
byte localAddress = 0xCC;     // address of this device
byte msgCount = 0;            // count of outgoing messages
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  Serial.println("LoRa Sender");
  setupLora();

  LoRa.onReceive(onReceive);
  LoRa.receive();

}

void loop() {
  // put your main code here, to run repeatedly:  
   if (!client.connected()) {
    reconnect();
  }
  client.loop();
  LoRa.receive();                     // go back into receive mode
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      
    char charBuf[50];
    int i = 0;
    for(i=0;i<16;i++){
        if(subscribeFlag[i] == 0)
          continue;
        topicsToSubscribe[i].toCharArray(charBuf, 50);
        client.subscribe(charBuf);
     }

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  JsonObject& root = jsonBuffer.parseObject(payload);
  root["key"] = String(topic).substring(String(topic).indexOf('/'));
  root["messege"] = payload;
  String output;
  root.printTo(output);
  sendMessege(output);
  Serial.println("Sending received messege on LoRa");
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
  
  char topicBuf[50],dataBuf[4096];
  
  JsonObject& root = jsonBuffer.parseObject(incoming);
  String topic = "sparrow_receive/" + root["key"].as<String>();
  topic.toCharArray(topicBuf, 50);
  incoming.toCharArray(dataBuf, 4096);
  
  client.publish(topicBuf,dataBuf);

  topic = "sparrow_response/" + root["key"].as<String>();
  topic.toCharArray(topicBuf, 50);
  client.subscribe(topicBuf);
  
  topicsToSubscribe[subscribeIndex]=topic;
  topicsToSubscribe[subscribeIndex++] = 1 << 7;
  if(subscribeIndex >= 16){
    subscribeIndex = subscribeIndex % 16;
    subscribeFlag[subscribeIndex] = 0;
  }
  
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("JSON parseObject() failed");
    return;
  }
  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  
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



