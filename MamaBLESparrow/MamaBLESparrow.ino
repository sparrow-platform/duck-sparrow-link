// MamaBLESparrow
//
// This file is a part of Sparrow project and helps connect smartphone devices to a TTGO LoRa hardware
//
// The connection between the device and the hardware is not very stable as the hardware takes longer time to connect to devices as a client


#include <SPI.h>              // include libraries
#include <LoRa.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Ticker.h>

Ticker BLEScanner;

#define NONE 0
String buffer[16];
byte bufferIndex=0;
byte recievedBy[16]={NONE};

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
static BLEAdvertisedDevice* BLEDevices[4];
static BLERemoteCharacteristic* pRemoteCharacteristic;

uint16_t BLEDeviceCount = 0;
static boolean doConnect[4],nextConnect = true;
static boolean connected = false;
static boolean doScan = false;

const int blueLED = LED_BUILTIN; 
byte localAddress = 0xBB;     // address of this device
byte msgCount = 0;            // count of outgoing messages
byte destination = 0xCC;      // destination to send to

#define SERVICE_UUID        "bc7a0884-8f30-11e9-bc42-526af7764f64"
static BLEUUID serviceUUID(SERVICE_UUID);
#define CHARACTERISTIC_UUID "bc7a0d48-8f30-11e9-bc42-526af7764f64"
static BLEUUID    charUUID(CHARACTERISTIC_UUID);

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
    sendMessage((char*)pData);
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      Serial.println("Found sparrow device");
      Serial.println(advertisedDevice.toString().c_str());
      BLEDeviceCount %= 5;
      
      if(BLEDeviceCount == 0 || BLEDevices[BLEDeviceCount-1]->getName() != advertisedDevice.getName()) {
        BLEDevices[BLEDeviceCount] = new BLEAdvertisedDevice(advertisedDevice);
        doConnect[BLEDeviceCount] = true;
        BLEDeviceCount += 1;
        doScan = true;
      }

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      int i=0;
      int j=0;
      deviceConnected = true;
      Serial.println("Client connected");      
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Clinet Disconnected");
      //deviceConnected = false;
    }
};

class MyDescriptorCallback: public BLEDescriptorCallbacks {
  void onRead(BLEDescriptor* pDescriptor) {
     // Do something before the read completes.
  }
 
  void onWrite(BLEDescriptor* pDescriptor) {
     // Do something because a new value was written.
     Serial.println("Descriptor onWrite");
     int i=0,j=0;
     for(i=0;i<16;i++){
        if(recievedBy[i] == 0)
          continue;
        for(j=0;j < buffer[i].length();j+=20) {
          uint8_t buf[20];
          buffer[i].substring(j,j+20).getBytes(buf,20);
          pCharacteristic->setValue(buf,20);
          pCharacteristic->notify();
        }
      }
  }
};

class MyClientCallback : public BLEClientCallbacks {
  private:
    int index;
  public:
  MyClientCallback(int i):BLEClientCallbacks() {
   index = i;
  }
  
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to Server");
    doConnect[index] = false;
    nextConnect = false;
  }

  void onDisconnect(BLEClient* pclient) {
    doConnect[index] = true;
    nextConnect = true;
    Serial.println("Disconnected from Server");
  }
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("LoRa Sender");
  setupLora();

  Serial.println("Server started");

  // Create the BLE Device
  BLEDevice::init("ESP32");
  startBLEServer();

  //Scanner Ticker
  BLEScanner.attach(10,startBLEScan);
  //startBLEScan();

}

void loop() {
  // put your main code here, to run repeatedly:  
    for(int i=0;i<5;i++){
      if(doConnect[i] && nextConnect){
        delay(500);
        connectToServer(BLEDevices[i],i);
      }
    }
    LoRa.receive();
    Serial.println("After Lora recieve");
    delay(6000);
}

void sendMessage(String outgoing) {
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
  recievedBy[bufferIndex++] = 1 << 7;
  if(bufferIndex >= 16){
    bufferIndex = bufferIndex % 16;
    recievedBy[bufferIndex] = 0;
  }
}

bool connectToServer(BLEAdvertisedDevice* advertisedDevice,int i) {
    Serial.print("Forming a connection to ");
    Serial.println(advertisedDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback(i));

    // Connect to the remove BLE Server.
    pClient->connect(advertisedDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if(pRemoteCharacteristic->canNotify()){
      Serial.println(" - Registering notification callback");
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    }

    delay(5000);
    pClient->disconnect();
    
}

void startBLEScan() {
  Serial.println("Starting BLE Scan");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(3, false);
}
void startBLEServer() {
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY);


  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  BLE2902* p2902Descriptor = new BLE2902();
  p2902Descriptor->setNotifications(true);
  p2902Descriptor->setCallbacks(new MyDescriptorCallback());
  pCharacteristic->addDescriptor(p2902Descriptor);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Started BLE server");
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

