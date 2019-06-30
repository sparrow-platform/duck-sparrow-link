# duck-sparrow-link
LoRa based bakcend for Sparrow Mesh

# Overview
This repository helps establish connecetion from smartphones to IBM watson backend via network of LoRa/BLE/WiFi interfaces.
The hardware primarily consists of 2 types of devices Mama and Papa devices. The Mama devices connect to smartphones via BLE and Wifi
and recieve data which is forwarded to Papa devices over LoRa. The Papa devices on recieving this data parse it and publish it on a 
appropriate mqtt topic so that it reaches the IBM watson backend. THe response is asynchronously recieved on a differernt mqtt topic
which is again sent back to Mama over LoRa and then is recieved by the smartphone
