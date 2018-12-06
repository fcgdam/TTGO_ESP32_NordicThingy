# TTGO Board ESP32 and Nordic Thingy52:

This program will connect to the Nordic Thingy:52 device to gather sensor data through BLE.

This is still a proof of concept to see how far the ESP32 BLE hardware and software goes...

The Nordic Thingy Notifications where not working due to this bug: https://github.com/nkolban/ESP32_BLE_Arduino/pull/17

Will the merge is not done to the ESP32 BLE Arduino Library we need to change the BLERemoteDescriptor.cpp file at line 151 to the new line:

From:

 ESP_GATT_WRITE_TYPE_NO_RSP,

to

 response ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP,

# Work to be done

Collect data, buffer it out, and send it to the Things Network.
