#include "BluetoothControl.h"

BluetoothControl::BluetoothControl(int rxPin, int txPin) 
  : bluetoothSerial(rxPin, txPin) {}

void BluetoothControl::begin(long baudRate) {
  bluetoothSerial.begin(baudRate);
}

bool BluetoothControl::readData(DataPacket &packet) {
  if (bluetoothSerial.available() >= sizeof(DataPacket)) {
    bluetoothSerial.readBytes((char *)&packet, sizeof(DataPacket));
    return true;
  }
  return false;
}
