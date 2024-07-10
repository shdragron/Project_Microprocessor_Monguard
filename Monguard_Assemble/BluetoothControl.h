#ifndef BluetoothControl_h
#define BluetoothControl_h

#include <SoftwareSerial.h>

struct DataPacket {
  char DIR_FBL;
  char DIR_FBR;
  char DIR_LR;
  int V_Left;
  int V_Right;
  char buttons[5]; // Buttons A, B, C, D, E
};

class BluetoothControl {
  public:
    BluetoothControl(int rxPin, int txPin);
    void begin(long baudRate);
    bool readData(DataPacket &packet);

  private:
    SoftwareSerial bluetoothSerial;
};

#endif
