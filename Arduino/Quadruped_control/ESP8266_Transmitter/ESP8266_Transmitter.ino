
// Fill-in information from your Blynk Template here
/*
#define BLYNK_TEMPLATE_ID "TMPLoua0H9la"
#define BLYNK_DEVICE_NAME "KendaliServo"
/*
#define BLYNK_TEMPLATE_ID "TMPL4giTRFIu"
#define BLYNK_DEVICE_NAME "Kelompok Sistem Embedded"
*/

#define BLYNK_TEMPLATE_ID "xxxxxxxxxxxx"
#define BLYNK_DEVICE_NAME "xxxxxxxxxxxx"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
//#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h"

int state = 0;


BLYNK_WRITE(V0)
{
  state = param.asInt();
  if(state == 1)Serial.write('a');
  else if(state == 2)Serial.write('b');
  else if(state == 3)Serial.write('c');
  else Serial.write(' ');
  delay(1000);
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  BlynkEdgent.begin();
}

void loop() {
  BlynkEdgent.run();
}
