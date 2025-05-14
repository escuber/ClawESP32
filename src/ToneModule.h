#ifndef TONE_MODULE_H
#define TONE_MODULE_H

enum ToneEvent {
  TONE_IDLE_OFFSET_SET,
  TONE_BLUETOOTH_NOT_CONNECTED,
  TONE_ERROR,
  TONE_WARNING,
  // Add more as needed
};

void initToneModule(int pin);  // Set the tone output pin
void playTone(ToneEvent event);

#endif