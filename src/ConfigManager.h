// ConfigManager.h
#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>

struct ConfigSettings {
    float reverseLimitPercent;   // 0.0 to 1.0
    float wheelieSuppressRatio;  // 0.0 to 1.0
    bool indoorModeEnabled;      // Indoor mode ON/OFF
    uint8_t version;
};

extern ConfigSettings currentConfig;

void loadConfig();
void saveConfig();
void printConfig();
void resetConfigToDefaults();

#endif //