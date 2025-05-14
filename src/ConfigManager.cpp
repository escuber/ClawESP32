
// ConfigManager.cpp
#include "ConfigManager.h"
#include <EEPROM.h>

#define EEPROM_SIZE 64
#define CONFIG_ADDRESS 32

ConfigSettings currentConfig;

void loadConfig() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(CONFIG_ADDRESS, currentConfig);
    if (currentConfig.version != 1) {
        Serial.println("[CONFIG] No valid config found, loading defaults.");
        resetConfigToDefaults();
    }
}

void saveConfig() {
    EEPROM.put(CONFIG_ADDRESS, currentConfig);
    EEPROM.commit();
    Serial.println("[CONFIG] Settings saved to EEPROM.");
}

void printConfig() {
    Serial.printf("[CONFIG] Reverse Limit: %.2f | Wheelie Suppress: %.2f | Indoor Mode: %s\n",
                  currentConfig.reverseLimitPercent,
                  currentConfig.wheelieSuppressRatio,
                  currentConfig.indoorModeEnabled ? "ON" : "OFF");
}

void resetConfigToDefaults() {
    currentConfig.reverseLimitPercent = 0.1f;
    currentConfig.wheelieSuppressRatio = 0.3f;
    currentConfig.indoorModeEnabled = false;
    currentConfig.version = 1;
    saveConfig();
    Serial.println("[CONFIG] ✅ Configuration reset to defaults.");
}
