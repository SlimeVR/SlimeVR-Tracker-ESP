#ifndef _OTA_H_
#define _OTA_H 1

#include <ArduinoOTA.h>

void otaSetup(const char * const otaPassword);
void otaUpdate();

#endif // _OTA_H_