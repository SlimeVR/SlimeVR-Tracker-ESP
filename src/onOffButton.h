#ifndef SLIMEVR_ONOFFBUTTON_H_
#define SLIMEVR_ONOFFBUTTON_H_

#include "globals.h"
#include "logging/Logger.h"

#ifndef ON_OFF_BUTTON_ACTIVE_LEVEL
#define ON_OFF_BUTTON_ACTIVE_LEVEL LOW
#endif

namespace SlimeVR {

class OnOffButton {
public:
	void setup();
	void update();

private:
	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("OnOffButton");
};

}  // namespace SlimeVR

#endif
