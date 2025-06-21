#include "SensorToggles.h"

void SensorToggleState::setToggle(SensorToggles toggle, bool state) {
	switch (toggle) {
		case SensorToggles::MagEnabled:
			magEnabled = state;
			break;
		case SensorToggles::CalibrationEnabled:
			calibrationEnabled = state;
			break;
		case SensorToggles::TempGradientCalibrationEnabled:
			tempGradientCalibrationEnabled = state;
			break;
	}
}

bool SensorToggleState::getToggle(SensorToggles toggle) const {
	switch (toggle) {
		case SensorToggles::MagEnabled:
			return magEnabled;
		case SensorToggles::CalibrationEnabled:
			return calibrationEnabled;
		case SensorToggles::TempGradientCalibrationEnabled:
			return tempGradientCalibrationEnabled;
	}
	return false;
}

void SensorToggleState::onToggleChange(
	std::function<void(SensorToggles, bool)>&& callback
) {
	this->callback = callback;
}

void SensorToggleState::emitToggleChange(SensorToggles toggle, bool state) const {
	if (callback) {
		(*callback)(toggle, state);
	}
}

const char* SensorToggleState::toggleToString(SensorToggles toggle) {
	switch (toggle) {
		case SensorToggles::MagEnabled:
			return "MagEnabled";
		case SensorToggles::CalibrationEnabled:
			return "CalibrationEnabled";
		case SensorToggles::TempGradientCalibrationEnabled:
			return "TempGradientCalibrationEnabled";
	}
	return "Unknown";
}
