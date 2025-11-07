#include "SensorToggles.h"

SensorToggleState::SensorToggleState(SensorToggleValues values)
	: values{values} {}

void SensorToggleState::setToggle(SensorToggles toggle, bool state) {
	switch (toggle) {
		case SensorToggles::MagEnabled:
			values.magEnabled = state;
			break;
		case SensorToggles::CalibrationEnabled:
			values.calibrationEnabled = state;
			break;
		case SensorToggles::TempGradientCalibrationEnabled:
			values.tempGradientCalibrationEnabled = state;
			break;
	}
}

bool SensorToggleState::getToggle(SensorToggles toggle) const {
	switch (toggle) {
		case SensorToggles::MagEnabled:
			return values.magEnabled;
		case SensorToggles::CalibrationEnabled:
			return values.calibrationEnabled;
		case SensorToggles::TempGradientCalibrationEnabled:
			return values.tempGradientCalibrationEnabled;
	}
	return false;
}

void SensorToggleState::onToggleChange(
	std::function<void(SensorToggles, bool)>&& callback
) {
	this->callback = callback;
}

SensorToggleValues SensorToggleState::getValues() const { return values; }

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
