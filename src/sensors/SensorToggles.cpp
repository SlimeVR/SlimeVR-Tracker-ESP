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

	for (auto& callback : onToggleChangeCallbacks) {
		callback(toggle, state);
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
	onToggleChangeCallbacks.push_back(callback);
}
