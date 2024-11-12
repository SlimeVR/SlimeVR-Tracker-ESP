/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "GyroTemperatureCalibrator.h"

#include "GlobalVars.h"

void GyroTemperatureCalibrator::resetCurrentTemperatureState() {
	if (!state.numSamples) {
		return;
	}
	state.numSamples = 0;
	state.tSum = 0;
	state.xSum = 0;
	state.ySum = 0;
	state.zSum = 0;
}

// must be called for every raw gyro sample
void GyroTemperatureCalibrator::updateGyroTemperatureCalibration(
	const float temperature,
	const bool restDetected,
	int16_t x,
	int16_t y,
	int16_t z
) {
	if (!restDetected) {
		return resetCurrentTemperatureState();
	}

	if (temperature < TEMP_CALIBRATION_MIN && !calibrationRunning) {
		calibrationRunning = true;
		configSaved = false;
		config.reset();
	}
	if (temperature > TEMP_CALIBRATION_MAX && calibrationRunning) {
		auto coeffs = poly.computeCoefficients();
		for (uint32_t i = 0; i < poly.numCoefficients; i++) {
			config.cx[i] = coeffs[0][i];
			config.cy[i] = coeffs[1][i];
			config.cz[i] = coeffs[2][i];
		}
		config.hasCoeffs = true;
		bst = 0.0f;
		bsx = 0;
		bsy = 0;
		bsz = 0;
		bn = 0;
		lastTemp = 0;
		calibrationRunning = false;
		if (!configSaveFailed && !configSaved) {
			saveConfig();
		}
	}
	if (calibrationRunning) {
		if (fabs(lastTemp - temperature) > 0.03f) {
			const double avgt = (double)bst / bn;
			const double avgValues[] = {
				(double)bsx / bn,
				(double)bsy / bn,
				(double)bsz / bn,
			};
			if (bn > 0) {
				poly.update(avgt, avgValues);
			}
			bst = 0.0f;
			bsx = 0;
			bsy = 0;
			bsz = 0;
			bn = 0;
			lastTemp = temperature;
		} else {
			bst += temperature;
			bsx += x;
			bsy += y;
			bsz += z;
			bn++;
		}
	}

	const int16_t idx = TEMP_CALIBRATION_TEMP_TO_IDX(temperature);

	if (idx < 0 || idx >= TEMP_CALIBRATION_BUFFER_SIZE) {
		return;
	}

	bool currentTempAlreadyCalibrated = config.samples[idx].t != 0.0f;
	if (currentTempAlreadyCalibrated) {
		return;
	}

	if (state.temperatureCurrentIdx != idx) {
		state.temperatureCurrentIdx = idx;
		resetCurrentTemperatureState();
	}

	float temperatureStepBoundsMin
		= TEMP_CALIBRATION_IDX_TO_TEMP(idx) - TEMP_CALIBRATION_MAX_DEVIATION_FROM_STEP;
	float temperatureStepBoundsMax
		= TEMP_CALIBRATION_IDX_TO_TEMP(idx) + TEMP_CALIBRATION_MAX_DEVIATION_FROM_STEP;
	bool isTemperatureOutOfDeviationRange = temperature < temperatureStepBoundsMin
										 || temperature > temperatureStepBoundsMax;
	if (isTemperatureOutOfDeviationRange) {
		return resetCurrentTemperatureState();
	}

	state.numSamples++;
	state.tSum += temperature;
	state.xSum += x;
	state.ySum += y;
	state.zSum += z;
	if (state.numSamples > samplesPerStep) {
		bool currentTempAlreadyCalibrated = config.samples[idx].t != 0.0f;
		if (!currentTempAlreadyCalibrated) {
			config.samplesTotal++;
		}
		config.samples[idx].t = state.tSum / state.numSamples;
		config.samples[idx].x = ((double)state.xSum / state.numSamples);
		config.samples[idx].y = ((double)state.ySum / state.numSamples);
		config.samples[idx].z = ((double)state.zSum / state.numSamples);

		config.minTemperatureRange
			= min(config.samples[idx].t, config.minTemperatureRange);
		config.maxTemperatureRange
			= max(config.samples[idx].t, config.maxTemperatureRange);
		config.minCalibratedIdx
			= TEMP_CALIBRATION_TEMP_TO_IDX(config.minTemperatureRange);
		config.maxCalibratedIdx
			= TEMP_CALIBRATION_TEMP_TO_IDX(config.maxTemperatureRange);
		resetCurrentTemperatureState();
	}
}

bool GyroTemperatureCalibrator::approximateOffset(
	const float temperature,
	float GOxyz[3]
) {
	if (!config.hasData()) {
		return false;
	}
	if (config.hasCoeffs) {
		if (lastApproximatedTemperature != 0.0f
			&& temperature == lastApproximatedTemperature) {
			GOxyz[0] = lastApproximatedOffsets[0];
			GOxyz[1] = lastApproximatedOffsets[1];
			GOxyz[2] = lastApproximatedOffsets[2];
		} else {
			float offsets[3] = {config.cx[3], config.cy[3], config.cz[3]};
			for (int32_t i = 2; i >= 0; i--) {
				offsets[0] = offsets[0] * temperature + config.cx[i];
				offsets[1] = offsets[1] * temperature + config.cy[i];
				offsets[2] = offsets[2] * temperature + config.cz[i];
			}
			lastApproximatedTemperature = temperature;
			lastApproximatedOffsets[0] = GOxyz[0] = offsets[0];
			lastApproximatedOffsets[1] = GOxyz[1] = offsets[1];
			lastApproximatedOffsets[2] = GOxyz[2] = offsets[2];
		}
		return true;
	}

	const float constrainedTemperature = constrain(
		temperature,
		config.minTemperatureRange,
		config.maxTemperatureRange
	);

	const int16_t idx = TEMP_CALIBRATION_TEMP_TO_IDX(constrainedTemperature);

	if (idx < 0 || idx >= TEMP_CALIBRATION_BUFFER_SIZE) {
		return false;
	}

	bool isCurrentTempCalibrated = config.samples[idx].t != 0.0f;
	if (isCurrentTempCalibrated) {
		GOxyz[0] = config.samples[idx].x;
		GOxyz[1] = config.samples[idx].y;
		GOxyz[2] = config.samples[idx].z;
		return true;
	}

	return false;
}

bool GyroTemperatureCalibrator::loadConfig(float newSensitivity) {
	bool ok = configuration.loadTemperatureCalibration(sensorId, config);
	if (ok) {
		config.rescaleSamples(newSensitivity);
		if (config.fullyCalibrated()) {
			configSaved = true;
		}
	} else {
		m_Logger.warn(
			"No temperature calibration data found for sensor %d, ignoring...",
			sensorId
		);
		// m_Logger.info("Temperature calibration is advised");
		m_Logger.info(
			"Temperature calibration is a work-in-progress feature; any changes to its "
			"parameters or updates will render the saved temperature curve invalid and "
			"unloadable."
		);
	}
	return configSaved;
}

bool GyroTemperatureCalibrator::saveConfig() {
	if (configuration.saveTemperatureCalibration(sensorId, config)) {
		m_Logger.info(
			"Saved temperature calibration config (%0.1f%) for sensorId:%i",
			config.getCalibrationDonePercent(),
			sensorId
		);
		if (config.fullyCalibrated()) {
			configSaved = true;
		} else {
			m_Logger.info("Calibration will resume from this checkpoint after reboot");
		}
	} else {
		configSaveFailed = true;
		m_Logger.error("Something went wrong");
	}
	return configSaved;
}
