#pragma once

// The OTA password is public, server should know it for OTA updates,
// and devices don't have any authentication anyway.
// We have password here to prevent random attacks on IOT things
// that might try to hack all esp-based devices and upload malicious
// firmware. We don't have any hardware buttons for the user to confirm
// OTA update, so this is the best way we have.
// OTA is allowed only for the first 60 seconds after device startup.
const char* otaPassword
	= "SlimeVR-OTA";  // YOUR OTA PASSWORD HERE, LEAVE EMPTY TO DISABLE OTA UPDATES

const char* provisioningPassword
	= "SlimeVR-Provisioning";  // YOUR PROVISIONING PASSWORD HERE, LEAVE EMPTY TO
							   // DISABLE WIFI PROVISIONING
