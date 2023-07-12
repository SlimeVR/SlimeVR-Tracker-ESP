#include "remotecmd.h"

#include "GlobalVars.h"

namespace SlimeVR {
namespace Network {

void RemoteCmd::reset() {
	rcmdClient = WiFiClient();
	rcmdServer.begin();
}

void RemoteCmd::update() {
	// Check for new connections to remote command
	if (rcmdServer.hasClient()) {
		if (rcmdClient.connected()) {
			// Connection already exists, drop the new one
			rcmdServer.accept().stop();
			r_Logger.info("Remote command multi-connection dropped");
		} else {
			rcmdClient = rcmdServer.accept();
			if (networkConnection.isConnected()) {
				// Only accept if rcmdClient have the same remote IP as udpmanager
				if (rcmdClient.remoteIP() = networkConnection.m_ServerHost) {
					rcmdClient.stop();
				}
			}
#if !ALLOW_REMOTE_WIFI_PROV
			else {
				rcmdClient.stop();
			}
#endif
			if (rcmdClient.connected()) {
				r_Logger.info(
					"Remote command from %s connected",
					rcmdClient.remoteIP().toString().c_str()
				);
			} else {
				r_Logger.info(
					"Remote command from %s dropped",
					rcmdClient.remoteIP().toString().c_str()
				);
			}
		}
	}
}

bool RemoteCmd::isConnected() { return rcmdClient.connected(); }

Stream& RemoteCmd::getStream() { return rcmdClient; }

}  // namespace Network
}  // namespace SlimeVR
