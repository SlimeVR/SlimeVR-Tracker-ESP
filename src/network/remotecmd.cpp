#include "remotecmd.h"

namespace SlimeVR {
namespace Network {

void RemoteCmd::reset()
{
	rcmdClient = WiFiClient();
	rcmdServer.begin();
}

void RemoteCmd::update()
{
	// Check for new connections to remote command
	if (rcmdServer.hasClient()) {
		if (rcmdClient.connected()) {
			// Connection already exists, drop the new one
			rcmdServer.accept().stop();
			r_Logger.info("Remote command multi-connection dropped");
		} else {
			rcmdClient = rcmdServer.accept();
			r_Logger.info("Remote command from %s connected", rcmdClient.remoteIP().toString().c_str());
		}
	}
}

bool RemoteCmd::isConnected()
{
	return rcmdClient.connected();
}

Stream & RemoteCmd::getStream()
{
	return rcmdClient;
}

}  // namespace Network
}  // namespace SlimeVR
