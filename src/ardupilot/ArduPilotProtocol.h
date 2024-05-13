#pragma once

#include "../network/websocket/WebSocketProtocol.h"
#include "../network/websocket/WebSocketServer.h"
#include "../world_interface/data.h"

#include <mutex>

#include <Eigen/Geometry>

namespace net {
namespace ardupilot {

/* @brief A protocol in order to handle messages from an ArduPilot enabled device */
class ArduPilotProtocol {
public:
	ArduPilotProtocol(net::websocket::SingleClientWSServer& websocketServer);
	~ArduPilotProtocol();
	ArduPilotProtocol(const ArduPilotProtocol& other) = delete;
	ArduPilotProtocol& operator=(const ArduPilotProtocol& other) = delete;

	/* @brief Gets the last reported GPS position (lat, lon)
	 *
	 * @return Last reported GPS position
	 */
	robot::types::DataPoint<navtypes::gpscoords_t> getGPS();

	/* @brief Gets the last reported orientation from the IMU
	 *
	 * @return Last reported orientation
	 */
	robot::types::DataPoint<Eigen::Quaterniond> getIMU();

private:
	/* @brief Validates GPS request
	 *
	 * @param json message
	 *
	 * @return True if message is valid, false otherwise
	 */
	static bool validateGPSRequest(const nlohmann::json& j);

	/* @brief Validates IMU request
	 *
	 * @param json message
	 *
	 * @return True if message is valid, false otherwise
	 */
	static bool validateIMURequest(const nlohmann::json& j);

	/* @brief Destructures GPS json message and updates last reported GPS values
	 *
	 * @param json message
	 */
	void handleGPSRequest(const nlohmann::json& j);

	/* @brief Destructures IMU json message and updates last reported GPS values
	 *
	 * @param json message
	 */
	void handleIMURequest(const nlohmann::json& j);

	/* @brief Checks if the ArduPilotProtocol is connected
	 *
	 * @return True if connected, false if not connected
	 */
	bool isArduPilotConnected();

	/* @brief Registers the protocol with the websocket server
	 *
	 * @param websocket server of which to add protocol
	 */
	void initArduPilotServer(net::websocket::SingleClientWSServer& websocketServer);

	/* @brief Connection handler for websocket server */
	void clientConnected();

	/* @brief Disconnect handler for websocket server */
	void clientDisconnected();

	std::mutex _connectionMutex;
	bool _arduPilotProtocolConnected;

	std::mutex _lastGPSMutex;
	robot::types::DataPoint<navtypes::gpscoords_t> _lastGPS;

	std::mutex _lastOrientationMutex;
	robot::types::DataPoint<Eigen::Quaterniond> _lastOrientation;
};

} // namespace ardupilot
} // namespace net
