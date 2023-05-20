#include "../network/websocket/WebSocketProtocol.h"
#include "../network/websocket/WebSocketServer.h"
#include "../world_interface/data.h"

#include <condition_variable>
#include <mutex>

using json = nlohmann::json;

namespace net {
namespace ardupilot {

class ArduPilotProtocol {
public:
	ArduPilotProtocol();
	~ArduPilotProtocol();
	ArduPilotProtocol(const ArduPilotProtocol& other) = delete;
	ArduPilotProtocol& operator=(const ArduPilotProtocol& other) = delete;

	// Registers the protocol with the websocket server
	void initArduPilotServer();
	// Connection handler for websocket server
	void clientConnected();
	// Disconnect handler for websocket server
	void clientDisconnected();

	/* Gets the last reported GPS position (lat, lon)
	 *
	 * @return Last reported GPS position
	 */
	robot::types::DataPoint<navtypes::gpscoords_t> getGPS();
	/* Gets the last reported IMU readings (roll, pitch, yaw)
	 *
	 * @return Last reported IMU readings
	 */
	robot::types::DataPoint<navtypes::orientation_t> getIMU();
	/* Gets the last reported heading (degrees)
	 *
	 * @return Last reported heading
	 */
	robot::types::DataPoint<int> getHeading();

	/* Validates GPS request
	 *
	 * @param json message
	 *
	 * @return True if message is valid, false otherwise
	 */
	static bool validateGPSRequest(const json& j);
	/* Validates IMU request
	 *
	 * @param json message
	 *
	 * @return True if message is valid, false otherwise
	 */
	static bool validateIMURequest(const json& j);
	/* Validates heading request
	 *
	 * @param json message
	 *
	 * @return True if message is valid, false otherwise
	 */
	static bool validateHeadingRequest(const json& j);
	/* Destructures GPS json message and updates last reported GPS values
	 *
	 * @param json message
	 */
	void handleGPSRequest(const json& j);
	/* Destructures IMU json message and updates last reported GPS values
	 *
	 * @param json message
	 */
	void handleIMURequest(const json& j);
	/* Destructures heading json message and updates last reported GPS values
	 *
	 * @param json message
	 */
	void handleHeadingRequest(const json& j);

private:
	std::condition_variable _connectionCV;
	std::mutex _connectionMutex;
	bool _arduPilotProtocolConnected;

	std::mutex _lastHeadingMutex;
	robot::types::DataPoint<int> _lastHeading;

	std::mutex _lastGPSMutex;
	robot::types::DataPoint<navtypes::gpscoords_t> _lastGPS;

	std::mutex _lastOrientationMutex;
	robot::types::DataPoint<navtypes::orientation_t> _lastOrientation;
};

} // namespace ardupilot
} // namespace net
