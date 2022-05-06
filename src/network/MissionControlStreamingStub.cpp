#include "MissionControlProtocol.h"
#include "../log.h"

namespace net {
namespace mc {
void MissionControlProtocol::videoStreamTask() {
	log(LOG_WARN, "Computer vision is disabled, so video streaming is unavailable\n");
}
} // namespace mc
} // namespace net
