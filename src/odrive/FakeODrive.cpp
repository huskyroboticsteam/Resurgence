#include "Odrive.h"

#include "../utils/json.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

// while running:
	// prompt to read or write endpoint

	// prompt which endpoint

	// prompt value to config to

	// put together and send to Jetson over WebSockets, then Jetson sends to ODrive control
