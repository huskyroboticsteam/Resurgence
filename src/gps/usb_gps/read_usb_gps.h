
#pragma once

namespace gps::usb {
/**
 * @brief Check if the USB GPS has acquired a fix.
 *
 * @return true if a fix was acquired, false otherwise.
 */
bool gpsHasFix();

/**
 * @brief Try to start the GPS thread. If this call succeeds (returns true) this method should
 * not be called again.
 *
 * @return true if the GPS thread was successfully started, false otherwise
 */
bool startGPSThread();
} // namespace gps::usb
