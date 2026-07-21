#pragma once

namespace robot {

class WorldInterface {
    public:
        WorldInterface() {}

        virtual void emergencyStop() { is_emergency_stopped = true; }
        bool isEmergencyStopped() { return is_emergency_stopped; }

        virtual void enableMotors(bool enabled) { motors_enabled = enabled; }
        bool areMotorsEnabled() { return motors_enabled; }
    private:
        bool is_emergency_stopped = false;
        bool motors_enabled = false;
};

}