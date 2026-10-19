#pragma once

// Pure interface: anything that can report barometric pressure, in Pa.
// No state, no relationship to SensorBase — a sensor implements this
// alongside whatever else it is, not instead of it.
class Barometer {
public:
    virtual ~Barometer() = default;
    virtual float getPressure() const = 0; // Pa
};
