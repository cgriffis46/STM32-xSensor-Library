#pragma once

// Pure interface: anything that can report temperature, in degrees C.
// No state, no relationship to SensorBase or any other interface here
// — a sensor implements this alongside whatever else it is.
class Thermometer {
public:
    virtual ~Thermometer() = default;
    virtual float getTemperature() const = 0; // degrees C
};
