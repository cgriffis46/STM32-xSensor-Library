#pragma once

// Pure interface: anything that can report relative humidity, in %RH.
// No state, no relationship to SensorBase or any other interface here
// — a sensor implements this alongside whatever else it is.
class Hygrometer {
public:
    virtual ~Hygrometer() = default;
    virtual float getHumidity() const = 0; // %RH, NAN until the first reading
};
