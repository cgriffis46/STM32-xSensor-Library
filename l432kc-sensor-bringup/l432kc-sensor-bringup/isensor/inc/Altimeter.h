#pragma once

// Pure interface: anything that can report altitude, in meters. How
// altitude is actually derived is entirely up to the implementer —
// from pressure (as BMP280Sensor does), from GPS, from a barometric
// formula referenced against a known point, whatever fits.
class Altimeter {
public:
    virtual ~Altimeter() = default;
    virtual float getAltitude() const = 0; // meters
};
