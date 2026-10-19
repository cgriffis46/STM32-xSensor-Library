#pragma once
#include <math.h>
#include "Barometer.h"

// A barometer's station pressure brought to sea level as the altimeter
// setting, the "barometer" figure weather services and most weather
// station consoles show: NOAA's formula (MADIS's form, the Smithsonian
// Meteorological Tables' equation), from station pressure and the
// station's elevation, with no temperature in it:
//
//     A = ((P - 0.3)^n + k * H)^(1/n)        hPa, H in metres
//     n = 0.190284, k = 8.4184960528e-5
//
// and its inverse, P = (A^n - k * H)^(1/n) + 0.3. Checked against WeeWX's
// uwxutils.StationToAltimeter ('aaMADIS', the algorithm WeeWX uses by
// default) and MetPy's altimeter_to_station_pressure, which give the same
// formula and its inverse (test/sea_level_barometer_test.cpp has numbers
// from both). NOAA's own pages couldn't be fetched here.
//
// True sea-level pressure (the "SLP" in a METAR) also corrects for the
// air column's temperature and humidity; that isn't done here.
//
//     bme280<Stm32HalI2CTransport> bme(param, &hi2c1, 0x77, i2c1Mutex);
//     SeaLevelBarometer baro(bme, 33.0f);      // 33 m above sea level
//     baro.calibrateTo(baro::inHgToPa(30.54f)); // match a reference once
//     float inHg = baro::paToInHg(baro.getPressure());
//
// The calibration offset is added to the source's station pressure,
// since that is where a sensor's error is: calibrateTo() works out the
// offset that makes the altimeter setting equal a reference (a nearby
// airport's, or weather.gov's) at the given elevation. With the
// elevation right, the offset is the sensor's own error and stays
// valid as the weather changes; with the elevation wrong it soaks up
// that error too, and is only right near the pressure it was set at.
//
// Inject, don't own: the source must outlive this object. One thread
// (the one that reads the sensor) uses it; nothing here is locked.

namespace baro {

const double kN = 0.190284;              // R * lapse rate / g, as the Smithsonian tables have it
const double kK = 8.4184960528e-5;       // (lapse rate / T0) * P0^n, hPa^n per metre
const double kStationCorrHpa = 0.3;      // the formula's 0.3 hPa
const float kPaPerInHg = 3386.389f;      // inches of mercury at 0 C (general knowledge)

inline float paToInHg(float pa) { return pa / kPaPerInHg; }
inline float inHgToPa(float inHg) { return inHg * kPaPerInHg; }

// Station pressure (Pa) at elevationM to the altimeter setting (Pa).
// NAN for a pressure at or below 0.3 hPa, or NAN input.
inline float altimeterSettingPa(float stationPa, float elevationM) {
    const double p = (double)stationPa / 100.0 - kStationCorrHpa;
    if (!(p > 0.0) || isnan(elevationM)) return NAN;
    const double base = pow(p, kN) + kK * (double)elevationM;
    if (!(base > 0.0)) return NAN;
    return (float)(pow(base, 1.0 / kN) * 100.0);
}

// The inverse: the station pressure (Pa) that gives altimeter setting
// altimeterPa at elevationM.
inline float stationFromAltimeterPa(float altimeterPa, float elevationM) {
    const double a = (double)altimeterPa / 100.0;
    if (!(a > 0.0) || isnan(elevationM)) return NAN;
    const double base = pow(a, kN) - kK * (double)elevationM;
    if (!(base > 0.0)) return NAN;
    return (float)((pow(base, 1.0 / kN) + kStationCorrHpa) * 100.0);
}

} // namespace baro

class SeaLevelBarometer : public Barometer {
public:
    // elevationM: the sensor's height above mean sea level, metres.
    // offsetPa: added to the source's pressure (0 until calibrated).
    SeaLevelBarometer(const Barometer& source, float elevationM, float offsetPa = 0.0f)
        : source_(source), elevationM_(elevationM), offsetPa_(offsetPa) {}

    // Barometer: the altimeter setting, Pa. NAN until the source has a reading.
    float getPressure() const override {
        return baro::altimeterSettingPa(stationPressure(), elevationM_);
    }

    // The source's pressure with the offset added, Pa.
    float stationPressure() const {
        const float p = source_.getPressure();
        return isnan(p) ? NAN : p + offsetPa_;
    }
    // The source's own pressure, Pa.
    float rawPressure() const { return source_.getPressure(); }

    float elevation() const { return elevationM_; }
    void setElevation(float metres) { elevationM_ = metres; }
    float offset() const { return offsetPa_; }
    void setOffset(float pa) { offsetPa_ = pa; }

    // Sets the offset so that getPressure() now reads referencePa (an
    // altimeter setting, Pa). False, offset unchanged, if the source has
    // no reading yet or the reference makes no sense at this elevation.
    bool calibrateTo(float referencePa) {
        const float raw = source_.getPressure();
        const float want = baro::stationFromAltimeterPa(referencePa, elevationM_);
        if (isnan(raw) || isnan(want)) return false;
        offsetPa_ = want - raw;
        return true;
    }

private:
    const Barometer& source_;
    float elevationM_;
    float offsetPa_;
};
