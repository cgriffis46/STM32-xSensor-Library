/*
 * bmp390.tpp
 *
 *  bmp390's member definitions and its calibration arithmetic.
 *  bmp390.h includes this file at its end, since templates have to be
 *  visible wherever they are used: include bmp390.h, not this file.
 */

#ifndef BMP390_TPP_
#define BMP390_TPP_

#include "../inc/bmp390.h"

namespace BMP390 {

// ---- calibration and compensation (Bosch's BMP3 Sensor API v2.0.6,
// floating point build: parse_calib_data(), compensate_temperature(),
// compensate_pressure()) ----

inline bmp390_calib_t bmp390_parse_calib(const uint8_t *r){
	bmp390_calib_t c;
	const uint16_t t1 = (uint16_t)((r[1] << 8) | r[0]);
	const uint16_t t2 = (uint16_t)((r[3] << 8) | r[2]);
	const int8_t t3 = (int8_t)r[4];
	const int16_t p1 = (int16_t)((r[6] << 8) | r[5]);
	const int16_t p2 = (int16_t)((r[8] << 8) | r[7]);
	const int8_t p3 = (int8_t)r[9];
	const int8_t p4 = (int8_t)r[10];
	const uint16_t p5 = (uint16_t)((r[12] << 8) | r[11]);
	const uint16_t p6 = (uint16_t)((r[14] << 8) | r[13]);
	const int8_t p7 = (int8_t)r[15];
	const int8_t p8 = (int8_t)r[16];
	const int16_t p9 = (int16_t)((r[18] << 8) | r[17]);
	const int8_t p10 = (int8_t)r[19];
	const int8_t p11 = (int8_t)r[20];
	c.par_t1 = (double)t1 * 256.0;							// / 2^-8
	c.par_t2 = (double)t2 / 1073741824.0;					// / 2^30
	c.par_t3 = (double)t3 / 281474976710656.0;				// / 2^48
	c.par_p1 = (double)(p1 - 16384) / 1048576.0;			// / 2^20
	c.par_p2 = (double)(p2 - 16384) / 536870912.0;			// / 2^29
	c.par_p3 = (double)p3 / 4294967296.0;					// / 2^32
	c.par_p4 = (double)p4 / 137438953472.0;					// / 2^37
	c.par_p5 = (double)p5 * 8.0;							// / 2^-3
	c.par_p6 = (double)p6 / 64.0;							// / 2^6
	c.par_p7 = (double)p7 / 256.0;							// / 2^8
	c.par_p8 = (double)p8 / 32768.0;						// / 2^15
	c.par_p9 = (double)p9 / 281474976710656.0;				// / 2^48
	c.par_p10 = (double)p10 / 281474976710656.0;			// / 2^48
	c.par_p11 = (double)p11 / 36893488147419103232.0;		// / 2^65
	return c;
}

inline double bmp390_compensate_temperature(const bmp390_calib_t &c, uint32_t adcT){
	const double d1 = (double)((int64_t)adcT) - c.par_t1;
	const double d2 = d1 * c.par_t2;
	double t = d2 + (d1 * d1) * c.par_t3;
	if (t < -40.0) t = -40.0;
	if (t > 85.0) t = 85.0;
	return t;
}

inline double bmp390_compensate_pressure(const bmp390_calib_t &c, double t, uint32_t adcP){
	const double p = (double)adcP;
	const double out1 = c.par_p5 + c.par_p6 * t + c.par_p7 * t * t + c.par_p8 * t * t * t;
	const double out2 = p * (c.par_p1 + c.par_p2 * t + c.par_p3 * t * t + c.par_p4 * t * t * t);
	const double d3 = p * p * (c.par_p9 + c.par_p10 * t);
	const double d4 = d3 + p * p * p * c.par_p11;
	double comp = out1 + out2 + d4;
	if (comp < 30000.0) comp = 30000.0;
	if (comp > 125000.0) comp = 125000.0;
	return comp;
}

inline uint32_t bmp390_measurement_us(bmp390_osr_t osrP, bmp390_osr_t osrT){
	return 234u + 392u + (2000u << (unsigned)osrP) + 313u + (2000u << (unsigned)osrT);
}

// ---- the driver ----

template <typename TTransport>
template <typename... TArgs>
bmp390<TTransport>::bmp390(const bmp390_param_t &param, TArgs&&... transportArgs)
	: base(bmp390_init_state, bmp390_error_state, bmp390_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	if(_param.osr_pressure>bmp390_osr_32x) _param.osr_pressure=bmp390_osr_32x;
	if(_param.osr_temperature>bmp390_osr_32x) _param.osr_temperature=bmp390_osr_32x;
	if(_param.iir>bmp390_iir_127) _param.iir=bmp390_iir_127;
	_calib=bmp390_calib_t();
	_variant=bmp390_variant_unknown;
	_newData=false;
	_start_measurement=false;
	phase_start=0;
	_id=0;
	_status=0;
	_err=0;
	onFail(); // readings NAN
}

template <typename TTransport>
void bmp390<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case bmp390_init_state:
		_start_measurement=false;
		if(this->checkDevice()){
			enter(bmp390_id_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;

	// The ID before anything is written, so a different chip at this
	// address is never reset.
	case bmp390_id_state:
		_id=0;
		issued(this->readRegs(bmp390_reg_chip_id,&_id,1),bmp390_wait_id_state,nowMs);
		break;
	case bmp390_wait_id_state:
		if(landed(nowMs)){
			if(_id==BMP390_CHIP_ID||_id==BMP388_CHIP_ID){
				phase_start=nowMs;
				enter(bmp390_cmd_rdy_state,nowMs);
			} else {
				fail(nowMs); // something else is at this address
			}
		}
		break;
	// Bosch's API only sends the reset once the chip says it can take a command.
	case bmp390_cmd_rdy_state:
		issued(this->readRegs(bmp390_reg_status,&_status,1),bmp390_wait_cmd_rdy_state,nowMs);
		break;
	case bmp390_wait_cmd_rdy_state:
		if(landed(nowMs)){
			if(_status&bmp390_status_cmd_rdy){
				enter(bmp390_reset_state,nowMs);
			} else if((nowMs-phase_start)>bmp390_cmd_rdy_timeout_ms){
				fail(nowMs);
			} else {
				enter(bmp390_cmd_rdy_poll_state,nowMs);
			}
		}
		break;
	case bmp390_cmd_rdy_poll_state:
		settled(nowMs,bmp390_poll_ms,bmp390_cmd_rdy_state);
		break;
	case bmp390_reset_state:
		issued(this->writeReg(bmp390_reg_cmd,BMP390_SOFT_RESET),bmp390_wait_reset_state,nowMs);
		break;
	case bmp390_wait_reset_state:
		// The chip may reset before it acknowledges; the ID read after
		// it tells whether it came back.
		if(finished(nowMs)){
			enter(bmp390_reset_settle_state,nowMs);
		}
		break;
	case bmp390_reset_settle_state:
		settled(nowMs,bmp390_reset_settle_ms,bmp390_id2_state);
		break;
	case bmp390_id2_state:
		_id=0;
		issued(this->readRegs(bmp390_reg_chip_id,&_id,1),bmp390_wait_id2_state,nowMs);
		break;
	case bmp390_wait_id2_state:
		if(landed(nowMs)){
			if(_id==BMP390_CHIP_ID||_id==BMP388_CHIP_ID){
				_variant=(bmp390_variant_t)_id;
				enter(bmp390_err_state,nowMs);
			} else {
				fail(nowMs); // didn't come back from the reset
			}
		}
		break;
	case bmp390_err_state:
		issued(this->readRegs(bmp390_reg_err,&_err,1),bmp390_wait_err_state,nowMs);
		break;
	case bmp390_wait_err_state:
		if(landed(nowMs)){
			if(_err&bmp390_err_cmd){
				fail(nowMs); // the reset command wasn't taken (Bosch's BMP3_E_CMD_EXEC_FAILED)
			} else {
				enter(bmp390_calib_state,nowMs);
			}
		}
		break;
	case bmp390_calib_state:
		issued(this->readRegs(bmp390_reg_calib,_calibReg,BMP390_LEN_CALIB),bmp390_wait_calib_state,nowMs);
		break;
	case bmp390_wait_calib_state:
		if(landed(nowMs)){
			_calib=bmp390_parse_calib(_calibReg);
			enter(bmp390_osr_state,nowMs);
		}
		break;
	// Write-only as far as this driver is concerned: whole values, built each time.
	case bmp390_osr_state:
		issued(this->writeReg(bmp390_reg_osr,(uint8_t)(((uint8_t)_param.osr_temperature<<3)|(uint8_t)_param.osr_pressure)),
				bmp390_wait_osr_state,nowMs);
		break;
	case bmp390_wait_osr_state:
		if(landed(nowMs)){
			enter(bmp390_config_state,nowMs);
		}
		break;
	case bmp390_config_state:
		issued(this->writeReg(bmp390_reg_config,(uint8_t)((uint8_t)_param.iir<<1)),bmp390_wait_config_state,nowMs);
		break;
	case bmp390_wait_config_state:
		if(landed(nowMs)){
			_start_measurement=true; // the first measurement at once
			enter(bmp390_done_state,nowMs);
		}
		break;

	case bmp390_done_state:
		// The period runs from the start of one measurement to the next.
		if((nowMs-phase_start)>=_param.period_ms||_start_measurement){
			_start_measurement=false;
			phase_start=nowMs;
			enter(bmp390_force_state,nowMs);
		} else {
			const uint32_t left=_param.period_ms-(nowMs-phase_start);
			sleep(left<bmp390_request_poll_ms?left:bmp390_request_poll_ms);
		}
		break;
	case bmp390_force_state:
		issued(this->writeReg(bmp390_reg_pwr_ctrl,
				(uint8_t)(bmp390_pwr_press_en|bmp390_pwr_temp_en|(bmp390_mode_forced<<4))),bmp390_wait_force_state,nowMs);
		break;
	case bmp390_wait_force_state:
		if(landed(nowMs)){
			enter(bmp390_measuring_state,nowMs);
		}
		break;
	case bmp390_measuring_state:
		settled(nowMs,measurementMs(),bmp390_status_state);
		break;
	case bmp390_status_state:
		issued(this->readRegs(bmp390_reg_status,&_status,1),bmp390_wait_status_state,nowMs);
		break;
	case bmp390_wait_status_state:
		if(landed(nowMs)){
			if((_status&bmp390_status_drdy_press)&&(_status&bmp390_status_drdy_temp)){
				enter(bmp390_data_state,nowMs);
			} else if((nowMs-phase_start)>measurementMs()+bmp390_phase_margin_ms){
				fail(nowMs); // the measurement never finished
			} else {
				enter(bmp390_status_poll_state,nowMs);
			}
		}
		break;
	case bmp390_status_poll_state:
		settled(nowMs,bmp390_poll_ms,bmp390_status_state);
		break;
	case bmp390_data_state:
		issued(this->readRegs(bmp390_reg_data,_data,BMP390_LEN_DATA),bmp390_wait_data_state,nowMs);
		break;
	case bmp390_wait_data_state:
		if(landed(nowMs)){
			convert();
			_newData=true;
			enter(bmp390_done_state,nowMs);
		}
		break;

	case bmp390_error_state:
		if(errorCleared(nowMs,bmp390_error_backoff_ms)){
			enter(bmp390_init_state,nowMs);
		}
		break;
	default:
		enter(bmp390_init_state,nowMs);
		break;
	}
}

// A pause: true (and on to next) once ms have passed in this state;
// otherwise sleeps for what is left.
template <typename TTransport>
bool bmp390<TTransport>::settled(uint32_t nowMs, uint32_t ms, bmp390_state_t next){
	if(elapsed(nowMs,ms)){
		enter(next,nowMs);
		return true;
	}
	sleepRemaining(nowMs,ms);
	return false;
}

template <typename TTransport>
void bmp390<TTransport>::convert(){
	const uint32_t adcP=(uint32_t)_data[0]|((uint32_t)_data[1]<<8)|((uint32_t)_data[2]<<16);
	const uint32_t adcT=(uint32_t)_data[3]|((uint32_t)_data[4]<<8)|((uint32_t)_data[5]<<16);
	_temperature=bmp390_compensate_temperature(_calib,adcT);   // first: pressure uses it
	_pressure=bmp390_compensate_pressure(_calib,_temperature,adcP);
}

template <typename TTransport>
uint32_t bmp390<TTransport>::measurementMs() const{
	return (bmp390_measurement_us(_param.osr_pressure,_param.osr_temperature)+999u)/1000u;
}

template <typename TTransport>
void bmp390<TTransport>::startMeasurement(){
	_start_measurement=true;
}

template <typename TTransport>
bool bmp390<TTransport>::newData(){
	const bool n=_newData;
	_newData=false;
	return n;
}

template <typename TTransport>
bool bmp390<TTransport>::GetTemperature(double *t){
	*t=_temperature;
	_newData=false;
	return !isnan(_temperature);
}

template <typename TTransport>
bool bmp390<TTransport>::GetPressure(double *p){
	*p=_pressure;
	_newData=false;
	return !isnan(_pressure);
}

template <typename TTransport>
void bmp390<TTransport>::onFail(){
	_temperature=NAN;
	_pressure=NAN;
}

} /* namespace BMP390 */

#endif /* BMP390_TPP_ */
