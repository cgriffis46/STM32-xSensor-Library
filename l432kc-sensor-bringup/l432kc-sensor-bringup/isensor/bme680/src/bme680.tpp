/*
 * bme680.tpp
 *
 *  bme680's member definitions, and the compensation as free
 *  functions. bme680.h includes this file at its end, since templates
 *  have to be visible wherever they are used: include bme680.h, not
 *  this file.
 *
 *  The compensation functions are Bosch Sensortec's integer ones from
 *  the BME68x Sensor API v4.4.8 (bme68x.c, BSD-3-Clause, Copyright (c)
 *  2023 Bosch Sensortec GmbH), kept expression for expression so the
 *  results match to the bit; test/bme680_ref_test.cpp checks that.
 *  Three changes: left shifts of values that can be negative are
 *  multiplications (the same on every compiler, and not undefined);
 *  the pressure's cubic term is multiplied in 64 bits, where Bosch's
 *  overflows at high pressure; and the BME688's gas resistance is
 *  worked out in 64 bits, without Bosch's 100 ohm steps.
 */

#ifndef BME680_TPP_
#define BME680_TPP_

#include "../inc/bme680.h"

namespace BME680 {

// ---- calibration and compensation (pure) ----------------------------------

// The 42 bytes as read: 23 from 0x8A, 14 from 0xE1, 5 from 0x00.
// Indices from Bosch's BME68X_IDX_*.
inline bme680_calib_t bme680_parse_calib(const uint8_t coeff[BME680_LEN_COEFF]){
	bme680_calib_t c;
	c.par_t1=(uint16_t)(((uint16_t)coeff[32]<<8)|coeff[31]);
	c.par_t2=(int16_t)(((uint16_t)coeff[1]<<8)|coeff[0]);
	c.par_t3=(int8_t)coeff[2];
	c.par_p1=(uint16_t)(((uint16_t)coeff[5]<<8)|coeff[4]);
	c.par_p2=(int16_t)(((uint16_t)coeff[7]<<8)|coeff[6]);
	c.par_p3=(int8_t)coeff[8];
	c.par_p4=(int16_t)(((uint16_t)coeff[11]<<8)|coeff[10]);
	c.par_p5=(int16_t)(((uint16_t)coeff[13]<<8)|coeff[12]);
	c.par_p6=(int8_t)coeff[15];
	c.par_p7=(int8_t)coeff[14];
	c.par_p8=(int16_t)(((uint16_t)coeff[19]<<8)|coeff[18]);
	c.par_p9=(int16_t)(((uint16_t)coeff[21]<<8)|coeff[20]);
	c.par_p10=coeff[22];
	// H1 and H2 share 0xE2: H1 its low nibble, H2 its high one.
	c.par_h1=(uint16_t)(((uint16_t)coeff[25]<<4)|(coeff[24]&0x0F));
	c.par_h2=(uint16_t)(((uint16_t)coeff[23]<<4)|(coeff[24]>>4));
	c.par_h3=(int8_t)coeff[26];
	c.par_h4=(int8_t)coeff[27];
	c.par_h5=(int8_t)coeff[28];
	c.par_h6=coeff[29];
	c.par_h7=(int8_t)coeff[30];
	c.par_gh1=(int8_t)coeff[35];
	c.par_gh2=(int16_t)(((uint16_t)coeff[34]<<8)|coeff[33]);
	c.par_gh3=(int8_t)coeff[36];
	c.res_heat_val=(int8_t)coeff[37];
	c.res_heat_range=(uint8_t)((coeff[39]&0x30)/16);
	c.range_sw_err=(int8_t)(((int8_t)(coeff[41]&0xF0))/16);
	return c;
}

inline int16_t bme680_calc_temperature(uint32_t adc, const bme680_calib_t &c, int32_t *t_fine){
	int64_t var1, var2, var3;
	var1=((int32_t)adc>>3)-((int32_t)c.par_t1<<1);
	var2=(var1*(int32_t)c.par_t2)>>11;
	var3=((var1>>1)*(var1>>1))>>12;
	var3=((var3)*((int32_t)c.par_t3*16))>>14;
	*t_fine=(int32_t)(var2+var3);
	return (int16_t)(((*t_fine*5)+128)>>8);
}

// A product as the 32 bit multiply leaves it, without the undefined
// behaviour of a signed overflow: where Bosch's terms can overflow with
// a reading far outside the chip's range.
inline int32_t bme680_wrap32(uint32_t v){
	return (int32_t)v;
}

inline uint32_t bme680_calc_pressure(uint32_t adc, const bme680_calib_t &c, int32_t t_fine){
	int32_t var1, var2, var3, pressure_comp;
	const int32_t pres_ovf_check=INT32_C(0x40000000);
	var1=(((int32_t)t_fine)>>1)-64000;
	var2=((((var1>>2)*(var1>>2))>>11)*(int32_t)c.par_p6)>>2;
	var2=var2+((var1*(int32_t)c.par_p5)*2);
	var2=(var2>>2)+((int32_t)c.par_p4*65536);
	var1=(((((var1>>2)*(var1>>2))>>13)*((int32_t)c.par_p3*32))>>3)+
	     (((int32_t)c.par_p2*var1)>>1);
	var1=var1>>18;
	var1=((32768+var1)*(int32_t)c.par_p1)>>15;
	pressure_comp=1048576-adc;
	pressure_comp=(int32_t)((pressure_comp-(var2>>12))*((uint32_t)3125));
	if(var1==0) return 0; // only with nonsense calibration; Bosch's code would divide by zero
	if(pressure_comp>=pres_ovf_check){
		pressure_comp=bme680_wrap32((uint32_t)(pressure_comp/var1)*2u);
	} else {
		pressure_comp=bme680_wrap32((uint32_t)pressure_comp*2u)/var1;
	}
	var1=((int32_t)c.par_p9*(int32_t)(bme680_wrap32((uint32_t)(pressure_comp>>3)*(uint32_t)(pressure_comp>>3))>>13))>>12;
	var2=((int32_t)(pressure_comp>>2)*(int32_t)c.par_p8)>>13;
	// Bosch multiplies this in 32 bits, which overflows (and its
	// integer build reads 2048 Pa low) from about 107 kPa with a
	// par_p10 of 30, from about 102.5 kPa with 35: 64 bits here, the
	// same result wherever Bosch's doesn't overflow.
	var3=(int32_t)(((int64_t)(pressure_comp>>8)*(pressure_comp>>8)*(pressure_comp>>8)*
	      (int64_t)c.par_p10)>>17);
	pressure_comp=(int32_t)(pressure_comp)+((var1+var2+var3+((int32_t)c.par_p7*128))>>4);
	return (uint32_t)pressure_comp;
}

inline uint32_t bme680_calc_humidity(uint16_t adc, const bme680_calib_t &c, int32_t t_fine){
	int32_t var1, var2, var3, var4, var5, var6, temp_scaled, calc_hum;
	temp_scaled=(((int32_t)t_fine*5)+128)>>8;
	var1=(int32_t)(adc-((int32_t)((int32_t)c.par_h1*16)))-
	     (((temp_scaled*(int32_t)c.par_h3)/((int32_t)100))>>1);
	var2=((int32_t)c.par_h2*
	      (((temp_scaled*(int32_t)c.par_h4)/((int32_t)100))+
	       (((temp_scaled*((temp_scaled*(int32_t)c.par_h5)/((int32_t)100)))>>6)/((int32_t)100))+
	       (int32_t)(1<<14)))>>10;
	var3=var1*var2;
	var4=(int32_t)c.par_h6<<7;
	var4=((var4)+((temp_scaled*(int32_t)c.par_h7)/((int32_t)100)))>>4;
	var5=((var3>>14)*(var3>>14))>>10;
	var6=(var4*var5)>>1;
	calc_hum=(((var3+var6)>>10)*((int32_t)1000))>>12;
	if(calc_hum>100000) calc_hum=100000;	// cap at 100 %RH
	else if(calc_hum<0) calc_hum=0;
	return (uint32_t)calc_hum;
}

// The BME680's gas resistance (Bosch: calc_gas_resistance_low).
inline uint32_t bme680_calc_gas_low(uint16_t adc, uint8_t range, const bme680_calib_t &c){
	static const uint32_t lookup_table1[16]={
		UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2143188679), UINT32_C(2136746228), UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647),
		UINT32_C(2147483647)
	};
	static const uint32_t lookup_table2[16]={
		UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000), UINT32_C(255744255),
		UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016), UINT32_C(8000000),
		UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000), UINT32_C(250000), UINT32_C(125000)
	};
	int64_t var1, var3;
	uint64_t var2;
	range&=0x0F;
	var1=(int64_t)((1340+(5*(int64_t)c.range_sw_err))*((int64_t)lookup_table1[range]))>>16;
	var2=(((int64_t)((int64_t)adc<<15)-(int64_t)(16777216))+var1);
	var3=(((int64_t)lookup_table2[range]*(int64_t)var1)>>9);
	if(var2==0) return 0; // not reachable from a 10 bit reading and a sane range_sw_err
	return (uint32_t)((var3+((int64_t)var2>>1))/(int64_t)var2);
}

// The BME688's gas resistance (Bosch: calc_gas_resistance_high).
inline uint32_t bme680_calc_gas_high(uint16_t adc, uint8_t range){
	range&=0x0F;
	uint32_t var1=UINT32_C(262144)>>range;
	int32_t var2=(int32_t)adc-INT32_C(512);
	var2*=INT32_C(3);
	var2=INT32_C(4096)+var2;
	// Bosch's integer build divides 10000 * var1 and then multiplies by
	// 100, to stay in 32 bits, which leaves steps of 100 ohms (6% at
	// range 15). 1000000 * var1 in 64 bits instead: what its floating
	// point build gives, truncated.
	return (uint32_t)((UINT64_C(1000000)*var1)/(uint32_t)var2);
}

// res_heat_x for a heater temperature at an ambient temperature.
inline uint8_t bme680_calc_res_heat(uint16_t temp_c, int8_t ambient_c, const bme680_calib_t &c){
	int32_t var1, var2, var3, var4, var5, heatr_res_x100;
	if(temp_c>bme680_heater_max_c) temp_c=bme680_heater_max_c;
	var1=(((int32_t)ambient_c*c.par_gh3)/1000)*256;
	var2=(c.par_gh1+784)*(((((c.par_gh2+154009)*temp_c*5)/100)+3276800)/10);
	var3=var1+(var2/2);
	var4=(var3/(c.res_heat_range+4));
	var5=(131*c.res_heat_val)+65536;
	heatr_res_x100=(int32_t)(((var4/var5)-250)*34);
	return (uint8_t)((heatr_res_x100+50)/100);
}

// gas_wait_x: 6 bits of ms and a 2 bit multiplier (1, 4, 16, 64).
inline uint8_t bme680_calc_gas_wait(uint16_t ms){
	if(ms>=0xFC0) return 0xFF;
	uint8_t factor=0;
	while(ms>0x3F){
		ms=ms/4;
		factor+=1;
	}
	return (uint8_t)(ms+(factor*64));
}

inline uint32_t bme680_gas_wait_ms(uint8_t gas_wait){
	return (uint32_t)(gas_wait&0x3F)<<(2*(gas_wait>>6));
}

// Bosch's bme68x_get_meas_dur() for forced mode, in us: 1963 us an
// oversampling cycle, 4 x 477 us switching between T, P and H, 5 x
// 477 us for the gas conversion, 1 ms to wake. The heater time is on
// top of this.
inline uint32_t bme680_meas_us(uint8_t os_t, uint8_t os_p, uint8_t os_h){
	static const uint8_t os_to_meas_cycles[6]={0, 1, 2, 4, 8, 16};
	if(os_t>5) os_t=5;
	if(os_p>5) os_p=5;
	if(os_h>5) os_h=5;
	uint32_t meas_cycles=os_to_meas_cycles[os_t];
	meas_cycles+=os_to_meas_cycles[os_p];
	meas_cycles+=os_to_meas_cycles[os_h];
	uint32_t meas_dur=meas_cycles*UINT32_C(1963);
	meas_dur+=UINT32_C(477*4);
	meas_dur+=UINT32_C(477*5);
	meas_dur+=UINT32_C(1000);
	return meas_dur;
}

// ---- the driver -------------------------------------------------------------

template <typename TTransport>
template <typename... TArgs>
bme680<TTransport>::bme680(const bme680_param_t &param, TArgs&&... transportArgs)
	: base(bme680_init_state, bme680_error_state, bme680_bus_timeout_ms, std::forward<TArgs>(transportArgs)...) {
	_param=param;
	if(_param.heater_ms==0) _param.heater_ms=1;
	if(_param.heater_ms>bme680_heater_max_ms) _param.heater_ms=bme680_heater_max_ms;
	_calib=bme680_calib_t();
	_variant=bme680_variant_unknown;
	_newData=false;
	_start_measurement=false;
	phase_start=0;
	_lastStart=0;
	_seaLevelPa=bme680_sea_level_default_pa;
	_page=0xFF;
	_pageWant=0;
	_pageReturn=bme680_init_state;
	_cfgIndex=0;
	_cfgCount=0;
	_id=0;
	_variantReg=0;
	_status=0;
	onFail(); // readings NAN
}

template <typename TTransport>
void bme680<TTransport>::main(uint32_t nowMs){
	switch(_state){
	case bme680_init_state:
		_start_measurement=false;
		_page=0xFF; // after a power cycle or a reset, not known
		if(this->checkDevice()){
			enter(bme680_id_state,nowMs);
		} else {
			fail(nowMs);
		}
		break;

	// SPI page change, entered from pageFor(); goes back to the state
	// that asked. Read, then written with only spi_mem_page changed, as
	// Bosch's set_mem_page() does.
	case bme680_page_read_state:
		issued(this->readRegs(bme680_reg_mem_page,&_status,1),bme680_wait_page_read_state,nowMs);
		break;
	case bme680_wait_page_read_state:
		if(landed(nowMs)){
			enter(bme680_page_write_state,nowMs);
		}
		break;
	case bme680_page_write_state:
		issued(this->writeReg(bme680_reg_mem_page,(uint8_t)((_status&~BME680_MEM_PAGE_BIT)|_pageWant)),
				bme680_wait_page_write_state,nowMs);
		break;
	case bme680_wait_page_write_state:
		if(landed(nowMs)){
			_page=_pageWant;
			enter(_pageReturn,nowMs);
		}
		break;

	// The ID before anything is written, so a different chip at this
	// address is never reset.
	case bme680_id_state:
		if(pageFor(bme680_reg_chip_id,nowMs)) break;
		issued(this->readRegs(bme680_reg_chip_id,&_id,1),bme680_wait_id_state,nowMs);
		break;
	case bme680_wait_id_state:
		if(landed(nowMs)){
			if(_id==BME680_CHIP_ID){
				enter(bme680_reset_state,nowMs);
			} else {
				fail(nowMs); // something else is at this address
			}
		}
		break;
	case bme680_reset_state:
		if(pageFor(bme680_reg_soft_reset,nowMs)) break;
		issued(this->writeReg(bme680_reg_soft_reset,BME680_SOFT_RESET),bme680_wait_reset_state,nowMs);
		break;
	case bme680_wait_reset_state:
		// Bosch's API ignores how the reset write ends: the chip may be
		// gone before it is acknowledged. The ID read after it tells.
		if(this->finished(nowMs)){
			_page=0xFF;
			enter(bme680_reset_settle_state,nowMs);
		}
		break;
	case bme680_reset_settle_state:
		settled(nowMs,bme680_reset_settle_ms,bme680_id2_state);
		break;
	case bme680_id2_state:
		if(pageFor(bme680_reg_chip_id,nowMs)) break;
		_id=0;
		issued(this->readRegs(bme680_reg_chip_id,&_id,1),bme680_wait_id2_state,nowMs);
		break;
	case bme680_wait_id2_state:
		if(landed(nowMs)){
			if(_id==BME680_CHIP_ID){
				enter(bme680_variant_state,nowMs);
			} else {
				fail(nowMs); // did not come back from the reset
			}
		}
		break;
	case bme680_variant_state:
		if(pageFor(bme680_reg_variant_id,nowMs)) break;
		issued(this->readRegs(bme680_reg_variant_id,&_variantReg,1),bme680_wait_variant_state,nowMs);
		break;
	case bme680_wait_variant_state:
		if(landed(nowMs)){
			if(_variantReg==bme680_variant_bme680||_variantReg==bme680_variant_bme688){
				_variant=(bme680_variant_t)_variantReg;
				enter(bme680_cal1_state,nowMs);
			} else {
				fail(nowMs); // a variant whose gas readout isn't known here
			}
		}
		break;
	case bme680_cal1_state:
		if(pageFor(bme680_reg_coeff1,nowMs)) break;
		issued(this->readRegs(bme680_reg_coeff1,_coeff,BME680_LEN_COEFF1),bme680_wait_cal1_state,nowMs);
		break;
	case bme680_wait_cal1_state:
		if(landed(nowMs)){
			enter(bme680_cal2_state,nowMs);
		}
		break;
	case bme680_cal2_state:
		if(pageFor(bme680_reg_coeff2,nowMs)) break;
		issued(this->readRegs(bme680_reg_coeff2,&_coeff[BME680_LEN_COEFF1],BME680_LEN_COEFF2),bme680_wait_cal2_state,nowMs);
		break;
	case bme680_wait_cal2_state:
		if(landed(nowMs)){
			enter(bme680_cal3_state,nowMs);
		}
		break;
	case bme680_cal3_state:
		if(pageFor(bme680_reg_coeff3,nowMs)) break;
		issued(this->readRegs(bme680_reg_coeff3,&_coeff[BME680_LEN_COEFF1+BME680_LEN_COEFF2],BME680_LEN_COEFF3),
				bme680_wait_cal3_state,nowMs);
		break;
	case bme680_wait_cal3_state:
		if(landed(nowMs)){
			_calib=bme680_parse_calib(_coeff);
			enter(bme680_ctrl_read_state,nowMs);
		}
		break;
	case bme680_ctrl_read_state:
		if(pageFor(bme680_reg_ctrl_gas0,nowMs)) break;
		issued(this->readRegs(bme680_reg_ctrl_gas0,_ctrl,BME680_LEN_CTRL),bme680_wait_ctrl_read_state,nowMs);
		break;
	case bme680_wait_ctrl_read_state:
		if(landed(nowMs)){
			buildConfig();
			_cfgIndex=0;
			enter(bme680_config_state,nowMs);
		}
		break;
	case bme680_config_state:
		if(_cfgIndex>=_cfgCount){
			_start_measurement=true; // the first measurement at once, not a period later
			enter(bme680_done_state,nowMs);
			break;
		}
		if(pageFor(_cfgReg[_cfgIndex],nowMs)) break;
		issued(this->writeReg(_cfgReg[_cfgIndex],_cfgVal[_cfgIndex]),bme680_wait_config_state,nowMs);
		break;
	case bme680_wait_config_state:
		if(landed(nowMs)){
			++_cfgIndex;
			enter(bme680_config_state,nowMs);
		}
		break;

	case bme680_done_state:
		// The period runs from the start of one measurement to the next.
		if((nowMs-_lastStart)>=_param.period_ms||_start_measurement){
			_start_measurement=false;
			phase_start=nowMs;
			this->nextPeriod(_lastStart,nowMs,_param.period_ms);
			enter(bme680_force_state,nowMs);
		} else {
			// No longer than bme680_request_poll_ms at a time, so a
			// startMeasurement() request is seen without waiting out
			// the whole period.
			const uint32_t left=_param.period_ms-(nowMs-_lastStart);
			sleep(left<bme680_request_poll_ms?left:bme680_request_poll_ms);
		}
		break;
	case bme680_force_state:
		if(pageFor(bme680_reg_ctrl_meas,nowMs)) break;
		issued(this->writeReg(bme680_reg_ctrl_meas,ctrlMeas(BME680_MODE_FORCED)),bme680_wait_force_state,nowMs);
		break;
	case bme680_wait_force_state:
		if(landed(nowMs)){
			enter(bme680_measuring_state,nowMs);
		}
		break;
	case bme680_measuring_state:
		settled(nowMs,measurementMs(),bme680_field_state);
		break;
	case bme680_field_state:
		if(pageFor(bme680_reg_field0,nowMs)) break;
		issued(this->readRegs(bme680_reg_field0,_field,BME680_LEN_FIELD),bme680_wait_field_state,nowMs);
		break;
	case bme680_wait_field_state:
		if(landed(nowMs)){
			if(_field[0]&BME680_NEW_DATA){
				convert();
				_newData=true;
				enter(bme680_done_state,nowMs);
			} else if((nowMs-phase_start)>measurementMs()+bme680_phase_margin_ms){
				fail(nowMs); // the measurement never finished
			} else {
				enter(bme680_field_poll_state,nowMs);
			}
		}
		break;
	case bme680_field_poll_state:
		settled(nowMs,bme680_field_poll_ms,bme680_field_state);
		break;

	case bme680_error_state:
		if(errorCleared(nowMs,bme680_error_backoff_ms)){
			enter(bme680_init_state,nowMs);
		}
		break;
	default:
		enter(bme680_init_state,nowMs);
		break;
	}
}

// SPI only. If reg is in the other page from the one selected, starts
// the page change, which comes back to the current state, and returns
// true: the caller breaks out and issues nothing this pass.
template <typename TTransport>
bool bme680<TTransport>::pageFor(uint8_t reg, uint32_t nowMs){
	if(_param.bus!=bme680_spi) return false;
	const uint8_t want=reg>0x7F?0:BME680_MEM_PAGE_BIT;
	if(want==_page) return false;
	_pageWant=want;
	_pageReturn=_state;
	enter(bme680_page_read_state,nowMs);
	return true;
}

// Moves on to next once ms have passed since the current state was
// entered, and sleeps for what is left until then.
template <typename TTransport>
bool bme680<TTransport>::settled(uint32_t nowMs, uint32_t ms, bme680_state_t next){
	if(elapsed(nowMs,ms)){
		enter(next,nowMs);
		return true;
	}
	sleepRemaining(nowMs,ms);
	return false;
}

template <typename TTransport>
uint8_t bme680<TTransport>::osTemperature() const{
	return _param.os_temperature==bme680_os_skip?(uint8_t)bme680_os_1x:(uint8_t)_param.os_temperature;
}

template <typename TTransport>
uint8_t bme680<TTransport>::ctrlMeas(uint8_t mode) const{
	return (uint8_t)((osTemperature()<<5)|((_param.os_pressure&0x07)<<2)|(mode&0x03));
}

// The configuration writes, in order. ctrl_hum, config, ctrl_gas_0 and
// ctrl_gas_1 keep the bits this driver doesn't set as they were read
// after the reset (Bosch's API reads and writes them back the same
// way); ctrl_meas is all ours. ctrl_hum only takes effect at the next
// ctrl_meas write, which comes after it here and at every measurement.
template <typename TTransport>
void bme680<TTransport>::buildConfig(){
	uint8_t n=0;
	_cfgReg[n]=bme680_reg_ctrl_hum;
	_cfgVal[n++]=(uint8_t)((_ctrl[2]&~BME680_OSRS_H_MSK)|(_param.os_humidity&BME680_OSRS_H_MSK));
	// odr<2:0> = 0 and odr<3> = 1 below: Bosch's ODR_NONE, for forced mode
	_cfgReg[n]=bme680_reg_config;
	_cfgVal[n++]=(uint8_t)((_ctrl[5]&~(BME680_FILTER_MSK|BME680_ODR20_MSK))|((_param.filter&0x07)<<2));
	_cfgReg[n]=bme680_reg_ctrl_meas;
	_cfgVal[n++]=ctrlMeas(0); // sleep mode
	if(gasOn()){
		_cfgReg[n]=bme680_reg_res_heat0;
		_cfgVal[n++]=bme680_calc_res_heat(_param.heater_c,_param.ambient_c,_calib);
		_cfgReg[n]=bme680_reg_gas_wait0;
		_cfgVal[n++]=bme680_calc_gas_wait(_param.heater_ms);
	}
	_cfgReg[n]=bme680_reg_ctrl_gas0;
	_cfgVal[n++]=(uint8_t)((_ctrl[0]&~BME680_HEAT_OFF)|(gasOn()?0:BME680_HEAT_OFF));
	// run_gas: bit 4 on the BME680, bit 5 on the BME688. nb_conv 0:
	// heater set-point 0.
	const uint8_t runGas=gasOn()?(_variant==bme680_variant_bme688?0x20:0x10):0x00;
	_cfgReg[n]=bme680_reg_ctrl_gas1;
	_cfgVal[n++]=(uint8_t)((_ctrl[1]&~(BME680_ODR3|BME680_RUN_GAS_MSK|BME680_NB_CONV_MSK))|BME680_ODR3|runGas);
	_cfgCount=n;
}

template <typename TTransport>
uint32_t bme680<TTransport>::measurementMs() const{
	uint32_t us=bme680_meas_us(osTemperature(),_param.os_pressure,_param.os_humidity);
	uint32_t ms=(us+999)/1000;
	if(gasOn()) ms+=bme680_gas_wait_ms(bme680_calc_gas_wait(_param.heater_ms));
	return ms;
}

// Field 0, as Bosch's read_field_data() takes it apart.
template <typename TTransport>
void bme680<TTransport>::convert(){
	const uint8_t *b=_field;
	const uint32_t adc_pres=((uint32_t)b[2]*4096)|((uint32_t)b[3]*16)|((uint32_t)b[4]/16);
	const uint32_t adc_temp=((uint32_t)b[5]*4096)|((uint32_t)b[6]*16)|((uint32_t)b[7]/16);
	const uint16_t adc_hum=(uint16_t)(((uint32_t)b[8]*256)|(uint32_t)b[9]);
	const bool high=_variant==bme680_variant_bme688;
	const uint8_t gas_lsb=high?b[16]:b[14];
	const uint16_t adc_gas=(uint16_t)((uint32_t)(high?b[15]:b[13])*4|((uint32_t)gas_lsb/64));
	const uint8_t gas_range=gas_lsb&BME680_GAS_RANGE_MSK;

	int32_t t_fine=0;
	const int16_t t=bme680_calc_temperature(adc_temp,_calib,&t_fine);
	_reading.temperature=(float)t/100.0f;

	_reading.pressure=_param.os_pressure==bme680_os_skip?NAN:(float)bme680_calc_pressure(adc_pres,_calib,t_fine);
	_reading.humidity=_param.os_humidity==bme680_os_skip?NAN:(float)bme680_calc_humidity(adc_hum,_calib,t_fine)/1000.0f;

	_reading.gas_valid=gasOn()&&(gas_lsb&BME680_GAS_VALID);
	_reading.heat_stable=gasOn()&&(gas_lsb&BME680_HEAT_STAB);
	if(_reading.gas_valid&&_reading.heat_stable){
		_reading.gas_resistance=(float)(high?bme680_calc_gas_high(adc_gas,gas_range)
		                                    :bme680_calc_gas_low(adc_gas,gas_range,_calib));
	} else {
		_reading.gas_resistance=NAN;
	}
}

template <typename TTransport>
void bme680<TTransport>::onFail(){
	_reading.temperature=NAN;
	_reading.pressure=NAN;
	_reading.humidity=NAN;
	_reading.gas_resistance=NAN;
	_reading.gas_valid=false;
	_reading.heat_stable=false;
}

// The international barometric formula (the standard atmosphere up to
// 11 km): h = 44330 * (1 - (p / p0)^(1 / 5.255)), as BMP280Sensor uses
// (general knowledge, not from Bosch's API, which has no altitude).
template <typename TTransport>
float bme680<TTransport>::getAltitude() const{
	return 44330.0f*(1.0f-powf(_reading.pressure/_seaLevelPa,1.0f/5.255f));
}

template <typename TTransport>
bool bme680<TTransport>::setSeaLevelPressure(float pa){
	if(!(pa>0.0f)||isinf(pa)) return false;
	_seaLevelPa=pa;
	return true;
}

template <typename TTransport>
void bme680<TTransport>::startMeasurement(){
	_start_measurement=true;
}

template <typename TTransport>
bool bme680<TTransport>::newData(){
	return _newData;
}

template <typename TTransport>
void bme680<TTransport>::getReading(bme680_reading_t *r){
	*r=_reading;
	_newData=false;
}

} /* namespace BME680 */

#endif /* BME680_TPP_ */
