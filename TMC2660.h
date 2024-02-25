#ifndef __TMC2660_H
#define __TMC2660_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "main.h"

	//Calculated for TMC2660-BOM with Rsense=100mOhm
	//CS = 32*R*I/V (with V = 0,31V oor 0,165V and I = 1000*current)
	//Irms=((CS+1)/32)*(Vfs/Rsense)*(1/sqrt*(2))
	#define TMC2660_SENS_LOW	0.0145983335
	#define TMC2660_SENS_HIGH	0.0274271721
	
	//some default values used in initialization
	#define ONE                     					  1
	#define DEFAULT_MICROSTEPPING_VALUE					8
  #define DRVCTRL_START												0x00003ul
	#define CHOPCONF_START											0x88004ul
	#define DRVCONF_START												0xE0010ul
	#define SGCSCONF_START											0xD0F1Ful
	#define SMARTEN_START												0xA2205ul
	//DRVSTATUS
	#define SG_pos															0
	#define OT_pos															1
	#define OTPW_pos														2
	#define S2GA_pos														3
	#define S2GB_pos														4
	#define OLA_pos															5
	#define OLB_pos															6
	#define STST_pos														7
	#define MSTEP_mask													10
	#define MSTEP_pos														10
	#define CPOL_pos														19
	//TMC26X register definitions
	#define DRVCTRL															0x00000ul
	//SDOFF=0
	#define MRES_mask														0xFFFF0 // Bit 0-3
	#define DEDGE_pos														8
	#define DEDGE_mask													(ONE << DEDGE_pos)
	#define INTPOL_pos													9
	#define INTPOL_mask													(ONE << INTPOL_pos) // Bit 9
	
	#define CHOPCONF														0x80000ul
	#define TOFF_mask														0xFFFF0 // Bit 0-3
	#define HSTRT_mask													0xFFF8F // Bit 4-6
	#define HSTRT_pos														4
	#define HEND_mask														0xFF87F // Bit 7-10
	#define HEND_pos														7
	#define HDEC_mask														0xFE7FF // Bit 11-12
	#define HDEC_pos														11
	#define COMP_mask														(ONE << 12)
	#define RNDTF_pos														13
	#define RNDTF_mask													(ONE << RNDTF_pos) // Bit 13
	#define CHM_pos															14
	#define CHM_mask														(ONE << CHM_pos) // Bit 14
	#define TBL_mask														0xE7FFF // Bit 15-16
	#define TBL_pos															15
	
	#define SMARTEN															0xA0000ul
	#define SEMIN_mask													0xFFFF0 // Bit 0-3
	#define SEUP_mask														0xFFF9F // Bit 5-6
	#define SEUP_pos														5	
	#define SEMAX_mask													0xFF0FF // Bit 8-11
	#define SEMAX_pos														8
	#define SEDN_mask														0xF9FFF // Bit 13-14
	#define SEDN_pos														13
	#define SEIMIN_pos													15
  #define SEIMIN_mask													(ONE << SEIMIN_pos) // Bit 15

	#define SGCSCONF														0xC0000ul
	#define CS_mask															0xFFFE0 // Bit 0-4
	#define SGT_mask														0xF80FF // Bit 8-14
	#define SGT_pos															8
	#define SFILT_pos														16
	#define SFILT_mask													(ONE << SFILT_pos) // Bit 16

	#define DRVCONF															0xE0000ul
	#define RDSEL_mask													0xFFFCF // Bit 4-5
	#define RDSEL_pos														4
	#define VSENSE_pos													6
	#define VSENSE_mask													(ONE << VSENSE_pos) // Bit 6
	#define SDOFF_pos														7
	#define SDOFF_mask													(ONE << SDOFF_pos) // Bit 7
	#define TS2G_mask														0xFFCFF // Bit 8-9
	#define TS2G_pos														8
	#define DISS2G_pos													10
	#define DISS2G_mask													(ONE << DISS2G_pos) // Bit 10
	#define SLPL_mask														0xFCFFF // Bit 12-13
	#define SLPL_pos														12
	#define SLPH_mask														0xF3FFF // Bit 14-15
	#define SLPH_pos														14
	#define TST_pos															16
    #define TST_mask													(ONE << TST_pos) // Bit 16
	
	typedef enum {
		m_STOPPED         = 0,
		m_STARTING        = 1,
		m_FULLSPEED       = 2,
		m_BREAKING        = 3,
		m_BREAKCORRECTION = 4
	} motor_status_t;

	typedef enum {
		CCW	= 0,
		CW	= 1
	} rotation_t;
	

	typedef struct {
		motor_status_t		mStatus;
		volatile int32_t	mCrtPos; //handle by interrupt
		rotation_t				mDirection;
		uint16_t					mStep;// no need to go to 1/256 microstepping
		uint32_t					mSPS;//Steps per second. For uStep of 32 we have max. 57600SPS
		uint32_t					mStepsDec;//When To start decelerating
		uint32_t					mStepsToTarget;//Steps to target
		uint32_t					mDRVCTRL;
		uint32_t					mDRVCONF;
		uint32_t					mCHOPCONF;
		uint32_t					mSGCSCONF;
		uint32_t					mSMARTEN;
		uint32_t					mDRVSTATUS;//this holds the value we get from controller
	} motor;

	void TMC2660_setDirection(uint8_t myMotor, uint8_t direction);
	void TMC2660_setSpeed(uint8_t myMotor, uint32_t speed);
	void TMC2660_init(uint8_t myMotor);
	void TMC2660_start(uint8_t myMotor);
	
	//Functions used to get different values from status
	uint8_t TMC2660_getStallGuardStatus(uint8_t myMotor);
	uint8_t TMC2660_getTempShutDown(uint8_t myMotor);
	uint8_t TMC2660_getTempWarning(uint8_t myMotor);
	uint8_t TMC2660_getShortA(uint8_t myMotor);
	uint8_t TMC2660_getShortB(uint8_t myMotor);
	uint8_t TMC2660_getOpenLoadA(uint8_t myMotor);
	uint8_t TMC2660_getOpenLoadB(uint8_t myMotor);
	uint8_t TMC2660_getStandStill(uint8_t myMotor);
	//For RDSEl = 2
	uint8_t TMC2660_getCoolStepValue(uint8_t MotorNo);
	uint8_t TMC2660_getStallGuardValue(uint8_t MotorNo);
	//For RDSEl = 0
	uint8_t TMC2660_getStepsCounter(uint8_t MotorNo);
	//For RDSEl = 1
	uint8_t TMC2660_getFullStallGuardValue(uint8_t MotorNo);
	//The main status function
	void TMC2660_getStatus(uint8_t MotorNo);
	
	//DRVCTRL register functions
	void TMC2660_setResolution(uint8_t MotorNo, uint16_t Resolution);
	uint16_t TMC2660_getResolution(uint8_t MotorNo);
	void TMC2660_setDoubleEdge(uint8_t MotorNo, uint8_t doubleEdge);
	uint8_t TMC2660_getDoubleEdge(uint8_t MotorNo);
	void TMC2660_setInterpolation(uint8_t MotorNo, uint8_t interpolation);
	uint8_t TMC2660_getInterpolation(uint8_t MotorNo);
	
	//CHOPCONF register functions
	void TMC2660_setTimeOff(uint8_t MotorNo, uint8_t TimeOff);
	uint8_t TMC2660_getTimeOff(uint8_t MotorNo);
	void TMC2660_setHysteresisStart(uint8_t MotorNo, uint8_t HysteresisStart);
	uint8_t TMC2660_getHysteresisStart(uint8_t MotorNo);
	void TMC2660_setHysteresisStart(uint8_t MotorNo, uint8_t HysteresisStart);
	uint8_t TMC2660_getHysteresisStart(uint8_t MotorNo);
	void TMC2660_setHysteresisEnd(uint8_t MotorNo, uint8_t HysteresisEnd);
	uint8_t TMC2660_getHysteresisEnd(uint8_t MotorNo);
	void TMC2660_setHysteresisDecrement(uint8_t MotorNo, uint8_t HysteresisDecrement);
	uint8_t TMC2660_getHysteresisDecrement(uint8_t MotorNo);
	void TMC2660_setRandomTime(uint8_t MotorNo, uint8_t randomTime);
	uint8_t TMC2660_getRandomTime(uint8_t MotorNo);
	void TMC2660_setChopperMode(uint8_t MotorNo, uint8_t ChopperMode);
	uint8_t TMC2660_getChopperMode(uint8_t MotorNo);
	void TMC2660_setBlankingTime(uint8_t MotorNo, uint8_t BlankingTime);
	uint8_t TMC2660_getBlankingTime(uint8_t MotorNo);
	
	//SMARTEN register functions
	void TMC2660_setCoolStepThldLow(uint8_t MotorNo, uint8_t LowerCoolStepThreshold);
	uint8_t TMC2660_getCoolStepThldLow(uint8_t MotorNo);	
	void TMC2660_setCurrentIncrement(uint8_t MotorNo, uint8_t CurrentIncrement);
	uint8_t TMC2660_getCurrentIncrement(uint8_t MotorNo);
	void TMC2660_setCoolStepThldHigh(uint8_t MotorNo, uint8_t UpperCoolStepThreshold);
	uint8_t TMC2660_getCoolStepThldHigh(uint8_t MotorNo);
	void TMC2660_setCurrentDecrementSpeed(uint8_t MotorNo, uint8_t CurrentDecrementSpeed);
	uint8_t TMC2660_getCurrentDecrementSpeed(uint8_t MotorNo);
	void TMC2660_setMinCoolStepCurrent(uint8_t MotorNo, uint8_t coolStepCurrent);
	uint8_t TMC2660_getMinCoolStepCurrent(uint8_t MotorNo);
	
	//SGCSCONF register functions
	void TMC2660_setRMS_Current(uint8_t MotorNo, uint16_t mA);
	void TMC2660_setCurrentScale(uint8_t MotorNo, uint8_t CurrentScale);
	uint8_t TMC2660_getCurrentScale(uint8_t MotorNo);
	void TMC2660_setStallGuardThld(uint8_t MotorNo, int8_t StallGuardThld);
	int8_t TMC2660_getStallGuardThld(uint8_t MotorNo);
	void TMC2660_setStallGuardFilter(uint8_t MotorNo, uint8_t GuardFilter);
	uint8_t TMC2660_getStallGuardFilter(uint8_t MotorNo);
	
	//DRVCONF register functions
	void TMC2660_setReadOut(uint8_t MotorNo, uint8_t ReadOut);
	uint8_t TMC2660_getReadOut(uint8_t MotorNo);
	void TMC2660_setVSENSE(uint8_t MotorNo, uint8_t vSense);
	uint8_t TMC2660_getVSENSE(uint8_t MotorNo);
	void TMC2660_setStepDir(uint8_t MotorNo, uint8_t StepDirSPI);
	uint8_t TMC2660_getStepDir(uint8_t MotorNo);
	void TMC2660_setGndDetectionTime(uint8_t MotorNo, uint8_t GndTimer);
	uint8_t TMC2660_getGndDetectionTime(uint8_t MotorNo);
	void TMC2660_setGndProtection(uint8_t MotorNo, uint8_t GndProtection);
	uint8_t TMC2660_getGndProtection(uint8_t MotorNo);
	void TMC2660_set_SlopeControlLowSide(uint8_t MotorNo, uint8_t SlopeControlLowSide);
	uint8_t TMC2660_getSlopeControlLowSide(uint8_t MotorNo);
	void TMC2660_set_SlopeControlHighSide(uint8_t MotorNo, uint8_t SlopeControlHighSide);
	uint8_t TMC2660_getSlopeControlHighSide(uint8_t MotorNo);
	void TMC2660_setTestMode(uint8_t MotorNo, uint8_t TestMode);
	uint8_t TMC2660_getTestMode(uint8_t MotorNo);
	
#ifdef __cplusplus
}
#endif

#endif /* __TMC2660_H */
