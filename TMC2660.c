#include <stdio.h>
#include <spi.h>
#include "TMC2660.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

	//SPI_AS5048;//Set SPI2->CR1-->CPOL Low
	//SPI_Default;//Set SPI2->CR1-->CPHA High
	motor myMotors[2];
	// SPI Modes
	//   Mode	  CPOL	  CPHA
	//	0		0		0
	//	1		0		1
	//	2		1		0
	//	3		1		1
	//	if(SPI_Mode == TM_SPI_Mode_0) {
	//		SPIHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	//		SPIHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	//	} else if(SPI_Mode == TM_SPI_Mode_1) {
	//		SPIHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	//		SPIHandle.Init.CLKPhase = SPI_PHASE_2EDGE;
	//	} else if(SPI_Mode == TM_SPI_Mode_2) {
	//		SPIHandle.Init.CLKPolarity = SPI_POLARITY_HIGH;
	//		SPIHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	//	} else if(SPI_Mode == TM_SPI_Mode_3) {
	//		SPIHandle.Init.CLKPolarity = SPI_POLARITY_HIGH;
	//		SPIHandle.Init.CLKPhase = SPI_PHASE_2EDGE;
	//	}

	static void print_Bits(uint32_t data) {
		int b = 19;//31
	//  for(; b>=24; b--){
	//    printf("%d",(data>>b)&1);
	//  }
	//  Serial.print(".");
		for(; b>=16; b--){
			printf("%d", (data>>b)&1);
		}
		printf(".");
		for(; b>=8; b--){
			printf("%d",(data>>b)&1);
		}
		printf(".");
		for(; b>=0; b--){
			printf("%d",(data>>b)&1);
		}
		printf("\n");
	}

	static void send(uint8_t MotorNo, uint32_t regVal) {
		// Change SPI mode 1 for AS5048 and 3 for TMC2660
		printf("%d -->\n", MotorNo);
		print_Bits(regVal);
		uint8_t toSend[3] = {((regVal >> 16) & 0xff), ((regVal >>  8) & 0xff),(regVal & 0xff) };
		printf("*****\n");
		uint8_t toGet[3] = {0, 0, 0};
		//select the TMC driver
		if(MotorNo){
			V_CS_Select;
		} else {
			H_CS_Select;
		}
		//write/read the values
		if(HAL_SPI_TransmitReceive(&hspi2, toSend, toGet, 3, PORT_TIMEOUT) != HAL_OK)
			printf ("HAL_SPI_TransmitReceive \n");
		//deselect the TMC chip
		if(MotorNo){
			V_CS_Release;
		} else {
			H_CS_Release;
		}
		//store the status result
		myMotors[MotorNo].mDRVSTATUS = ((toGet[0] << 16) | (toGet[1] << 8) | (toGet[2] >> 4));
		//printf("->>mDRVSTATUS Motor %d = ", MotorNo);
		//print_Bits(Motors[MotorNo].mDRVSTATUS);
	}

	void TMC2660_setSpeed(uint8_t MotorNo, uint32_t speed){
		myMotors[MotorNo].mSPS = speed;
		uint32_t NewTimPeriod = ((TimTickFreq/myMotors[MotorNo].mSPS) - 1);
		uint32_t PulseLen = ((NewTimPeriod + 1)/2) - 1;
		switch(MotorNo){
			case 0:
				htim2.Instance->ARR = NewTimPeriod;
				htim2.Instance->CNT = 0;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PulseLen);
			case 1:
				htim5.Instance->ARR = NewTimPeriod;
				htim5.Instance->CNT = 0;
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PulseLen);
			break;
		}	
	}

	void TMC2660_setDirection(uint8_t MotorNo, uint8_t direction){
		switch(MotorNo) {
			case 0:
				if(direction){
					H_DIR_A;
				} else {
					H_DIR_B;
				}		
			break;
			case 1:
				if(direction){
					V_DIR_A;
				} else {
					V_DIR_B;
				}		
			break;		
		}	
	}

	void TMC2660_init(uint8_t MotorNo) {
		//init the structures
		myMotors[MotorNo].mStatus					= m_STOPPED;
		myMotors[MotorNo].mCrtPos					= 0;//handle by interrupt
		myMotors[MotorNo].mDirection			= CW;
		myMotors[MotorNo].mStep						= 0;//0=256uS 8=full step
		myMotors[MotorNo].mSPS 						= 1;//Steps per second (actual frequency of the timer) this is why 32 bit timer is nice
		myMotors[MotorNo].mStepsDec 			= 2;//When To start decelerating
		myMotors[MotorNo].mStepsToTarget 	= 0;//Steps to target
		
		//setting the default register values
		myMotors[MotorNo].mDRVCTRL  = DRVCTRL;
		myMotors[MotorNo].mCHOPCONF = CHOPCONF;
		myMotors[MotorNo].mSMARTEN  = SMARTEN;
		myMotors[MotorNo].mSGCSCONF = SGCSCONF;
		myMotors[MotorNo].mDRVCONF  = DRVCONF;
//		//All setting below can be done in a single pass
//		//This long way of seeting up registers is to show the functions
//		
		//DRVCTRL settings
		TMC2660_setResolution(MotorNo, myMotors[MotorNo].mStep);
		TMC2660_setDoubleEdge(MotorNo, 0);//use single edge for stepping
		TMC2660_setInterpolation(MotorNo, 0);//do not interpolate steps
		//CHOPCONF settings
		TMC2660_setTimeOff(MotorNo, 4);//if 0 then the driver is disabled
		TMC2660_setHysteresisStart(MotorNo, 4);// !!! Valuse are moved to 0...7 not 1..8 HEND+HSTRT must be <= 17 (15 in datasheet)
		TMC2660_setHysteresisEnd(MotorNo, 3);// !!! remember, add +3 Valuse are moved to 0...15 not -3..12 HEND+HSTRT must be <= 17(15 in datasheet)
		TMC2660_setHysteresisDecrement(MotorNo, 0);
		TMC2660_setRandomTime(MotorNo, 0);
		TMC2660_setChopperMode(MotorNo, 0);//Standard mode (spreadCycle)
		//Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54
		TMC2660_setBlankingTime(MotorNo, 2);
		//DRVCONF settings
		TMC2660_setReadOut(MotorNo, 1);//get stall guard value
		TMC2660_setVSENSE(MotorNo, 1);//sense resistor voltage is 310mV - will be changed by TMC2660_setRMS_Current if needed
		TMC2660_setStepDir(MotorNo, 0);//Select the Step/Dir interface
		TMC2660_setGndDetectionTime(MotorNo, 0);//3.2us
		TMC2660_setGndProtection(MotorNo, 0);//enable short to ground protection
		TMC2660_set_SlopeControlLowSide(MotorNo, 3);// medium
		TMC2660_set_SlopeControlHighSide(MotorNo, 3);// medium
		//SMARTEN settings
		TMC2660_setCoolStepThldLow(MotorNo, 2);//if not 0 then coolStep is enabled
		TMC2660_setCurrentIncrement(MotorNo, 1);
		TMC2660_setCoolStepThldHigh(MotorNo, 2);
		TMC2660_setCurrentDecrementSpeed(MotorNo, 32);
		TMC2660_setMinCoolStepCurrent(MotorNo, 1);//½ CS current setting
		//SGCSCONF settings
		TMC2660_setCurrentScale(MotorNo, 31);//or use TMC2660_setRMS_Current scale
		//TMC2660_setRMS_Current(MotorNo, 1697);//current in mA 2.4A max/
		TMC2660_setStallGuardThld(MotorNo, -5);//increase/decrese depending on the torque needed -64 to 63
		TMC2660_setStallGuardFilter(MotorNo, 1);//enable filter for better accuracy
		TMC2660_setSpeed(MotorNo, myMotors[MotorNo].mSPS);//set steps per second
		printf("------------------------------------------\n");
		print_Bits(myMotors[MotorNo].mDRVCTRL);
		print_Bits(myMotors[MotorNo].mCHOPCONF);
		print_Bits(myMotors[MotorNo].mSMARTEN);
		print_Bits(myMotors[MotorNo].mSGCSCONF);
		print_Bits(myMotors[MotorNo].mDRVCONF);
		printf("------------------------------------------\n");
	}

	void TMC2660_start(uint8_t MotorNo){
		printf("------------------------------------------\n");
		printf("Motor = %d\n", MotorNo);
		printf("DRVCTRL  ");
		send(MotorNo, myMotors[MotorNo].mDRVCTRL);

		printf("CHOPCONF ");
		send(MotorNo, myMotors[MotorNo].mCHOPCONF);

		printf("SMARTEN  ");
		send(MotorNo, myMotors[MotorNo].mSMARTEN);

		printf("SGCSCONF ");
		send(MotorNo, myMotors[MotorNo].mSGCSCONF);

		printf("DRVCONF  ");
		send(MotorNo, myMotors[MotorNo].mDRVCONF);
		printf("------------------------------------------\n");		
//		send(MotorNo, 0x80008);
//		send(MotorNo, 0x88008);
//		send(MotorNo, 0x88004);
//		send(MotorNo, 0x88004);
//		send(MotorNo, 0xE0000);
//		send(MotorNo, 0xC0017);
		TMC2660_setDirection(MotorNo, myMotors[MotorNo].mDirection);
		//save that we are in running mode
		myMotors[MotorNo].mStatus = m_STARTING;
		//Enable pin set to low
		if(MotorNo){
			__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
			//Motor 1 PWM
			//HAL_TIM_Base_Start(&htim5);
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
			V_EN_On;
		} else {
			__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
			//Motor 0 PWM
			//HAL_TIM_Base_Start(&htim2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			H_EN_On;
		}
	}


	//0: No motor stall detected.
	//1: stallGuard2 threshold has been reached, and the SG_TST output is driven high.
	uint8_t TMC2660_getStallGuardStatus(uint8_t MotorNo){
		return myMotors[MotorNo].mDRVSTATUS & ONE;
	}

	//0: No overtemperature shutdown condition.
	//1: Overtemperature shutdown has occurred.
	uint8_t TMC2660_getTempShutDown(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> OT_pos) & ONE);
	}

	//0: No overtemperature warning condition.
	//1: Warning threshold is active.
	uint8_t TMC2660_getTempWarning(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> OTPW_pos) & ONE);
	}

	//0: No short to ground shutdown condition.
	//1: Short to ground shutdown condition. The short counter is incremented by each short
	//circuit and the chopper cycle is suspended. The counter is decremented for each phase
	//polarity change. The MOSFETs are shut off when the counter reaches 3 and remain shut off
	//until the shutdown condition is cleared by disabling and re-enabling the driver.
	//The shutdown conditions reset by deasserting the ENN input or clearing the TOFF parameter.
	uint8_t TMC2660_getShortA(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> S2GA_pos) & ONE);
	}

	uint8_t TMC2660_getShortB(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> S2GB_pos) & ONE);
	}

	//0: No open load condition detected.
	//1: No chopper event has happened during the last period with constant coil polarity.
	//Only a current above 1/16 of the maximum setting can clear this bit!
	//Hint: This bit is only a status indicator. The chip takes no other action when this bit is set.
	//False indications may occur during fast motion and at standstill. Check this bit only during slow motion.
	uint8_t TMC2660_getOpenLoadA(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> OLA_pos) & ONE);
	}

	uint8_t TMC2660_getOpenLoadB(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> OLB_pos) & ONE);
	}

	//0: No standstill condition detected.
	//1: No active edge occurred on the STEP input during the last 220 system clock cycles.
	uint8_t TMC2660_getStandStill(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVSTATUS >> STST_pos) & ONE);
	}

	void TMC2660_getStatus(uint8_t MotorNo){
		
		printf("->getStat DRVCONF %d = \n", MotorNo);
		print_Bits(myMotors[MotorNo].mDRVCONF);
		send(MotorNo, myMotors[MotorNo].mDRVCONF);
		printf("Result \n");
		print_Bits(myMotors[MotorNo].mDRVCONF);
		uint32_t response = myMotors[MotorNo].mDRVSTATUS & 0xFFCFF;
		printf("& 0xFFCFF \n");
		print_Bits(response);
		myMotors[MotorNo].mDRVSTATUS = (response & 0xFF);
		printf("& 0xFF \n");
		print_Bits(myMotors[MotorNo].mDRVCONF);
		myMotors[MotorNo].mDRVSTATUS |= (response & 0xFFC00);
		printf("& 0xFFC00 \n");
		print_Bits(myMotors[MotorNo].mDRVCONF);
		printf("getStatus response %d = ", MotorNo);
		print_Bits(myMotors[MotorNo].mDRVSTATUS);
		if(TMC2660_getStallGuardStatus(MotorNo))
			printf("Motor %d StallGuard detected \n", MotorNo);
		if(TMC2660_getTempShutDown(MotorNo))
			printf("Motor %d TempShutDown detected \n", MotorNo);
		if(TMC2660_getTempWarning(MotorNo))
			printf("Motor %d TempWarning detected \n", MotorNo);		
		if(TMC2660_getShortA(MotorNo))
			printf("Motor %d A Short to GND detected \n", MotorNo);
		if(TMC2660_getShortB(MotorNo))
			printf("Motor %d B Short to GND detected \n", MotorNo);
		if(TMC2660_getOpenLoadA(MotorNo))
			printf("Motor %d A OpenLoad detected \n", MotorNo);
		if(TMC2660_getOpenLoadB(MotorNo))
			printf("Motor %d B OpenLoad detected \n", MotorNo);	
		if(TMC2660_getStandStill(MotorNo))
			printf("Motor %d StandStill detected \n", MotorNo);
		//printf("Motor %d StallGuardValue %d \n", MotorNo, TMC2660_getFullStallGuardValue(MotorNo));			
	}
	
//we start with DRVCTRL register
//0			1			2			3			4	5	6	7	8			9		
//MRES0	MRES1	MRES2	MRES3	0	0	0	0	DEDGE	INTPOL	
//10	11	12	13	14	15	16	17	18	19																			
//0	0	0	0	0	0	0	0	0	0

//Microstep resolution for STEP/DIR mode
//Microsteps per 90°:
//%0000:0 256
//%0001:1 128
//%0010:2 64
//%0011:3 32
//%0100:4 16
//%0101:5 8
//%0110:6 4
//%0111:7 2 (halfstep)
//%1000:8 1 (fullstep)
void TMC2660_setResolution(uint8_t MotorNo, uint16_t Resolution){
	myMotors[MotorNo].mDRVCTRL = (myMotors[MotorNo].mDRVCTRL & MRES_mask) | Resolution;
	if(myMotors[MotorNo].mStatus)
		send(MotorNo, myMotors[MotorNo].mDRVCTRL);
}

uint16_t TMC2660_getResolution(uint8_t MotorNo){
    return (myMotors[MotorNo].mDRVCTRL & (~MRES_mask));
}

	//Enable double edge STEP pulses
	//0: Rising STEP pulse edge is active, falling edge is inactive.
	//1: Both rising and falling STEP pulse edges are active.
	void TMC2660_setDoubleEdge(uint8_t MotorNo, uint8_t doubleEdge){
	if(doubleEdge){
		myMotors[MotorNo].mDRVCTRL |= DEDGE_mask;
	} else {
		myMotors[MotorNo].mDRVCTRL &= ~DEDGE_mask;
	}
	if(myMotors[MotorNo].mStatus)
		send(MotorNo, myMotors[MotorNo].mDRVCTRL);
	}

	uint8_t TMC2660_getDoubleEdge(uint8_t MotorNo){
		return (myMotors[MotorNo].mDRVCTRL >> DEDGE_pos) & ONE;
	}

	//Enable STEP interpolation
	//0: Disable STEP pulse interpolation.
	//1: Enable STEP pulse multiplication by 16.
	void TMC2660_setInterpolation(uint8_t MotorNo, uint8_t interpolation){
	if(interpolation){
		myMotors[MotorNo].mDRVCTRL |= INTPOL_mask;
	} else {
		myMotors[MotorNo].mDRVCTRL &= ~INTPOL_mask;
	}
	if(myMotors[MotorNo].mStatus)
		send(MotorNo, myMotors[MotorNo].mDRVCTRL);
	}

	uint8_t TMC2660_getInterpolation(uint8_t MotorNo){
		return (myMotors[MotorNo].mDRVCTRL >> INTPOL_pos) & ONE;
	}

	//CHOPCONF register
	//0			1			2			3			4				5				6				7			8			9			
	//TOFF0	TOFF1	TOFF2	TOFF3	HSTRT0	HSTRT1	HSTRT2	HEND0	HEND1	HEND2	
	//10		11		12		13		14	15		16		17	18	19
	//HEND3	HDEC0	HDEC1	RNDTF	CHM	TBL0	TBL1	0		0		1

	//Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off.
	//If TOFF is nonzero, slow decay time is a multiple of system clock
	//periods: NCLK= 12 + (32 x TOFF) (Minimum time is 64clocks.)
	//%0000: Driver disable, all bridges off
	//%0001: 1 (use with TBL of minimum 24 clocks)
	//%0010 … %1111: 2 … 15
	void TMC2660_setTimeOff(uint8_t MotorNo, uint8_t TimeOff){
	if(TimeOff > 15)
		TimeOff = 15;
	myMotors[MotorNo].mCHOPCONF = (myMotors[MotorNo].mCHOPCONF & TOFF_mask) | TimeOff;
	if(myMotors[MotorNo].mStatus)
		send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getTimeOff(uint8_t MotorNo){
		return (myMotors[MotorNo].mCHOPCONF & (~TOFF_mask));
	}

	//CHM=0 Hysteresis start offset from HEND:
	//%000: 1 %001: 2 %010: 3 %011: 4 %100: 5 %101: 6 %110: 7 %111: 8
	//Effective: HEND+HSTRT must be = 15
	//CHM=1 Three least-significant bits of the duration of the fast decay phase. The MSB is HDEC0.
	//Fast decay time is a multiple of system clock periods: NCLK= 32 x (HDEC0+HSTRT)
	// !!! Valuse are moved to 0...7 not 1..8
	void TMC2660_setHysteresisStart(uint8_t MotorNo, uint8_t HysteresisStart){
		if(HysteresisStart > 7)
			HysteresisStart = 7;
		int8_t hEND = TMC2660_getHysteresisEnd(MotorNo);
		if((hEND + HysteresisStart) > 17)
			HysteresisStart = 17 - hEND;
		myMotors[MotorNo].mCHOPCONF = (myMotors[MotorNo].mCHOPCONF & HSTRT_mask) | ((uint32_t)HysteresisStart << HSTRT_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getHysteresisStart(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF & (~HSTRT_mask)) >> HSTRT_pos); 
	}

	//CHM=0 %0000 … %1111:
	//Hysteresis is -3, -2, -1, 0, 1, …, 12 (1/512 of this setting adds to current setting)
	//This is the hysteresis value which becomes used for the hysteresis chopper
	//CHM=1 %0000 … %1111: Offset is -3, -2, -1, 0, 1, …, 12
	//This is the sine wave offset and 1/512 of the value becomes added to the absolute value
	//of each sine wave entry.
	// !!! Valuse are moved to 0...15 not -3..12
	void TMC2660_setHysteresisEnd(uint8_t MotorNo, uint8_t HysteresisEnd){
		if(HysteresisEnd > 15)
			HysteresisEnd = 15;
		uint8_t hSTA = TMC2660_getHysteresisStart(MotorNo);
		if((hSTA + HysteresisEnd) > 17)
			HysteresisEnd = 17 - hSTA;
		myMotors[MotorNo].mCHOPCONF = (myMotors[MotorNo].mCHOPCONF & HEND_mask) | ((uint32_t)HysteresisEnd << HEND_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getHysteresisEnd(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF & (~HEND_mask)) >> HEND_pos);
	}

	//Hysteresis decrement interval or Fast decay mode
	//CHM=0 Hysteresis decrement period setting, in system clock periods: %00: 16 %01: 32 %10: 48 %11: 64
	//CHM=1 HDEC1=0: current comparator can terminate the fast decay phase before timer expires.
	//HDEC1=1: only the timer terminates the fast decay phase.
	//HDEC0: MSB of fast decay time setting
	void TMC2660_setHysteresisDecrement(uint8_t MotorNo, uint8_t hysteresisDecrement){
		if(hysteresisDecrement > 3)
			hysteresisDecrement = 3;
		myMotors[MotorNo].mCHOPCONF = (myMotors[MotorNo].mCHOPCONF & HDEC_mask) | ((uint32_t)hysteresisDecrement << HDEC_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getHysteresisDecrement(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF & (~HDEC_mask)) >> HDEC_pos);
	}

	//Enable randomizing the slow decay phase duration:
	//0: Chopper off time is fixed as set by bits tOFF
	//1: Random mode, tOFF is random modulated by dNCLK= -12 … +3 clocks.
	void TMC2660_setRandomTime(uint8_t MotorNo, uint8_t randomTime){
		if(randomTime){
			myMotors[MotorNo].mCHOPCONF |= RNDTF_mask;
		} else {
			myMotors[MotorNo].mCHOPCONF &= ~RNDTF_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getRandomTime(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF >> RNDTF_pos) & ONE);
	}

	// This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below
	//0 - Standard mode (spreadCycle)
	//1 - Constant tOFF with fast decay time. Fast decay time is also terminated when the
	//negative nominal current is reached. Fast decay is after on time.

	void TMC2660_setChopperMode(uint8_t MotorNo, uint8_t ChopperMode){
		if(ChopperMode){
			myMotors[MotorNo].mCHOPCONF |= CHM_mask;
		} else {
			myMotors[MotorNo].mCHOPCONF &= ~CHM_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getChopperMode(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF >> CHM_pos) & ONE);
	}

	//Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54
	void TMC2660_setBlankingTime(uint8_t MotorNo, uint8_t BlankingTime){
		if(BlankingTime > 3)
			BlankingTime = 3;
		myMotors[MotorNo].mCHOPCONF = (myMotors[MotorNo].mCHOPCONF & TBL_mask) | ((uint32_t)BlankingTime << TBL_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mCHOPCONF);
	}

	uint8_t TMC2660_getBlankingTime(uint8_t MotorNo){
		return ((myMotors[MotorNo].mCHOPCONF & (~TBL_mask)) >> TBL_pos);
	}

	//Next on the list is SMARTEN register
	//0		1		2		3		4	5		6		7	8		9		
	//SEMIN0	SEMIN1	SEMIN2	SEMIN3	0	SEUP0	SEUP1	0	SEMAX0	SEMAX1	
	//10		11		12	13		14		15		16	17	18	19																			
	//SEMAX2	SEMAX3	0	SEDN0	SEDN1	SEIMIN	0	1	0	1

	//Lower coolStep threshold/coolStep disable
	//If SEMIN is 0, coolStep is disabled. If SEMIN is nonzero and the stallGuard2 value SG falls below SEMIN x 32,
	//the coolStep current scaling factor is increased.

	void TMC2660_setCoolStepThldLow(uint8_t MotorNo, uint8_t LowerCoolStepThreshold){
		if(LowerCoolStepThreshold > 15)
			LowerCoolStepThreshold = 15;
		myMotors[MotorNo].mSMARTEN = (myMotors[MotorNo].mSMARTEN & SEMIN_mask) | LowerCoolStepThreshold;
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSMARTEN);
	}

	uint8_t TMC2660_getCoolStepThldLow(uint8_t MotorNo){
		return (myMotors[MotorNo].mSMARTEN & (~SEMIN_mask));
	}

	//Current increment size
	//Number of current increment steps for each time that the stallGuard2 value SG is sampled below the lower
	//threshold: %00: 1 %01: 2 %10: 4 %11: 8
	void TMC2660_setCurrentIncrement(uint8_t MotorNo, uint8_t CurrentIncrement){
		if(CurrentIncrement > 3)
			CurrentIncrement = 3;
		myMotors[MotorNo].mSMARTEN = (myMotors[MotorNo].mSMARTEN & SEUP_mask) | ((uint32_t)CurrentIncrement << SEUP_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSMARTEN);
	}

	uint8_t TMC2660_getCurrentIncrement(uint8_t MotorNo){
		return ((myMotors[MotorNo].mSMARTEN & (~SEUP_mask)) >> SEUP_pos);
	}

	//Upper coolStep threshold as an offset from the lower threshold
	//If the stallGuard2 measurement value SG is sampled equal to or above (SEMIN+SEMAX+1) x 32
	//enough times, then the coil current scaling factor is decremented.
	void TMC2660_setCoolStepThldHigh(uint8_t MotorNo, uint8_t UpperCoolStepThreshold){
		if(UpperCoolStepThreshold > 15)
			UpperCoolStepThreshold = 15;
		myMotors[MotorNo].mSMARTEN = (myMotors[MotorNo].mSMARTEN & SEMAX_mask) | ((uint32_t)UpperCoolStepThreshold << SEMAX_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSMARTEN);
	}

	uint8_t TMC2660_getCoolStepThldHigh(uint8_t MotorNo){
		return ((myMotors[MotorNo].mSMARTEN & (~SEMAX_mask)) >> SEMAX_pos);
	}

	//Current decrement speed
	//Number of times that the stallGuard2 value must be sampled equal to or above the
	//upper threshold for each decrement of the coil current: %00: 32 %01: 8 %10: 2 %11: 1
	void TMC2660_setCurrentDecrementSpeed(uint8_t MotorNo, uint8_t CurrentDecrementSpeed){
		if(CurrentDecrementSpeed > 3)
			CurrentDecrementSpeed = 3;
		myMotors[MotorNo].mSMARTEN = (myMotors[MotorNo].mSMARTEN & SEDN_mask) | ((uint32_t)CurrentDecrementSpeed<<SEDN_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSMARTEN);
	}

	uint8_t TMC2660_getCurrentDecrementSpeed(uint8_t MotorNo){
		return (myMotors[MotorNo].mSMARTEN & (~SEDN_mask)) >> SEDN_pos;
	}

	//Minimum coolStep current
	//0: ½ CS current setting
	//1: ¼ CS current setting
	void TMC2660_setMinCoolStepCurrent(uint8_t MotorNo, uint8_t coolStepCurrent){
		if(coolStepCurrent){
			myMotors[MotorNo].mSMARTEN |= SEIMIN_mask;
		} else {
			myMotors[MotorNo].mSMARTEN &= ~SEIMIN_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSMARTEN);
	}

	uint8_t TMC2660_getMinCoolStepCurrent(uint8_t MotorNo){
		return ((myMotors[MotorNo].mSMARTEN >> SEIMIN_pos) & ONE);
	}

	//Next on the list is SGCSCONF register
	//0		1	2	3	4	5	6	7	8		9		
	//CS0	CS1	CS2	CS3	CS4	0	0	0	SGT0	SGT1	
	//10	11		12		13		14		15	16		17	18	19																			
	//SGT2	SGT3	SGT4	SGT5	SGT6	0	SFILT	0		1		1

	//Requested current = mA = I_rms/1000
	//Equation for current:
	//I_rms = (CS+1)/32 * (V_fs/R_sense) * (1/sqrt(2))
	//Solve for CS ->
	//CS = 32*sqrt(2)*I_rms*R_sense/V_fs - 1
	//  Example:
	//  vsense = 0b0 -> V_fs = 0.310V //Typical
	//  mA = 1650mA = I_rms/1000 = 1.65A
	//  R_sense = 0.100 Ohm
	//  CS = 32*sqrt(2)*1.65*0.100/0.310 - 1 = 24,09
	//  CS = 24
	void TMC2660_setRMS_Current(uint8_t MotorNo, uint16_t mA) {
		uint8_t CS = TMC2660_SENS_LOW*(double)mA - 1.0;
		// If Current Scale is too low, turn on high sensitivity R_sense and calculate again
		if (CS < 16) {
			TMC2660_setVSENSE(MotorNo, 1);
			CS = TMC2660_SENS_HIGH*(double)mA - 1.0;
		} else { // If CS >= 16, turn off high_sense_r
			TMC2660_setVSENSE(MotorNo, 0);
		}
		if(CS > 31)
			CS = 31;
		TMC2660_setCurrentScale(MotorNo, CS);
	}
	
	
	//Current scale (scales digital currents A and B)
	//Current scaling for SPI and step/direction operation. %00000 … %11111: 1/32, 2/32, 3/32, … 32/32
	//This value is biased by 1 and divided by 32, so the range is 1/32 to 32/32. Example: CS=0 is 1/32 current
	void TMC2660_setCurrentScale(uint8_t MotorNo, uint8_t CurrentScale){
		if(CurrentScale > 31)
			CurrentScale = 31;
		myMotors[MotorNo].mSGCSCONF = (myMotors[MotorNo].mSGCSCONF & CS_mask) | CurrentScale;
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSGCSCONF);
	}

	uint8_t TMC2660_getCurrentScale(uint8_t MotorNo){
		return (myMotors[MotorNo].mSGCSCONF & (~CS_mask));
	}

	//stallGuard2 threshold value
	//The stallGuard2 threshold value controls the optimum measurement range for readout.
	//A lower value results in a higher sensitivity and requires less torque to indicate a stall.
	//The value is a two’s complement signed integer. Values below -10 are not recommended.
	//Range: -64 to +63
	void TMC2660_setStallGuardThld(uint8_t MotorNo, int8_t StallGuardThld){
		if(StallGuardThld > 63) StallGuardThld = 63;
		if(StallGuardThld < -64) StallGuardThld = -64;
		StallGuardThld &= 0x7f;
		myMotors[MotorNo].mSGCSCONF = (myMotors[MotorNo].mSGCSCONF & SGT_mask) | ((uint32_t)StallGuardThld << SGT_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSGCSCONF);
	}

	int8_t TMC2660_getStallGuardThld(uint8_t MotorNo){
    uint8_t tmp = ((myMotors[MotorNo].mSGCSCONF & (~SGT_mask)) >> SGT_pos);
    if(tmp > 63){
	    return tmp - 128;
    } else {
			return tmp;
    }
	}

	//stallGuard2 filter enable
	//0: Standard mode, fastest response time.
	//1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	void TMC2660_setStallGuardFilter(uint8_t MotorNo, uint8_t GuardFilter){
		if(GuardFilter){
			myMotors[MotorNo].mSGCSCONF |= SFILT_mask;
		} else {
			myMotors[MotorNo].mSGCSCONF &= ~SFILT_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mSGCSCONF);
	}

	uint8_t TMC2660_getStallGuardFilter(uint8_t MotorNo){
		return ((myMotors[MotorNo].mSGCSCONF >> SFILT_pos) & ONE);
	}	

	//Next on the list is DRVCONF register
	//0	1	2	3	4		5		6		7		8		9		
	//0	0	0	0	RDSEL0	RDSEL1	VSENSE	SDOFF	TS2G0	TS2G1	
	//10		11	12		13		14		15		16	17	18	19																			
	//DISS2G	0	SLPL0	SLPL1	SLPH0	SLPH1	TST	1	1	1

	//Select value for read out (RD bits)
	//%00 Microstep position read back
	//%01 stallGuard2 level read back
	//%10 stallGuard2 and coolStep current level read back
	//%11 Reserved, do not use
	void TMC2660_setReadOut(uint8_t MotorNo, uint8_t ReadOut){
		if(ReadOut > 2)
			ReadOut = 2;
		myMotors[MotorNo].mDRVCONF = (myMotors[MotorNo].mDRVCONF & RDSEL_mask) | ((uint32_t)ReadOut << RDSEL_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getReadOut(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF & (~RDSEL_mask)) >> RDSEL_pos);
	}

	//Sense resistor voltage-based current scaling
	//0: Full-scale sense resistor voltage is 305mV.
	//1: Full-scale sense resistor voltage is 165mV.
	//(Full-scale refers to a current setting of 31 and a DAC value of 255.)
	void TMC2660_setVSENSE(uint8_t MotorNo, uint8_t vSense){
		if(vSense){
			myMotors[MotorNo].mDRVCONF |= VSENSE_mask;
		} else {
			myMotors[MotorNo].mDRVCONF &= ~VSENSE_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getVSENSE(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF >> VSENSE_pos) & ONE);
	}
	
	//STEP/DIR interface disable
	//0: Enable STEP and DIR interface.
	//1: Disable STEP and DIR interface. SPI interface is used to move motor.
	void TMC2660_setStepDir(uint8_t MotorNo, uint8_t StepDirSPI){
		if(StepDirSPI){
			myMotors[MotorNo].mDRVCONF |= SDOFF_mask;
		} else {
			myMotors[MotorNo].mDRVCONF &= ~SDOFF_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getStepDir(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF >> SDOFF_pos) & ONE);
	}
	
	//Short to GND detection timer
	//%00: 3.2µs. %01: 1.6µs. %10: 1.2µs. %11: 0.8µs.
	void TMC2660_setGndDetectionTime(uint8_t MotorNo, uint8_t GndTimer){
		if(GndTimer > 3)
			GndTimer = 3;
		myMotors[MotorNo].mDRVCONF = (myMotors[MotorNo].mDRVCONF & TS2G_mask) | ((uint32_t)GndTimer << TS2G_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getGndDetectionTime(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF & (~TS2G_mask)) >> TS2G_pos);
	}
	
	//Short to GND protection disable
	//0: Short to GND protection is enabled.
	//1: Short to GND protection is disabled.
	void TMC2660_setGndProtection(uint8_t MotorNo, uint8_t GndProtection){
		if(GndProtection){
			myMotors[MotorNo].mDRVCONF |= DISS2G_mask;
		} else {
			myMotors[MotorNo].mDRVCONF &= ~DISS2G_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getGndProtection(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF >> DISS2G_pos) & ONE);
	}

	//Slope control, low side
	//%00: Minimum.	%01: Minimum.	%10: Medium.	%11: Maximum.
	void TMC2660_set_SlopeControlLowSide(uint8_t MotorNo, uint8_t SlopeControlLowSide){
		if(SlopeControlLowSide > 3)
			SlopeControlLowSide = 3;
		myMotors[MotorNo].mDRVCONF = (myMotors[MotorNo].mDRVCONF & SLPL_mask) | ((uint32_t)SlopeControlLowSide << SLPL_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getSlopeControlLowSide(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF & (~SLPL_mask)) >> SLPL_pos);
	}
	
	//Slope control, high side
	//%00: Minimum	%01: Minimum temperature compensation mode.	%10: Medium temperature compensation mode.
	//%11: Maximum	In temperature compensated mode (tc), the MOSFET gate driver strength is increased if the overtemperature
	//warning temperature is reached. This compensates for temperature dependency of high-side slope control.
	void TMC2660_set_SlopeControlHighSide(uint8_t MotorNo, uint8_t SlopeControlHighSide){
		if(SlopeControlHighSide > 3)
			SlopeControlHighSide = 3;
		myMotors[MotorNo].mDRVCONF = (myMotors[MotorNo].mDRVCONF & SLPH_mask) | ((uint32_t)SlopeControlHighSide << SLPH_pos);
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getSlopeControlHighSide(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF & (~SLPH_mask)) >> SLPH_pos);
	}
	
	//Reserved TEST mode
	//Must be cleared for normal operation. When set, the SG_TST output exposes digital test values,
	//and the TEST_ANA output exposes analog test values. Test value selection is controlled by SGT1 and SGT0:
	//TEST_ANA: %00: anatest_2vth, %01: anatest_dac_out, %10: anatest_vdd_half.
	//SG_TST: %00: comp_A, %01: comp_B, %10: CLK, %11: on_state_xy
	void TMC2660_setTestMode(uint8_t MotorNo, uint8_t TestMode){
		if(TestMode){
			myMotors[MotorNo].mDRVCONF |= TST_mask;
		} else {
			myMotors[MotorNo].mDRVCONF &= ~TST_mask;
		}
		if(myMotors[MotorNo].mStatus)
			send(MotorNo, myMotors[MotorNo].mDRVCONF);
	}

	uint8_t TMC2660_getTestMode(uint8_t MotorNo){
		return ((myMotors[MotorNo].mDRVCONF >> TST_pos) & ONE);
	}

	