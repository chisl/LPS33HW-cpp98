/*
 * name:        LPS33HW
 * description: MEMS pressure sensor: 260-1260 hPa absolute digital output barometer with water-resistant package
 * manuf:       STMicroelectronics
 * version:     Version 0.1
 * url:         http://www.st.com/resource/en/datasheet/lps33hw.pdf
 * date:        2018-01-02
 * author       https://chisl.io/
 * file:        LPS33HW.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class LPS33HW_Base and implement the read and write functions! */

/* LPS33HW: MEMS pressure sensor: 260-1260 hPa absolute digital output barometer with water-resistant package */
class LPS33HW_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG INTERRUPT_CFG                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG INTERRUPT_CFG:
	 * 8.1 Interrupt configuration
	 * To generate an interrupt event based on a user-defined threshold, the DIFF_EN bit must be
	 * set to '1' and the threshold values stored in THS_P_L (0Ch) and THS_P_H (0Dh).
	 * When DIFF_EN = '1', the PHE bit or PLE bit (or both bits) has to be enabled. The PHE and
	 * PLE bits enable the interrupt generation on the positive or negative event respectively.
	 * When DIFF_EN is enabled and AUTOZERO or AUTORIFP is enabled, the defined pressure
	 * threshold values in THS_P (0Ch, 0Dh) is compared with:
	 * P_DIFF_IN = measured pressure - pressure reference
	 * The value of the pressure reference is assigned depending on the AUTOZERO and
	 * AUTORIFP modes given in the next two paragraphs.
	 * 
	 * If the AUTOZERO bit is set to '1', the measured pressure is used as a reference on the
	 * register REF_P (15h, 16h and 17h). From that point on, the output pressure registers
	 * PRESS_OUT (PRESS_OUT_H (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h))
	 * are updated and the same value is also used for the interrupt generation:
	 * – PRESS_OUT = measured pressure - REF_P
	 * After the first conversion the AUTOZERO bit is automatically set to '0'. To return back to
	 * normal mode, the RESET_AZ bit has to be set to '1'. This resets also the content of the
	 * REF_P registers.
	 * If the AUTORIFP bit is set to '1', the measured pressure is used as a reference on the
	 * register REF_P (15h, 16h and 17h). The output registers PRESS_OUT (PRESS_OUT_H
	 * (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h)) show the difference between the
	 * measured pressure and the content of the RPDS registers (18h and 19h):
	 * – PRESS_OUT = measured pressure - RPDS*256
	 * After the first conversion the AUTORIFP bit is automatically set to '0'. To return back to
	 * normal mode, the RESET_ARP bit has to be set to '1'.
	 */
	struct INTERRUPT_CFG
	{
		static const uint16_t __address = 11;
		
		/* Bits AUTORIFP: */
		/* AUTORIFP: Enable AutoRifP function.   */
		struct AUTORIFP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t AUTORIFP_ENABLED = 0b1; // 
		};
		/* Bits RESET_ARP: */
		/* Reset AutoRifP function.  */
		struct RESET_ARP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t RESET_AUTORIFP = 0b1; // 
		};
		/* Bits AUTOZERO: */
		/* Enable Autozero.  */
		struct AUTOZERO
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t AUTOZERO_ENABLED = 0b1; // 
		};
		/* Bits RESET_AZ: */
		/* Reset Autozero function.  */
		struct RESET_AZ
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t RESET_AUTOZERO = 0b1; // 
		};
		/* Bits DIFF_EN: */
		/* Enable interrupt generation.  */
		struct DIFF_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t INTERRUPT_GEN_DISABLED = 0b0; // 
			static const uint8_t INTERRUPT_GEN_ENABLED = 0b1; // 
		};
		/* Bits LIR: */
		/* Latch interrupt request to the INT_SOURCE register.  */
		struct LIR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NOT_LATCHED = 0b0; // 
			static const uint8_t LATCHED = 0b1; // 
		};
		/* Bits PLE: */
		/* Enable interrupt generation on differential pressure low event.  */
		struct PLE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // disable interrupt request
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value lower than preset threshold)
		};
		/* Bits PHE: */
		/* Enable interrupt generation on differential pressure high event.  */
		struct PHE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // disable interrupt request
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value higher than preset threshold)
		};
	};
	
	/* Set register INTERRUPT_CFG */
	void setINTERRUPT_CFG(uint8_t value)
	{
		write(INTERRUPT_CFG::__address, value, 8);
	}
	
	/* Get register INTERRUPT_CFG */
	uint8_t getINTERRUPT_CFG()
	{
		return read8(INTERRUPT_CFG::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG THS_P_L                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG THS_P_L:
	 * 8.2 Least significant bits of the threshold value for pressure interrupt generation.
	 * The threshold value for pressure interrupt generation is a 16-bit unsigned right-justified
	 * value composed of THS_P_H (0Dh) and THS_P_L (0Ch). The value is expressed as:
	 * Interrupt threshold (hPA) = (THS_P) / 16.
	 * To enable the interrupt event based on this user-defined threshold, the DIFF_EN bit in
	 * INTERRUPT_CFG (0Bh) must be set to '1', the PHE bit or PLE bit (or both bits) in
	 * INTERRUPT_CFG (0Bh) has to be enabled.
	 */
	struct THS_P_L
	{
		static const uint16_t __address = 12;
		
		/* Bits THS_P_L: */
		struct THS_P_L_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register THS_P_L */
	void setTHS_P_L(uint8_t value)
	{
		write(THS_P_L::__address, value, 8);
	}
	
	/* Get register THS_P_L */
	uint8_t getTHS_P_L()
	{
		return read8(THS_P_L::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG THS_P_H                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG THS_P_H:
	 * 8.3 Most significant bits of the threshold value for pressure interrupt generation.
	 */
	struct THS_P_H
	{
		static const uint16_t __address = 13;
		
		/* Bits THS_P_H: */
		struct THS_P_H_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register THS_P_H */
	void setTHS_P_H(uint8_t value)
	{
		write(THS_P_H::__address, value, 8);
	}
	
	/* Get register THS_P_H */
	uint8_t getTHS_P_H()
	{
		return read8(THS_P_H::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG WHO_AM_I                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG WHO_AM_I:
	 * 8.4 Device Who am I
	 */
	struct WHO_AM_I
	{
		static const uint16_t __address = 15;
		
		/* Bits WHO_AM_I: */
		struct WHO_AM_I_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register WHO_AM_I */
	void setWHO_AM_I(uint8_t value)
	{
		write(WHO_AM_I::__address, value, 8);
	}
	
	/* Get register WHO_AM_I */
	uint8_t getWHO_AM_I()
	{
		return read8(WHO_AM_I::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG1:
	 * 8.5 Control register 1
	 * Table 17. Output data rate bit configurations
	 * ODR2    ODR1 ODR0 Pressure (Hz) Temperature (Hz)
	 * 0    0 0 Power-down / One-shot mode enabled
	 * 0    0 1 1 Hz 1 Hz
	 * 0    1 0 10 Hz 10 Hz
	 * 0    1 1 25 Hz 25 Hz
	 * 1    0 0 50 Hz 50 Hz
	 * 1    0 1 75 Hz 75 Hz
	 * When the ODR bits are set to '000' the device is in Power-down mode. When the device
	 * is in power-down mode, almost all internal blocks of the device are switched off to
	 * minimize power consumption. The I2C interface is still active to allow communication with
	 * the device. The content of the configuration registers is preserved and output data
	 * registers are not updated, therefore keeping the last data sampled in memory before
	 * going into power-down mode.
	 * If the ONE_SHOT bit in CTRL_REG2 (11h) is set to '1', One-shot mode is triggered and a
	 * new acquisition starts when it is required. Enabling this mode is possible only if the device
	 * was previously in power-down mode (ODR bits set to '000'). Once the acquisition is
	 * completed and the output registers updated, the device automatically enters in power-
	 * down mode. The ONE_SHOT bit self-clears itself.
	 * When the ODR bits are set to a value different than '000', the device is in Continuous
	 * mode and automatically acquires a set of data (pressure and temperature) at the
	 * frequency selected through the ODR[2:0] bits.
	 * Once the additional low-pass filter has been enabled through the EN_LPFP bit, it is possible
	 * to configure the device bandwidth acting on the LPFP_CFG bit. See Table 18 for low-pass
	 * filter configurations.
	 * Table 18. Low-pass filter configurations
	 * EN_LPFP    LPFP_CFG Additional low-pass filter status Device bandwidth
	 * 0    x Disabled ODR/2
	 * 1    0 Enabled ODR/9
	 * 1    1 Enabled ODR/20
	 * The BDU bit is used to inhibit the update of the output registers between the reading of the
	 * upper and lower register parts. In default mode (BDU = ‘0’), the lower and upper register
	 * parts are updated continuously. When the BDU is activated (BDU = ‘1’), the content of the
	 * output registers is not updated until PRESS_OUT_H (2Ah) is read, avoiding the reading of
	 * values related to different samples.
	 */
	struct CTRL_REG1
	{
		static const uint16_t __address = 16;
		
		/* Bits unused_0: */
		/* This bit must be set to ‘0’ for proper operation of the device.  */
		struct unused_0
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits ODR: */
		/*
		 * Output data rate selection.
		 * Refer to Table 17.
		 */
		struct ODR
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b01110000; // [4,5,6]
		};
		/* Bits EN_LPFP: */
		/* Enable low-pass filter on pressure data.  */
		struct EN_LPFP
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // Low-pass filter disabled
			static const uint8_t ENABLE = 0b1; // Low-pass filter enabled
		};
		/* Bits LPFP_CFG: */
		/*
		 * Low-pass configuration register.
		 * Refer to Table 18.
		 */
		struct LPFP_CFG
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
		};
		/* Bits BDU: */
		/* Block data update.  */
		struct BDU
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t CONTINUOUS = 0b0; // continuous update
			static const uint8_t NOT_UPDATED_UNTIL_READ = 0b1; // output registers not updated until MSB and LSB have been read
		};
		/* Bits SIM: */
		/* SPI Serial Interface Mode selection.  */
		struct SIM
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t SIM_4WIRE = 0b0; // 4-wire interface
			static const uint8_t SIM_3WIRE = 0b1; // 3-wire interface
		};
	};
	
	/* Set register CTRL_REG1 */
	void setCTRL_REG1(uint8_t value)
	{
		write(CTRL_REG1::__address, value, 8);
	}
	
	/* Get register CTRL_REG1 */
	uint8_t getCTRL_REG1()
	{
		return read8(CTRL_REG1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG2:
	 * 8.6 Control register 2
	 */
	struct CTRL_REG2
	{
		static const uint16_t __address = 17;
		
		/* Bits BOOT: */
		/*
		 * Reboot memory content.
		 * The bit is self-cleared when the BOOT is completed.
		 */
		struct BOOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode;
			static const uint8_t REBOOT_MEM = 0b1; // reboot memory content.
		};
		/* Bits FIFO_EN: */
		/* FIFO enable.  */
		struct FIFO_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits STOP_ON_FTH: */
		/* Stop on FIFO threshold. Enable FIFO watermark level use.  */
		struct STOP_ON_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_ADD_INC: */
		/*
		 * Register address automatically incremented during a multiple byte access with a
		 * serial interface (I2C or SPI).
		 * The BOOT bit is used to refresh the content of the internal registers stored in the Flash
		 * memory block. At device power-up the content of the Flash memory block is transferred to
		 * the internal registers related to the trimming functions to allow correct behavior of the device
		 * itself. If for any reason the content of the trimming registers is modified, it is sufficient to use
		 * this bit to restore the correct values. When the BOOT bit is set to ‘1’, the content of the
		 * internal Flash is copied inside the corresponding internal registers and is used to calibrate
		 * the device. These values are factory trimmed and they are different for every device. They
		 * allow correct behavior of the device and normally they should not be changed. At the end of
		 * the boot process the BOOT bit is set again to ‘0’ by hardware. The BOOT bit takes effect
		 * after one ODR clock cycle.
		 * SWRESET is the software reset bit. The following device registers (INTERRUPT_CFG
		 * (0Bh), THS_P_L (0Ch), THS_P_H (0Dh), CTRL_REG1 (10h), CTRL_REG2 (11h),
		 * CTRL_REG3 (12h), FIFO_CTRL (14h), REF_P_XL (15h), REF_P_L (16h), REF_P_H
		 * (17h)) are reset to the default value if the SWRESET bit is set to '1'. The SWRESET bit
		 * returns back to '0' by hardware.
		 * The ONE_SHOT bit is used to start a new conversion when the ODR[2,0] bits in
		 * CTRL_REG1 (10h) are set to ‘000’. Writing a ‘1’ in ONE_SHOT triggers a single
		 * measurement of pressure and temperature. Once the measurement is done, the
		 * ONE_SHOT bit will self-clear, the new data are available in the output registers, and the
		 * STATUS (27h) bits are updated.
		 */
		struct F_ADD_INC
		{
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits I2C_DIS: */
		/* Disable I2C interface.  */
		struct I2C_DIS
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t I2C_ENABLED = 0b0; // 
			static const uint8_t I2C_DISABLED = 0b1; // 
		};
		/* Bits SWRESET: */
		/*
		 * Software reset.
		 * The bit is self-cleared when the reset is completed.
		 */
		struct SWRESET
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t SOFTWARE_RESET = 0b1; // 
		};
		/* Bits unused_0: */
		/* This bit must be set to ‘0’ for proper operation of the device  */
		struct unused_0
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits ONE_SHOT: */
		/* One-shot enable.  */
		struct ONE_SHOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t IDLE_MODE = 0b0; // 
			static const uint8_t NEW_DATASET_ACQUIRED = 0b1; // a new dataset is acquired)
		};
	};
	
	/* Set register CTRL_REG2 */
	void setCTRL_REG2(uint8_t value)
	{
		write(CTRL_REG2::__address, value, 8);
	}
	
	/* Get register CTRL_REG2 */
	uint8_t getCTRL_REG2()
	{
		return read8(CTRL_REG2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG3                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG3:
	 * 8.7 Control register 3 - INT_DRDY pin control register
	 */
	struct CTRL_REG3
	{
		static const uint16_t __address = 18;
		
		/* Bits INT_H_L: */
		/* Interrupt active-high/low.  */
		struct INT_H_L
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t ACTIVE_HIGH = 0b0; // 
			static const uint8_t ACTIVE_LOW = 0b1; // 
		};
		/* Bits PP_OD: */
		/* Push-pull/open-drain selection on interrupt pads.  */
		struct PP_OD
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t PUSH_PULL = 0b0; // 
			static const uint8_t OPEN_DRAIN = 0b1; // 
		};
		/* Bits F_FSS5: */
		/* FIFO full flag on INT_DRDY pin.  */
		struct F_FSS5
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_FTH: */
		/* FIFO threshold (watermark) status on INT_DRDY pin.  */
		struct F_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_OVR: */
		/* FIFO overrun interrupt on INT_DRDY pin.  */
		struct F_OVR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits DRDY: */
		/* Data-ready signal on INT_DRDY pin.  */
		struct DRDY
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits INT_S: */
		/*
		 * Data signal on INT_DRDY pin control bits.
		 * Refer to Table 19.
		 */
		struct INT_S
		{
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t DATA_SIGNAL = 0b00; // Data signal (in order of priority: DRDY or F_FTH or F_OVR or F_FSS5
			static const uint8_t PRESSURE_HIGH = 0b01; // Pressure high (P_high)
			static const uint8_t PRESSURE_LOW = 0b10; // Pressure low (P_low)
			static const uint8_t PRESSURE_LOW_OR_HIGH = 0b11; // Pressure low OR high
		};
	};
	
	/* Set register CTRL_REG3 */
	void setCTRL_REG3(uint8_t value)
	{
		write(CTRL_REG3::__address, value, 8);
	}
	
	/* Get register CTRL_REG3 */
	uint8_t getCTRL_REG3()
	{
		return read8(CTRL_REG3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG FIFO_CTRL                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_CTRL:
	 * 8.8 FIFO control register
	 */
	struct FIFO_CTRL
	{
		static const uint16_t __address = 20;
		
		/* Bits F_MODE: */
		/*
		 * FIFO mode selection.
		 * Refer to Table 20 and Section 4 for additional details.
		 */
		struct F_MODE
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t BYPASS = 0b00; // Bypass mode        
			static const uint8_t FIFO = 0b01; // FIFO mode        
			static const uint8_t STREAM = 0b10; // Stream mode        
			static const uint8_t STREAM_TO_FIFO = 0b11; // Stream-to-FIFO mode        
			static const uint8_t BYPASS_TO_STREAM = 0b100; // Bypass-to-Stream mode        
			static const uint8_t reserved_0 = 0b101; // 
			static const uint8_t DYNAMIC_STREAM = 0b110; // Dynamic-Stream mode        
			static const uint8_t BYPASS_TO_FIFO = 0b111; // Bypass-to-FIFO mode        
		};
		/* Bits WTM: */
		/* FIFO watermark level selection.  */
		struct WTM
		{
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
		};
	};
	
	/* Set register FIFO_CTRL */
	void setFIFO_CTRL(uint8_t value)
	{
		write(FIFO_CTRL::__address, value, 8);
	}
	
	/* Get register FIFO_CTRL */
	uint8_t getFIFO_CTRL()
	{
		return read8(FIFO_CTRL::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG REF_P_XL                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG REF_P_XL:
	 * 8.9 Reference pressure (LSB
	 * This register contains the low part of the reference pressure value.
	 * The reference pressure value is 24-bit data and is composed of REF_P_H (17h), REF_P_L
	 * (16h) and REF_P_XL (15h). The value is expressed as 2’s complement.
	 * The reference pressure value is used when the AUTOZERO or AUTORIFP function is
	 * enabled (refer to CTRL_REG3 (12h)) and for the Autozero function (refer to the
	 * INTERRUPT_CFG (0Bh)).
	 */
	struct REF_P_XL
	{
		static const uint16_t __address = 21;
		
		/* Bits REF_P_XL: */
		struct REF_P_XL_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register REF_P_XL */
	void setREF_P_XL(uint8_t value)
	{
		write(REF_P_XL::__address, value, 8);
	}
	
	/* Get register REF_P_XL */
	uint8_t getREF_P_XL()
	{
		return read8(REF_P_XL::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG REF_P_L                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG REF_P_L:
	 * 8.1 Reference pressure (middle part)
	 * This register contains the mid part of the reference pressure value.
	 * Refer to REF_P_XL (15h).
	 */
	struct REF_P_L
	{
		static const uint16_t __address = 22;
		
		/* Bits REF_P_L: */
		struct REF_P_L_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register REF_P_L */
	void setREF_P_L(uint8_t value)
	{
		write(REF_P_L::__address, value, 8);
	}
	
	/* Get register REF_P_L */
	uint8_t getREF_P_L()
	{
		return read8(REF_P_L::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                           REG REF_P_H                                            *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG REF_P_H:
	 * 8.11 Reference pressure (MSB part)
	 * This register contains the high part of the reference pressure value.
	 * Refer to REF_P_XL (15h).
	 */
	struct REF_P_H
	{
		static const uint16_t __address = 23;
		
		/* Bits REF_P_H: */
		struct REF_P_H_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register REF_P_H */
	void setREF_P_H(uint8_t value)
	{
		write(REF_P_H::__address, value, 8);
	}
	
	/* Get register REF_P_H */
	uint8_t getREF_P_H()
	{
		return read8(REF_P_H::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG RPDS_L                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RPDS_L:
	 * 8.12 Pressure offset (LSB data)
	 * This register contains the low part of the pressure offset value.
	 * 
	 * If, after the soldering of the component, a residual offset is still present, it can be removed
	 * with a one-point calibration.
	 * After soldering, the measured offset can be stored in the RPDS_H (19h) and RPDS_L (18h)
	 * registers and automatically subtracted from the pressure output registers: the output
	 * pressure register PRESS_OUT (28h, 29h and 2Ah) is provided as the difference between
	 * the measured pressure and the content of the register 256*RPDS (18h, 19h)*.
	 * *DIFF_EN = '0', AUTOZERO = '0', AUTORIFP = '0'
	 */
	struct RPDS_L
	{
		static const uint16_t __address = 24;
		
		/* Bits RPDS_L: */
		struct RPDS_L_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RPDS_L */
	void setRPDS_L(uint8_t value)
	{
		write(RPDS_L::__address, value, 8);
	}
	
	/* Get register RPDS_L */
	uint8_t getRPDS_L()
	{
		return read8(RPDS_L::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG RPDS_H                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RPDS_H:
	 * 8.13 Pressure offset (MSB data)
	 * This register contains the high part of the pressure offset value.
	 * Refer to RPDS_L (18h).
	 */
	struct RPDS_H
	{
		static const uint16_t __address = 25;
		
		/* Bits RPDS_H: */
		struct RPDS_H_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register RPDS_H */
	void setRPDS_H(uint8_t value)
	{
		write(RPDS_H::__address, value, 8);
	}
	
	/* Get register RPDS_H */
	uint8_t getRPDS_H()
	{
		return read8(RPDS_H::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG RES_CONF                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RES_CONF:
	 * 8.14 Low-power mode configuration
	 */
	struct RES_CONF
	{
		static const uint16_t __address = 26;
		
		/* Bits unused_0: */
		/* These bits must be set to ‘0’ for proper operation of the device.  */
		struct unused_0
		{
			static const uint8_t dflt = 0b000000; // 6'd0
			static const uint8_t mask = 0b11111100; // [2,3,4,5,6,7]
		};
		/* Bits reserved_1: */
		/* The content of this bit must not be modified for proper operation of the device  */
		struct reserved_1
		{
			static const uint8_t mask = 0b00000010; // [1]
		};
		/* Bits LC_EN: */
		/* Low-current mode enable.  */
		struct LC_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NORMAL = 0b0; // normal mode (low-noise mode)
			static const uint8_t LOW_CURRENT = 0b1; // low-current mode
		};
	};
	
	/* Set register RES_CONF */
	void setRES_CONF(uint8_t value)
	{
		write(RES_CONF::__address, value, 8);
	}
	
	/* Get register RES_CONF */
	uint8_t getRES_CONF()
	{
		return read8(RES_CONF::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG INT_SOURCE                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG INT_SOURCE:
	 * 8.15 Interrupt source
	 */
	struct INT_SOURCE
	{
		static const uint16_t __address = 37;
		
		/* Bits BOOT_STATUS: */
		/* If ‘1’ indicates that the Boot (Reboot) phase is running.  */
		struct BOOT_STATUS
		{
			static const uint8_t mask = 0b10000000; // [7]
		};
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t dflt = 0b0000; // 4'd0
			static const uint8_t mask = 0b01111000; // [3,4,5,6]
		};
		/* Bits IA: */
		/* Interrupt active.  */
		struct IA
		{
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // one or more interrupt events have been generated
		};
		/* Bits PL: */
		/* Differential pressure Low.  */
		struct PL
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // Low differential pressure event has occurred
		};
		/* Bits PH: */
		/* Differential pressure High.  */
		struct PH
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // High differential pressure event has occurred).
		};
	};
	
	/* Set register INT_SOURCE */
	void setINT_SOURCE(uint8_t value)
	{
		write(INT_SOURCE::__address, value, 8);
	}
	
	/* Get register INT_SOURCE */
	uint8_t getINT_SOURCE()
	{
		return read8(INT_SOURCE::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FIFO_STATUS                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_STATUS:
	 * 8.16 IFO status
	 * Table 21. FIFO_STATUS example: OVR/FSS details
	 * FTH    OVRN    FSS5 FSS4 FSS3 FSS2 FSS1 FSS0 Description
	 * 0        0    0 0 0 0 0 0  FIFO empty
	 * 1        0    0 0 0 0 0 1  1 unread sample
	 * ...
	 * 1        0    1 0 0 0 0 0  32 unread samples
	 * 1        1    1 0 0 0 0 0  At least one sample has been overwritten
	 * When the number of unread samples in FIFO is greater than the threshold level set in register
	 * FIFO_CTRL (14h), FTH value is ‘1’.
	 */
	struct FIFO_STATUS
	{
		static const uint16_t __address = 38;
		
		/* Bits FTH_FIFO: */
		/* FIFO threshold status.  */
		struct FTH_FIFO
		{
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t BELOW = 0b0; // FIFO filling is lower than the threshold level,
			static const uint8_t AT_OR_ABOVE = 0b1; // FIFO filling is equal or higher than the threshold level).
		};
		/* Bits OVR: */
		/* FIFO overrun status.  */
		struct OVR
		{
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t NOT_FULL = 0b0; // FIFO is not completely full;
			static const uint8_t FULL = 0b1; // FIFO is full and at least one sample in the FIFO has been overwritten).
		};
		/* Bits FSS: */
		/* FIFO stored data level.  */
		struct FSS
		{
			static const uint8_t mask = 0b00111111; // [0,1,2,3,4,5]
			static const uint8_t EMPTY = 0b0000; // FIFO empty
			static const uint8_t FULL = 0b100000; // FIFO is full and has 32 unread samples).
		};
	};
	
	/* Set register FIFO_STATUS */
	void setFIFO_STATUS(uint8_t value)
	{
		write(FIFO_STATUS::__address, value, 8);
	}
	
	/* Get register FIFO_STATUS */
	uint8_t getFIFO_STATUS()
	{
		return read8(FIFO_STATUS::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                            REG STATUS                                             *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG STATUS:
	 * 8.17 Status register
	 */
	struct STATUS
	{
		static const uint16_t __address = 39;
		
		/* Bits unused_0: */
		struct unused_0
		{
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits T_OR: */
		/* Temperature data overrun.  */
		struct T_OR
		{
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // a new data for temperature has overwritten the previous one
		};
		/* Bits P_OR: */
		/* Pressure data overrun.  */
		struct P_OR
		{
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // new data for pressure has overwritten the previous one)
		};
		/* Bits unused_1: */
		struct unused_1
		{
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits T_DA: */
		/* Temperature data available.  */
		struct T_DA
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for temperature is not yet available;
			static const uint8_t NEW_DATA = 0b1; // new data for temperature is available)
		};
		/* Bits P_DA: */
		/* Pressure data available.  */
		struct P_DA
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for pressure is not yet available;
			static const uint8_t NEW_DATA = 0b1; // new data for pressure is available)
		};
	};
	
	/* Set register STATUS */
	void setSTATUS(uint8_t value)
	{
		write(STATUS::__address, value, 8);
	}
	
	/* Get register STATUS */
	uint8_t getSTATUS()
	{
		return read8(STATUS::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                         REG PRESS_OUT_XL                                          *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT_XL:
	 * 8.18 Pressure output value (LSB)
	 * This register contains the low part of the pressure output value.
	 * 
	 * The pressure output value is 24-bit data that contains the measured pressure. It is
	 * composed of PRESS_OUT_H (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h).
	 * The value is expressed as 2’s complement.
	 * The output pressure register PRESS_OUT is provided as the difference between the
	 * measured pressure and the content of the register RPDS (18h, 19h)*.
	 * Please refer to Section 3.4: Interpreting pressure readings for additional info.
	 * *DIFF_EN = '0', AUTOZERO = '0', AUTORIFP = '0'
	 */
	struct PRESS_OUT_XL
	{
		static const uint16_t __address = 40;
		
		/* Bits PRESS_OUT_XL: */
		struct PRESS_OUT_XL_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PRESS_OUT_XL */
	void setPRESS_OUT_XL(uint8_t value)
	{
		write(PRESS_OUT_XL::__address, value, 8);
	}
	
	/* Get register PRESS_OUT_XL */
	uint8_t getPRESS_OUT_XL()
	{
		return read8(PRESS_OUT_XL::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG PRESS_OUT_L                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT_L:
	 * 8.19 Pressure output value (mid part)
	 * This register contains the mid part of the pressure output value. Refer to
	 * PRESS_OUT_XL (28h)
	 */
	struct PRESS_OUT_L
	{
		static const uint16_t __address = 41;
		
		/* Bits PRESS_OUT_L: */
		struct PRESS_OUT_L_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PRESS_OUT_L */
	void setPRESS_OUT_L(uint8_t value)
	{
		write(PRESS_OUT_L::__address, value, 8);
	}
	
	/* Get register PRESS_OUT_L */
	uint8_t getPRESS_OUT_L()
	{
		return read8(PRESS_OUT_L::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG PRESS_OUT_H                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT_H:
	 * 8.2 Pressure output value (MSB)
	 * This register contains the high part of the pressure output value.
	 * POUT[23:16]
	 * Refer to PRESS_OUT_XL (28h)
	 */
	struct PRESS_OUT_H
	{
		static const uint16_t __address = 42;
		
		/* Bits PRESS_OUT_H: */
		struct PRESS_OUT_H_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register PRESS_OUT_H */
	void setPRESS_OUT_H(uint8_t value)
	{
		write(PRESS_OUT_H::__address, value, 8);
	}
	
	/* Get register PRESS_OUT_H */
	uint8_t getPRESS_OUT_H()
	{
		return read8(PRESS_OUT_H::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG TEMP_OUT_L                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT_L:
	 * 0.21 Temperature output value (LSB)
	 * This register contains the low part of the temperature output value.
	 * The temperature output value is 16-bit data that contains the measured temperature. It is
	 * composed of TEMP_OUT_H (2Ch), and TEMP_OUT_L (2Bh). The value is expressed as
	 * 2’s complement.
	 */
	struct TEMP_OUT_L
	{
		static const uint16_t __address = 43;
		
		/* Bits TEMP_OUT_L: */
		struct TEMP_OUT_L_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register TEMP_OUT_L */
	void setTEMP_OUT_L(uint8_t value)
	{
		write(TEMP_OUT_L::__address, value, 8);
	}
	
	/* Get register TEMP_OUT_L */
	uint8_t getTEMP_OUT_L()
	{
		return read8(TEMP_OUT_L::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG TEMP_OUT_H                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT_H:
	 * 8.22 Temperature output value (MSB)
	 * This register contains the high part of the temperature output value.
	 * The temperature output value is 16-bit data that contains the measured temperature. It is
	 * composed of TEMP_OUT_H (2Ch) and TEMP_OUT_L (2Bh). The value is expressed as 2’s
	 * complement.
	 */
	struct TEMP_OUT_H
	{
		static const uint16_t __address = 44;
		
		/* Bits TEMP_OUT_H: */
		struct TEMP_OUT_H_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register TEMP_OUT_H */
	void setTEMP_OUT_H(uint8_t value)
	{
		write(TEMP_OUT_H::__address, value, 8);
	}
	
	/* Get register TEMP_OUT_H */
	uint8_t getTEMP_OUT_H()
	{
		return read8(TEMP_OUT_H::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG LPFP_RES                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG LPFP_RES:
	 * 8.23 Low-pass filter reset register.
	 * I f the LPFP is active, in order to avoid the transitory phase, the filter can be reset by
	 * reading this register before generating pressure measurements.
	 */
	struct LPFP_RES
	{
		static const uint16_t __address = 51;
		
		/* Bits LPFP_RES: */
		struct LPFP_RES_
		{
			/* MODE - */
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register LPFP_RES */
	void setLPFP_RES(uint8_t value)
	{
		write(LPFP_RES::__address, value, 8);
	}
	
	/* Get register LPFP_RES */
	uint8_t getLPFP_RES()
	{
		return read8(LPFP_RES::__address, 8);
	}
	
};
