#ifndef __TMC5160_REG_H
#define __TMC5160_REG_H

#ifdef __cplusplus
extern "C" {
#endif


// Registers
typedef enum{
	// GENERAL CONFIGURATION REGISTERS (0X00…0X0F)
	GCONF			= 0x00,	// RW
	GSTAT			= 0x01,	// R + WC
	IFCNT			= 0x02, // R
	SLAVECONF		= 0x03, // W
	IOIN			= 0x04, // R
	OUTPUT			= 0x04, // W --- In UART mode, SDO_CFG0 is an output.
	X_COMPARE		= 0x05, // W
	OTP_PROG		= 0x06, // W
	OTP_READ		= 0x07, // R
	FACTORY_CONF	= 0x08, // RW
	SHORT_CONF		= 0x09, // W
	DRV_CONF		= 0x0A, // W
	GLOBALSCALER	= 0x0B, // W
	OFFSET_READ		= 0x0C, // R
	// VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0X10…0X1F)
	IHOLD_IRUN		= 0x10, // W
	TPOWERDOWN		= 0x11, // W
	TSTEP			= 0x12, // R
	TPWMTHRS		= 0x13, // W
	TCOOLTHRS		= 0x14, // W
	THIGH			= 0x15, // W
	// RAMP GENERATOR MOTION CONTROL REGISTER SET (0X20…0X2D)
	RAMPMODE		= 0x20, // RW
	XACTUAL			= 0x21, // RW
	VACTUAL			= 0x22, // R
	VSTART			= 0x23, // W
	A1				= 0x24, // W
	V1				= 0x25, // W
	AMAX			= 0x26, // W
	VMAX			= 0x27, // W
	DMAX			= 0x28, // W
	D1				= 0x2A, // W
	VSTOP			= 0x2B, // W
	TZEROWAIT		= 0x2C, // W
	XTARGET			= 0x2D, // RW
	// RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0X30…0X36)
	VDCMIN			= 0x33, // W
	SW_MODE			= 0x34, // RW
	RAMP_STAT		= 0x35, // R + WC
	XLATCH			= 0x36, // R
	// ENCODER REGISTER SET (0X38…0X3C)
	ENCMODE			= 0x38, // RW
	X_ENC			= 0x39, // RW
	ENC_CONST		= 0x3A, // W
	ENC_STATUS		= 0x3B, // R + WC
	ENC_LATCH		= 0x3C, // R
	ENC_DEVIATION	= 0x3D, // W
	// DRIVER REGISTER SET (0X6C…0X7F)
	CHOPCONF		= 0x6C, // RW
	COOLCONF		= 0x6D, // W
	DCCTRL			= 0x6E, // W
	DRV_STATUS		= 0x6F, // R
	PWMCONF			= 0x70, // W
	PWM_SCALE		= 0x71, // R
	PWM_AUTO		= 0x72, // R
	LOST_STEPS		= 0x73, // R
	REG_MAX			= 0x74
}TMC5160_reg_addresses;

typedef enum {
    READ,
    WRITE,
    READ_WRITE,
    READ_WRITE_CLEAR
} reg_access_t;

// Struct for sparse lookup
typedef struct {
	TMC5160_reg_addresses addr;       // Register address
    reg_access_t access; // Access type
} RegAccess_t;

// Sparse array of defined registers (47 entries, ~94 bytes)
static const RegAccess_t reg_access[] = {
    {GCONF,         READ_WRITE},
    {GSTAT,         READ_WRITE_CLEAR}, // R + WC
    {IFCNT,         READ},
    {SLAVECONF,     WRITE},
    {IOIN,          READ},             // Default assumption
    {OUTPUT,        WRITE},            // UART mode specific
    {X_COMPARE,     WRITE},
    {OTP_PROG,      WRITE},
    {OTP_READ,      READ},
    {FACTORY_CONF,  READ_WRITE},
    {SHORT_CONF,    WRITE},
    {DRV_CONF,      WRITE},
    {GLOBALSCALER,  WRITE},
    {OFFSET_READ,   READ},
    {IHOLD_IRUN,    WRITE},
    {TPOWERDOWN,    WRITE},
    {TSTEP,         READ},
    {TPWMTHRS,      WRITE},
    {TCOOLTHRS,     WRITE},
    {THIGH,         WRITE},
    {RAMPMODE,      READ_WRITE},
    {XACTUAL,       READ_WRITE},
    {VACTUAL,       READ},
    {VSTART,        WRITE},
    {A1,            WRITE},
    {V1,            WRITE},
    {AMAX,          WRITE},
    {VMAX,          WRITE},
    {DMAX,          WRITE},
    {D1,            WRITE},
    {VSTOP,         WRITE},
    {TZEROWAIT,     WRITE},
    {XTARGET,       READ_WRITE},
    {VDCMIN,        WRITE},
    {SW_MODE,       READ_WRITE},
    {RAMP_STAT,     READ_WRITE_CLEAR}, // R + WC
    {XLATCH,        READ},
    {ENCMODE,       READ_WRITE},
    {X_ENC,         READ_WRITE},
    {ENC_CONST,     WRITE},
    {ENC_STATUS,    READ_WRITE_CLEAR}, // R + WC
    {ENC_LATCH,     READ},
    {ENC_DEVIATION, WRITE},
    {CHOPCONF,      READ_WRITE},
    {COOLCONF,      WRITE},
    {DCCTRL,        WRITE},
    {DRV_STATUS,    READ},
    {PWMCONF,       WRITE},
    {PWM_SCALE,     READ},
    {PWM_AUTO,      READ},
    {LOST_STEPS,    READ}
};



#ifdef __cplusplus
}
#endif

#endif /*  __TMC5160_REG_H */
