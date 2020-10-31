#ifndef CXD2837_H
#define CXD2837_H

#define CXD2837_I2C_ADDR	0xd8
#define CXD2837_CHECK_ADDR	0xfd
#define CXD2837ER_CHIP_ID	0xb1


#define CXD2841ER_USE_GATECTRL	1	/* bit 0 */
#define CXD2841ER_AUTO_IFHZ		2	/* bit 1 */
#define CXD2841ER_TS_SERIAL		4	/* bit 2 */
#define CXD2841ER_ASCOT			8	/* bit 3 */
#define CXD2841ER_EARLY_TUNE	16	/* bit 4 */
#define CXD2841ER_NO_WAIT_LOCK	32	/* bit 5 */
#define CXD2841ER_NO_AGCNEG		64	/* bit 6 */
#define CXD2841ER_TSBITS		128	/* bit 7 */

enum cxd2841er_xtal {
	SONY_XTAL_20500, /* 20.5 MHz */
	SONY_XTAL_24000, /* 24 MHz */
	SONY_XTAL_41000 /* 41 MHz */
};

enum cxd2841er_state {
	STATE_SHUTDOWN = 0,
	STATE_SLEEP_TC,
	STATE_ACTIVE_TC
};

#define I2C_SLVX			0
#define I2C_SLVT			1

struct cxd2841er_priv {
	uint8_t							i2c_addr_slvt;
	uint8_t							i2c_addr_slvx;
	enum cxd2841er_state			state;
	uint8_t							system;
	enum cxd2841er_xtal				xtal;
	uint32_t						flags;
	void 							*rtl_dev;
};

int cxd2837_init(struct cxd2841er_priv *priv);
int cxd2837_exit(struct cxd2841er_priv *priv);
int cxd2837_read_signal_strength(struct cxd2841er_priv *priv);

#endif
