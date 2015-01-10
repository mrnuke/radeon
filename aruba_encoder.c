#include <drm/drmP.h>
#include <drm/radeon_drm.h>
#include "radeon.h"

#include <stdbool.h>


#define DP_LINK_TRAINING_CTL				(0x1cc0 << 2)
#define  DP_LINK_TRAINING_EN				BIT(4)

//ucEncoderMode
#define ATOM_ENCODER_MODE_DP					0
#define ATOM_ENCODER_MODE_LVDS					1
#define ATOM_ENCODER_MODE_DVI					2
#define ATOM_ENCODER_MODE_HDMI					3
#define ATOM_ENCODER_MODE_SDVO					4
#define ATOM_ENCODER_MODE_DP_AUDIO				5
#define ATOM_ENCODER_MODE_TV					13
#define ATOM_ENCODER_MODE_CV					14
#define ATOM_ENCODER_MODE_CRT					15
#define ATOM_ENCODER_MODE_DVO					16
#define ATOM_ENCODER_MODE_DP_SST				ATOM_ENCODER_MODE_DP    // For DP1.2
#define ATOM_ENCODER_MODE_DP_MST				5                       // For DP1.2

// define ucBitPerColor:
#define PANEL_BPC_UNDEFINE					0x00
#define PANEL_6BIT_PER_COLOR					0x01
#define PANEL_8BIT_PER_COLOR					0x02
#define PANEL_10BIT_PER_COLOR					0x03
#define PANEL_12BIT_PER_COLOR					0x04
#define PANEL_16BIT_PER_COLOR					0x05

uint16_t get_uniphy_reg_offset(uint8_t huge, uint8_t tits)
{
	/*
	 * big = par[0] >> 6
	 * small = par[0] & 0x7
	 */
	static const uint16_t really[3][6] = {
		{ 0x0000, 0x0300, 0x2600, 0x2900, 0x2c00, 0x2f00 },  /* 0x00 */
		{ 0x0000, 0x0002, 0x002c, 0x002d, 0x002e, 0x002f },  /* 0x40 */
		{ 0x0000, 0x0001, 0x0007, 0x0008, 0x000b, 0x000c }}; /* 0x80 */

	return really[huge][tits];
}

static const uint8_t ucLVDS_Misc = 0x1c;
#define PANEL_MISC_DUAL                   0x01
#define PANEL_MISC_FPDI                   0x02
#define PANEL_MISC_8BIT_PER_COLOR         0x20


extern int aruba_iotrace;

static void aruba_write(struct radeon_device *rdev, uint32_t reg, uint32_t value)
{
	if (aruba_iotrace)
		printk("\tradeon_write(0x%04x, 0x%08x);\n", reg >> 2, value);
	WREG32(reg, value);
}

static uint32_t aruba_read(struct radeon_device *rdev, uint32_t reg)
{
	uint32_t val = RREG32(reg);
	if (aruba_iotrace)
		printk("\tradeon_read(0x%04x); /* %08x */\n", reg >> 2, val);
	return val;
}


static void aruba_mask(struct radeon_device *rdev, uint32_t reg, uint32_t clrbits, uint32_t setbits)
{
	uint32_t reg32 = aruba_read(rdev, reg);
	reg32 &= ~clrbits;
	reg32 |= setbits;
	aruba_write(rdev, reg, reg32);
}

void aruba_encoder_setup_dp(struct radeon_device *rdev, uint8_t id,
			 uint16_t pixel_clock, uint8_t lane_num,
			 uint8_t bpc_mask, uint32_t dp_link_rate)
{
	uint8_t something;
	uint16_t regptr;
	uint32_t quot;

	regptr =  get_uniphy_reg_offset(0, id);

	//   004b: CLEAR  reg[1c83]  [XXXX]
	aruba_write(rdev, (0x1c83 + regptr) << 2, 0);
	//   0056: MOVE   reg[1c03]  [..XX]  <-  001f
	aruba_mask(rdev, (0x1c03 + regptr) << 2, 0xffff, 0x001f);

	//   0071: AND    reg[1c00]  [.X..]  <-  fe
	aruba_mask(rdev, (0x1c00 + regptr) << 2, BIT(16), 0);
	//   0076: CLEAR  reg[1cc1]  [..XX]
	aruba_mask(rdev, (0x1cc1 + regptr) << 2, 0xffff, 0);
	//   007a: COMP   cfg->ucBitPerColor  <-  00
	//   007e: JUMP_NotEqual  0085
	if (bpc_mask == PANEL_BPC_UNDEFINE)
		//   0081: MOVE   cfg->ucBitPerColor  <-  02
		bpc_mask = PANEL_8BIT_PER_COLOR;
	//   0085: SUB    cfg->ucBitPerColor  <-  01
	bpc_mask--;
	//   0089: MOVE   reg[1cc1]  [X...]  <-  cfg->ucBitPerColor
	aruba_mask(rdev, (0x1cc1 + regptr) << 2, 0xff << 24, bpc_mask << 24);
	//   008e: TEST   reg[1cc3]  [...X]  <-  01
	//   0093: JUMP_NotEqual  00e3
	if (!(aruba_read(rdev, (0x1cc3 + regptr) << 2) & BIT(0))) {
		//   0096: SET_DATA_BLOCK  ff  (this table)
		//   0098: ADD    WS_DATAPTR [..XX]  <-  01c2
		//   009d: MOVE   WS_REMIND/HI32 [...X]  <-  cfg->ucConfig
		//   00a1: AND    WS_REMIND/HI32 [XXXX]  <-  00000003
		//// cfg->ucDPLinkRate;
		//   00a8: ADD    WS_DATAPTR [..XX]  <-  WS_REMIND/HI32 [..XX]
		//   00ac: ADD    WS_DATAPTR [..XX]  <-  WS_REMIND/HI32 [..XX]
		//   00b0: MOVE   WS_REMIND/HI32 [..XX]  <-  data[0000] [..XX]
		////dp_link_rate = dp_link_rates[cfg->acConfig.ucDPLinkRate];
		//   00b5: AND    reg[1cc9]  [..X.]  <-  fe
		aruba_mask(rdev, (0x1cc9 + regptr) << 2, BIT(8), 0);
		//   00ba: MOVE   reg[1cca]  [XXXX]  <-  00008000
		aruba_write(rdev, (0x1cca + regptr) << 2, 0x00008000);
		//   00c2: CLEAR  WS_QUOT/LOW32 [XXXX]
		//   00c5: MOVE   WS_QUOT/LOW32 [..XX]  <-  cfg->usPixelClock
		//   00c9: MUL    WS_QUOT/LOW32 [XXXX]  <-  00008000
		//   00d0: DIV    WS_QUOT/LOW32 [XXXX]  <-  WS_REMIND/HI32 [XXXX]
		quot = pixel_clock * 0x00008000 / dp_link_rate;
		//   00d4: MOVE   reg[1ccb]  [XXXX]  <-  WS_QUOT/LOW32 [XXXX]
		aruba_write(rdev, (0x1ccb + regptr) << 2, quot);
		//   00d9: OR     reg[1ccc]  [X...]  <-  10
		aruba_mask(rdev, (0x1ccc + regptr) << 2, 0, BIT(28));
		//   00de: OR     reg[1c00]  [..X.]  <-  04
		aruba_mask(rdev, (0x1c00 + regptr) << 2, 0, BIT(10));
	}
	//   00e3: MOVE   param[01]  [...X]  <-  reg[1c00]  [...X]
	something = aruba_read(rdev, (0x1c00 + regptr) << 2) & 0xff;
	//   00e8: AND    param[01]  [...X]  <-  07
	something &= 7;
	//   00ec: CALL_TABLE  14  (ASIC_StaticPwrMgtStatusChange/SetUniphyInstance)
	regptr = get_uniphy_reg_offset(0, something);
	//   00ee: SHIFT_LEFT  cfg->ucLaneNum  by  04
	//   00f2: MOVE   reg[1b9c]  [.X..]  <-  cfg->ucLaneNum
	aruba_mask(rdev, (0x1b9c + regptr) << 2, 0xff << 16, lane_num << 20); // FIXME: bitmap last
	//   00f7: SET_REG_BLOCK  0000
	//   00fa: EOT
	return;
}

void aruba_encoder_setup_other(struct radeon_device *rdev, uint8_t id,
			 uint8_t mode,
			 uint8_t lane_num)
{
	uint16_t regptr;

	regptr =  get_uniphy_reg_offset(0, id);

	//   004b: CLEAR  reg[1c83]  [XXXX]
	aruba_write(rdev, (0x1c83 + regptr) << 2, 0);
	//   004f: COMP   param[01]  [...X]  <-  01
	//   0053: JUMP_Equal  010c
	//if (mode != ATOM_ENCODER_MODE_LVDS) {
	//   0056: MOVE   reg[1c03]  [..XX]  <-  001f
	aruba_mask(rdev, (0x1c03 + regptr) << 2, 0xffff, 0x001f);

	//   005c: COMP   param[01]  [...X]  <-  ATOM_ENCODER_MODE_DP
	//   0060: JUMP_Equal  0071
	//   0063: COMP   param[01]  [...X]  <-  ATOM_ENCODER_MODE_DVI
	//   0067: JUMP_Equal  00fb
	//   006a: COMP   param[01]  [...X]  <-  ATOM_ENCODER_MODE_DP_AUDIO
	//   006e: JUMP_NotEqual  0103

	switch (mode) {
		case ATOM_ENCODER_MODE_DVI:
		//   00fb: MOVE   reg[1c83]  [XXXX]  <-  00000000
		aruba_write(rdev, (0x1c83 + regptr) << 2, 0);
	default:		/* Fall through */
		//   0103: AND    reg[1c7c]  [..XX]  <-  fcef
		aruba_mask(rdev, (0x1c7c + regptr) << 2, 0x0310, 0);
		//   0109: JUMP   0145
		break;
	///} else {
	case ATOM_ENCODER_MODE_LVDS:
		//   010c: MOVE   reg[1c03]  [..XX]  <-  0063
		aruba_mask(rdev, (0x1c03 + regptr) << 2, 0xffff, 0x0063);
		//   0112: SET_DATA_BLOCK  06  (LVDS_Info)
		//   0114: MOVE   WS_REMIND/HI32 [X...]  <-  data[0028] [...X]
		//   0119: TEST   WS_REMIND/HI32 [X...]  <-  20
		//   011d: JUMP_Equal  0136
		if (ucLVDS_Misc & PANEL_MISC_8BIT_PER_COLOR) {
			//   0120: OR     reg[1c8c]  [...X]  <-  01
			aruba_mask(rdev, (0x1c8c + regptr) << 2, 0, BIT(0));
			//   0125: MOVE   WS_REMIND/HI32 [X...]  <-  data[0028] [...X]
			//   012a: TEST   WS_REMIND/HI32 [X...]  <-  02
			//   012e: JUMP_Equal  0136
			if (ucLVDS_Misc & PANEL_MISC_FPDI)
				//   0131: OR     reg[1c8c]  [...X]  <-  10
				aruba_mask(rdev, (0x1c8c + regptr) << 2, 0, BIT(4));
		}
		//   0136: TEST   WS_REMIND/HI32 [X...]  <-  01
		//   013a: JUMP_Equal  0154
		if (ucLVDS_Misc & PANEL_MISC_DUAL) {
			if (lane_num > 4) {
				DRM_DEBUG_KMS("Changing lane num for LVDS from %d to 4.\n", lane_num);
				lane_num = 4;
			}
		} else {
			//   013d: OR     reg[1c8c]  [..X.]  <-  01
			aruba_mask(rdev, (0x1c8c + regptr) << 2, 0, BIT(8));
			//   0142: JUMP   014c
			if (lane_num <= 4) {
				DRM_DEBUG_KMS("Changing lane num for LVDS from %d to 8.\n", lane_num);
				lane_num = 8;
			}
		}
	}
	//   0145: COMP   param[01]  [..X.]  <-  04
	//   0149: JUMP_BelowOrEq  0154
	if (lane_num > 4)
		//   014c: OR     reg[1c00]  [.X..]  <-  01
		aruba_mask(rdev, (0x1c00 + regptr) << 2, 0, BIT(16));
		//   0151: JUMP   0159
	else
		//   0154: AND    reg[1c00]  [.X..]  <-  fe
		aruba_mask(rdev, (0x1c00 + regptr) << 2, BIT(16), 0);
	//   0159: OR     reg[1c00]  [..X.]  <-  04
	aruba_mask(rdev, (0x1c00 + regptr) << 2, 0, BIT(11));
	//   015e: SET_REG_BLOCK  0000
	//   0161: EOT
	return;
}

// command_table  0000e742  #04  (SetClocksRatio/DIGxEncoderControl):
//
//   Size         01c8
//   Format Rev.  01
//   Param Rev.   00
//   Content Rev. 04
//   Attributes:  Work space size        00 longs
//                Parameter space size   01 longs
//                Table update indicator 0

void aruba_encoder_setup_panel_mode(struct radeon_device *rdev,
				    uint8_t encoder_id, uint8_t panel_mode)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   01b2: OR     reg[1cd5]  [...X]  <-  10
	aruba_mask(rdev, (0x1cd5 + regptr) << 2, 0, 0x10);
	//   01b7: MOVE   reg[1cde]  [...X]  <-  param[01]  [...X]
	aruba_mask(rdev, (0x1cde + regptr) << 2, 0xff, panel_mode);
}

void aruba_encoder_video_on(struct radeon_device *rdev, uint8_t encoder_id)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   0199: OR     reg[1cc9]  [..X.]  <-  01
	aruba_mask(rdev, (0x1cc9 + regptr) << 2, 0, 1 << 8);
	//   019e: DELAY_MicroSec  0a
	usleep_range(10, 12);
	//   01a0: AND    reg[1cc4]  [...X]  <-  fe
	aruba_mask(rdev, (0x1cc4 + regptr) << 2, 1, 0);
	//   01a5: DELAY_MicroSec  0a
	usleep_range(10, 12);
	//   01a7: MOVE   reg[1cc3]  [..XX]  <-  0201
	aruba_mask(rdev, (0x1cc3 + regptr) << 2, 0xffff, 0x0201);
	//   01ad: DELAY_MicroSec  c8
	usleep_range(200, 240);
}

void aruba_encoder_video_off(struct radeon_device *rdev, uint8_t encoder_id)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   0189: CLEAR  reg[1cc3]  [..XX]
	aruba_mask(rdev, (0x1cc3 + regptr) << 2, 0xffff, 0);
	//   018d: DELAY_MicroSec  c8
	usleep_range(200, 240);
	//   018f: OR     reg[1cc4]  [...X]  <-  01
	aruba_mask(rdev, (0x1cc4 + regptr) << 2, 0, 1);
	//   0194: DELAY_MicroSec  32
	usleep_range(50, 60);
}

void aruba_encoder_link_training_start(struct radeon_device *rdev,
				       uint8_t encoder_id)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   0179: AND    reg[1cc0]  [...X]  <-  ef
	aruba_mask(rdev, (0x1cc0 + regptr) << 2, BIT(4), 0);
}

void aruba_encoder_link_training_pattern(struct radeon_device *rdev,
				       uint8_t encoder_id, uint8_t pattern)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   0162: CLEAR  reg[1cd1]  [...X]
	aruba_mask(rdev, (0x1cd1 + regptr) << 2, 0xff, pattern - 1);
}

void aruba_encoder_link_training_finish(struct radeon_device *rdev,
				       uint8_t encoder_id)
{
	uint16_t regptr;
	regptr = get_uniphy_reg_offset(0, encoder_id);
	//   0181: OR     reg[1cc0]  [...X]  <-  10
	aruba_mask(rdev, (0x1cc0 + regptr) << 2, 0, BIT(4));
}
 
