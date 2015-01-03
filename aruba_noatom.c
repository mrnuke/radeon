#include <drm/drmP.h>
#include <drm/radeon_drm.h>
#include "radeon.h"

#include "atom.h"
#include "atom-bits.h"
#include <drm/drm_dp_helper.h>

/* NOTE: These names are based on what _we_think_ these registers do */
#define REG_DP_AUX_CTL			(0x1880 << 2)
#define REG_DP_AUX_FIFO_CTL		(0x1881 << 2)
#define REG_DP_AUX_CTL2			(0x1883 << 2)
#define REG_DP_AUX_STATUS		(0x1884 << 2)
#define REG_DP_AUX_FIFO			(0x1886 << 2)
#define REG_DP_PAD_EN_CTL		(0x194c << 2)

/*
 * This tells us the offset of each AUX control block from the fisrt block.
 * It is given in number of 32-bit registers, so it needs to be multiplied by
 * 4 before converting it to an address offset.
 */
static const uint16_t aux_ch_reg[] = {0, 0x14, 0x28, 0x40, 0x54, 0x68};

static int aruba_iotrace = 0;

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

static void aux_channel_fifo_write_start(struct radeon_device *rdev, uint8_t channel, uint8_t data)
{
	uint32_t reg;
	reg = REG_DP_AUX_FIFO + (aux_ch_reg[channel] << 2);
	aruba_write(rdev, reg, (data << 8) | (1 << 31));
}

static void aux_channel_fifo_write(struct radeon_device *rdev, uint8_t channel, uint8_t data)
{
	uint32_t reg;
	reg = REG_DP_AUX_FIFO + (aux_ch_reg[channel] << 2);
	aruba_write(rdev, reg, data << 8);
}

static uint8_t aux_channel_fifo_read(struct radeon_device *rdev, uint8_t channel)
{
	uint32_t reg;
	reg = REG_DP_AUX_FIFO + (aux_ch_reg[channel] << 2);
	return (aruba_read(rdev, reg) >> 8) & 0xff;
}

static int do_aux_tran(struct radeon_device *rdev,
		       uint8_t channel_id, uint8_t delay, uint8_t hpd_id,
		       const uint8_t *msg, uint8_t send_bytes,
		       uint8_t *recv, uint8_t recv_size, uint8_t *reply)
{
	int i, wait;
	uint32_t regptr;
	uint8_t num_bytes_received;

	regptr = channel_id * 0x04 << 2;
	aruba_mask(rdev, REG_DP_PAD_EN_CTL + regptr, 0, 0x01 << 16);
	aruba_mask(rdev, REG_DP_PAD_EN_CTL + regptr, 0xffff, 0);

	regptr = aux_ch_reg[channel_id] << 2;

	aruba_mask(rdev, REG_DP_AUX_CTL + regptr, 0x7 << 20, (hpd_id & 0x7) << 20);
	aruba_mask(rdev, REG_DP_AUX_CTL + regptr, 0, 0x0101);

	/* Tell controller how many bytes we want to send */
	aruba_mask(rdev, REG_DP_AUX_FIFO_CTL + regptr, 0xff << 16, send_bytes << 16);
	aruba_mask(rdev, REG_DP_AUX_FIFO_CTL + regptr, 0xff, (delay << 4) & 0xff);

	/* Caller should make sure message is 16 bytes or less */
	aux_channel_fifo_write_start(rdev, channel_id, *msg++);
	while (--send_bytes)
		aux_channel_fifo_write(rdev, channel_id, *msg++);

	aruba_mask(rdev, REG_DP_AUX_CTL2 + regptr, 0, 0x02);
	aruba_mask(rdev, REG_DP_AUX_FIFO_CTL + regptr, 0, 0x01);

	wait = (delay * 10 + 0x32);
	while ((aruba_read(rdev, REG_DP_AUX_STATUS + regptr) & 0xff) != 0x01) {
		usleep_range(10, 10);
		if (--wait != 0)
			continue;
		/* Timed out */
		return -ETIMEDOUT;
	}

	if (aruba_read(rdev, REG_DP_AUX_STATUS + regptr) == 0x00ff8ff0)
		return -EBUSY;

	aruba_write(rdev, REG_DP_AUX_FIFO + regptr, 0x80000001);

	*reply = aux_channel_fifo_read(rdev, channel_id);
	num_bytes_received = (aruba_read(rdev, REG_DP_AUX_STATUS + regptr) >> 24) & 0x1f;

	if (num_bytes_received == 0)
		return -EIO;

	/* First byte is the reply field. The others are the data bytes */
	if (--num_bytes_received == 0)
		return num_bytes_received;

	for (i = 0; i < min(num_bytes_received, recv_size); i++)
		recv[i] = aux_channel_fifo_read(rdev, channel_id);

	/* This extra data is lost forever. TODO: Signal an error? */
	if (i < num_bytes_received)
		DRM_DEBUG_KMS("Got %d extra bytez", num_bytes_received - i);
	for (; i < num_bytes_received; i++)
		aux_channel_fifo_read(rdev, channel_id);

	return num_bytes_received;
}

static int radeon_process_aux_ch_wrapper(struct radeon_i2c_chan *chan,
					 uint8_t *send, int send_bytes,
					 uint8_t *recv, int recv_size,
					 uint8_t delay, uint8_t *reply)
{
	struct drm_device *dev = chan->dev;
	struct radeon_device *rdev = dev->dev_private;

	int ret;
	uint8_t hpd_id, ch_id;

	/* TODO: Actually, there are only five HPD pins and five AUX channels */
	hpd_id = chan->rec.hpd & 0x7;
	ch_id = chan->rec.i2c_id & 0x7;

	ret = do_aux_tran(rdev, ch_id, delay / 10, hpd_id, send, send_bytes,
			  recv, recv_size, reply);
	/* The hardware gives us a full byte, but we need bits [4:7] */
	*reply >>= 4;

	if (ret == -EIO) {
		DRM_DEBUG_KMS("AUX channel error\n");
		return -EIO;
	}

	if (ret == -ETIMEDOUT) {
		DRM_DEBUG_KMS("AUX channel timeout\n");
		return -ETIMEDOUT;
	}

	if (ret == -EBUSY) {
		DRM_DEBUG_KMS("AUX channel busy\n");
		return -EBUSY;
	}

	return ret;
}

#define BARE_ADDRESS_SIZE 3
#define HEADER_SIZE (BARE_ADDRESS_SIZE + 1)

ssize_t aruba_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct radeon_i2c_chan *chan =
		container_of(aux, struct radeon_i2c_chan, aux);
	struct drm_device *dev = chan->dev;
	struct radeon_device *rdev = dev->dev_private;

	int ret;
	u8 tx_buf[20];
	size_t tx_size;
	u8 ack, delay = 0;
	
	if (WARN_ON(msg->size > 16))
		return -E2BIG;
	
	mutex_lock(&rdev->mode_info.atom_context->mutex);
	mutex_lock(&chan->mutex);

	tx_buf[0] = (msg->request << 4) | ((msg->address >> 16) & 0xf);
	tx_buf[1] = msg->address >> 8;
	tx_buf[2] = msg->address & 0xff;
	tx_buf[3] = msg->size ? (msg->size - 1) : 0;

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
		tx_size = (msg->size) ? (HEADER_SIZE + msg->size) : BARE_ADDRESS_SIZE;

		memcpy(tx_buf + HEADER_SIZE, msg->buffer, msg->size);
		ret = radeon_process_aux_ch_wrapper(chan,
					    tx_buf, tx_size, NULL, 0, delay, &ack);
		if (ret >= 0)
			/* Return payload size. */
			ret = msg->size;
		break;
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		tx_size = (msg->size) ? HEADER_SIZE : BARE_ADDRESS_SIZE;
		ret = radeon_process_aux_ch_wrapper(chan,
					    tx_buf, tx_size, msg->buffer, msg->size, delay, &ack);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret >= 0)
		msg->reply = ack;

	mutex_unlock(&chan->mutex);
	mutex_unlock(&rdev->mode_info.atom_context->mutex);
	
	return ret;
}

#define DIV_UP(x, y)     (((x) + (y) - 1) / (y))
#define PWM_MAGIC_CONST 	25000000	// 0x017d7840
#define PWM_RESOLUTION		4095		// 0xfff

static void aruba_brightness_control(struct radeon_device *rdev,
			      uint16_t bl_pwm_freq_hz, uint8_t bl_level)
{
	uint32_t ticks_per_cycle, cycle_div, cycle_ctr, high_ctr;

	if (bl_pwm_freq_hz == 0)
		return;
	if (((aruba_read(rdev, 0x019c << 2) >> 16) & 0xff) != 0x01)
		aruba_mask(rdev, 0x1921, 0, 0x01);

	ticks_per_cycle = bl_pwm_freq_hz * PWM_RESOLUTION;
	cycle_div = DIV_UP(PWM_MAGIC_CONST, ticks_per_cycle);

	aruba_mask(rdev, 0x191b << 2, 0xffff << 16, cycle_div << 16);

	cycle_ctr = PWM_MAGIC_CONST / (cycle_div * bl_pwm_freq_hz);

	aruba_mask(rdev, 0x1920 << 2, 0xff << 16, 0x0c << 16);
	aruba_mask(rdev, 0x1920 << 2, 0xffff, cycle_ctr & 0xffff);

	high_ctr = cycle_ctr * (bl_level + 1);
	high_ctr >>= 4;
	aruba_mask(rdev, 0x191e << 2, 0xffff, high_ctr & 0xffff);
	aruba_mask(rdev, 0x191e << 2, 0, 0xc0 << 24);

	aruba_mask(rdev, 0x1921 << 2, (1 << 0), 0);
	while (((aruba_read(rdev, 0x1921 << 2) >> 8) & 0xff) == 0x01);
}

static void aruba_lcd_blon(struct radeon_device *rdev)
{
	if ((aruba_read(rdev, 0x1919 << 2) >> 24) != 0x01)
		return;

	msleep(0x32);

	if (((aruba_read(rdev, 0x191d << 2) >> 8) & 0xff) == 0) {
		aruba_mask(rdev, 0x191d << 2, 0, 0x03 << 24);
	} else {
		aruba_mask(rdev, 0x1940 << 2, 0, (1 << 16));
		aruba_mask(rdev, 0x1919 << 2, 0, 0x03 << 24);
		msleep(0x04);
		aruba_mask(rdev, 0x1940 << 2, (1 << 16), 0);
	}
}

static void aruba_lcd_bloff(struct radeon_device *rdev, uint8_t max_pclk)
{
	if ((aruba_read(rdev, 0x1919 << 2) >> 24) == 0x01)
		return;

	if (max_pclk != 0)
		return;

	if (((aruba_read(rdev, 0x191d << 2) >> 8) & 0xff) != 0) {
		aruba_mask(rdev, 0x1940 << 2, 0, 0x01 << 16);
		msleep(0x04);
		aruba_mask(rdev, 0x1919 << 2, (1 << 24), (1 << 25));
		aruba_mask(rdev, 0x1940 << 2, (1 << 16), 0);
	} else {
		aruba_mask(rdev, 0x1919 << 2, (1 << 24), (1 << 25));
	}
	msleep(1);
}

u8 aruba_get_backlight_level(struct radeon_encoder *radeon_encoder)
{
	struct radeon_encoder_atom_dig *dig = radeon_encoder->enc_priv;
	
	DRM_DEBUG_KMS("ARUBA: %s", __func__);

	if (dig)
		return dig->backlight_level;
	else
		return 0;
}

void aruba_set_backlight_level(struct radeon_encoder *radeon_encoder, u8 level)
{
	struct drm_device *dev = radeon_encoder->base.dev;
	struct radeon_device *rdev = dev->dev_private;
	struct radeon_encoder_atom_dig *dig = radeon_encoder->enc_priv;;

	DRM_DEBUG_KMS("ARUBA: %s", __func__);

	if (level == 0) {
		aruba_lcd_bloff(rdev, 0);
	} else {
		aruba_brightness_control(rdev, 200, level);
		aruba_lcd_blon(rdev);
	}
	
	if (dig)
		dig->backlight_level = level;
}
