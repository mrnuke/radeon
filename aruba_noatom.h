#ifndef _ARUBA_NOATOM_H
#define _ARUBA_NOATOM_H

u8 aruba_get_backlight_level(struct radeon_encoder *radeon_encoder);
void aruba_set_backlight_level(struct radeon_encoder *radeon_encoder, u8 level);
void aruba_encoder_init(struct radeon_device *rdev, uint8_t connector_id);

#endif /* _ARUBA_NOATOM_H */
