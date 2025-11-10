extern void *g_IOMHandle;
int sgp40setup(float sampling_interval);
uint16_t sgp40_get_voc_sraw(uint16_t *raw, uint32_t *index);
uint16_t sgp40_get_voc_sraw_lowpower(uint16_t *raw, uint32_t *index);
int sgp40PinEnable();
int sgp40PinDisable();
