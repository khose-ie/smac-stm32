

#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32-device-queue.h>

smacRetCode_t smac_mcu_initialize(void)
{
    stm32_device_queue_initialize();
    stm32_device_event_queue_initialize();
    return SMAC_RET_OK;
}
