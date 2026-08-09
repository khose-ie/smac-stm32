#include <smac.h>
#include <smac-stm32.h>

smacRetCode_t stm32_cast_code(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return SMAC_RET_OK;
        case HAL_ERROR:
            return SMAC_RET_LOW_LEVEL_FAILURE;
        case HAL_BUSY:
            return SMAC_RET_BUSY;
        case HAL_TIMEOUT:
            return SMAC_RET_TIMEOUT;
        default:
            return SMAC_RET_UNKNOWN;
    }
}
