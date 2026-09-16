#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32.h>

/// @brief Create a Timer instance within the MCU abstraction layer.
/// @details This function creates a Timer instance within the MCU abstraction layer, associating it
/// with the provided handle.
smacTim_t smac_tim_create(void* handle)
{
    return (smacTim_t)stm32_device_queue_allocate(handle, 0);
}

/// @brief Drop a Timer instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified Timer instance.
void smac_tim_drop(smacTim_t tim)
{
    stm32_device_queue_free((stm32Device_t*)tim);
    stm32_device_cache_queue_free((stm32Device_t*)tim);
}

/// @brief Set Timer event callbacks for the specified Timer instance.
smacRetCode_t smac_tim_set_event(smacTim_t tim, smacTimEvent_t* event,
                                 smacMcuEventData_t event_data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)tim, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

/// @brief Clean Timer event callbacks for the specified Timer instance.
void smac_tim_clean_event(smacTim_t tim)
{
    stm32_device_event_queue_free((stm32Device_t*)tim);
}

/// @brief Get the current count of the specified Timer instance.
/// @details This function retrieves the current count value of the specified Timer instance within
/// the MCU abstraction layer.
uint32_t smac_tim_count(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef*)device->handle);
}

/// @brief Activate the specified Timer instance.
/// @details This function starts the Timer instance within the MCU abstraction layer.
smacRetCode_t smac_tim_activate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Start(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Deactivate the specified Timer instance.
/// @details This function stops the Timer instance within the MCU abstraction layer.
smacRetCode_t smac_tim_deactivate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Stop(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously activate the specified Timer instance.
/// @details This function initiates an asynchronous activation of the Timer instance within the MCU
/// abstraction layer.
smacRetCode_t smac_tim_async_activate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Start_IT(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously deactivate the specified Timer instance.
/// @details This function initiates an asynchronous deactivation of the Timer instance within the
/// MCU abstraction layer.
smacRetCode_t smac_tim_async_deactivate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Stop_IT(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously activate the specified Timer instance with associated data.
/// @details This function initiates an asynchronous activation of the Timer instance within the MCU
/// abstraction layer, with the provided data.
smacRetCode_t smac_tim_async_activate_data(smacTim_t tim, const uint32_t* data, uint16_t size)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Start_DMA(device->handle, data, size))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously deactivate the specified Timer instance with associated data.
/// @details This function initiates an asynchronous deactivation of the Timer instance within the
/// MCU abstraction layer.
smacRetCode_t smac_tim_async_deactivate_data(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_TIM_Base_Stop_DMA(device->handle))
               : SMAC_RET_NULL_REF;
}
