#include <assert.h>
#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32.h>

/// @brief Create a Timer instance within the MCU abstraction layer.
/// @details This function creates a Timer instance within the MCU abstraction layer, associating it
/// with the provided handle.
smacTim_t smac_tim_create(void* handle)
{
    assert(handle != NULL);
    return (smacTim_t)stm32_device_queue_allocate(handle);
}

/// @brief Drop a Timer instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified Timer instance.
void smac_tim_drop(smacTim_t tim)
{
    assert(tim != NULL);

    stm32_device_queue_free((stm32Device_t*)tim);
    stm32_device_cache_queue_free((stm32Device_t*)tim);
}

/// @brief Set Timer event callbacks for the specified Timer instance.
smacRetCode_t smac_tim_set_event(smacTim_t tim, smacTimEvent_t* event,
                                 smacMcuEventData_t event_data)
{
    assert(tim != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)tim, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

/// @brief Clean Timer event callbacks for the specified Timer instance.
void smac_tim_clean_event(smacTim_t tim)
{
    assert(tim != NULL);
    stm32_device_event_queue_free((stm32Device_t*)tim);
}

/// @brief Get the current count of the specified Timer instance.
/// @details This function retrieves the current count value of the specified Timer instance within
/// the MCU abstraction layer.
uint32_t smac_tim_count(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef*)device->handle);
}

/// @brief Activate the specified Timer instance.
/// @details This function starts the Timer instance within the MCU abstraction layer.
smacRetCode_t smac_tim_activate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_TIM_Base_Start(device->handle));
}

/// @brief Deactivate the specified Timer instance.
/// @details This function stops the Timer instance within the MCU abstraction layer.
smacRetCode_t smac_tim_deactivate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_TIM_Base_Stop(device->handle));
}

/// @brief Asynchronously activate the specified Timer instance.
/// @details This function initiates an asynchronous activation of the Timer instance within the MCU
/// abstraction layer.
smacRetCode_t smac_tim_async_activate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_TIM_Base_Start_IT(device->handle));
}

/// @brief Asynchronously deactivate the specified Timer instance.
/// @details This function initiates an asynchronous deactivation of the Timer instance within the
/// MCU abstraction layer.
smacRetCode_t smac_tim_async_deactivate(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_TIM_Base_Stop_IT(device->handle));
}

/// @brief Asynchronously activate the specified Timer instance with associated data.
/// @details This function initiates an asynchronous activation of the Timer instance within the MCU
/// abstraction layer, with the provided data.
smacRetCode_t smac_tim_async_activate_data(smacTim_t tim, const uint32_t* data, uint16_t size)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(HAL_TIM_Base_Start_DMA(device->handle, data, size));
}

/// @brief Asynchronously deactivate the specified Timer instance with associated data.
/// @details This function initiates an asynchronous deactivation of the Timer instance within the
/// MCU abstraction layer.
smacRetCode_t smac_tim_async_deactivate_data(smacTim_t tim)
{
    stm32Device_t* device = (stm32Device_t*)tim;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_TIM_Base_Stop_DMA(device->handle));
}

/// ===============================================================================================
/// @name TIM Callback Implementations
/// @brief Implementation of TIM callback functions for handling various TIM events.
/// ===============================================================================================

void HAL_TIM_PeriodElapsedCallback_Custom(TIM_HandleTypeDef* htim)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)htim);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->tim.timeout != NULL)
        {
            event->event->tim.timeout(event->device, event->event_data);
        }
    }
}

__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    HAL_TIM_PeriodElapsedCallback_Custom(htim);
}

// void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef* htim) {}

// void HAL_TIM_TriggerCallback(TIM_HandleTypeDef* htim) {}

// void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef* htim) {}
