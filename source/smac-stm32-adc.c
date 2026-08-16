#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stddef.h>
#include <stm32-queue.h>
#include <stm32.h>

static smacAdcEventConvertComplete on_convert_complete;
static smacAdcEventOverThreshold on_over_threshold;
static smacAdcEventError on_error;

/// @brief Set ADC event callbacks for the MCU abstraction layer.
/// @param on_convert_complete Callback for conversion complete event.
/// @param on_over_threshold Callback for over-threshold event.
/// @param on_error Callback for error event.
/// @return @ref SMAC_RET_OK if the callbacks are set successfully, otherwise an error code.
smacRetCode_t smac_mcu_set_adc_event(smacAdcEventConvertComplete on_convert_complete,
                                     smacAdcEventOverThreshold on_over_threshold,
                                     smacAdcEventError on_error)
{
    if (on_convert_complete == NULL || on_over_threshold == NULL || on_error == NULL)
    {
        return SMAC_RET_PARAM_ERR;
    }

    on_convert_complete = on_convert_complete;
    on_over_threshold   = on_over_threshold;
    on_error            = on_error;

    return SMAC_RET_OK;
}

/// @brief Create an ADC instance within the MCU abstraction layer.
/// @details This function creates an ADC instance within the MCU abstraction layer, associating it
/// with the provided handle.
/// @param handle The handle associated with the ADC instance.
/// @return The created ADC instance handle.
smacAdc_t smac_adc_create(void* handle)
{
    return (smacAdc_t)stm32_device_queue_allocate(handle, 0);
}

/// @brief Drop an ADC instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified ADC instance.
/// @param adc The ADC instance to be dropped.
void smac_adc_drop(smacAdc_t adc)
{
    stm32_device_queue_free((stm32Device*)adc);
    stm32_device_event_queue_free((stm32Device*)adc);
}

/// @brief Set ADC event callbacks for the specified ADC instance.
/// @param adc The ADC instance.
/// @param data The event data to be associated with the ADC instance.
/// @return @ref SMAC_RET_OK if the event is set successfully, otherwise an error code.
/// @note If you don't want to use the interrupt/event of the ADC instance, you could don't call
/// this function.
/// @note This function cannot enable the interrupt and also needs you to enable the interrupt in
/// MCU driver.
smacRetCode_t smac_adc_set_event(smacAdc_t adc, smacMcuEventData_t data)
{
    return stm32_device_event_queue_allocate((stm32Device*)adc, data);
}

/// @brief Clean ADC event callbacks for the specified ADC instance.
/// @param adc The ADC instance.
/// @note This function will remove all event callbacks associated with the specified ADC instance.
void smac_adc_clean_event(smacAdc_t adc)
{
    stm32_device_event_queue_free((stm32Device*)adc);
}

/// @brief Perform a conversion on the specified ADC instance.
/// @details This function performs a conversion on the specified ADC instance with the provided
/// data.
/// @param adc The ADC instance.
/// @param data Pointer to the variable where the converted data will be stored.
/// @param timeout The timeout for the conversion operation. @ref SMAC_MCU_WAIT_NOW for no wait,
/// @ref SMAC_MCU_WAIT_FOREVER for indefinite wait.
/// @return @ref SMAC_RET_OK if the conversion is successful, otherwise an error code.
smacRetCode_t smac_adc_convert(smacAdc_t adc, uint32_t* data, uint32_t timeout)
{
    smacRetCode_t code;
    stm32Device* device = (stm32Device*)adc;

    if (device == NULL || device->handle == NULL)
    {
        return SMAC_RET_NULL_REF;
    }

    code = stm32_cast_code(HAL_ADC_Start(device->handle));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    code = stm32_cast_code(HAL_ADC_PollForConversion(device->handle, timeout));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    *data = HAL_ADC_GetValue(device->handle);
    return SMAC_RET_OK;
}

/// @brief Perform an asynchronous conversion on the specified ADC instance.
/// @details This function initiates an asynchronous conversion on the specified ADC instance.
/// @param adc The ADC instance.
/// @return @ref SMAC_RET_OK if the asynchronous conversion is initiated successfully, otherwise an
/// error code.
smacRetCode_t smac_adc_async_convert(smacAdc_t adc)
{
    stm32Device* device = (stm32Device*)adc;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_ADC_Start_IT(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Start an asynchronous conversion on the specified ADC instance.
/// @details This function starts an asynchronous conversion on the specified ADC instance with the
/// provided data buffer and size.
/// @param adc The ADC instance.
/// @param data The buffer to store the conversion data.
/// @param size The size of the data buffer.
/// @return @ref SMAC_RET_OK if the asynchronous conversion is started successfully, otherwise an
/// error code.
smacRetCode_t smac_adc_async_conversion_start(smacAdc_t adc, uint32_t* data, uint32_t size)
{
    stm32Device* device = (stm32Device*)adc;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_ADC_Start_DMA(device->handle, data, size))
               : SMAC_RET_NULL_REF;
}

/// @brief Stop an asynchronous conversion on the specified ADC instance.
/// @details This function stops an ongoing asynchronous conversion on the specified ADC instance.
/// @param adc The ADC instance.
/// @return @ref SMAC_RET_OK if the asynchronous conversion is stopped successfully, otherwise an
/// error code.
smacRetCode_t smac_adc_async_conversion_stop(smacAdc_t adc)
{
    stm32Device* device = (stm32Device*)adc;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_ADC_Stop_DMA(device->handle))
               : SMAC_RET_NULL_REF;
}

/// ===============================================================================================
/// @name ADC Callback Implementations
/// @brief Implementation of ADC callback functions for handling various ADC events.
/// ===============================================================================================

/// @brief  Regular conversion complete callback in non blocking mode
/// @param  hadc pointer to a ADC_HandleTypeDef structure that contains
///         the configuration information for the specified ADC.
/// @retval None
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (on_convert_complete != NULL)
    {
        stm32DeviceEvent* event = stm32_device_event_queue_search((stm32DeviceHandle)hadc);

        if (event != NULL)
        {
            on_convert_complete((smacAdc_t)event->device, (smacMcuEventData_t)event->event_data,
                                HAL_ADC_GetValue((ADC_HandleTypeDef*)event->device->handle));
        }
    }
}

// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {}

/// @brief  Analog watchdog callback in non blocking mode
/// @param  hadc pointer to a ADC_HandleTypeDef structure that contains
///         the configuration information for the specified ADC.
/// @retval None
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    if (on_over_threshold != NULL)
    {
        stm32DeviceEvent* event = stm32_device_event_queue_search((stm32DeviceHandle)hadc);

        if (event != NULL)
        {
            on_over_threshold((smacAdc_t)event->device, (smacMcuEventData_t)event->event_data);
        }
    }
}

/// @brief  Error ADC callback.
/// @note   In case of error due to overrun when using ADC with DMA transfer
///         (HAL ADC handle parameter "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
///         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
///         - If needed, restart a new ADC conversion using function
///           "HAL_ADC_Start_DMA()"
///           (this function is also clearing overrun flag)
/// @param  hadc pointer to a ADC_HandleTypeDef structure that contains
///         the configuration information for the specified ADC.
/// @retval None
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
    if (on_error != NULL)
    {
        stm32DeviceEvent* event = stm32_device_event_queue_search((stm32DeviceHandle)hadc);

        if (event != NULL)
        {
            on_error((smacAdc_t)event->device, (smacMcuEventData_t)event->event_data,
                     hadc->ErrorCode);
        }
    }
}
