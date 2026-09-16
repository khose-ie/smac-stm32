#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32.h>

#define io_state_stm32_to_smac(x) ((x) == GPIO_PIN_SET ? SMAC_IO_SET : SMAC_IO_RST)

#define io_state_smac_to_stm32(x) ((x) == SMAC_IO_SET ? GPIO_PIN_SET : GPIO_PIN_RESET)

/// @brief Create an IO instance within the MCU abstraction layer.
/// @details This function creates an IO instance within the MCU abstraction layer, associating it
/// with the provided handle and pin.
smacIo_t smac_io_create(void* handle, uint32_t pin)
{
    return (smacIo_t)(stm32_device_queue_allocate_with_addition(handle, pin));
}

/// @brief Drop an IO instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified IO instance.
void smac_io_drop(smacIo_t io)
{
    stm32_device_queue_free((stm32Device_t*)io);
    stm32_device_event_queue_free((stm32Device_t*)io);
}

/// @brief Set IO event callbacks for the specified IO instance.
smacRetCode_t smac_io_set_event(smacIo_t io, smacIoEvent_t* event, smacMcuEventData_t data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)io, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean IO event callbacks for the specified IO instance.
void smac_io_clean_event(smacIo_t io)
{
    stm32_device_event_queue_free((stm32Device_t*)io);
}

/// @brief Get the current state of the specified IO instance.
/// @details This function retrieves the current state of the IO instance within the MCU abstraction
/// layer.
smacIoState smac_io_state(smacIo_t io)
{
    stm32Device_t* device = (stm32Device_t*)io;
    return (device != NULL) && (device->handle != NULL)
               ? io_state_stm32_to_smac(
                     HAL_GPIO_ReadPin((GPIO_TypeDef*)device->handle, device->addition))
               : SMAC_IO_RST;
}

/// @brief Set the state of the specified IO instance.
/// @details This function sets the state of the IO instance within the MCU abstraction layer.
smacRetCode_t smac_io_set_state(smacIo_t io, smacIoState state)
{
    stm32Device_t* device = (stm32Device_t*)io;

    if ((device == NULL) || (device->handle == NULL))
    {
        return SMAC_RET_NULL_REF;
    }

    HAL_GPIO_WritePin((GPIO_TypeDef*)device->handle, device->addition,
                      io_state_smac_to_stm32(state));
    return SMAC_RET_OK;
}

/// @brief Reverse the state of the specified IO instance.
/// @details This function toggles the state of the IO instance within the MCU abstraction layer.
smacRetCode_t smac_io_reverse_state(smacIo_t io)
{
    stm32Device_t* device = (stm32Device_t*)io;

    if ((device == NULL) || (device->handle == NULL))
    {
        return SMAC_RET_NULL_REF;
    }

    HAL_GPIO_TogglePin((GPIO_TypeDef*)device->handle, device->addition);
    return SMAC_RET_OK;
}

/// ===============================================================================================
/// @name IO Callback Implementations for old STM32 platform
/// @brief Implementation of IO callback functions for handling various IO events.
/// ===============================================================================================

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search_with_addition(NULL, GPIO_Pin);

    if ((event != NULL) && (event->event != NULL) && (event->event->io.state_change != NULL))
    {
        event->event->io.state_change(event->device, event->event_data);
    }
}

/// ===============================================================================================
/// @name IO Callback Implementations for new STM32 platform
/// @brief Implementation of IO callback functions for handling various IO events.
/// ===============================================================================================

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    HAL_GPIO_EXTI_Callback(GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    HAL_GPIO_EXTI_Callback(GPIO_Pin);
}
