#include <assert.h>
#include <smac/middleware/smac-mcu.h>
#include <stm32.h>

smacUart_t smac_uart_create(void* handle)
{
    assert(handle != NULL);
    return (smacUart_t)stm32_device_queue_allocate(handle);
}

void smac_uart_drop(smacUart_t uart)
{
    assert(uart != NULL);

    stm32_device_queue_free((stm32Device_t*)uart);
    stm32_device_event_queue_free((stm32Device_t*)uart);
}

smacRetCode_t smac_uart_set_event(smacUart_t uart, smacUartEvent_t* event,
                                  smacMcuEventData_t event_data)
{
    assert(uart != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)uart, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

void smac_uart_clean_event(smacUart_t uart)
{
    assert(uart != NULL);
    stm32_device_event_queue_free((stm32Device_t*)uart);
}

smacRetCode_t smac_uart_transmit(smacUart_t uart, const uint8_t* data, uint32_t size,
                                 uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(HAL_UART_Transmit(device->handle, data, size, timeout));
}

smacRetCode_t smac_uart_receive(smacUart_t uart, uint8_t* data, uint32_t size,
                                uint32_t* received_size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(
        HAL_UARTEx_ReceiveToIdle(device->handle, data, size, (uint16_t*)received_size, timeout));
}

smacRetCode_t smac_uart_receive_size(smacUart_t uart, uint8_t* data, uint32_t size,
                                     uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(HAL_UART_Receive(device->handle, data, size, timeout));
}

smacRetCode_t smac_uart_async_transmit(smacUart_t uart, const uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    assert(device != NULL);
    assert(handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmatx == NULL ? HAL_UART_Transmit_IT(handle, data, size)
                                                  : HAL_UART_Transmit_DMA(handle, data, size));
}

smacRetCode_t smac_uart_async_receive(smacUart_t uart, uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    assert(device != NULL);
    assert(handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmarx == NULL
                               ? HAL_UARTEx_ReceiveToIdle_IT(handle, data, size)
                               : HAL_UARTEx_ReceiveToIdle_DMA(handle, data, size));
}

smacRetCode_t smac_uart_async_receive_size(smacUart_t uart, uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    assert(device != NULL);
    assert(handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmarx == NULL ? HAL_UART_Receive_IT(handle, data, size)
                                                  : HAL_UART_Receive_DMA(handle, data, size));
}

smacRetCode_t smac_uart_async_abort(smacUart_t uart)
{
    stm32Device_t* device = (stm32Device_t*)uart;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_UART_Abort(device->handle));
}

/// ===============================================================================================
/// @name UART Callback Implementations
/// @brief Implementation of UART callback functions for handling various UART events.
/// ===============================================================================================

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->uart.tx_complete != NULL)
        {
            event->event->uart.tx_complete((smacUart_t)event->device, event->event_data);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->uart.rx_size_complete != NULL)
        {
            event->event->uart.rx_size_complete((smacUart_t)event->device, event->event_data);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->uart.error != NULL)
        {
            event->event->uart.error((smacUart_t)event->device, event->event_data,
                                     huart->ErrorCode);
        }
    }
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->uart.abort_complete != NULL)
        {
            event->event->uart.abort_complete((smacUart_t)event->device, event->event_data);
        }
    }
}

// void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef* huart) {}

// void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* huart) {}

/// @retval None
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->uart.rx_complete != NULL)
        {
            event->event->uart.rx_complete((smacUart_t)event->device, event->event_data, Size);
        }
    }
}
