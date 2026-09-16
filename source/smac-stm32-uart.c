#include <smac-mcu.h>
#include <stm32.h>

smacUart_t smac_uart_create(void* handle)
{
    return (smacUart_t)stm32_device_queue_allocate(handle);
}

void smac_uart_drop(smacUart_t uart)
{
    stm32_device_queue_free((stm32Device_t*)uart);
    stm32_device_event_queue_free((stm32Device_t*)uart);
}

smacRetCode_t smac_uart_set_event(smacUart_t uart, smacUartEvent_t* event,
                                  smacMcuEventData_t event_data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)uart, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

void smac_uart_clean_event(smacUart_t uart)
{
    stm32_device_event_queue_free((stm32Device_t*)uart);
}

smacRetCode_t smac_uart_transmit(smacUart_t uart, const uint8_t* data, uint32_t size,
                                 uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_UART_Transmit(device->handle, data, size, timeout))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_receive(smacUart_t uart, uint8_t* data, uint32_t size,
                                uint32_t* received_size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_UARTEx_ReceiveToIdle(device->handle, data, size,
                                                          (uint16_t*)received_size, timeout))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_receive_size(smacUart_t uart, uint8_t* data, uint32_t size,
                                     uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)uart;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_UART_Receive(device->handle, data, size, timeout))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_async_transmit(smacUart_t uart, const uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    return (device != NULL) && (handle != NULL)
               ? stm32_cast_code(handle->hdmatx == NULL ? HAL_UART_Transmit_IT(handle, data, size)
                                                        : HAL_UART_Transmit_DMA(handle, data, size))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_async_receive(smacUart_t uart, uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    return (device != NULL) && (handle != NULL)
               ? stm32_cast_code(handle->hdmarx == NULL
                                     ? HAL_UARTEx_ReceiveToIdle_IT(handle, data, size)
                                     : HAL_UARTEx_ReceiveToIdle_DMA(handle, data, size))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_async_receive_size(smacUart_t uart, uint8_t* data, uint32_t size)
{
    stm32Device_t* device      = (stm32Device_t*)uart;
    UART_HandleTypeDef* handle = (UART_HandleTypeDef*)device->handle;

    return (device != NULL) && (handle != NULL)
               ? stm32_cast_code(handle->hdmarx == NULL ? HAL_UART_Receive_IT(handle, data, size)
                                                        : HAL_UART_Receive_DMA(handle, data, size))
               : SMAC_RET_NULL_REF;
}

smacRetCode_t smac_uart_async_abort(smacUart_t uart)
{
    stm32Device_t* device = (stm32Device_t*)uart;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_UART_Abort(device->handle))
               : SMAC_RET_NULL_REF;
}

/// ===============================================================================================
/// @name UART Callback Implementations
/// @brief Implementation of UART callback functions for handling various UART events.
/// ===============================================================================================

/// @brief  Tx Transfer completed callbacks.
/// @param  huart  Pointer to a UART_HandleTypeDef structure that contains
///                the configuration information for the specified UART module.
/// @retval None
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if ((event != NULL) && (event->event != NULL) && (event->event->uart.tx_complete != NULL))
    {
        event->event->uart.tx_complete((smacUart_t)event->device,
                                       (smacMcuEventData_t)event->event_data);
    }
}

/// @brief  Tx Half Transfer completed callbacks.
/// @param  huart  Pointer to a UART_HandleTypeDef structure that contains
///                the configuration information for the specified UART module.
/// @retval None
// void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef* huart) {}

/// @brief  Rx Transfer completed callbacks.
/// @param  huart  Pointer to a UART_HandleTypeDef structure that contains
///                the configuration information for the specified UART module.
/// @retval None
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if ((event != NULL) && (event->event != NULL) && (event->event->uart.rx_size_complete != NULL))
    {
        event->event->uart.rx_size_complete((smacUart_t)event->device,
                                            (smacMcuEventData_t)event->event_data);
    }
}

/// @brief  Rx Half Transfer completed callbacks.
/// @param  huart  Pointer to a UART_HandleTypeDef structure that contains
///                the configuration information for the specified UART module.
/// @retval None
// void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart) {}

/// @brief  UART error callbacks.
/// @param  huart  Pointer to a UART_HandleTypeDef structure that contains
///                the configuration information for the specified UART module.
/// @retval None
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if ((event != NULL) && (event->event != NULL) && (event->event->uart.error != NULL))
    {
        event->event->uart.error((smacUart_t)event->device, (smacMcuEventData_t)event->event_data,
                                 huart->ErrorCode);
    }
}

/// @brief  UART Abort Complete callback.
/// @param  huart UART handle.
/// @retval None
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef* huart)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if ((event != NULL) && (event->event != NULL) && (event->event->uart.abort_complete != NULL))
    {
        event->event->uart.abort_complete((smacUart_t)event->device,
                                          (smacMcuEventData_t)event->event_data);
    }
}

/// @brief  UART Abort Complete callback.
/// @param  huart UART handle.
/// @retval None
// void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef* huart) {}

/// @brief  UART Abort Receive Complete callback.
/// @param  huart UART handle.
/// @retval None
// void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* huart) {}

/// @brief  Reception Event Callback (Rx event notification called after use of advanced reception
/// service).
/// @param  huart UART handle
/// @param  Size  Number of data available in application reception buffer (indicates a position in
///               reception buffer until which, data are available)
/// @retval None
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)huart);

    if ((event != NULL) && (event->event != NULL) && (event->event->uart.rx_complete != NULL))
    {
        event->event->uart.rx_complete((smacUart_t)event->device,
                                       (smacMcuEventData_t)event->event_data, Size);
    }
}
