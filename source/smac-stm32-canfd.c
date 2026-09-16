#include <assert.h>
#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32.h>
#include <string.h>

#define CAN_FD_ROLE_MASK    (0x80000000)
#define CAN_FD_ROLE_FD      (0x00000000)
#define CAN_FD_ROLE_CLASSIC (0x80000000)

#define cast_to_stm32_id_type(frame_kind)                                                          \
    ((frame_kind == SMAC_CAN_FRAME_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID)

#define cast_to_stm32_frame_type(request_kind)                                                     \
    ((request_kind == SMAC_CAN_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME)

#define cast_to_stm32_error_state(error_state)                                                     \
    ((error_state == SMAC_CAN_ERROR_ACTIVE) ? FDCAN_ESI_ACTIVE : FDCAN_ESI_PASSIVE)

#define cast_to_stm32_format(format)                                                               \
    ((format == SMAC_CAN_CLASSIC) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN)

#define cast_to_stm32_bit_rate_switch(switch_bit_rate)                                             \
    ((switch_bit_rate) ? FDCAN_BRS_ON : FDCAN_BRS_OFF)

#define cast_from_stm32_id_type(frame_kind)                                                        \
    ((frame_kind == FDCAN_STANDARD_ID) ? SMAC_CAN_FRAME_STANDARD : SMAC_CAN_FRAME_EXTENDED)

#define cast_from_stm32_frame_type(request_kind)                                                   \
    ((request_kind == FDCAN_DATA_FRAME) ? SMAC_CAN_DATA : SMAC_CAN_REMOTE)

#define cast_from_stm32_error_state(error_state)                                                   \
    ((error_state == FDCAN_ESI_ACTIVE) ? SMAC_CAN_ERROR_ACTIVE : SMAC_CAN_ERROR_PASSIVE)

#define cast_from_stm32_format(format)                                                             \
    ((format == FDCAN_CLASSIC_CAN) ? SMAC_CAN_CLASSIC : SMAC_CAN_FD)

#define cast_from_stm32_bit_rate_switch(switch_bit_rate)                                           \
    ((switch_bit_rate == FDCAN_BRS_ON) ? true : false)

#ifdef HAL_FDCAN_MODULE_ENABLED

/// @brief Create a CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_create.
smacCanFd_t smac_can_fd_create(void* handle)
{
    assert(handle != NULL);

    stm32Device_t* device = stm32_device_queue_allocate_with_addition(handle, CAN_FD_ROLE_FD);

    if (device == NULL)
    {
        return NULL;
    }
    // Initialize the cache for this device in the STM32 device cache queue.
    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        stm32_device_queue_free(device);
        return NULL;
    }

    return (smacCanFd_t)device;
}

/// @brief Release the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_drop.
void smac_can_fd_drop(smacCanFd_t canfd)
{
    assert(canfd != NULL);

    stm32_device_event_queue_free((stm32Device_t*)canfd);
    stm32_device_cache_queue_free((stm32Device_t*)canfd);
    stm32_device_queue_free((stm32Device_t*)canfd);
}

/// @brief Set an event for the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_set_event.
smacRetCode_t smac_can_fd_set_event(smacCanFd_t canfd, smacCanFdEvent_t* event,
                                    smacMcuEventData_t data)
{
    assert(canfd != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)canfd,
                                             (stm32DeviceEventHandle_t*)event, data);
}

/// @brief Clean up events associated with the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_clean_event.
void smac_can_fd_clean_event(smacCanFd_t canfd)
{
    assert(canfd != NULL);
    stm32_device_event_queue_free((stm32Device_t*)canfd);
}

/// @brief Activate a CAN FD message using the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_active.
smacRetCode_t smac_can_fd_active(smacCanFd_t canfd)
{
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_FDCAN_Start(device->handle));
}

/// @brief Deactivate a CAN FD message using the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_deactive.
smacRetCode_t smac_can_fd_deactive(smacCanFd_t canfd)
{
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_FDCAN_Stop(device->handle));
}

/// @brief Transmit a CAN FD message using the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_transmit.
smacRetCode_t smac_can_fd_transmit(smacCanFd_t canfd, const smacCanFdMessage* message,
                                   uint32_t timeout)
{
    FDCAN_TxHeaderTypeDef head;
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(message != NULL);

    memset(&head, 0, sizeof(head));

    head.Identifier          = message->head.ident;
    head.IdType              = cast_to_stm32_id_type(message->head.frame_kind);
    head.TxFrameType         = cast_to_stm32_frame_type(message->head.request_kind);
    head.DataLength          = message->head.data_length;
    head.ErrorStateIndicator = cast_to_stm32_error_state(message->head.error_state);
    head.BitRateSwitch       = cast_to_stm32_bit_rate_switch(message->head.switch_bit_rate);
    head.FDFormat            = cast_to_stm32_format(message->head.format);
    head.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    head.MessageMarker       = 0;

    smacRetCode_t code =
        stm32_cast_code(HAL_FDCAN_AddMessageToTxFifoQ(device->handle, &head, message->data));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    uint32_t buffer_idx = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(device->handle);

    uint32_t start_tick = HAL_GetTick();
    uint32_t tick       = start_tick;

    while (HAL_FDCAN_IsTxBufferMessagePending(device->handle, buffer_idx))
    {
        tick = HAL_GetTick();

        if ((tick - start_tick) >= timeout)
        {
            return SMAC_RET_TIMEOUT;
        }
    }

    return SMAC_RET_OK;
}

/// @brief Receive a CAN FD message from the specified channel using the specified CAN FD instance.
/// @param canfd The CAN FD instance.
/// @param channel The channel from which to receive the message.
/// @param message Pointer to the structure to store the received message.
/// @param timeout The timeout duration for the reception.
/// @return @ref SMAC_RET_OK if the message is successfully received, otherwise an error code.
static smacRetCode_t smac_can_fd_receive(smacCanFd_t canfd, uint32_t channel,
                                         smacCanFdMessage* message, uint32_t timeout)
{
    FDCAN_RxHeaderTypeDef head;
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(message != NULL);

    memset(&head, 0, sizeof(head));
    memset(message, 0, sizeof(*message));

    uint32_t start_tick = HAL_GetTick();

    while (HAL_FDCAN_GetRxFifoFillLevel(device->handle, channel) == 0)
    {
        if ((HAL_GetTick() - start_tick) >= timeout)
        {
            return SMAC_RET_TIMEOUT;
        }

        HAL_Delay(1);
    }

    smacRetCode_t code =
        stm32_cast_code(HAL_FDCAN_GetRxMessage(device->handle, channel, &head, message->data));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    message->head.ident           = head.Identifier;
    message->head.frame_kind      = cast_from_stm32_id_type(head.IdType);
    message->head.request_kind    = cast_from_stm32_frame_type(head.RxFrameType);
    message->head.data_length     = head.DataLength;
    message->head.error_state     = cast_from_stm32_error_state(head.ErrorStateIndicator);
    message->head.switch_bit_rate = cast_from_stm32_bit_rate_switch(head.BitRateSwitch);
    message->head.format          = cast_from_stm32_format(head.FDFormat);

    return SMAC_RET_OK;
}

/// @brief Asynchronously receive a CAN FD message from channel 0 using the specified CAN FD
/// instance.
/// @details This function is a convenience wrapper around @ref smac_can_fd_receive_channel0.
smacRetCode_t smac_can_fd_receive_channel0(smacCanFd_t canfd, smacCanFdMessage* message,
                                           uint32_t timeout)
{
    return smac_can_fd_receive(canfd, FDCAN_RX_FIFO0, message, timeout);
}

/// @brief Asynchronously receive a CAN FD message from channel 1 using the specified CAN FD
/// instance.
/// @details This function is a convenience wrapper around @ref smac_can_fd_receive_channel1.
smacRetCode_t smac_can_fd_receive_channel1(smacCanFd_t canfd, smacCanFdMessage* message,
                                           uint32_t timeout)
{
    return smac_can_fd_receive(canfd, FDCAN_RX_FIFO1, message, timeout);
}

/// @brief Asynchronously transmit a CAN FD message using the specified CAN FD instance.
/// @details The specific implementation of @ref smac_can_fd_async_transmit for CAN FD.
smacRetCode_t smac_can_fd_async_transmit(smacCanFd_t canfd, const smacCanFdMessage* message)
{
    FDCAN_TxHeaderTypeDef head;
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(message != NULL);

    memset(&head, 0, sizeof(head));

    head.Identifier          = message->head.ident;
    head.IdType              = cast_to_stm32_id_type(message->head.frame_kind);
    head.TxFrameType         = cast_to_stm32_frame_type(message->head.request_kind);
    head.DataLength          = message->head.data_length;
    head.ErrorStateIndicator = cast_to_stm32_error_state(message->head.error_state);
    head.BitRateSwitch       = cast_to_stm32_bit_rate_switch(message->head.switch_bit_rate);
    head.FDFormat            = cast_to_stm32_format(message->head.format);
    head.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    head.MessageMarker       = 0;

    return stm32_cast_code(HAL_FDCAN_AddMessageToTxFifoQ(device->handle, &head, message->data));
}

/// @brief Asynchronously receive a CAN FD message from the specified channel using the specified
/// CAN FD instance.
/// @param canfd The CAN FD instance.
/// @param channel The channel from which to receive the message.
/// @param message Pointer to the structure to store the received message.
/// @return @ref SMAC_RET_OK if the message is successfully queued for asynchronous reception,
/// otherwise an error code.
static smacRetCode_t smac_can_fd_async_receive(smacCanFd_t canfd, uint32_t channel,
                                               smacCanFdMessage* message)
{
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(message != NULL);

    memset(message, 0, sizeof(*message));

    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        return SMAC_RET_STACK_OVERFLOW;
    }

    return stm32_device_cache_queue_set_cache(device, channel, (stm32DeviceCacheData_t)message);
}

/// @brief Asynchronously receive a CAN FD message from channel 0 using the specified CAN FD
/// instance.
/// @details This function is a convenience wrapper around @ref smac_can_fd_async_receive,
/// specifically for channel 0.
smacRetCode_t smac_can_fd_async_receive_channel0(smacCanFd_t canfd, smacCanFdMessage* message)
{
    return smac_can_fd_async_receive(canfd, 0, message);
}

/// @brief Asynchronously receive a CAN FD message from channel 1 using the specified CAN FD
/// instance.
/// @details This function is a convenience wrapper around @ref smac_can_fd_async_receive,
/// specifically for channel 1.
smacRetCode_t smac_can_fd_async_receive_channel1(smacCanFd_t canfd, smacCanFdMessage* message)
{
    return smac_can_fd_async_receive(canfd, 1, message);
}

/// ===============================================================================================
/// @defgroup can_fd_classic_interface CAN FD Classic Interface
/// @brief Interface for handling CAN FD events in a classic manner.
/// ===============================================================================================

/// @brief Create a classic CAN instance.
/// @details The specific implementation of @ref smac_can_fd_create_classic.
smacCanFd_t smac_can_fd_create_classic(void* handle)
{
    assert(handle != NULL);

    // Allocate a device from the STM32 device queue for the classic CAN instance.
    stm32Device_t* device = stm32_device_queue_allocate_with_addition(handle, CAN_FD_ROLE_CLASSIC);

    if (device == NULL)
    {
        return NULL;
    }
    // Initialize the cache for this device in the STM32 device cache queue.
    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        stm32_device_queue_free(device);
        return NULL;
    }

    return (smacCanFd_t)device;
}

/// @brief Set the CAN FD event handling mode to classic for the specified CAN FD instance.
/// @details This function configures the CAN FD instance to use the classic event handling mode,
/// where events are handled using the traditional CAN event callbacks.
smacRetCode_t smac_can_fd_set_event_classic(smacCanFd_t canfd, smacCanEvent_t* event,
                                            smacMcuEventData_t data)
{
    assert(canfd != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)canfd,
                                             (stm32DeviceEventHandle_t*)event, data);
}

/// @brief Transmit a CAN FD message using the classic event handling mode.
/// @details This function transmits a CAN FD message using the classic event handling mode,
/// where the message is sent using the traditional CAN FD transmit function.
smacRetCode_t smac_can_fd_transmit_classic(smacCanFd_t canfd, const smacCanMessage* message,
                                           uint32_t timeout)
{
    assert(canfd != NULL);
    assert(message != NULL);

    smacCanFdMessage can_fd_message;

    can_fd_message.head.ident           = message->head.ident;
    can_fd_message.head.frame_kind      = message->head.frame_kind;
    can_fd_message.head.request_kind    = message->head.request_kind;
    can_fd_message.head.data_length     = message->head.data_length;
    can_fd_message.head.format          = SMAC_CAN_CLASSIC;
    can_fd_message.head.error_state     = SMAC_CAN_ERROR_ACTIVE;
    can_fd_message.head.switch_bit_rate = false;

    smac_can_fd_message_set_data(&can_fd_message, message->data);

    return smac_can_fd_transmit(canfd, &can_fd_message, timeout);
}

/// @brief Receive a CAN FD message from channel 0 using the classic event handling mode.
/// @details This function receives a CAN FD message from channel 0 using the classic event handling
/// mode, where the message is received using the traditional CAN FD receive function.
smacRetCode_t smac_can_fd_receive_channel0_classic(smacCanFd_t canfd, smacCanMessage* message,
                                                   uint32_t timeout)
{
    assert(canfd != NULL);
    assert(message != NULL);

    smacCanFdMessage can_fd_message;

    smacRetCode_t code = smac_can_fd_receive_channel0(canfd, &can_fd_message, timeout);

    if (code == SMAC_RET_OK)
    {
        message->head.ident        = can_fd_message.head.ident;
        message->head.frame_kind   = can_fd_message.head.frame_kind;
        message->head.request_kind = can_fd_message.head.request_kind;
        message->head.data_length  = can_fd_message.head.data_length;

        smac_can_message_set_data(message, can_fd_message.data);
    }

    return code;
}

/// @brief Receive a CAN FD message from channel 1 using the classic event handling mode.
/// @details This function receives a CAN FD message from channel 1 using the classic event handling
/// mode, where the message is received using the traditional CAN FD receive function.
smacRetCode_t smac_can_fd_receive_channel1_classic(smacCanFd_t canfd, smacCanMessage* message,
                                                   uint32_t timeout)
{
    assert(canfd != NULL);
    assert(message != NULL);

    smacCanFdMessage can_fd_message;

    smacRetCode_t code = smac_can_fd_receive_channel1(canfd, &can_fd_message, timeout);

    if (code == SMAC_RET_OK)
    {
        message->head.ident        = can_fd_message.head.ident;
        message->head.frame_kind   = can_fd_message.head.frame_kind;
        message->head.request_kind = can_fd_message.head.request_kind;
        message->head.data_length  = can_fd_message.head.data_length;

        smac_can_message_set_data(message, can_fd_message.data);
    }

    return code;
}

/// @brief Asynchronously transmit a CAN FD message using the classic event handling mode.
/// @details This function initiates the asynchronous transmission of a CAN FD message using the
/// classic event handling mode, where the message is sent using the traditional CAN FD asynchronous
/// transmit function.
smacRetCode_t smac_can_fd_async_transmit_classic(smacCanFd_t canfd, const smacCanMessage* message)
{
    assert(canfd != NULL);
    assert(message != NULL);

    smacCanFdMessage can_fd_message;

    can_fd_message.head.ident           = message->head.ident;
    can_fd_message.head.frame_kind      = message->head.frame_kind;
    can_fd_message.head.request_kind    = message->head.request_kind;
    can_fd_message.head.data_length     = message->head.data_length;
    can_fd_message.head.format          = SMAC_CAN_CLASSIC;
    can_fd_message.head.error_state     = SMAC_CAN_ERROR_ACTIVE;
    can_fd_message.head.switch_bit_rate = false;

    smac_can_fd_message_set_data(&can_fd_message, message->data);

    return smac_can_fd_async_transmit(canfd, &can_fd_message);
}

/// @brief Asynchronously receive a CAN FD message from the specified channel using the classic
/// event handling mode.
/// @details This function initiates the asynchronous reception of a CAN FD message from the
/// specified channel using the classic event handling mode, where the message is received using the
/// traditional CAN FD asynchronous receive function.
static smacRetCode_t smac_can_fd_async_receive_classic(smacCanFd_t canfd, uint32_t channel,
                                                       smacCanMessage* message)
{
    stm32Device_t* device = (stm32Device_t*)canfd;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(message != NULL);

    memset(message, 0, sizeof(*message));

    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        return SMAC_RET_STACK_OVERFLOW;
    }

    return stm32_device_cache_queue_set_cache(device, channel, (stm32DeviceCacheData_t)message);
}

/// @brief Asynchronously receive a CAN FD message from channel 0 using the classic event handling
/// mode.
/// @details This function initiates the asynchronous reception of a CAN FD message from channel 0
/// using the classic event handling mode, where the message is received using the traditional CAN
/// FD asynchronous receive function.
smacRetCode_t smac_can_fd_async_receive_channel0_classic(smacCanFd_t canfd, smacCanMessage* message)
{
    return smac_can_fd_async_receive_classic(canfd, 0, message);
}

/// @brief Asynchronously receive a CAN FD message from channel 1 using the classic event handling
/// mode.
/// @details This function initiates the asynchronous reception of a CAN FD message from channel 1
/// using the classic event handling mode, where the message is received using the traditional CAN
/// FD asynchronous receive function.
smacRetCode_t smac_can_fd_async_receive_channel1_classic(smacCanFd_t canfd, smacCanMessage* message)
{
    return smac_can_fd_async_receive_classic(canfd, 1, message);
}

/// ===============================================================================================
/// @name CAN classic interface
/// @brief Implementation of classic CAN interface functions for handling various CAN events with
/// CAN FD device.
/// ===============================================================================================

/// @brief Create a CAN instance within the MCU abstraction layer.
/// @details Implement the creation API of smac mcu, please see @ref smac_can_create for details.
smacCan_t smac_can_create(void* handle)
{
    return smac_can_fd_create_classic(handle);
}

/// @brief Drop a CAN instance within the MCU abstraction layer.
/// @details Implement the dropping API of smac mcu, please see @ref smac_can_drop for details.
void smac_can_drop(smacCan_t can)
{
    smac_can_fd_drop(can);
}

/// @brief Implement the setting of CAN event callbacks for the specified CAN instance within the
/// MCU abstraction layer.
/// @details Implement the setting API of smac mcu, please see @ref smac_can_set_event for details.
smacRetCode_t smac_can_set_event(smacCan_t can, smacCanEvent_t* event, smacMcuEventData_t data)
{
    return smac_can_fd_set_event_classic(can, event, data);
}

/// @brief Clean CAN event callbacks for the specified CAN instance.
/// @details Implement the cleaning API of smac mcu, please see @ref smac_can_clean_event for
/// details.
void smac_can_clean_event(smacCan_t can)
{
    smac_can_fd_clean_event(can);
}

/// @brief Activate a CAN instance within the MCU abstraction layer.
/// @details Implement the activation API of smac mcu, please see @ref smac_can_active for details.
smacRetCode_t smac_can_active(smacCan_t can)
{
    return smac_can_fd_active(can);
}

/// @brief Deactivate a CAN instance within the MCU abstraction layer.
/// @details Implement the deactivation API of smac mcu, please see @ref smac_can_deactive for
/// details.
smacRetCode_t smac_can_deactive(smacCan_t can)
{

    return smac_can_fd_deactive(can);
}

/// @brief Transmit a message over the specified CAN instance.
/// @details Implement the transmission API of smac mcu, please see @ref smac_can_transmit for
/// details.
smacRetCode_t smac_can_transmit(smacCan_t can, const smacCanMessage* message, uint32_t timeout)
{
    return smac_can_fd_transmit_classic(can, message, timeout);
}

/// @brief Receive a message from channel 0 of the specified CAN instance.
/// @details Implement the reception API of smac mcu for channel 0, please see @ref
/// smac_can_receive_channel0 for details.
smacRetCode_t smac_can_receive_channel0(smacCan_t can, smacCanMessage* message, uint32_t timeout)
{
    return smac_can_fd_receive_channel0_classic(can, message, timeout);
}
/// @brief Receive a message from channel 1 of the specified CAN instance.
/// @details Implement the reception API of smac mcu for channel 1, please see @ref
/// smac_can_receive_channel1 for details.
smacRetCode_t smac_can_receive_channel1(smacCan_t can, smacCanMessage* message, uint32_t timeout)
{
    return smac_can_fd_receive_channel1_classic(can, message, timeout);
}

/// @brief Asynchronously transmit a message over the specified CAN instance.
/// @details Implement the asynchronous transmission API of smac mcu, please see @ref
/// smac_can_async_transmit for details.
smacRetCode_t smac_can_async_transmit(smacCan_t can, const smacCanMessage* message)
{

    return smac_can_fd_async_transmit_classic(can, message);
}

/// @brief Asynchronously receive a message over channel 0 of the specified CAN instance.
/// @details Implement the asynchronous reception API of smac mcu for channel 0, please see @ref
/// smac_can_async_receive_channel0 for details.
smacRetCode_t smac_can_async_receive_channel0(smacCan_t can, smacCanMessage* message)
{

    return smac_can_fd_async_receive_channel0_classic(can, message);
}

/// @brief Asynchronously receive a message over channel 1 of the specified CAN instance.
/// @details Implement the asynchronous reception API of smac mcu for channel 1, please see @ref
/// smac_can_async_receive_channel1 for details.
smacRetCode_t smac_can_async_receive_channel1(smacCan_t can, smacCanMessage* message)
{
    return smac_can_fd_async_receive_channel1_classic(can, message);
}

/// ===============================================================================================
/// @name CAN FD Callback Implementations
/// @brief Implementation of CAN FD callback functions for handling various CAN FD events.
/// ===============================================================================================

/// @brief Handle the reception of a CAN FD message from the specified FIFO.
/// @param hfdcan Pointer to the FDCAN handle.
/// @param RxFifoITs Interrupt status flags for the RX FIFO.
/// @param fifo The FIFO number from which the message is received.
static void on_rx_fifo_callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifoITs, uint32_t fifo)
{
    (void)RxFifoITs;

    stm32DeviceCache_t* cache = stm32_device_cache_queue_search((stm32DeviceHandle_t)hfdcan);

    if ((cache == NULL) || (cache->cache_data[fifo] == (uintptr_t)NULL))
    {
        return;
    }

    if (smac_can_fd_receive(cache->device, fifo, (smacCanFdMessage*)cache->cache_data[fifo], 0) !=
        SMAC_RET_OK)
    {
        return;
    }

    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hfdcan);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & CAN_FD_ROLE_MASK) == CAN_FD_ROLE_FD)
        {
            if (event->event->can_fd.rx_complete != NULL)
            {
                event->event->can_fd.rx_complete(event->device, event->event_data);
            }
        }
        else if ((event->device->addition & CAN_FD_ROLE_MASK) == CAN_FD_ROLE_CLASSIC)
        {
            if (event->event->can.rx_complete != NULL)
            {
                event->event->can.rx_complete(event->device, event->event_data);
            }
        }
    }
}

// void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t TxEventFifoITs);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    on_rx_fifo_callback(hfdcan, RxFifo0ITs, 0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    on_rx_fifo_callback(hfdcan, RxFifo1ITs, 1);
}

// void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef* hfdcan);

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes)
{
    (void)BufferIndexes;

    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hfdcan);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & CAN_FD_ROLE_MASK) == CAN_FD_ROLE_FD)
        {
            if ((event->event->can_fd.tx_complete != NULL))
            {
                event->event->can_fd.tx_complete(event->device, event->event_data);
            }
        }
        else if ((event->device->addition & CAN_FD_ROLE_MASK) == CAN_FD_ROLE_CLASSIC)
        {
            if ((event->event->can.tx_complete != NULL))
            {
                event->event->can.tx_complete(event->device, event->event_data);
            }
        }
    }
}

// void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes);
// void HAL_FDCAN_HighPriorityMessageCallback(FDCAN_HandleTypeDef* hfdcan);
// void HAL_FDCAN_TimestampWraparoundCallback(FDCAN_HandleTypeDef* hfdcan);
// void HAL_FDCAN_TimeoutOccurredCallback(FDCAN_HandleTypeDef* hfdcan);
// void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan);
// void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs);

#endif // HAL_FDCAN_MODULE_ENABLED
