#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32-queue.h>
#include <stm32.h>
#include <string.h>

#if defined(STM32H5) || defined(STM32C5)
#define STM32_CAN_FD
#endif // defined(STM32H5)

#ifndef STM32_CAN_FD
#define STM32_CAN_CLASSIC
#endif // STM32_CAN_FD

#define cast_to_stm32_rtr(request_kind)                                                            \
    ((request_kind == SMAC_CAN_DATA) ? CAN_RTR_DATA : CAN_RTR_REMOTE)

#define cast_to_stm32_ide(frame_kind)                                                              \
    ((frame_kind == SMAC_CAN_FRAME_STANDARD) ? CAN_ID_STD : CAN_ID_EXT)

#define cast_from_stm32_rtr(request_kind)                                                          \
    ((request_kind == CAN_RTR_DATA) ? SMAC_CAN_DATA : SMAC_CAN_REMOTE)

#define cast_from_stm32_ide(frame_kind)                                                            \
    ((frame_kind == CAN_ID_STD) ? SMAC_CAN_FRAME_STANDARD : SMAC_CAN_FRAME_EXTENDED)

static smacCanEventTxComplete _on_tx_complete    = NULL;
static smacCanEventRxComplete _on_rx_complete[2] = { NULL, NULL };

smacRetCode_t set_can_classic_event(smacCanEventTxComplete on_tx_complete,
                                    smacCanEventRxComplete on_rx_complete0,
                                    smacCanEventRxComplete on_rx_complete1)
{
    _on_tx_complete    = on_tx_complete;
    _on_rx_complete[0] = on_rx_complete0;
    _on_rx_complete[1] = on_rx_complete1;

    return SMAC_RET_OK;
}

#ifdef STM32_CAN_CLASSIC

/// @brief Create a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_create for classic CAN instances.
static smacCan_t can_classic_create(void* handle)
{
    // Allocate a device from the STM32 device queue for the classic CAN instance.
    stm32Device* device = stm32_device_queue_allocate(handle, 0);

    if (device == NULL)
    {
        return NULL;
    }
    // Initialize the cache for this device in the STM32 device cache queue.
    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        stm32_device_queue_free(device);
    }

    return (smacCan_t)device;
}

/// @brief Drop a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_drop for classic CAN instances.
static void can_classic_drop(smacCan_t can)
{
    if (can != NULL)
    {
        stm32_device_event_queue_free((stm32Device*)can);
        stm32_device_cache_queue_free((stm32Device*)can);
        stm32_device_queue_free((stm32Device*)can);
    }
}

/// @brief Set an event for a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_set_event for classic CAN instances.
static smacRetCode_t can_classic_set_event(smacCan_t can, smacMcuEventData_t data)
{
    stm32Device* device = (stm32Device*)can;
    return device != NULL ? stm32_device_event_queue_allocate(device, data) : SMAC_RET_NULL_REF;
}

/// @brief Clean events for a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_clean_event for classic CAN instances.
static void can_classic_clean_event(smacCan_t can)
{
    if (can != NULL)
    {
        stm32_device_event_queue_free((stm32Device*)can);
    }
}

/// @brief Activate a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_active for classic CAN instances.
static smacRetCode_t can_classic_active(smacCan_t can)
{
    stm32Device* device = (stm32Device*)can;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_CAN_Start(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Deactivate a classic CAN instance within the MCU abstraction layer.
/// @details The specific implementation of @ref smac_can_deactive for classic CAN instances.
static smacRetCode_t can_classic_deactive(smacCan_t can)
{
    stm32Device* device = (stm32Device*)can;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_CAN_Stop(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Transmit a classic CAN message using the specified classic CAN instance.
/// @details The specific implementation of @ref smac_can_transmit for classic CAN instances.
static smacRetCode_t can_classic_transmit(smacCan_t can, const smacCanMessage* message,
                                          uint32_t timeout)
{
    CAN_TxHeaderTypeDef head;
    stm32Device* device = (stm32Device*)can;

    if ((device == NULL) || (device->handle == NULL) || (message == NULL))
    {
        return SMAC_RET_PARAM_ERR;
    }

    memset(&head, 0, sizeof(head));

    head.StdId = message->head.frame_kind == SMAC_CAN_FRAME_STANDARD ? message->head.ident : 0;
    head.ExtId = message->head.frame_kind == SMAC_CAN_FRAME_EXTENDED ? message->head.ident : 0;
    head.RTR   = cast_to_stm32_rtr(message->head.request_kind);
    head.IDE   = cast_to_stm32_ide(message->head.frame_kind);
    head.DLC   = message->head.data_length;

    uint32_t mailbox = 0;

    smacRetCode_t code =
        stm32_cast_code(HAL_CAN_AddTxMessage(device->handle, &head, message->data, &mailbox));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    uint32_t tick       = HAL_GetTick();
    uint32_t start_tick = tick;

    while (HAL_CAN_IsTxMessagePending(device->handle, mailbox))
    {
        tick = HAL_GetTick();

        if ((tick - start_tick) >= timeout)
        {
            return SMAC_RET_TIMEOUT;
        }

        HAL_Delay(1);
    }

    return SMAC_RET_OK;
}

/// @brief Receive a classic CAN message from the specified channel using the specified classic CAN
/// instance.
/// @param can The classic CAN instance to use for reception.
/// @param channel The channel (FIFO) from which to receive the message.
/// @param message Pointer to the structure where the received message will be stored.
/// @param timeout The maximum time to wait for a message, in milliseconds.
/// @return The result of the reception operation, as a @ref smacRetCode_t value.
static smacRetCode_t can_classic_receive(smacCan_t can, uint32_t channel, smacCanMessage* message,
                                         uint32_t timeout)
{
    CAN_RxHeaderTypeDef head;
    stm32Device* device = (stm32Device*)can;

    uint32_t start_tick = HAL_GetTick();

    while (HAL_CAN_GetRxFifoFillLevel(device->handle, channel) == 0)
    {
        if ((HAL_GetTick() - start_tick) >= timeout)
        {
            return SMAC_RET_TIMEOUT;
        }

        HAL_Delay(1);
    }

    smacRetCode_t code =
        stm32_cast_code(HAL_CAN_GetRxMessage(device->handle, channel, &head, message->data));

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    message->head.ident        = head.IDE == CAN_ID_STD ? head.StdId : head.ExtId;
    message->head.request_kind = cast_from_stm32_rtr(head.RTR);
    message->head.frame_kind   = cast_from_stm32_ide(head.IDE);
    message->head.data_length  = head.DLC;

    return SMAC_RET_OK;
}

/// @brief Receive a classic CAN message from channel 0 using the specified classic CAN instance.
/// @details The specific implementation of @ref smac_can_receive_channel for channel 0.
static smacRetCode_t can_classic_receive_channel0(smacCan_t can, smacCanMessage* message,
                                                  uint32_t timeout)
{
    return can_classic_receive(can, CAN_RX_FIFO0, message, timeout);
}

/// @brief Receive a classic CAN message from channel 1 using the specified classic CAN instance.
/// @details The specific implementation of @ref smac_can_receive_channel for channel 1.
static smacRetCode_t can_classic_receive_channel1(smacCan_t can, smacCanMessage* message,
                                                  uint32_t timeout)
{
    return can_classic_receive(can, CAN_RX_FIFO1, message, timeout);
}

/// @brief Asynchronously transmit a classic CAN message using the specified classic CAN instance.
/// @details The specific implementation of @ref smac_can_async_transmit for classic CAN.
static smacRetCode_t can_classic_async_transmit(smacCan_t can, const smacCanMessage* message)
{
    CAN_TxHeaderTypeDef head;
    stm32Device* device = (stm32Device*)can;

    if ((device == NULL) || (device->handle == NULL) || (message == NULL))
    {
        return SMAC_RET_PARAM_ERR;
    }

    memset(&head, 0, sizeof(head));

    head.StdId = message->head.frame_kind == SMAC_CAN_FRAME_STANDARD ? message->head.ident : 0;
    head.ExtId = message->head.frame_kind == SMAC_CAN_FRAME_EXTENDED ? message->head.ident : 0;
    head.RTR   = cast_to_stm32_rtr(message->head.request_kind);
    head.IDE   = cast_to_stm32_ide(message->head.frame_kind);
    head.DLC   = message->head.data_length;

    uint32_t mailbox = 0;

    return stm32_cast_code(HAL_CAN_AddTxMessage(device->handle, &head, message->data, &mailbox));
}

/// @brief Asynchronously receive a classic CAN message from the specified channel using the
/// specified classic CAN instance.
/// @param can The classic CAN instance.
/// @param channel The channel from which to receive the message.
/// @param message Pointer to the structure to store the received message.
/// @return @ref SMAC_RET_OK if the message is successfully queued for asynchronous reception,
/// otherwise an error code.
static smacRetCode_t can_classic_async_receive(smacCan_t can, uint32_t channel,
                                               smacCanMessage* message)
{
    stm32Device* device = (stm32Device*)can;

    if ((device == NULL) || (device->handle == NULL) || (message == NULL))
    {
        return SMAC_RET_PARAM_ERR;
    }

    memset(message, 0, sizeof(*message));

    if (stm32_device_cache_queue_allocate(device) != SMAC_RET_OK)
    {
        return SMAC_RET_STACK_OVERFLOW;
    }

    return stm32_device_cache_queue_set_cache(device, channel, (stm32DeviceCacheData)message);
}

/// @brief Asynchronously receive a classic CAN message from channel 0 using the specified classic
/// CAN instance.
/// @details This function is a convenience wrapper around @ref can_classic_async_receive,
/// specifically for channel 0.
static smacRetCode_t can_classic_async_receive_channel0(smacCan_t can, smacCanMessage* message)
{
    return can_classic_async_receive(can, 0, message);
}

/// @brief Asynchronously receive a classic CAN message from channel 1 using the specified classic
/// CAN instance.
/// @details This function is a convenience wrapper around @ref can_classic_async_receive,
/// specifically for channel 1.
static smacRetCode_t can_classic_async_receive_channel1(smacCan_t can, smacCanMessage* message)
{
    return can_classic_async_receive(can, 1, message);
}

#else // defined(STM32_CAN_FD)

static smacCanMessage* can_message[2];
static smacCanFdMessage can_fd_message[2];

static void _on_fd_tx_complete(smacCan_t can, smacMcuEventData_t event_data)
{
    if (_on_tx_complete != NULL)
    {
        _on_tx_complete(can, event_data);
    }
}

static void _on_fd_rx_complete(smacCan_t can, uint32_t channel, smacMcuEventData_t event_data)
{
    if (_on_rx_complete[channel] != NULL)
    {
        can_message[channel]->head.ident        = can_fd_message[channel].head.ident;
        can_message[channel]->head.frame_kind   = can_fd_message[channel].head.frame_kind;
        can_message[channel]->head.request_kind = can_fd_message[channel].head.request_kind;
        can_message[channel]->head.data_length  = can_fd_message[channel].head.data_length;

        smac_can_message_set_data(can_message[channel], can_fd_message[channel].data);

        _on_rx_complete[channel](can, event_data);
    }
}

static void _on_fd_rx_complete0(smacCan_t can, smacMcuEventData_t event_data)
{
    _on_fd_rx_complete(can, 0, event_data);
}

static void _on_fd_rx_complete1(smacCan_t can, smacMcuEventData_t event_data)
{
    _on_fd_rx_complete(can, 1, event_data);
}

#endif // STM32_CAN_CLASSIC

/// @brief Set CAN event callbacks for the MCU abstraction layer.
/// @param on_rx_complete Callback for reception complete event.
/// @return @ref SMAC_RET_OK if the callbacks are set successfully, otherwise an error code.
smacRetCode_t smac_mcu_set_can_event(smacCanEventTxComplete on_tx_complete,
                                     smacCanEventRxComplete on_rx_complete0,
                                     smacCanEventRxComplete on_rx_complete1)
{
#if defined(STM32_CAN_CLASSIC)

    return set_can_classic_event(on_tx_complete, on_rx_complete0, on_rx_complete1);

#else // defined(STM32_CAN_FD)

    set_can_classic_event(on_tx_complete, on_rx_complete0, on_rx_complete1);
    return smac_mcu_set_can_fd_event(_on_fd_tx_complete, _on_fd_rx_complete0, _on_fd_rx_complete1);

#endif // STM32_CAN_CLASSIC
}

/// @brief Create a CAN instance within the MCU abstraction layer.
/// @details Implement the creation API of smac mcu, please see @ref smac_can_create for details.
smacCan_t smac_can_create(void* handle)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_create(handle);

#else // defined(STM32_CAN_FD)

    return smac_can_fd_create(handle);

#endif // STM32_CAN_CLASSIC
}

/// @brief Drop a CAN instance within the MCU abstraction layer.
/// @details Implement the dropping API of smac mcu, please see @ref smac_can_drop for details.
void smac_can_drop(smacCan_t can)
{
#if defined(STM32_CAN_CLASSIC)

    can_classic_drop(can);

#else // defined(STM32_CAN_FD)

    smac_can_fd_drop(can);

#endif // STM32_CAN_CLASSIC
}

/// @brief Implement the setting of CAN event callbacks for the specified CAN instance within the
/// MCU abstraction layer.
/// @details Implement the setting API of smac mcu, please see @ref smac_can_set_event for details.
smacRetCode_t smac_can_set_event(smacCan_t can, smacMcuEventData_t data)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_set_event(can, data);

#else // defined(STM32_CAN_FD)

    return smac_can_fd_set_event(can, data);

#endif // STM32_CAN_CLASSIC
}

/// @brief Clean CAN event callbacks for the specified CAN instance.
/// @details Implement the cleaning API of smac mcu, please see @ref smac_can_clean_event for
/// details.
void smac_can_clean_event(smacCan_t can)
{
#if defined(STM32_CAN_CLASSIC)

    can_classic_clean_event(can);

#else // defined(STM32_CAN_FD)

    smac_can_fd_clean_event(can);

#endif // STM32_CAN_CLASSIC
}

/// @brief Activate a CAN instance within the MCU abstraction layer.
/// @details Implement the activation API of smac mcu, please see @ref smac_can_active for details.
smacRetCode_t smac_can_active(smacCan_t can)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_active(can);

#else // defined(STM32_CAN_FD)

    return smac_can_fd_active(can);

#endif // STM32_CAN_CLASSIC
}

/// @brief Deactivate a CAN instance within the MCU abstraction layer.
/// @details Implement the deactivation API of smac mcu, please see @ref smac_can_deactive for
/// details.
smacRetCode_t smac_can_deactive(smacCan_t can)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_deactive(can);

#else // defined(STM32_CAN_FD)

    return smac_can_fd_deactive(can);

#endif // STM32_CAN_CLASSIC
}

/// @brief Transmit a message over the specified CAN instance.
/// @details Implement the transmission API of smac mcu, please see @ref smac_can_transmit for
/// details.
smacRetCode_t smac_can_transmit(smacCan_t can, const smacCanMessage* message, uint32_t timeout)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_transmit(can, message, timeout);

#else // defined(STM32_CAN_FD)

    smacCanFdMessage can_fd_message;

    can_fd_message.head.ident           = message->head.ident;
    can_fd_message.head.frame_kind      = message->head.frame_kind;
    can_fd_message.head.request_kind    = message->head.request_kind;
    can_fd_message.head.data_length     = message->head.data_length;
    can_fd_message.head.format          = SMAC_CAN_CLASSIC;
    can_fd_message.head.error_state     = SMAC_CAN_ERROR_ACTIVE;
    can_fd_message.head.switch_bit_rate = false;

    smac_can_fd_message_set_data(&can_fd_message, message->data);

    return smac_can_fd_transmit(can, &can_fd_message, timeout);

#endif // STM32_CAN_CLASSIC
}

/// @brief Receive a message from channel 0 of the specified CAN instance.
/// @details Implement the reception API of smac mcu for channel 0, please see @ref
/// smac_can_receive_channel0 for details.
smacRetCode_t smac_can_receive_channel0(smacCan_t can, smacCanMessage* message, uint32_t timeout)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_receive_channel0(can, message, timeout);

#else // defined(STM32_CAN_FD)

    smacCanFdMessage can_fd_message;

    smacRetCode_t code = smac_can_fd_receive_channel0(can, &can_fd_message, timeout);

    if (code == SMAC_RET_OK)
    {
        message->head.ident        = can_fd_message.head.ident;
        message->head.frame_kind   = can_fd_message.head.frame_kind;
        message->head.request_kind = can_fd_message.head.request_kind;
        message->head.data_length  = can_fd_message.head.data_length;

        smac_can_message_set_data(message, can_fd_message.data);
    }

    return code;

#endif // STM32_CAN_CLASSIC
}
/// @brief Receive a message from channel 1 of the specified CAN instance.
/// @details Implement the reception API of smac mcu for channel 1, please see @ref
/// smac_can_receive_channel1 for details.
smacRetCode_t smac_can_receive_channel1(smacCan_t can, smacCanMessage* message, uint32_t timeout)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_receive_channel1(can, message, timeout);

#else // defined(STM32_CAN_FD)

    smacCanFdMessage can_fd_message;

    smacRetCode_t code = smac_can_fd_receive_channel1(can, &can_fd_message, timeout);

    if (code == SMAC_RET_OK)
    {
        message->head.ident        = can_fd_message.head.ident;
        message->head.frame_kind   = can_fd_message.head.frame_kind;
        message->head.request_kind = can_fd_message.head.request_kind;
        message->head.data_length  = can_fd_message.head.data_length;

        smac_can_message_set_data(message, can_fd_message.data);
    }

    return code;

#endif // STM32_CAN_CLASSIC
}

/// @brief Asynchronously transmit a message over the specified CAN instance.
/// @details Implement the asynchronous transmission API of smac mcu, please see @ref
/// smac_can_async_transmit for details.
smacRetCode_t smac_can_async_transmit(smacCan_t can, const smacCanMessage* message)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_async_transmit(can, message);

#else // defined(STM32_CAN_FD)

    smacCanFdMessage can_fd_message;

    can_fd_message.head.ident           = message->head.ident;
    can_fd_message.head.frame_kind      = message->head.frame_kind;
    can_fd_message.head.request_kind    = message->head.request_kind;
    can_fd_message.head.data_length     = message->head.data_length;
    can_fd_message.head.format          = SMAC_CAN_CLASSIC;
    can_fd_message.head.error_state     = SMAC_CAN_ERROR_ACTIVE;
    can_fd_message.head.switch_bit_rate = false;

    smac_can_fd_message_set_data(&can_fd_message, message->data);

    return smac_can_fd_async_transmit(can, &can_fd_message);

#endif // STM32_CAN_CLASSIC
}

/// @brief Asynchronously receive a message over channel 0 of the specified CAN instance.
/// @details Implement the asynchronous reception API of smac mcu for channel 0, please see @ref
/// smac_can_async_receive_channel0 for details.
smacRetCode_t smac_can_async_receive_channel0(smacCan_t can, smacCanMessage* message)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_async_receive_channel0(can, message);

#else // defined(STM32_CAN_FD)

    can_message[0] = message;
    return smac_can_fd_async_receive_channel0(can, &can_fd_message[0]);

#endif // STM32_CAN_CLASSIC
}

/// @brief Asynchronously receive a message over channel 1 of the specified CAN instance.
/// @details Implement the asynchronous reception API of smac mcu for channel 1, please see @ref
/// smac_can_async_receive_channel1 for details.
smacRetCode_t smac_can_async_receive_channel1(smacCan_t can, smacCanMessage* message)
{
#if defined(STM32_CAN_CLASSIC)

    return can_classic_async_receive_channel1(can, message);

#else // defined(STM32_CAN_FD)

    can_message[1] = message;
    return smac_can_fd_async_receive_channel1(can, &can_fd_message[1]);

#endif // STM32_CAN_CLASSIC
}

/// ===============================================================================================
/// @name ADC Callback Implementations
/// @brief Implementation of ADC callback functions for handling various ADC events.
/// ===============================================================================================

#ifdef STM32_CAN_CLASSIC

static void on_rx_fifo_msg_pending_callback(CAN_HandleTypeDef* hcan, uint32_t fifo)
{
    stm32DeviceCache* cache = stm32_device_cache_queue_search(hcan);

    if ((cache == NULL) || (cache->cache_data[fifo] == (uintptr_t)NULL))
    {
        return;
    }

    smacRetCode_t code =
        can_classic_receive(cache->device, fifo, (smacCanMessage*)cache->cache_data[fifo], 0);

    if (code != SMAC_RET_OK)
    {
        return;
    }

    if (_on_rx_complete[fifo] != NULL)
    {
        stm32DeviceEvent* event = stm32_device_event_queue_search(hcan);

        if (event == NULL)
        {
            return;
        }

        _on_rx_complete[fifo](event->device, event->event_data);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
{
    if (_on_tx_complete != NULL)
    {
        stm32DeviceEvent* event = stm32_device_event_queue_search(hcan);

        if (event == NULL)
        {
            return;
        }

        _on_tx_complete(event->device, event->event_data);
    }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_TxMailbox0CompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_TxMailbox0CompleteCallback(hcan);
}

// void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
// void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
// void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    on_rx_fifo_msg_pending_callback(hcan, 0);
}

// void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    on_rx_fifo_msg_pending_callback(hcan, 1);
}

// void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
// void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
// void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);

#endif // STM32_CAN_CLASSIC
