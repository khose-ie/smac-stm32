#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32-i2c.h>
#include <stm32.h>

/// @brief Create an I2C slave instance within the MCU abstraction layer.
/// @details This function creates an I2C slave instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacI2c_t smac_i2c_slave_create(void* handle)
{
    return (smacI2c_t)stm32_device_queue_allocate(handle, STM32_I2C_ROLE_SLAVE);
}

/// @brief Drop an I2C slave instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C slave instance.
/// @param i2c The I2C slave instance to be dropped.
void smac_i2c_slave_drop(smacI2c_t i2c)
{
    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C slave event callbacks for the specified I2C slave instance.
smacRetCode_t smac_i2c_slave_set_event(smacI2c_t i2c, smacI2cSlaveEvent_t* event,
                                       smacMcuEventData_t data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C slave event callbacks for the specified I2C slave instance.
void smac_i2c_slave_clean_event(smacI2c_t i2c)
{
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Listen for incoming communication on the specified I2C slave instance.
/// @details This function puts the I2C slave instance into a listening state, ready to respond to
/// master requests.
smacRetCode_t smac_i2c_slave_listen(smacI2c_t i2c)
{
    stm32Device_t* device = (stm32Device_t*)i2c;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_I2C_EnableListen_IT(device->handle))
               : SMAC_RET_NULL_REF;
}

/// @brief Transmit data over the specified I2C slave instance.
/// @details This function transmits the specified data to the given address over the I2C slave
/// instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_transmit(smacI2c_t i2c, const uint8_t* data, uint32_t size,
                                      uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(
                     HAL_I2C_Slave_Transmit(device->handle, (uint8_t*)data, size, timeout))
               : SMAC_RET_NULL_REF;
}

/// @brief Receive data over the specified I2C slave instance.
/// @details This function receives data over the I2C slave instance within
/// the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_receive(smacI2c_t i2c, uint8_t* data, uint32_t size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;
    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_I2C_Slave_Receive(device->handle, data, size, timeout))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously transmit data over the specified I2C slave instance.
/// @details This function initiates an asynchronous transmission of the specified data over the I2C
/// slave instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_async_transmit(smacI2c_t i2c, const uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(
                     handle->hdmatx == NULL
                         ? HAL_I2C_Slave_Transmit_IT(device->handle, (uint8_t*)data, size)
                         : HAL_I2C_Slave_Transmit_DMA(handle, (uint8_t*)data, size))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously receive data over the specified I2C slave instance.
/// @details This function initiates an asynchronous reception of data over
/// the I2C slave instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_async_receive(smacI2c_t i2c, uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(handle->hdmarx == NULL
                                     ? HAL_I2C_Slave_Receive_IT(handle, data, size)
                                     : HAL_I2C_Slave_Receive_DMA(handle, data, size))
               : SMAC_RET_NULL_REF;
}

/// ===============================================================================================
/// @name I2C Slave Callback Implementations
/// @brief Implementation of I2C slave callback functions for handling various I2C slave events.
/// ===============================================================================================

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_slave.tx_complete != NULL))
    {
        event->event->i2c_slave.tx_complete((smacI2c_t)event->device,
                                            (smacMcuEventData_t)event->event_data);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_slave.rx_complete != NULL))
    {
        event->event->i2c_slave.rx_complete((smacI2c_t)event->device,
                                            (smacMcuEventData_t)event->event_data);
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_slave.selected != NULL))
    {
        event->event->i2c_slave.selected((smacI2c_t)event->device,
                                         (smacMcuEventData_t)event->event_data);
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if ((event != NULL) && (event->event != NULL) &&
        (event->event->i2c_slave.listen_complete != NULL))
    {
        event->event->i2c_slave.listen_complete((smacI2c_t)event->device,
                                                (smacMcuEventData_t)event->event_data);
    }
}
