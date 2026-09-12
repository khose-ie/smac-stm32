#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32-i2c.h>
#include <stm32.h>

#define STM32_I2C_MAX_TIMEOUT (500)

/// @brief Create an I2C master instance within the MCU abstraction layer.
/// @details This function creates an I2C master instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacI2c_t smac_i2c_master_create(void* handle)
{
    return (smacI2c_t)stm32_device_queue_allocate(handle, STM32_I2C_ROLE_MASTER);
}

/// @brief Drop an I2C master instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C master instance.
void smac_i2c_master_drop(smacI2c_t i2c)
{
    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C master event callbacks for the specified I2C master instance.
/// @details This function sets the event callbacks for the specified I2C master instance.
smacRetCode_t smac_i2c_master_set_event(smacI2c_t i2c, smacI2cMasterEvent_t* event,
                                        smacMcuEventData_t data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C master event callbacks for the specified I2C master instance.
/// @details This function removes all event callbacks associated with the specified I2C master
/// instance.
void smac_i2c_master_clean_event(smacI2c_t i2c)
{
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Checks if the specified I2C slave device is ready for communication.
/// @details This function checks whether the specified I2C slave device associated with the given
/// I2C master instance is ready for communication.
smacRetCode_t smac_i2c_master_selected_device_in_ready_state(smacI2c_t i2c, uint16_t slave,
                                                             uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    if (device == NULL)
    {
        return SMAC_RET_NULL_REF;
    }

    if (HAL_I2C_IsDeviceReady(device->handle, slave, 1, timeout) == HAL_OK)
    {
        return SMAC_RET_OK;
    }

    if (timeout <= STM32_I2C_MAX_TIMEOUT)
    {
        return SMAC_RET_MCU_I2C_BUSY;
    }

    HAL_I2C_DeInit(device->handle);
    HAL_I2C_Init(device->handle);

    if (HAL_I2C_IsDeviceReady(device->handle, slave, 1, timeout - STM32_I2C_MAX_TIMEOUT) != HAL_OK)
    {
        return SMAC_RET_MCU_I2C_FAULT;
    }

    return SMAC_RET_OK;
}

/// @brief Transmit data over the specified I2C master instance.
/// @details This function transmits the specified data to the given address over the I2C master
/// instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_master_transmit(smacI2c_t i2c, uint16_t slave, const uint8_t* data,
                                       uint32_t size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    smacRetCode_t code = smac_i2c_master_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(
                     HAL_I2C_Master_Transmit(device->handle, slave, (uint8_t*)data, size, timeout))
               : SMAC_RET_NULL_REF;
}

/// @brief Receive data over the specified I2C master instance.
/// @details This function receives data from the given address over the I2C master instance within
/// the MCU abstraction layer.
smacRetCode_t smac_i2c_master_receive(smacI2c_t i2c, uint16_t slave, uint8_t* data, uint32_t size,
                                      uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    smacRetCode_t code = smac_i2c_master_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_I2C_Master_Receive(device->handle, slave, data, size, timeout))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously transmit data over the specified I2C master instance.
/// @details This function initiates an asynchronous transmission of the specified data to the given
/// address over the I2C master instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_master_async_transmit(smacI2c_t i2c, uint16_t slave, const uint8_t* data,
                                             uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(
                     handle->hdmatx == NULL
                         ? HAL_I2C_Master_Transmit_IT(handle, slave, (uint8_t*)data, size)
                         : HAL_I2C_Master_Transmit_DMA(handle, slave, (uint8_t*)data, size))
               : SMAC_RET_NULL_REF;
}

/// @brief Asynchronously receive data over the specified I2C master instance.
/// @details This function initiates an asynchronous reception of data from the given address over
/// the I2C master instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_master_async_receive(smacI2c_t i2c, uint16_t slave, uint8_t* data,
                                            uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(handle->hdmarx == NULL
                                     ? HAL_I2C_Master_Receive_IT(handle, slave, data, size)
                                     : HAL_I2C_Master_Receive_DMA(handle, slave, data, size))
               : SMAC_RET_NULL_REF;
}

/// ===============================================================================================
/// @name I2C Master Callback Implementations
/// @brief Implementation of I2C master callback functions for handling various I2C master events.
/// ===============================================================================================

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_master.tx_complete != NULL))
    {
        event->event->i2c_master.tx_complete((smacI2c_t)event->device, event->event_data);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_master.rx_complete != NULL))
    {
        event->event->i2c_master.rx_complete((smacI2c_t)event->device, event->event_data);
    }
}
