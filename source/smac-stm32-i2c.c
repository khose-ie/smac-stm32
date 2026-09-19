#include <assert.h>
#include <smac/middleware/smac-mcu.h>
#include <stm32.h>

#define STM32_I2C_MAX_TIMEOUT (500)
#define STM32_I2C_ROLE_MASK   (0xC0000000)
#define STM32_I2C_ROLE_MEM    (0x00000000)
#define STM32_I2C_ROLE_MASTER (0x80000000)
#define STM32_I2C_ROLE_SLAVE  (0xC0000000)

/// ===============================================================================================
/// @name I2C Mem Interface
/// @brief Implementation of I2C mem interface.
/// ===============================================================================================

static uint16_t stm32_cast_i2c_mem_addr_size(smacI2cMemAddrSize mem_addr_size)
{
    switch (mem_addr_size)
    {
        case SMAC_I2C_MEM_ADDR_BIT8:
            return I2C_MEMADD_SIZE_8BIT;
        case SMAC_I2C_MEM_ADDR_BIT16:
            return I2C_MEMADD_SIZE_16BIT;
    }

    return I2C_MEMADD_SIZE_8BIT; // Default to 8-bit if unknown
}

/// @brief Create an I2C memory instance within the MCU abstraction layer.
/// @details This function creates an I2C memory instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacI2c_t smac_i2c_mem_create(void* handle)
{
    assert(handle != NULL);
    return (smacI2c_t)stm32_device_queue_allocate_with_addition(handle, STM32_I2C_ROLE_MEM);
}

/// @brief Drop an I2C memory instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C memory instance.
void smac_i2c_mem_drop(smacI2c_t i2c)
{
    assert(i2c != NULL);

    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C memory event callbacks for the specified I2C memory instance.
/// @param i2c The I2C memory instance.
smacRetCode_t smac_i2c_mem_set_event(smacI2c_t i2c, smacI2cMemEvent_t* event,
                                     smacMcuEventData_t data)
{
    assert(i2c != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C memory event callbacks for the specified I2C memory instance.
void smac_i2c_mem_clean_event(smacI2c_t i2c)
{
    assert(i2c != NULL);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Check if the I2C memory device is in a ready state.
/// @details This function checks whether the I2C memory device associated with the specified I2C
/// instance is ready for communication.
smacRetCode_t smac_i2c_mem_selected_device_in_ready_state(smacI2c_t i2c, uint16_t address,
                                                          uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);

    if (HAL_I2C_IsDeviceReady(device->handle, address, 1, timeout) == HAL_OK)
    {
        return SMAC_RET_OK;
    }

    if (timeout <= STM32_I2C_MAX_TIMEOUT)
    {
        return SMAC_RET_MCU_I2C_BUSY;
    }

    HAL_I2C_DeInit(device->handle);
    HAL_I2C_Init(device->handle);

    if (HAL_I2C_IsDeviceReady(device->handle, address, 1, timeout - STM32_I2C_MAX_TIMEOUT) !=
        HAL_OK)
    {
        return SMAC_RET_MCU_I2C_FAULT;
    }

    return SMAC_RET_OK;
}

/// @brief Write data to the specified I2C memory device.
/// @details This function writes the specified data to the given memory address of the I2C memory
/// device associated with the specified I2C instance.
smacRetCode_t smac_i2c_mem_write(smacI2c_t i2c, uint16_t slave, uint16_t mem_addr,
                                 smacI2cMemAddrSize mem_addr_size, const uint8_t* data,
                                 uint16_t size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    smacRetCode_t code = smac_i2c_mem_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return stm32_cast_code(HAL_I2C_Mem_Write(device->handle, slave, mem_addr,
                                             stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                             (uint8_t*)data, size, timeout));
}

/// @brief Read data from the specified I2C memory device.
/// @details This function reads data from the given memory address of the I2C memory device
/// associated with the specified I2C instance.
smacRetCode_t smac_i2c_mem_read(smacI2c_t i2c, uint16_t slave, uint16_t mem_addr,
                                smacI2cMemAddrSize mem_addr_size, uint8_t* data, uint16_t size,
                                uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    smacRetCode_t code = smac_i2c_mem_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return stm32_cast_code(HAL_I2C_Mem_Read(device->handle, slave, mem_addr,
                                            stm32_cast_i2c_mem_addr_size(mem_addr_size), data, size,
                                            timeout));
}

/// @brief Asynchronously write data to the specified I2C memory device.
/// @details This function initiates an asynchronous write of the specified data to the given memory
/// address of the I2C memory device associated with the specified I2C instance.
smacRetCode_t smac_i2c_mem_async_write(smacI2c_t i2c, uint16_t slave, uint16_t mem_addr,
                                       smacI2cMemAddrSize mem_addr_size, const uint8_t* data,
                                       uint16_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmatx == NULL
                               ? HAL_I2C_Mem_Write_IT(handle, slave, mem_addr,
                                                      stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                                      (uint8_t*)data, size)
                               : HAL_I2C_Mem_Write_DMA(handle, slave, mem_addr,
                                                       stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                                       (uint8_t*)data, size));
}

/// @brief Asynchronously read data from the specified I2C memory device.
/// @details This function initiates an asynchronous read of data from the given memory address of
/// the I2C memory device associated with the specified I2C instance.
smacRetCode_t smac_i2c_mem_async_read(smacI2c_t i2c, uint16_t slave, uint16_t mem_addr,
                                      smacI2cMemAddrSize mem_addr_size, uint8_t* data,
                                      uint16_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(
        handle->hdmarx == NULL
            ? HAL_I2C_Mem_Read_IT(handle, slave, mem_addr,
                                  stm32_cast_i2c_mem_addr_size(mem_addr_size), data, size)
            : HAL_I2C_Mem_Read_DMA(handle, slave, mem_addr,
                                   stm32_cast_i2c_mem_addr_size(mem_addr_size), data, size));
}

/// ===============================================================================================
/// @name I2C Master Interface
/// @brief Implementation of I2C master interface.
/// ===============================================================================================

/// @brief Create an I2C master instance within the MCU abstraction layer.
/// @details This function creates an I2C master instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacI2c_t smac_i2c_master_create(void* handle)
{
    assert(handle != NULL);
    return (smacI2c_t)stm32_device_queue_allocate_with_addition(handle, STM32_I2C_ROLE_MASTER);
}

/// @brief Drop an I2C master instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C master instance.
void smac_i2c_master_drop(smacI2c_t i2c)
{
    assert(i2c != NULL);

    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C master event callbacks for the specified I2C master instance.
/// @details This function sets the event callbacks for the specified I2C master instance.
smacRetCode_t smac_i2c_master_set_event(smacI2c_t i2c, smacI2cMasterEvent_t* event,
                                        smacMcuEventData_t data)
{
    assert(i2c != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C master event callbacks for the specified I2C master instance.
/// @details This function removes all event callbacks associated with the specified I2C master
/// instance.
void smac_i2c_master_clean_event(smacI2c_t i2c)
{
    assert(i2c != NULL);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Checks if the specified I2C slave device is ready for communication.
/// @details This function checks whether the specified I2C slave device associated with the given
/// I2C master instance is ready for communication.
smacRetCode_t smac_i2c_master_selected_device_in_ready_state(smacI2c_t i2c, uint16_t slave,
                                                             uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);

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

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

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

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

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

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmatx == NULL
                               ? HAL_I2C_Master_Transmit_IT(handle, slave, (uint8_t*)data, size)
                               : HAL_I2C_Master_Transmit_DMA(handle, slave, (uint8_t*)data, size));
}

/// @brief Asynchronously receive data over the specified I2C master instance.
/// @details This function initiates an asynchronous reception of data from the given address over
/// the I2C master instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_master_async_receive(smacI2c_t i2c, uint16_t slave, uint8_t* data,
                                            uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmarx == NULL
                               ? HAL_I2C_Master_Receive_IT(handle, slave, data, size)
                               : HAL_I2C_Master_Receive_DMA(handle, slave, data, size));
}

/// ===============================================================================================
/// @name I2C Slave Interface
/// @brief Implementation of I2C slave interface.
/// ===============================================================================================

/// @brief Create an I2C slave instance within the MCU abstraction layer.
/// @details This function creates an I2C slave instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacI2c_t smac_i2c_slave_create(void* handle)
{
    assert(handle != NULL);
    return (smacI2c_t)stm32_device_queue_allocate_with_addition(handle, STM32_I2C_ROLE_SLAVE);
}

/// @brief Drop an I2C slave instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C slave instance.
/// @param i2c The I2C slave instance to be dropped.
void smac_i2c_slave_drop(smacI2c_t i2c)
{
    assert(i2c != NULL);

    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C slave event callbacks for the specified I2C slave instance.
smacRetCode_t smac_i2c_slave_set_event(smacI2c_t i2c, smacI2cSlaveEvent_t* event,
                                       smacMcuEventData_t data)
{
    assert(i2c != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C slave event callbacks for the specified I2C slave instance.
void smac_i2c_slave_clean_event(smacI2c_t i2c)
{
    assert(i2c != NULL);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Listen for incoming communication on the specified I2C slave instance.
/// @details This function puts the I2C slave instance into a listening state, ready to respond to
/// master requests.
smacRetCode_t smac_i2c_slave_listen(smacI2c_t i2c)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_I2C_EnableListen_IT(device->handle));
}

/// @brief Transmit data over the specified I2C slave instance.
/// @details This function transmits the specified data to the given address over the I2C slave
/// instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_transmit(smacI2c_t i2c, const uint8_t* data, uint32_t size,
                                      uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_I2C_Slave_Transmit(device->handle, (uint8_t*)data, size, timeout));
}

/// @brief Receive data over the specified I2C slave instance.
/// @details This function receives data over the I2C slave instance within
/// the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_receive(smacI2c_t i2c, uint8_t* data, uint32_t size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    assert(device != NULL);
    assert(device->handle != NULL);

    return stm32_cast_code(HAL_I2C_Slave_Receive(device->handle, data, size, timeout));
}

/// @brief Asynchronously transmit data over the specified I2C slave instance.
/// @details This function initiates an asynchronous transmission of the specified data over the I2C
/// slave instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_async_transmit(smacI2c_t i2c, const uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmatx == NULL
                               ? HAL_I2C_Slave_Transmit_IT(device->handle, (uint8_t*)data, size)
                               : HAL_I2C_Slave_Transmit_DMA(handle, (uint8_t*)data, size));
}

/// @brief Asynchronously receive data over the specified I2C slave instance.
/// @details This function initiates an asynchronous reception of data over
/// the I2C slave instance within the MCU abstraction layer.
smacRetCode_t smac_i2c_slave_async_receive(smacI2c_t i2c, uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)i2c;
    I2C_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmarx == NULL ? HAL_I2C_Slave_Receive_IT(handle, data, size)
                                                  : HAL_I2C_Slave_Receive_DMA(handle, data, size));
}

/// ===============================================================================================
/// @name I2C Callback Implementations
/// @brief Implementation of I2C callback functions for handling various I2C events.
/// ===============================================================================================

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_master.tx_complete != NULL)
        {
            event->event->i2c_master.tx_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_master.rx_complete != NULL)
        {
            event->event->i2c_master.rx_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_slave.tx_complete != NULL)
        {
            event->event->i2c_slave.tx_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_slave.rx_complete != NULL)
        {
            event->event->i2c_slave.rx_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_slave.selected != NULL)
        {
            event->event->i2c_slave.selected((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t*)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_slave.listen_complete != NULL)
        {
            event->event->i2c_slave.listen_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_mem.write_complete != NULL)
        {
            event->event->i2c_mem.write_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->i2c_mem.read_complete != NULL)
        {
            event->event->i2c_mem.read_complete((smacI2c_t)event->device, event->event_data);
        }
    }
}

// void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & STM32_I2C_ROLE_MASK) == STM32_I2C_ROLE_MEM)
        {
            if (event->event->i2c_mem.error != NULL)
            {
                event->event->i2c_mem.error((smacI2c_t)event->device, event->event_data);
            }
        }
        else if ((event->device->addition & STM32_I2C_ROLE_MASK) == STM32_I2C_ROLE_MASTER)
        {
            if (event->event->i2c_master.error != NULL)
            {
                event->event->i2c_master.error((smacI2c_t)event->device, event->event_data);
            }
        }
        else if ((event->device->addition & STM32_I2C_ROLE_MASK) == STM32_I2C_ROLE_SLAVE)
        {
            if (event->event->i2c_slave.error != NULL)
            {
                event->event->i2c_slave.error((smacI2c_t)event->device, event->event_data);
            }
        }
    }
}
