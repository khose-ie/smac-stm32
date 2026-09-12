#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32-i2c.h>
#include <stm32.h>

#define STM32_I2C_MAX_TIMEOUT (500)

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
    return (smacI2c_t)stm32_device_queue_allocate(handle, STM32_I2C_ROLE_MEM);
}

/// @brief Drop an I2C memory instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified I2C memory instance.
void smac_i2c_mem_drop(smacI2c_t i2c)
{
    stm32_device_queue_free((stm32Device_t*)i2c);
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Set I2C memory event callbacks for the specified I2C memory instance.
/// @param i2c The I2C memory instance.
smacRetCode_t smac_i2c_mem_set_event(smacI2c_t i2c, smacI2cMemEvent_t* event,
                                     smacMcuEventData_t data)
{
    return stm32_device_event_queue_allocate((stm32Device_t*)i2c, (stm32DeviceEventHandle_t*)event,
                                             data);
}

/// @brief Clean I2C memory event callbacks for the specified I2C memory instance.
void smac_i2c_mem_clean_event(smacI2c_t i2c)
{
    stm32_device_event_queue_free((stm32Device_t*)i2c);
}

/// @brief Check if the I2C memory device is in a ready state.
/// @details This function checks whether the I2C memory device associated with the specified I2C
/// instance is ready for communication.
smacRetCode_t smac_i2c_mem_selected_device_in_ready_state(smacI2c_t i2c, uint16_t address,
                                                          uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    if (device == NULL)
    {
        return SMAC_RET_NULL_REF;
    }

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

    smacRetCode_t code = smac_i2c_mem_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_I2C_Mem_Write(device->handle, slave, mem_addr,
                                                   stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                                   (uint8_t*)data, size, timeout))
               : SMAC_RET_NULL_REF;
}

/// @brief Read data from the specified I2C memory device.
/// @details This function reads data from the given memory address of the I2C memory device
/// associated with the specified I2C instance.
smacRetCode_t smac_i2c_mem_read(smacI2c_t i2c, uint16_t slave, uint16_t mem_addr,
                                smacI2cMemAddrSize mem_addr_size, uint8_t* data, uint16_t size,
                                uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)i2c;

    smacRetCode_t code = smac_i2c_mem_selected_device_in_ready_state(i2c, slave, timeout);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(HAL_I2C_Mem_Read(device->handle, slave, mem_addr,
                                                  stm32_cast_i2c_mem_addr_size(mem_addr_size), data,
                                                  size, timeout))
               : SMAC_RET_NULL_REF;
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

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(
                     handle->hdmatx == NULL
                         ? HAL_I2C_Mem_Write_IT(handle, slave, mem_addr,
                                                stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                                (uint8_t*)data, size)
                         : HAL_I2C_Mem_Write_DMA(handle, slave, mem_addr,
                                                 stm32_cast_i2c_mem_addr_size(mem_addr_size),
                                                 (uint8_t*)data, size))
               : SMAC_RET_NULL_REF;
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

    return (device != NULL) && (device->handle != NULL)
               ? stm32_cast_code(handle->hdmarx == NULL
                                     ? HAL_I2C_Mem_Read_IT(
                                           handle, slave, mem_addr,
                                           stm32_cast_i2c_mem_addr_size(mem_addr_size), data, size)
                                     : HAL_I2C_Mem_Read_DMA(
                                           handle, slave, mem_addr,
                                           stm32_cast_i2c_mem_addr_size(mem_addr_size), data, size))
               : SMAC_RET_NULL_REF;
}

/// ===============================================================================================
/// @name I2C Mem Callback Implementations
/// @brief Implementation of I2C memory callback functions for handling various I2C memory events.
/// ===============================================================================================

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_mem.write_complete != NULL))
    {
        event->event->i2c_mem.write_complete((smacI2c_t)event->device, event->event_data);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if ((event != NULL) && (event->event != NULL) && (event->event->i2c_mem.read_complete != NULL))
    {
        event->event->i2c_mem.read_complete((smacI2c_t)event->device, event->event_data);
    }
}

// void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search((stm32DeviceHandle_t)hi2c);

    if ((event != NULL) && (event->event != NULL))
    {
        if (((event->device->addition & STM32_I2C_ROLE_MARK) == STM32_I2C_ROLE_MEM) &&
            (event->event->i2c_mem.error != NULL))
        {
            event->event->i2c_mem.error((smacI2c_t)event->device, event->event_data);
        }
        else if (((event->device->addition & STM32_I2C_ROLE_MARK) == STM32_I2C_ROLE_MASTER) &&
                 (event->event->i2c_master.error != NULL))
        {
            event->event->i2c_master.error((smacI2c_t)event->device, event->event_data);
        }
        else if (((event->device->addition & STM32_I2C_ROLE_MARK) == STM32_I2C_ROLE_SLAVE) &&
                 (event->event->i2c_slave.error != NULL))
        {
            event->event->i2c_slave.error((smacI2c_t)event->device, event->event_data);
        }
    }
}
