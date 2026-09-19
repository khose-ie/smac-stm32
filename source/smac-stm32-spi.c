#include <assert.h>
#include <smac/middleware/smac-mcu.h>
#include <stm32.h>

#define STM32_SPI_ROLE_MASK   (0x80000000)
#define STM32_SPI_ROLE_MASTER (0x00000000)
#define STM32_SPI_ROLE_SLAVE  (0x80000000)

/// ===============================================================================================
/// @name SPI Master Interface
/// @brief Implementation of SPI Master interface.
/// ===============================================================================================

/// @brief Create an SPI Master instance within the MCU abstraction layer.
/// @details This function creates an SPI Master instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacSpi_t smac_spi_master_create(void* handle)
{
    assert(handle != NULL);
    return (smacSpi_t)stm32_device_queue_allocate_with_addition(handle, STM32_SPI_ROLE_MASTER);
}

/// @brief Drop an SPI Master instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified SPI Master instance.
void smac_spi_master_drop(smacSpi_t spi)
{
    assert(spi != NULL);

    stm32_device_queue_free((stm32Device_t*)spi);
    stm32_device_event_queue_free((stm32Device_t*)spi);
}

/// @brief Set SPI Master event callbacks for the specified SPI Master instance.
smacRetCode_t smac_spi_master_set_event(smacSpi_t spi, smacSpiMasterEvent_t* event,
                                        smacMcuEventData_t event_data)
{
    assert(spi != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)spi, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

/// @brief Clean SPI Master event callbacks for the specified SPI Master instance.
void smac_spi_master_clean_event(smacSpi_t spi)
{
    assert(spi != NULL);
    stm32_device_event_queue_free((stm32Device_t*)spi);
}

/// @brief Transmit data over the specified SPI Master instance.
/// @details This function transmits the specified data over the SPI Master instance within the MCU
/// abstraction layer.
smacRetCode_t smac_spi_master_transmit(smacSpi_t spi, smacIo_t nss, const uint8_t* data,
                                       uint32_t size, uint32_t timeout)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    stm32Device_t* nss_device = (stm32Device_t*)nss;

    smacRetCode_t code = SMAC_RET_OK;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(nss_device != NULL);
    assert(nss_device->handle != NULL);

    code = smac_io_set_state(nss, SMAC_IO_RST);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    code = stm32_cast_code(HAL_SPI_Transmit(device->handle, (uint8_t*)data, size, timeout));

    if (code != SMAC_RET_OK)
    {
        smac_io_set_state(nss, SMAC_IO_SET);
        return code;
    }

    return smac_io_set_state(nss, SMAC_IO_SET);
}

/// @brief Transmit and receive data over the specified SPI Master instance.
/// @details This function transmits and receives data over the SPI Master instance within the MCU
/// abstraction layer.
smacRetCode_t smac_spi_master_transmit_receive(smacSpi_t spi, smacIo_t nss, const uint8_t* tx_data,
                                               uint8_t* rx_data, uint32_t size, uint32_t timeout)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    stm32Device_t* nss_device = (stm32Device_t*)nss;

    smacRetCode_t code = SMAC_RET_OK;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(nss_device != NULL);
    assert(nss_device->handle != NULL);

    code = smac_io_set_state(nss, SMAC_IO_RST);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    code = stm32_cast_code(
        HAL_SPI_TransmitReceive(device->handle, (uint8_t*)tx_data, rx_data, size, timeout));

    if (code != SMAC_RET_OK)
    {
        smac_io_set_state(nss, SMAC_IO_SET);
        return code;
    }

    return smac_io_set_state(nss, SMAC_IO_SET);
}

/// @brief Asynchronously transmit data over the specified SPI Master instance.
/// @details This function initiates an asynchronous transmission of the specified data over the SPI
/// Master instance within the MCU abstraction layer.
smacRetCode_t smac_spi_master_async_transmit(smacSpi_t spi, smacIo_t nss, const uint8_t* data,
                                             uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    stm32Device_t* nss_device = (stm32Device_t*)nss;

    smacRetCode_t code        = SMAC_RET_OK;
    SPI_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(nss_device != NULL);
    assert(nss_device->handle != NULL);

    code = smac_io_set_state(nss, SMAC_IO_RST);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    code = stm32_cast_code(handle->hdmatx == NULL
                               ? HAL_SPI_Transmit_IT(handle, (uint8_t*)data, size)
                               : HAL_SPI_Transmit_DMA(handle, (uint8_t*)data, size));

    if (code != SMAC_RET_OK)
    {
        smac_io_set_state(nss, SMAC_IO_SET);
        return code;
    }

    return smac_io_set_state(nss, SMAC_IO_SET);
}

/// @brief Asynchronously transmit and receive data over the specified SPI Master instance.
/// @details This function initiates an asynchronous transmission and reception of data over the SPI
/// Master instance within the MCU abstraction layer.
smacRetCode_t smac_spi_master_async_transmit_receive(smacSpi_t spi, smacIo_t nss,
                                                     const uint8_t* tx_data, uint8_t* rx_data,
                                                     uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    stm32Device_t* nss_device = (stm32Device_t*)nss;

    smacRetCode_t code        = SMAC_RET_OK;
    SPI_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(nss_device != NULL);
    assert(nss_device->handle != NULL);

    code = smac_io_set_state(nss, SMAC_IO_RST);

    if (code != SMAC_RET_OK)
    {
        return code;
    }

    code = stm32_cast_code(
        handle->hdmatx == NULL
            ? HAL_SPI_TransmitReceive_IT(handle, (uint8_t*)tx_data, rx_data, size)
            : HAL_SPI_TransmitReceive_DMA(handle, (uint8_t*)tx_data, rx_data, size));

    if (code != SMAC_RET_OK)
    {
        smac_io_set_state(nss, SMAC_IO_SET);
        return code;
    }

    return smac_io_set_state(nss, SMAC_IO_SET);
}

/// ===============================================================================================
/// @name SPI Slave Interface
/// @brief Implementation of SPI Slave interface.
/// ===============================================================================================

/// @brief Create an SPI Slave instance within the MCU abstraction layer.
/// @details This function creates an SPI Slave instance within the MCU abstraction layer,
/// associating it with the provided handle.
smacSpi_t smac_spi_slave_create(void* handle)
{
    assert(handle != NULL);
    return (smacSpi_t)stm32_device_queue_allocate_with_addition(handle, STM32_SPI_ROLE_SLAVE);
}

/// @brief Drop an SPI Slave instance within the MCU abstraction layer.
/// @details This function releases the resources associated with the specified SPI Slave instance.
void smac_spi_slave_drop(smacSpi_t spi)
{
    assert(spi != NULL);

    stm32_device_queue_free((stm32Device_t*)spi);
    stm32_device_event_queue_free((stm32Device_t*)spi);
}

/// @brief Set SPI Slave event callbacks for the specified SPI Slave instance.
smacRetCode_t smac_spi_slave_set_event(smacSpi_t spi, smacSpiSlaveEvent_t* event,
                                       smacMcuEventData_t event_data)
{
    assert(spi != NULL);
    assert(event != NULL);

    return stm32_device_event_queue_allocate((stm32Device_t*)spi, (stm32DeviceEventHandle_t*)event,
                                             event_data);
}

/// @brief Clean SPI Slave event callbacks for the specified SPI Slave instance.
void smac_spi_slave_clean_event(smacSpi_t spi)
{
    assert(spi != NULL);
    stm32_device_event_queue_free((stm32Device_t*)spi);
}

/// @brief Transmit data over the specified SPI Slave instance.
/// @details This function transmits the specified data over the SPI Slave instance within the MCU
/// abstraction layer.
smacRetCode_t smac_spi_slave_transmit(smacSpi_t spi, const uint8_t* data, uint32_t size,
                                      uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)spi;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(HAL_SPI_Transmit(device->handle, (uint8_t*)data, size, timeout));
}

/// @brief Receive data over the specified SPI Slave instance.
/// @details This function receives data over the SPI Slave instance within the MCU abstraction
/// layer.
smacRetCode_t smac_spi_slave_receive(smacSpi_t spi, uint8_t* data, uint32_t size, uint32_t timeout)
{
    stm32Device_t* device = (stm32Device_t*)spi;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(HAL_SPI_Receive(device->handle, data, size, timeout));
}

/// @brief Asynchronously transmit data over the specified SPI Slave instance.
/// @details This function initiates an asynchronous transmission of the specified data over the SPI
/// Slave instance within the MCU abstraction layer.
smacRetCode_t smac_spi_slave_async_transmit(smacSpi_t spi, const uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    SPI_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmatx == NULL
                               ? HAL_SPI_Transmit_IT(handle, (uint8_t*)data, size)
                               : HAL_SPI_Transmit_DMA(handle, (uint8_t*)data, size));
}

/// @brief Asynchronously receive data over the specified SPI Slave instance.
/// @details This function initiates an asynchronous reception of data over the SPI Slave instance
/// within the MCU abstraction layer.
smacRetCode_t smac_spi_slave_async_receive(smacSpi_t spi, uint8_t* data, uint32_t size)
{
    stm32Device_t* device     = (stm32Device_t*)spi;
    SPI_HandleTypeDef* handle = device->handle;

    assert(device != NULL);
    assert(device->handle != NULL);
    assert(data != NULL);

    return stm32_cast_code(handle->hdmarx == NULL ? HAL_SPI_Receive_IT(handle, data, size)
                                                  : HAL_SPI_Receive_DMA(handle, data, size));
}

/// ===============================================================================================
/// @name SPI Callback Implementations
/// @brief Implementation of SPI callback functions for handling various SPI events.
/// ===============================================================================================

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search(hspi);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_MASTER)
        {
            if (event->event->spi_master.tx_complete != NULL)
            {
                event->event->spi_master.tx_complete(event->device, event->event_data);
            }
        }
        else if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_SLAVE)
        {
            if (event->event->spi_slave.tx_complete != NULL)
            {
                event->event->spi_slave.tx_complete(event->device, event->event_data);
            }
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search(hspi);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->spi_slave.rx_complete != NULL)
        {
            event->event->spi_slave.rx_complete(event->device, event->event_data);
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search(hspi);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if (event->event->spi_master.tx_rx_complete != NULL)
        {
            event->event->spi_master.tx_rx_complete(event->device, event->event_data);
        }
    }
}

// void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef* hspi) {}

// void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef* hspi) {}

// void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef* hspi) {}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search(hspi);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_MASTER)
        {
            if (event->event->spi_master.error != NULL)
            {
                event->event->spi_master.error(event->device, event->event_data);
            }
        }
        else if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_SLAVE)
        {
            if (event->event->spi_slave.error != NULL)
            {
                event->event->spi_slave.error(event->device, event->event_data);
            }
        }
    }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi)
{
    stm32DeviceEvent_t* event = stm32_device_event_queue_search(hspi);

    if (event != NULL)
    {
        assert(event->device != NULL);
        assert(event->event != NULL);

        if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_MASTER)
        {
            if (event->event->spi_master.abort_complete != NULL)
            {
                event->event->spi_master.abort_complete(event->device, event->event_data);
            }
        }
        else if ((event->device->addition & STM32_SPI_ROLE_MASK) == STM32_SPI_ROLE_SLAVE)
        {
            if (event->event->spi_slave.abort_complete != NULL)
            {
                event->event->spi_slave.abort_complete(event->device, event->event_data);
            }
        }
    }
}
