#ifndef _STM32_H_
#define _STM32_H_

#include <smac/configuration/smac-stm32.h>
#include <smac/middleware/smac-mcu.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define STM32_DEVICE_CACHE_DATA_MAX (3)

/// @brief Handle type for STM32 devices.
/// @details This type represents a generic handle to an STM32 device within the abstraction layer.
typedef void* stm32DeviceHandle_t;

/// @brief Addition type for STM32 devices.
/// @details This type represents additional information or attributes associated with an STM32
/// device within the abstraction layer.
typedef uint32_t stm32DeviceAddition_t;

typedef union
{
    smacAdcEvent_t adc;
    smacCanEvent_t can;
    smacCanFdEvent_t can_fd;
    smacI2cMasterEvent_t i2c_master;
    smacI2cSlaveEvent_t i2c_slave;
    smacI2cMemEvent_t i2c_mem;
    smacIoEvent_t io;
    smacPwmEvent_t pwm;
    smacSpiMasterEvent_t spi_master;
    smacSpiSlaveEvent_t spi_slave;
    smacTimEvent_t tim;
    smacUartEvent_t uart;
} stm32DeviceEventHandle_t;

/// @brief Event data type for STM32 devices.
/// @details This type represents the event data associated with an STM32 device event within the
/// abstraction layer.
typedef smacMcuEventData_t stm32DeviceEventData_t;

/// @brief Cache data type for STM32 devices.
/// @details This type represents the cache data associated with an STM32 device within the
/// abstraction layer.
typedef uintptr_t stm32DeviceCacheData_t;

/// @brief Structure representing an STM32 device.
/// @details This structure contains the handle and addition information for an STM32 device within
/// the abstraction layer.
typedef struct
{
    stm32DeviceHandle_t handle;
    stm32DeviceAddition_t addition;
} stm32Device_t;

/// @brief Structure representing an STM32 device event.
/// @details This structure contains the device and event data associated with an STM32 device event
/// within the abstraction layer.
typedef struct
{
    stm32Device_t* device;
    stm32DeviceEventHandle_t* event;
    stm32DeviceEventData_t event_data;
} stm32DeviceEvent_t;

/// @brief Structure representing an STM32 device cache.
/// @details This structure contains the device and cache data associated with an STM32 device
/// within the abstraction layer.
typedef struct
{
    stm32Device_t* device;
    stm32DeviceCacheData_t cache_data[STM32_DEVICE_CACHE_DATA_MAX];
} stm32DeviceCache_t;

/// @brief Initialize the device queue with the specified number of devices.
/// @param device_queue Pointer to the device queue to initialize.
/// @param num Number of devices in the queue.
void stm32_device_queue_initialize(void);

/// @brief Allocate a device from the device queue.
/// @param handle Handle of the device to allocate.
/// @return Pointer to the allocated device, or NULL if allocation failed.
stm32Device_t* stm32_device_queue_allocate(stm32DeviceHandle_t handle);

/// @brief Allocate a device from the device queue with addition information.
/// @param handle Handle of the device to allocate.
/// @param addition Addition information for the device.
/// @return Pointer to the allocated device, or NULL if allocation failed.
stm32Device_t* stm32_device_queue_allocate_with_addition(stm32DeviceHandle_t handle,
                                                         stm32DeviceAddition_t addition);

/// @brief Free the specified device from the device queue.
/// @param device Pointer to the device to free.
void stm32_device_queue_free(stm32Device_t* device);

/// @brief Initialize the device event queue with the specified number of events.
void stm32_device_event_queue_initialize(void);

/// @brief Allocate an event from the device event queue.
/// @param device Pointer to the device associated with the event.
/// @param event_handle Pointer to the handle of the allocated event.
/// @param event_data Data associated with the event.
/// @return SMAC_RET_OK if allocation was successful, otherwise an error code.
smacRetCode_t stm32_device_event_queue_allocate(stm32Device_t* device,
                                                stm32DeviceEventHandle_t* event_handle,
                                                stm32DeviceEventData_t event_data);

/// @brief Free the specified event from the device event queue.
/// @param device Pointer to the device associated with the event.
void stm32_device_event_queue_free(stm32Device_t* device);

/// @brief Search for an event in the device event queue by device handle.
/// @param handle Handle of the device associated with the event.
/// @return Pointer to the found event, or NULL if not found.
stm32DeviceEvent_t* stm32_device_event_queue_search(stm32DeviceHandle_t handle);

/// @brief Search for an event in the device event queue by device handle and addition information.
/// @param handle Handle of the device associated with the event.
/// @param addition Addition information for the device.
/// @return Pointer to the found event, or NULL if not found.
stm32DeviceEvent_t* stm32_device_event_queue_search_with_addition(stm32DeviceHandle_t handle,
                                                                  stm32DeviceAddition_t addition);

/// @brief Initialize the device cache queue with the specified number of cache entries.
void stm32_device_cache_queue_initialize(void);

/// @brief Allocate a cache entry from the device cache queue.
/// @param device Pointer to the device associated with the cache entry.
/// @return SMAC_RET_OK if allocation was successful, otherwise an error code.
smacRetCode_t stm32_device_cache_queue_allocate(stm32Device_t* device);

/// @brief Free a cache entry from the device cache queue.
/// @param device Pointer to the device associated with the cache entry.
void stm32_device_cache_queue_free(stm32Device_t* device);

/// @brief Set the cache data for a specific cache entry in the device cache queue.
/// @param device Pointer to the device associated with the cache entry.
/// @param index Index of the cache entry to set.
/// @param cache_data Data to set for the cache entry.
/// @return SMAC_RET_OK if the operation was successful, otherwise an error code.
smacRetCode_t stm32_device_cache_queue_set_cache(stm32Device_t* device, uint32_t index,
                                                 stm32DeviceCacheData_t cache_data);

/// @brief Search for a cache entry in the device cache queue by device handle.
/// @param handle Handle of the device associated with the cache entry.
/// @return Pointer to the found cache entry, or NULL if not found.
stm32DeviceCache_t* stm32_device_cache_queue_search(stm32DeviceHandle_t handle);

/// @brief Search for a cache entry in the device cache queue by device handle and addition
/// information.
/// @param handle Handle of the device associated with the cache entry.
/// @param addition Addition information for the device.
/// @return Pointer to the found cache entry, or NULL if not found.
stm32DeviceCache_t* stm32_device_cache_queue_search_with_addition(stm32DeviceHandle_t handle,
                                                                  stm32DeviceAddition_t addition);

/// @brief Cast a HAL status code to a SMAC return code.
/// @param hal_status The HAL status code to be cast.
/// @return The corresponding SMAC return code.
smacRetCode_t stm32_cast_code(HAL_StatusTypeDef hal_status);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _STM32_H_
