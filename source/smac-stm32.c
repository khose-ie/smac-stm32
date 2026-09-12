

#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stm32.h>
#include <string.h>

#ifndef SMAC_STM32_PERIPH_NUM
#define SMAC_STM32_PERIPH_NUM (8)
#endif // SMAC_STM32_PERIPH_NUM

#ifndef SMAC_STM32_EVENTABLE_PERIPH_NUM
#define SMAC_STM32_EVENTABLE_PERIPH_NUM (0)
#endif // SMAC_STM32_EVENTABLE_PERIPH_NUM

#ifndef SMAC_STM32_CACHEABLE_PERIPH_NUM
#define SMAC_STM32_CACHEABLE_PERIPH_NUM (0)
#endif // SMAC_STM32_CACHEABLE_PERIPH_NUM

#ifndef SMAC_STM32_STACK_EX_MEM_SIZE
#define SMAC_STM32_STACK_EX_MEM_SIZE (0)
#endif // SMAC_STM32_STACK_EX_MEM_SIZE

/// @brief Calculate the required stack size for the STM32 device queues.
/// @details This macro calculates the total stack size needed for the device queue, device event
/// queue, and device cache queue based on the configured number of peripherals for each type.
#define STM32_STACK_REQ_SIZE                                                                       \
    (SMAC_STM32_PERIPH_NUM * sizeof(stm32Device_t) +                                               \
     SMAC_STM32_EVENTABLE_PERIPH_NUM * sizeof(stm32DeviceEvent_t) +                                \
     SMAC_STM32_CACHEABLE_PERIPH_NUM * sizeof(stm32DeviceCache_t))

/// @brief Define the stack memory for the STM32 device queues.
/// @details This section defines the stack memory used for the STM32 device queues. If an external
/// stack memory is not provided, it uses a static array. Otherwise, it uses the externally defined
/// stack memory.
#ifndef SMAC_STM32_STACK_EX_MEM
#define EXT              static
#define STM32_STACK      (mcu_stack)
#define STM32_STACK_SIZE (STM32_STACK_REQ_SIZE)
#else // SMAC_STM32_STACK_EX_MEM
#define EXT              extern
#define STM32_STACK      (SMAC_STM32_STACK_EX_MEM)
#define STM32_STACK_SIZE (SMAC_STM32_STACK_EX_MEM_SIZE)
#endif // SMAC_STM32_STACK_EX_MEM

/// @brief Declare the stack memory for the STM32 device queues.
/// @details This declaration uses the `EXT` macro to determine whether the stack memory is defined
/// as static or extern, based on the configuration. The `STM32_STACK` and `STM32_STACK_SIZE` macros
/// are used to specify the stack memory and its size.
EXT uint8_t STM32_STACK[STM32_STACK_SIZE];

/// @brief Define the structure representing the STM32 device stack.
/// @details This structure contains the device queue, device event queue, and device cache queue
/// for the STM32 platform. It is used to organize the memory layout of the STM32 device queues
/// within the stack memory.
typedef struct
{
    stm32Device_t device_queue[SMAC_STM32_PERIPH_NUM];
    stm32DeviceEvent_t device_event_queue[SMAC_STM32_EVENTABLE_PERIPH_NUM];
    stm32DeviceCache_t device_cache_queue[SMAC_STM32_CACHEABLE_PERIPH_NUM];
} stm32Stack_t;

/// @brief Get a pointer to the STM32 device stack.
/// @details This macro casts the stack memory to a pointer to the `stm32Stack_t` structure,
/// allowing access to the device queue, device event queue, and device cache queue within the stack
/// memory.
#define stm32_stack() ((stm32Stack_t*)STM32_STACK)

/// @brief Initialize the SMAC MCU and the STM32 device queues.
/// @details This function initializes the device queue, device event queue, and device cache queue
/// for the STM32 platform. It should be called before using any other SMAC STM32 functions.
smacRetCode_t smac_mcu_initialize(void)
{
    stm32_device_queue_initialize();
    stm32_device_event_queue_initialize();
    stm32_device_cache_queue_initialize();
    return SMAC_RET_OK;
}

/// @brief Introduce a delay for the specified number of milliseconds.
/// @details This function introduces a blocking delay for the specified number of milliseconds.
void smac_mcu_delay(uint32_t milliseconds)
{
    HAL_Delay(milliseconds);
}

/// @brief Initialize the device queue with the specified number of devices.
/// @details This function initializes the device queue by setting all device entries to zero.
void stm32_device_queue_initialize(void)
{
    memset(stm32_stack()->device_queue, 0, SMAC_STM32_PERIPH_NUM * sizeof(stm32Device_t));
}

/// @brief Allocate a device from the device queue.
/// @details This function first searches for an existing device with the specified handle and
/// addition. If found, it returns the existing device. If not found, it allocates a new device
/// entry in the queue with the specified handle and addition, and returns it. If the queue is full,
/// it returns NULL.
stm32Device_t* stm32_device_queue_allocate(stm32DeviceHandle_t handle,
                                           stm32DeviceAddition_t addition)
{
    if (handle == NULL)
    {
        return NULL;
    }

    for (stm32Device_t* device = stm32_stack()->device_queue;
         device < stm32_stack()->device_queue + SMAC_STM32_PERIPH_NUM; device++)
    {
        if ((device->handle == handle) && (device->addition == addition))
        {
            return device;
        }
    }

    for (stm32Device_t* device = stm32_stack()->device_queue;
         device < stm32_stack()->device_queue + SMAC_STM32_PERIPH_NUM; device++)
    {
        if (device->handle == NULL)
        {
            device->handle   = handle;
            device->addition = addition;
            return device;
        }
    }

    return NULL;
}

/// @brief Free a device from the device queue.
/// @details This function frees a device from the device queue by setting its handle to NULL and
/// its addition to 0.
void stm32_device_queue_free(stm32Device_t* device)
{
    if (device != NULL)
    {
        device->handle   = NULL;
        device->addition = 0;
    }
}

/// @brief Initialize the device event queue with the specified number of events.
/// @details This function initializes the device event queue by setting all event entries to zero.
void stm32_device_event_queue_initialize(void)
{
    memset(stm32_stack()->device_event_queue, 0,
           SMAC_STM32_EVENTABLE_PERIPH_NUM * sizeof(stm32DeviceEvent_t));
}

/// @brief Allocate an event from the device event queue.
/// @details This function first searches for an existing event associated with the specified
/// device. If found, it updates the event data and returns SMAC_RET_OK. If not found, it allocates
/// a new event entry in the queue with the specified device, event handle, and event data, and
/// returns SMAC_RET_OK. If the queue is full, it returns SMAC_RET_STACK_OVERFLOW.
smacRetCode_t stm32_device_event_queue_allocate(stm32Device_t* device,
                                                stm32DeviceEventHandle_t* event_handle,
                                                stm32DeviceEventData_t event_data)
{
    if ((device == NULL) || (event_handle == NULL))
    {
        return SMAC_RET_PARAM_ERR;
    }

    for (stm32DeviceEvent_t* event = stm32_stack()->device_event_queue;
         event < stm32_stack()->device_event_queue + SMAC_STM32_EVENTABLE_PERIPH_NUM; event++)
    {
        if (event->device == device)
        {
            event->event_data = event_data;
            return SMAC_RET_OK;
        }
    }

    for (stm32DeviceEvent_t* event = stm32_stack()->device_event_queue;
         event < stm32_stack()->device_event_queue + SMAC_STM32_EVENTABLE_PERIPH_NUM; event++)
    {
        if (event->device == NULL)
        {
            event->device     = device;
            event->event      = event_handle;
            event->event_data = event_data;
            return SMAC_RET_OK;
        }
    }

    return SMAC_RET_STACK_OVERFLOW;
}

/// @brief Free an event from the device event queue.
/// @details This function searches for an event associated with the specified device and frees it
/// by setting the device pointer and event data to NULL.
void stm32_device_event_queue_free(stm32Device_t* device)
{
    if (device != NULL)
    {
        stm32DeviceEvent_t* event =
            stm32_device_event_queue_search_with_addition(device->handle, device->addition);

        if (event != NULL)
        {
            event->device     = NULL;
            event->event_data = NULL;
        }
    }
}

/// @brief Search for an event in the device event queue by device handle.
/// @details This function searches for an event associated with the specified device handle in the
/// device event queue. It returns a pointer to the found event, or NULL if not found.
stm32DeviceEvent_t* stm32_device_event_queue_search(stm32DeviceHandle_t handle)
{
    return stm32_device_event_queue_search_with_addition(handle, 0);
}

/// @brief Search for an event in the device event queue by device handle and addition information.
/// @details This function searches for an event associated with the specified device handle and
/// addition information in the device event queue. It returns a pointer to the found event, or NULL
/// if not found.
stm32DeviceEvent_t* stm32_device_event_queue_search_with_addition(stm32DeviceHandle_t handle,
                                                                  stm32DeviceAddition_t addition)
{
    if (handle == NULL)
    {
        return NULL;
    }

    for (stm32DeviceEvent_t* event = stm32_stack()->device_event_queue;
         event < stm32_stack()->device_event_queue + SMAC_STM32_EVENTABLE_PERIPH_NUM; event++)
    {
        if (event->device != NULL)
        {
            if (((event->device->handle == handle) && (event->device->addition == addition)) ||
                ((handle == NULL) && (addition != 0) && (event->device->addition == addition)))
            {
                return event;
            }
        }
    }

    return NULL;
}

/// @brief Initialize the device cache queue.
/// @details This function initializes the device cache queue by setting all entries to zero.
void stm32_device_cache_queue_initialize(void)
{
    memset(stm32_stack()->device_cache_queue, 0,
           SMAC_STM32_CACHEABLE_PERIPH_NUM * sizeof(stm32DeviceCache_t));
}

/// @brief Allocate a cache entry for a device in the device cache queue.
/// @details This function searches for an existing cache entry for the specified device. If found,
/// it returns SMAC_RET_OK. If not found, it allocates a new cache entry for the device and
/// initializes it. If the cache queue is full, it returns SMAC_RET_STACK_OVERFLOW.
smacRetCode_t stm32_device_cache_queue_allocate(stm32Device_t* device)
{
    if (device == NULL)
    {
        return SMAC_RET_PARAM_ERR;
    }

    for (stm32DeviceCache_t* cache = stm32_stack()->device_cache_queue;
         cache < stm32_stack()->device_cache_queue + SMAC_STM32_CACHEABLE_PERIPH_NUM; cache++)
    {
        if (cache->device == device)
        {
            // The cache for this device has already been allocated, no need to reinitialize it.
            return SMAC_RET_OK;
        }
    }

    for (stm32DeviceCache_t* cache = stm32_stack()->device_cache_queue;
         cache < stm32_stack()->device_cache_queue + SMAC_STM32_CACHEABLE_PERIPH_NUM; cache++)
    {
        if (cache->device == NULL)
        {
            cache->device = device;
            memset(cache->cache_data, 0,
                   STM32_DEVICE_CACHE_DATA_MAX * sizeof(stm32DeviceCacheData_t));
            return SMAC_RET_OK;
        }
    }

    return SMAC_RET_STACK_OVERFLOW;
}

/// @brief Free a cache entry for a device in the device cache queue.
/// @details This function searches for the cache entry corresponding to the specified device and
/// frees it by setting the device pointer to NULL and clearing the cache data.
void stm32_device_cache_queue_free(stm32Device_t* device)
{
    stm32DeviceCache_t* cache =
        stm32_device_cache_queue_search_with_addition(device->handle, device->addition);

    if (cache != NULL)
    {
        cache->device = NULL;
        memset(cache->cache_data, 0, STM32_DEVICE_CACHE_DATA_MAX * sizeof(stm32DeviceCacheData_t));
    }
}

/// @brief Set the cache data for a device in the device cache queue.
/// @details This function searches for the cache entry corresponding to the specified device and
/// sets the cache data at the specified index. If the device is not found in the cache queue, it
/// returns SMAC_RET_INSTANCE_NOT_FOUND.
smacRetCode_t stm32_device_cache_queue_set_cache(stm32Device_t* device, uint32_t index,
                                                 stm32DeviceCacheData_t cache_data)
{
    if ((device == NULL) || (index >= STM32_DEVICE_CACHE_DATA_MAX))
    {
        return SMAC_RET_PARAM_ERR;
    }

    for (stm32DeviceCache_t* cache = stm32_stack()->device_cache_queue;
         cache < stm32_stack()->device_cache_queue + SMAC_STM32_CACHEABLE_PERIPH_NUM; cache++)
    {
        if (cache->device == device)
        {
            cache->cache_data[index] = cache_data;
            return SMAC_RET_OK;
        }
    }

    return SMAC_RET_INSTANCE_NOT_FOUND;
}

/// @brief Search for a cache entry for a device in the device cache queue.
/// @details This function searches for the cache entry corresponding to the specified device
/// handle. If the device is not found, it returns NULL.
stm32DeviceCache_t* stm32_device_cache_queue_search(stm32DeviceHandle_t handle)
{
    return stm32_device_cache_queue_search_with_addition(handle, 0);
}

/// @brief Search for a cache entry for a device in the device cache queue with a specific addition.
/// @details This function searches for the cache entry corresponding to the specified device handle
/// and addition. If the device is not found, it returns NULL.
stm32DeviceCache_t* stm32_device_cache_queue_search_with_addition(stm32DeviceHandle_t handle,
                                                                  stm32DeviceAddition_t addition)
{
    if (handle == NULL)
    {
        return NULL;
    }

    for (stm32DeviceCache_t* cache = stm32_stack()->device_cache_queue;
         cache < stm32_stack()->device_cache_queue + SMAC_STM32_CACHEABLE_PERIPH_NUM; cache++)
    {
        if (cache->device != NULL)
        {
            if (((cache->device->handle == handle) && (cache->device->addition == addition)) ||
                ((handle == NULL) && (addition != 0) && (cache->device->addition == addition)))
            {
                return cache;
            }
        }
    }

    return NULL;
}

smacRetCode_t stm32_cast_code(HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
        case HAL_OK:
            return SMAC_RET_OK;
        case HAL_ERROR:
            return SMAC_RET_LOW_LEVEL_FAILURE;
        case HAL_BUSY:
            return SMAC_RET_BUSY;
        case HAL_TIMEOUT:
            return SMAC_RET_TIMEOUT;
        default:
            return SMAC_RET_UNKNOWN;
    }
}
