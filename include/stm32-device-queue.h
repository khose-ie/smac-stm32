#ifndef _STM32_DEVICE_QUEUE_H_
#define _STM32_DEVICE_QUEUE_H_

#include <smac-mcu.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef void* stm32DeviceHandle;

typedef uint32_t stm32DeviceAddition;

typedef void* stm32DeviceEventData;

typedef uintptr_t stm32DeviceCacheData;

typedef struct
{
    stm32DeviceHandle handle;
    stm32DeviceAddition addition;
} stm32Device;

typedef struct
{
    stm32Device* device;
    stm32DeviceEventData event_data;
} stm32DeviceEvent;

typedef struct
{
    stm32Device* device;
    stm32DeviceCacheData cache_data;
} stm32DeviceCache;

void stm32_device_queue_initialize(void);

stm32Device* stm32_device_queue_allocate(stm32DeviceHandle handle, stm32DeviceAddition addition);

void stm32_device_queue_free(stm32Device* device);

void stm32_device_event_queue_initialize(void);

smacRetCode_t stm32_device_event_queue_allocate(stm32Device* device,
                                                stm32DeviceEventData event_data);

void stm32_device_event_queue_free(stm32Device* device);

stm32DeviceEvent* stm32_device_event_queue_search(stm32DeviceHandle handle);

stm32DeviceEvent* stm32_device_event_queue_search_with_addition(stm32DeviceHandle handle,
                                                                stm32DeviceAddition addition);

smacRetCode_t stm32_device_cache_queue_allocate(stm32Device* device,
                                                stm32DeviceCacheData cache_data);

void stm32_device_cache_queue_free(stm32Device* device);

smacRetCode_t stm32_device_cache_queue_set_cache(stm32Device* device,
                                                 stm32DeviceCacheData cache_data);

stm32DeviceCache* stm32_device_cache_queue_search(stm32DeviceHandle handle);

stm32DeviceCache* stm32_device_cache_queue_search_with_addition(stm32DeviceHandle handle,
                                                                stm32DeviceAddition addition);
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _STM32_DEVICE_QUEUE_H_
