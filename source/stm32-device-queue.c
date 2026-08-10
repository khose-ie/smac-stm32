#include <smac-stm32.h>
#include <stm32-device-queue.h>
#include <string.h>

static stm32Device _device_queue[SMAC_STM32_PERIPH_NUM];

static stm32DeviceEvent _device_event_queue[SMAC_STM32_EVENTABLE_PERIPH_NUM];

void stm32_device_queue_initialize(void)
{
    memset(_device_queue, 0, sizeof(_device_queue));
}

stm32Device* stm32_device_queue_allocate(stm32DeviceHandle handle, stm32DeviceAddition addition)
{
    for (int i = 0; i < SMAC_STM32_PERIPH_NUM; i++)
    {
        if ((_device_queue[i].handle == handle) && (_device_queue[i].addition == addition))
        {
            return &_device_queue[i];
        }
    }

    for (int i = 0; i < SMAC_STM32_PERIPH_NUM; i++)
    {
        if (_device_queue[i].handle == NULL)
        {
            _device_queue[i].handle   = handle;
            _device_queue[i].addition = addition;
            return &_device_queue[i];
        }
    }

    return NULL;
}

void stm32_device_queue_free(stm32Device* device)
{
    if (device != NULL)
    {
        device->handle   = NULL;
        device->addition = 0;
    }
}

void stm32_device_event_queue_initialize(void)
{
    memset(_device_event_queue, 0, sizeof(_device_event_queue));
}

smacRetCode_t stm32_device_event_queue_allocate(stm32Device* device,
                                                stm32DeviceEventData event_data)
{
    if ((device == NULL) || (event_data == NULL))
    {
        return SMAC_RET_PARAM_ERR;
    }

    for (int i = 0; i < SMAC_STM32_EVENTABLE_PERIPH_NUM; i++)
    {
        if (_device_event_queue[i].device == device)
        {
            _device_event_queue[i].event_data = event_data;
            return SMAC_RET_OK;
        }
    }

    for (int i = 0; i < SMAC_STM32_EVENTABLE_PERIPH_NUM; i++)
    {
        if (_device_event_queue[i].device == NULL)
        {
            _device_event_queue[i].device     = device;
            _device_event_queue[i].event_data = event_data;
            return SMAC_RET_OK;
        }
    }

    return SMAC_RET_STACK_OVERFLOW;
}

void stm32_device_event_queue_free(stm32Device* device)
{
    if (device != NULL)
    {
        for (int i = 0; i < SMAC_STM32_EVENTABLE_PERIPH_NUM; i++)
        {
            if (_device_event_queue[i].device == device)
            {
                _device_event_queue[i].device     = NULL;
                _device_event_queue[i].event_data = NULL;
            }
        }
    }
}

stm32DeviceEvent* stm32_device_event_queue_search(stm32DeviceHandle handle)
{
    return stm32_device_event_queue_search_with_addition(handle, 0);
}

stm32DeviceEvent* stm32_device_event_queue_search_with_addition(stm32DeviceHandle handle,
                                                                stm32DeviceAddition addition)
{
    if (handle != NULL)
    {
        for (int i = 0; i < SMAC_STM32_EVENTABLE_PERIPH_NUM; i++)
        {
            if (((_device_event_queue[i].device->handle == handle) &&
                 (_device_event_queue[i].device->addition == addition)) ||
                ((handle == NULL) && (_device_event_queue[i].device->addition == addition)))
            {
                return &_device_event_queue[i];
            }
        }
    }

    return NULL;
}
