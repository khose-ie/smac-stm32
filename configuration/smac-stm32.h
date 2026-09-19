#ifndef _SMAC_STM32_H_
#define _SMAC_STM32_H_

/// Include the STM32 HAL header for specific STM32 series here.
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/// @brief The number of STM32 peripheral instances available.
/// @note This value should be updated according to the actual number of STM32 peripheral instances
/// available on the target platform.
#define SMAC_STM32_PERIPH_NUM (10)

/// @brief The number of STM32 peripheral instances that want to support events/interrupts.
/// @details This value represents the number of STM32 peripheral instances that are capable of
/// generating events or interrupts and need to be managed accordingly.
/// @note This value should be updated according to the actual number of STM32 peripheral instances
/// that need event/interrupt support on the target platform.
/// @note This value should be less than @ref SMAC_STM32_PERIPH_NUM.
#define SMAC_STM32_EVENTABLE_PERIPH_NUM (10)

/// @brief The number of STM32 peripheral instances that want to support caching.
/// @details This value represents the number of STM32 peripheral instances that are capable of
/// being cached and need to be managed accordingly.
/// @note This value should be updated according to the actual number of STM32 peripheral instances
/// that need caching support on the target platform.
/// @note This value should be less than @ref SMAC_STM32_PERIPH_NUM.
/// @example For the CAN peripheral, if you want to enable async receive, you should set cacheable
/// list > 0 and the CAN peripheral will set the message ptr to cache.
#define SMAC_STM32_CACHEABLE_PERIPH_NUM (1)

/// @brief External memory used for the STM32 stack.
/// @details This macro defines the external memory location used for the STM32 stack within the
/// abstraction layer.
// #define SMAC_STM32_STACK_EX_MEM (mcu_stack)

/// @brief The size of the external memory used for the STM32 stack.
/// @details This macro defines the size of the external memory allocated for the STM32 stack within
/// the abstraction layer.
/// @note The requires size is decided by @ref SMAC_STM32_EVENTABLE_PERIPH_NUM, @ref
/// SMAC_STM32_EVENTABLE_PERIPH_NUM and @ref SMAC_STM32_CACHEABLE_PERIPH_NUM. The relation is:
/// 32-bit machine:
/// SMAC_STM32_STACK_EX_MEM_SIZE >= SMAC_STM32_PERIPH_NUM * 8 + SMAC_STM32_EVENTABLE_PERIPH_NUM * 12
/// + SMAC_STM32_CACHEABLE_PERIPH_NUM * 16;
/// 64-bit machine:
/// SMAC_STM32_STACK_EX_MEM_SIZE >= SMAC_STM32_PERIPH_NUM * 16 + SMAC_STM32_EVENTABLE_PERIPH_NUM *
/// 24 + SMAC_STM32_CACHEABLE_PERIPH_NUM * 32;
// #define SMAC_STM32_STACK_EX_MEM_SIZE (256)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _SMAC_STM32_H_
