#ifndef _STM32_I2C_H_
#define _STM32_I2C_H_

#include <smac-mcu.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define STM32_I2C_ROLE_MARK   (0xC0000000)
#define STM32_I2C_ROLE_MASTER (0x80000000)
#define STM32_I2C_ROLE_MEM    (0x00000000)
#define STM32_I2C_ROLE_SLAVE  (0xC0000000)

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _STM32_I2C_H_
