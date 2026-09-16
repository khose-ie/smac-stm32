#include <assert.h>
#include <smac-mcu.h>
#include <smac-stm32.h>
#include <stddef.h>
#include <stm32.h>

#if defined(STM32H5)
#define STM32_WRITE_INPUT_DATA_ADDRESS
#endif // defined(STM32H5)

#define FLASH_QUADWORD_SIZE   (16)
#define FLASH_DOUBLEWORD_SIZE (8)
#define FLASH_WORD_SIZE       (4)
#define FLASH_HALFWORD_SIZE   (2)
#define FLASH_BYTE_SIZE       (1)

static uint32_t main_flash_choice_data_width(uint32_t address, uint32_t size)
{
#if defined(FLASH_TYPEPROGRAM_QUADWORD)

    if ((size % FLASH_QUADWORD_SIZE == 0) && ((address & 0x0F) == 0))
    {
        return FLASH_QUADWORD_SIZE;
    }

#endif // defined(FLASH_TYPEPROGRAM_QUADWORD)

#if defined(FLASH_TYPEPROGRAM_DOUBLEWORD)

    if ((size % FLASH_DOUBLEWORD_SIZE == 0) && ((address & 0x07) == 0))
    {
        return FLASH_DOUBLEWORD_SIZE;
    }

#endif // defined(FLASH_TYPEPROGRAM_DOUBLEWORD)

#if defined(FLASH_TYPEPROGRAM_WORD)

    if ((size % FLASH_WORD_SIZE == 0) && ((address & 0x03) == 0))
    {
        return FLASH_WORD_SIZE;
    }

#endif // defined(FLASH_TYPEPROGRAM_WORD)

#if defined(FLASH_TYPEPROGRAM_HALFWORD)

    if ((size % FLASH_HALFWORD_SIZE == 0) && ((address & 0x01) == 0))
    {
        return FLASH_HALFWORD_SIZE;
    }

#endif // defined(FLASH_TYPEPROGRAM_HALFWORD)

#if defined(FLASH_TYPEPROGRAM_BYTE)

    if ((size % FLASH_BYTE_SIZE == 0) && ((address & 0x00) == 0))
    {
        return FLASH_BYTE_SIZE;
    }

#endif // defined(FLASH_TYPEPROGRAM_BYTE)

    return 0;
}

static uint32_t main_flash_width_to_programtype(uint32_t width)
{
#if defined(FLASH_TYPEPROGRAM_QUADWORD)
    if (width == FLASH_QUADWORD_SIZE)
    {
        return FLASH_TYPEPROGRAM_QUADWORD;
    }
#endif

#if defined(FLASH_TYPEPROGRAM_DOUBLEWORD)
    if (width == FLASH_DOUBLEWORD_SIZE)
    {
        return FLASH_TYPEPROGRAM_DOUBLEWORD;
    }
#endif

#if defined(FLASH_TYPEPROGRAM_WORD)
    if (width == FLASH_WORD_SIZE)
    {
        return FLASH_TYPEPROGRAM_WORD;
    }
#endif

#if defined(FLASH_TYPEPROGRAM_HALFWORD)
    if (width == FLASH_HALFWORD_SIZE)
    {
        return FLASH_TYPEPROGRAM_HALFWORD;
    }
#endif

#if defined(FLASH_TYPEPROGRAM_BYTE)
    if (width == FLASH_BYTE_SIZE)
    {
        return FLASH_TYPEPROGRAM_BYTE;
    }
#endif

    return 0;
}

/// @brief Erase the specified Internal FLASH instance.
/// @details The specific implementation of @ref smac_flash_erase for the STM32 platform.
smacRetCode_t smac_flash_erase(uint32_t bank, uint32_t sector, uint32_t num)
{
    uint32_t error;
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks     = bank;
    erase_init.Sector    = sector;
    erase_init.NbSectors = num;

    status = HAL_FLASH_Lock();

    if (status != HAL_OK)
        return stm32_cast_code(status);

    status = HAL_FLASHEx_Erase(&erase_init, &error);
    stm32_cast_code(HAL_FLASH_Unlock());

    return stm32_cast_code(status);
}

smacRetCode_t smac_flash_write(uint32_t bank, uint32_t address, const uint8_t* data, uint32_t size)
{
    (void)bank;

    assert(data != NULL);

    uint32_t width = main_flash_choice_data_width(address, size);

    if (width == 0)
    {
        return SMAC_RET_NOT_SUPPORT;
    }

#if defined(STM32_WRITE_INPUT_DATA_ADDRESS)

    uint32_t addr;
    uint32_t data_addr;

    for (addr = address, data_addr = (uint32_t)data; addr < address + size;
         addr += width, data_addr += width)
    {
        if (HAL_FLASH_Program(main_flash_width_to_programtype(width), addr, data_addr) != HAL_OK)
        {
            return SMAC_RET_LOW_LEVEL_FAILURE;
        }
    }

#else // STM32_WRITE_INPUT_DATA64

    uint32_t addr;
    const uint8_t* data64;

    for (addr = address, data64 = data; addr < address + size; addr += width, data64 += width)
    {
        if (HAL_FLASH_Program(main_flash_width_to_programtype(width), addr, *(uint64_t*)data64) !=
            HAL_OK)
        {
            return SMAC_RET_LOW_LEVEL_FAILURE;
        }
    }

#endif // defined(STM32_WRITE_INPUT_DATA_ADDRESS)

    return SMAC_RET_OK;
}
