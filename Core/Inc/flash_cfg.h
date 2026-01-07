/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_cfg.h
  * @brief          : Log-structured flash config storage
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef FLASH_CFG_H
#define FLASH_CFG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t mode;
  uint8_t run_state;
  uint8_t param_level[10];
  uint8_t reserved[21];
} Config;

bool FlashCfg_Load(Config *out);
bool FlashCfg_Append(const Config *in);
void FlashCfg_InitDefaults(Config *out);

#endif /* FLASH_CFG_H */
