#pragma once

/*
 * TTGO T8 V1.7 腳位 (推論韌體)。相機沿用錄影韌體的 camera_pinout.h，不重複定義；
 * mic 腳位與 main/audio_record.c 相同。
 *
 * 一塊板子只跑一個角色 (infer_config.h 的 INFER_NODE_ROLE)，沒選到的管線完全不初始化，
 * 所以 mic 的 26/25 與相機的 SIOD(26) / VSYNC(25) 重疊沒有關係。
 * SD 是 SDMMC slot 1 (1-bit)：CLK 14、CMD 15、D0 2，見 camera_pinout.h 末段。
 */
#include "camera_pinout.h"

/* 麥克風介面：1 = 標準 I2S (INMP441 / ICS-43434)，0 = PDM */
#ifndef MIC_INTERFACE_I2S
#define MIC_INTERFACE_I2S   1
#endif

#if MIC_INTERFACE_I2S
#define MIC_PIN_CLK         26      /* BCLK / SCK */
#define MIC_PIN_WS          25      /* WS / LRCLK */
#define MIC_PIN_DIN         33      /* SD (麥克風 DOUT) */
#else
#define MIC_PIN_CLK         26      /* PDM CLK */
#define MIC_PIN_WS          (-1)    /* PDM 沒有 WS */
#define MIC_PIN_DIN         25      /* PDM DATA */
#endif
