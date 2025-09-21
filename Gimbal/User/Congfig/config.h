/**
 * @file config.h
 * @author lez
 * @brief 工程配置文件
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

#ifndef CONFIG_H
#define CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"
/* Exported macros -----------------------------------------------------------*/
#define MINIPC_COMM_CAN
// #define MINIPC_COMM_USB

// #define IMAGE_VT12
#define IMAGE_VT13

// #define CHASSIS
#define GIMBAL

#ifdef GIMBAL

#define BULLET_SPEED_PID

#endif

#ifdef CHASSIS

#define SPEED_SLOPE
#define POWER_LIMIT

#ifdef POWER_LIMIT

#define POWER_LIMIT_BUFFER_LOOP
#define BIG_P_ALLOCATE

#endif

#endif

// #define DISABLE_SUPEACAP

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
