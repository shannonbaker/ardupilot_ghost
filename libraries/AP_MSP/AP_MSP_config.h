#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 1
#endif

// define for enabling MSP sensor drivers
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED HAL_MSP_ENABLED
#endif

// define for enabling MSP DisplayPort
#ifndef HAL_WITH_MSP_DISPLAYPORT
#define HAL_WITH_MSP_DISPLAYPORT HAL_MSP_ENABLED
#endif

// MSP VTX control needs both the MSP layer and AP_VideoTX. It is non-essential
// and off by default; boards with MSP and the video hardware opt in.
#ifndef AP_MSP_VIDEOTX_ENABLED
#define AP_MSP_VIDEOTX_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 2048)
#endif

// Native GHOST field subscriptions and mission transport over MSP DisplayPort.
// This remains disabled unless explicitly selected by a custom build.
#ifndef AP_MSP_GHOST_DP_ENABLED
#define AP_MSP_GHOST_DP_ENABLED 0
#endif

#if AP_MSP_GHOST_DP_ENABLED && (!HAL_MSP_ENABLED || !HAL_WITH_MSP_DISPLAYPORT)
#error "AP_MSP_GHOST_DP_ENABLED requires HAL_MSP_ENABLED and HAL_WITH_MSP_DISPLAYPORT"
#endif
