/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-06     MYGuo        the first version
 */

#ifndef __SENSOR_INTF_BMI088_H__
#define __SENSOR_INTF_BMI088_H__

#include <rtthread.h>
#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 1, 0))
        #include "drivers/sensor.h"
    #else
        #include "sensor.h"
    #endif
#endif
#include "BMI088.h"


rt_err_t rt_hw_bmi088_init(const char *name, struct rt_sensor_config *acc_cfg, struct rt_sensor_config *gyr_cfg);

#endif
