/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2020-03-10     MyGuo       the first version
 */
 
#include <rtthread.h>
#include <rtdbg.h> 
#include "bmi088.h"
#include "sensor_intf_bmi088.h"
#include "drv_spi.h"
#include "bmi088_sample.h"

#define BMI088_BUS_NAME "spi1"
#define BMI088A_SPI_NAME "spi10"
#define BMI088G_SPI_NAME "spi11"

static rt_device_t acce_device_t;
static rt_device_t gyro_device_t;
struct rt_sensor_data acc_test;
struct rt_sensor_data gyr_test;

void imu_thread_entry(void* parameter)
{
    // 配置spi,配置片选引脚(要在acc、gyr初始化之前配置,因为器件初始化中涉及到引脚操作)
    rt_hw_spi_device_attach(BMI088_BUS_NAME, BMI088A_SPI_NAME, GPIOB, GPIO_PIN_14);
    rt_hw_spi_device_attach(BMI088_BUS_NAME, BMI088G_SPI_NAME, GPIOB, GPIO_PIN_15);

    // 注册传感器
    struct rt_sensor_config acc_cfg = {0};
    struct rt_sensor_config gyr_cfg = {0};
    
    acc_cfg.intf.dev_name = BMI088A_SPI_NAME;
    gyr_cfg.intf.dev_name = BMI088G_SPI_NAME;
    
    rt_hw_bmi088_init("bmi", &acc_cfg, &gyr_cfg);
    
    acce_device_t = rt_device_find("acce_bmi");
    if (acce_device_t == RT_NULL)
    {
        LOG_E("Can't find acce device\r\n");
    }
    else 
    {
        rt_device_open(acce_device_t, RT_DEVICE_OFLAG_RDWR);
    }
    
    gyro_device_t = rt_device_find("gyro_bmi");
    if (gyro_device_t == RT_NULL)
    {
        LOG_E("Can't find gyro device\r\n");
    }
    else
    {
        rt_device_open(gyro_device_t, RT_DEVICE_OFLAG_RDWR);
    }
    
    while (1)
    {
        rt_device_read(acce_device_t, 0, &acc_test, 1);
        rt_device_read(gyro_device_t, 0, &gyr_test, 1);

        rt_thread_mdelay(5);
    }
    
}
