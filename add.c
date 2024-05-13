#include "common_36xx_led.h"
#include "led_driver.h"
#include "gpio.h"
#include "sysctl.h"
#include "syslog.h"
#include "udevice.h"
#if USE_RTOS
#include <stdio.h>
#else
#include "aiva_sleep.h"
#endif
#include <math.h>

static led_channel_t s_chn = CHANNEL_0;

typedef struct Common_36xxParams {
    /** current_ma = current_level * current_step_ma + current_min_ma*/
    float current_step_ma;
    float current_min_ma;
    uint8_t chip_id;
    uint8_t device_id;
} Common_36xxParams_t;

typedef enum {
    LM3644 = 0,
    AW36515,
    AW3644
} ProductType_t;

static Common_36xxParams_t supported_36xx_products[] = {
    [LM3644] = {
    .current_step_ma = 11.725f,
    .current_min_ma = 10.9f,
    .chip_id = 0x00,
    .device_id = 0x02,
    },
    [AW36515] = {
    .current_step_ma = 7.83f,
    .current_min_ma = 3.91f,
    .chip_id = 0x30,
    .device_id = 0x02,
    },
    [AW3644] = {
    .current_step_ma = 11.72f,
    .current_min_ma = 11.35f,
    .chip_id = 0x36,
    .device_id = 0x02,
    }
};
// default is LM3644
static ProductType_t current_36xx_type = LM3644;

#define MAX_BRIGHTNESS (0x7F)
#define LED_BRIGHTNESS_INIT (0x20)

#define TAG                             "36XX"
#define LM3644_SLAVE_ADDR               (0x63 << 0)

#define CHIP_ID_REG                     (0x00) // only aw3644 aw36515 has this reg
#define ENABLE_REG                      (0X01)
#define IVFM_REG                        (0x02)
#define LED1_FLASH_REG                  (0x03)
#define LED2_FLASH_REG                  (0x04)
#define LED1_TORCH_REG                  (0x05)
#define LED2_TORCH_REG                  (0x06)
#define BOOST_REG                       (0x07)
#define TIMING_REG                      (0x08)
#define TEMP_REG                        (0x09)
#define FLAGS1_REG                      (0x0A)
#define FLAGS2_REG                      (0x0B)
#define DEVICE_ID_REG                   (0x0C)
#define LAST_FLASH_REG                  (0x0D)

enum Common_36xx_Time_Out_Val {
    LM3644_TIMEOUT_10MS   = 0,
    LM3644_TIMEOUT_20MS   = 1,
    LM3644_TIMEOUT_30MS   = 2,
    LM3644_TIMEOUT_40MS   = 3,
    LM3644_TIMEOUT_50MS   = 4,
    LM3644_TIMEOUT_60MS   = 5,
    LM3644_TIMEOUT_70MS   = 6,
    LM3644_TIMEOUT_80MS   = 7,
    LM3644_TIMEOUT_90MS   = 8,
    LM3644_TIMEOUT_100MS  = 9,
    LM3644_TIMEOUT_150MS  = 10,
    LM3644_TIMEOUT_200MS  = 11,
    LM3644_TIMEOUT_250MS  = 12,
    LM3644_TIMEOUT_300MS  = 13,
    LM3644_TIMEOUT_350MS  = 14,
    LM3644_TIMEOUT_400MS  = 15,
};

typedef struct _LM3644_Config_T {
    uint8_t     reg_addr;                             // Register address
    uint8_t     data;                                 // Data
}LM3644_Config_T;

/* 
   if only LED_2 is connected tolm3644, setting reg 0x1 to 0x27, LED_2 won't work;
   if LED_1 and LED_2 is connected tolm3644, setting reg 0x1 to 0x27, LED_1 and LED_2 works.
   */   
static LM3644_Config_T common_36xx_init_settings[] = {
/*#ifdef USE_SL18*/
    /*// IR mode, 0x26, enable LED2,disable LED1.*/
    /*{0x1, 0x26},       // Reg 1  ()    */
/*#else*/
    /*// IR mode, 0x25, enable LED1,disable LED2.*/
    /*{0x1, 0x25},       // Reg 1  ()    */
/*#endif*/
    // IR mode, 0x27, disable LED2, LED1.
    {ENABLE_REG, 0x24},       // Reg 1  ()

    {IVFM_REG, 0x01},       // Reg 2  ()
    {LED1_FLASH_REG, LED_BRIGHTNESS_INIT},       // Reg 3  ()
    {LED2_FLASH_REG, LED_BRIGHTNESS_INIT},       // Reg 4  ()
    {LED1_TORCH_REG, 0x8F},       // Reg 5  ()
    {LED2_TORCH_REG, 0x0F},       // Reg 6  ()
    {BOOST_REG, 0x09},       // Reg 7  ()
    /*{TIMING_REG, 0x1F},       // Reg 8  ()*/
    {TIMING_REG, 0x13},       // Reg 8  ()  40ms
    /*{TIMING_REG, 0x14},       // Reg 8  ()  50ms*/
    /*{TIMING_REG, 0x11},       // Reg 8  ()  20ms*/
    {TEMP_REG, 0x08},       // Reg 9  ()
};

typedef struct _LED_Status {
    uint8_t     brightness_reg_addr;
    uint8_t     brightness;                             
} LED_Status;

static LED_Status led_status[] = {
    {LED1_FLASH_REG, LED_BRIGHTNESS_INIT},
    {LED2_FLASH_REG, LED_BRIGHTNESS_INIT}
};

static int Common_36xx_Write_Reg(i2c_device_number_t dev, uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
    int ret;
    uint8_t data_buf[2];

    data_buf[0] = reg;
    data_buf[1] = data;
    ret = i2c_send_data(dev, i2c_addr, data_buf, 2);
    CHECK_RET(ret);

    return ret;
}

static uint8_t Common_36xx_Read_Reg(i2c_device_number_t dev, uint8_t i2c_addr, uint8_t reg_addr)
{
    int ret;
    uint8_t reg_val;

    ret = i2c_send_data(dev, i2c_addr, &reg_addr, 1);
    CHECK_RET(ret);

    ret = i2c_recv_data(dev, i2c_addr, 0, 0, &reg_val, 1);
    CHECK_RET(ret);

    return reg_val;
}

static int Common_36xx_Get_Lock(led_dev_t *dev)
{
    int ret = 0;
#ifdef USE_RTOS
    if (!xPortIsInISR()) {
        ret = pthread_mutex_lock(&(((Common_led_t*)dev->priv)->mutex));
    }
#endif
    return ret;
}

static int Common_36xx_Release_Lock(led_dev_t *dev)
{
    int ret = 0;
#ifdef USE_RTOS
    if (!xPortIsInISR()) {
        ret = pthread_mutex_unlock(&(((Common_led_t*)dev->priv)->mutex));
    }
#endif
    return ret;
}

static enum Common_36xx_Time_Out_Val Timeout_Ms2Level(int timeout_ms)
{
    enum Common_36xx_Time_Out_Val timeout_level = LM3644_TIMEOUT_10MS;
    if (timeout_ms < 5)
    {
        timeout_level = LM3644_TIMEOUT_10MS;
    }
    else if (timeout_ms >= 5 && timeout_ms <= 100)
    {
        timeout_level = (enum Common_36xx_Time_Out_Val)((timeout_ms + 5) / 10 - 1);
    }
    else if (timeout_ms <= 400)
    {
        timeout_level = (enum Common_36xx_Time_Out_Val)((timeout_ms + 25) / 50 - 2 + LM3644_TIMEOUT_100MS);
    }
    else
    {
        timeout_level = LM3644_TIMEOUT_400MS;
    }

    return timeout_level;
}

static int Common_36xx_Set_Flash_Timeout(led_dev_t *dev, int timeout_level)//设置闪光灯的超时时间
{
    Common_36xx_Get_Lock(dev);//代码获取一个锁，以确保在设置超时时间期间不会被其他操作打断

    Common_led_t *_dev = (Common_led_t *)dev->priv;
    i2c_device_number_t i2c_num = _dev->i2c_num;
    i2c_init(i2c_num, _dev->i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);

    int ret = Common_36xx_Write_Reg(i2c_num,
            _dev->i2c_addr,
            TIMING_REG, 
            timeout_level);

    Common_36xx_Release_Lock(dev);

    return ret;
}

// void Common_36xx_Test(void)
// {    
//     Common_36xx_Init(I2C_DEVICE_2, GPIO_PIN3, true);
//     Common_36xx_Set_Flash_Timeout(LM3644_TIMEOUT_10MS);

//     while (1)
//     {
//         Common_36xx_Power_Off();        
//         Common_36xx_Power_On();
//         Common_36xx_Busy_Delay(3000);
//     }
// }


#ifdef __cplusplus
 extern "C" {
#endif

#if USE_RTOS
static void Timeout_Cb(TimerHandle_t timer)
{
    led_dev_t* led_dev = (led_dev_t*)pvTimerGetTimerID(timer);
    Common_led_t* common_36xx_dev = (Common_led_t*)led_dev->priv;
    gpio_pin_value_t pin_val = gpio_get_pin(common_36xx_dev->power_pin);
    if(pin_val == GPIO_PV_LOW)
    {
        // set power pin high to turn on led, and start timer to turn off led when timeout 
        gpio_set_pin(common_36xx_dev->power_pin, GPIO_PV_HIGH);
       // LOGI("", "%d GPIO_PV_HIGH\n", (int)common_36xx_dev->power_pin);
        //
        TimerHandle_t _timer = common_36xx_dev->timer;
        TickType_t period = xTimerGetPeriod(_timer);
       TickType_t expect_period = led_dev->timeout_ms[s_chn] / portTICK_PERIOD_MS;
        if (period!= expect_period)
        {
            xTimerChangePeriod(_timer, expect_period, 0);
        }
        xTimerStart(_timer, 0);
    }
    else
    {
        // reset power pin to turn off led
       // LOGI("", "%d GPIO_PV_LOW\n", (int)common_36xx_dev->power_pin);
        gpio_set_pin(common_36xx_dev->power_pin, GPIO_PV_LOW);
    }
}
#endif

int Common_36xx_Init(led_dev_t *dev)
{
    Common_led_t* _dev = (Common_led_t*)dev->priv;
    uint8_t pin = _dev->power_pin;
    int timeout_ms = dev->timeout_ms;
 
    i2c_device_number_t i2c_num = _dev->i2c_num;

    Common_36xx_Get_Lock(dev);

    i2c_init(i2c_num, _dev->i2c_addr, _dev->i2c_addr_width, _dev->i2c_clk);
    
    // read 0x00 to get product id
    uint8_t chip_id = Common_36xx_Read_Reg(i2c_num, _dev->i2c_addr, CHIP_ID_REG);
#if 0 // LM3644 not tell us what will return when read 0x00, so we can not assume it not return 0x30 0r 0x36
    for (size_t i = 0; i < sizeof(supported_36xx_Products) / sizeof(supported_36xx_Products[0]); i++)
    {
        if (supported_36xx_Products[i].chip_id == chip_id)
        {
            current_36xx_type = (ProductType_t)i;
            break;
        }
    }
#else
    if (chip_id == supported_36xx_Products[AW36515].chip_id)
    {
        current_36xx_type = AW36515;
    }
    else if (chip_id == supported_36xx_Products[AW3644].chip_id)
    {
        current_36xx_type = AW3644;
    }
#endif

    int ret = 0;
    for (int i = 0; i < (int)AIVA_ARRAY_LEN(common_36xx_init_settings); i++)
    {
        ret = Common_36xx_Write_Reg(i2c_num,
                _dev->i2c_addr,
                common_36xx_init_settings[i].reg_addr, 
                common_36xx_init_settings[i].data);
        if (ret!= 0)
        {
            Common_36xx_Release_Lock(dev);
            goto out;
        }
    }

    // enum Common_36xx_Time_Out_Val timeout_level = Timeout_Ms2Level(timeout_ms);
    // Common_36xx_Write_Reg(i2c_num,
    //     _dev->i2c_addr, 
    //     TIMING_REG, 
    //     timeout_level);

    Common_36xx_Release_Lock(dev);

    Sysctl_Io_Switch_t io_switch = IO_SWITCH_GPIO0 + pin;
    Sysctl_Set_Io_Switch(io_switch, 1);

    if (_dev->use_strobe)
    {
        gpio_set_drive_mode(pin, GPIO_DM_INPUT);
    }
    else
    {
        gpio_set_drive_mode(pin, GPIO_DM_OUTPUT);
#if USE_RTOS
        int pulse_in_ms = 2;
        const led_trig_param_t *trig = dev->trig_param;
        if (trig!= NULL && trig->pulse_in_us >= 2000 && trig->pulse_in_us <= 10000)
        {
            pulse_in_ms = (trig->pulse_in_us / 1000);
        }
        char timer_name[20] = {0};
        snprintf(timer_name, sizeof(timer_name) / sizeof(timer_name[0]), "LM3644%d_LED_TIMER", (int)pin);
        // create software timer
        _dev->timer = xTimerCreate
                    (timer_name,
                    pdMS_TO_TICKS(pulse_in_ms),
                    0,
                    (void *)dev,
                    Timeout_Cb);
#endif
    }

    LOGI(TAG, "%s done\n", current_36xx_type == AW3644? "AW3644" : (current_36xx_type == AW36515? "