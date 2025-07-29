// ESP-S3-LCD-EV开发板相关头文件
#include "wifi_board.h"               // WiFi板子基类
#include "codecs/box_audio_codec.h"   // 音频编解码器
#include "display/lcd_display.h"      // LCD显示器
#include "application.h"              // 应用程序主类
#include "button.h"                   // 按键控制
#include "led/single_led.h"          // LED控制
#include "pin_config.h"               // 引脚配置

#include "config.h"                   // 配置文件

// ESP-IDF系统库
#include <wifi_station.h>            // WiFi连接
#include <esp_log.h>                 // 日志输出
#include <driver/i2c_master.h>       // I2C主机驱动
#include "esp_lcd_gc9503.h"          // GC9503 LCD驱动
#include <esp_lcd_panel_io.h>        // LCD面板IO
#include <esp_lcd_panel_ops.h>       // LCD面板操作
#include <esp_lcd_panel_io_additions.h> // LCD面板IO扩展

#include "esp_io_expander_tca9554.h" // TCA9554 IO扩展器

#define TAG "ESP_S3_LCD_EV_Board"

// 字体声明
LV_FONT_DECLARE(font_puhui_30_4);    // 普惠字体 30号 4位色深
LV_FONT_DECLARE(font_awesome_30_4);  // Awesome图标字体 30号 4位色深

// ESP-S3-LCD-EV开发板类，继承自WiFi板子基类
class ESP_S3_LCD_EV_Board : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;  // 音频编解码器I2C总线句柄
    Button boot_button_;                     // 启动按键
    LcdDisplay* display_;                    // LCD显示器指针

    // 添加EV开发板LCD支持 - IO扩展器句柄
    esp_io_expander_handle_t expander = NULL;

    // 初始化RGB GC9503V显示屏
    void InitializeRGB_GC9503V_Display() {
        ESP_LOGI(TAG, "Init GC9503V");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        
        // 添加EV开发板LCD支持 - 配置VSYNC引脚
        gpio_config_t io_conf = {
            .pin_bit_mask = BIT64(GC9503V_PIN_NUM_VSYNC),  // VSYNC引脚位掩码
            .mode = GPIO_MODE_OUTPUT,                       // 输出模式
            .pull_up_en = GPIO_PULLUP_ENABLE,              // 使能上拉
        };

        gpio_config(&io_conf);                             // 配置GPIO
        gpio_set_level(GC9503V_PIN_NUM_VSYNC, 1);         // 设置VSYNC为高电平

        ESP_LOGI(TAG, "Install 3-wire SPI  panel IO");
        // 3线SPI线路配置
        spi_line_config_t line_config = {
            .cs_io_type = IO_TYPE_EXPANDER,           // CS片选信号使用IO扩展器
            .cs_expander_pin = GC9503V_LCD_IO_SPI_CS_1,  // CS扩展器引脚
            .scl_io_type = IO_TYPE_EXPANDER,          // SCL时钟信号使用IO扩展器
            .scl_expander_pin = GC9503V_LCD_IO_SPI_SCL_1, // SCL扩展器引脚
            .sda_io_type = IO_TYPE_EXPANDER,          // SDA数据信号使用IO扩展器
            .sda_expander_pin = GC9503V_LCD_IO_SPI_SDO_1, // SDA扩展器引脚
            .io_expander = expander,                  // IO扩展器句柄
        };

        // 3线SPI面板IO配置
        // esp_lcd_panel_io_3wire_spi_config_t: 用于配置3线SPI通信的参数结构
        // GC9503_PANEL_IO_3WIRE_SPI_CONFIG宏展开后包含以下配置：
        // - line_config: SPI信号线配置（CS、SCL、SDA都通过TCA9554 IO扩展器）
        // - expect_clk_speed: 期望的SPI时钟频率（通常为500kHz）
        // - spi_mode: SPI通信模式（0-3，通常使用模式0）
        // - lcd_cmd_bytes: LCD命令字节数（1-4字节，通常为1字节）
        // - lcd_param_bytes: LCD参数字节数（1-4字节，通常为1字节）
        // - flags.use_dc_bit: 是否使用DC位区分命令和数据
        // - flags.cs_high_active: CS信号是否高电平有效
        esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
        // 创建3线SPI面板IO实例
        // esp_lcd_new_panel_io_3wire_spi: 创建软件模拟的3线SPI接口，用于向LCD控制器发送初始化命令
        // 该函数内部会：
        // 1. 分配panel_io内存并初始化相关函数指针
        // 2. 保存SPI配置参数（时钟频率、模式、信号线配置等）
        // 3. 初始化IO扩展器的SPI引脚为输出模式
        // 4. 创建用于软件模拟SPI时序的相关资源
        // 注意：这里创建的是控制接口，用于发送配置命令，不是数据传输接口
        int espok = esp_lcd_new_panel_io_3wire_spi(&io_config, &panel_io);
        ESP_LOGI(TAG, "Install 3-wire SPI  panel IO:%d",espok);


        ESP_LOGI(TAG, "Install RGB LCD panel driver");
        esp_lcd_panel_handle_t panel_handle = NULL;
        // RGB LCD面板配置
        esp_lcd_rgb_panel_config_t rgb_config = {
            .clk_src = LCD_CLK_SRC_PLL160M,              // 时钟源：160MHz PLL
            // .timings = GC9503_376_960_PANEL_60HZ_RGB_TIMING(),  // 原始时序配置
            // 添加EV开发板支持 - 使用480x480分辨率时序
            .timings = GC9503_480_480_PANEL_60HZ_RGB_TIMING(),
            .data_width = 16,                            // RGB565并行模式，16位数据宽度
            .bits_per_pixel = 16,                        // 每像素16位
            .num_fbs = GC9503V_LCD_RGB_BUFFER_NUMS,      // 帧缓冲区数量
            .bounce_buffer_size_px = GC9503V_LCD_H_RES * GC9503V_LCD_RGB_BOUNCE_BUFFER_HEIGHT, // 缓冲区大小
            .dma_burst_size = 64,                        // DMA突发传输大小
            .hsync_gpio_num = GC9503V_PIN_NUM_HSYNC,     // 水平同步引脚
            .vsync_gpio_num = GC9503V_PIN_NUM_VSYNC,     // 垂直同步引脚
            .de_gpio_num = GC9503V_PIN_NUM_DE,           // 数据使能引脚
            .pclk_gpio_num = GC9503V_PIN_NUM_PCLK,       // 像素时钟引脚
            .disp_gpio_num = GC9503V_PIN_NUM_DISP_EN,    // 显示使能引脚
            .data_gpio_nums = {                          // 数据引脚配置
                GC9503V_PIN_NUM_DATA0,                   // 数据位0
                GC9503V_PIN_NUM_DATA1,                   // 数据位1
                GC9503V_PIN_NUM_DATA2,                   // 数据位2
                GC9503V_PIN_NUM_DATA3,                   // 数据位3
                GC9503V_PIN_NUM_DATA4,                   // 数据位4
                GC9503V_PIN_NUM_DATA5,                   // 数据位5
                GC9503V_PIN_NUM_DATA6,                   // 数据位6
                GC9503V_PIN_NUM_DATA7,                   // 数据位7
                GC9503V_PIN_NUM_DATA8,                   // 数据位8
                GC9503V_PIN_NUM_DATA9,                   // 数据位9
                GC9503V_PIN_NUM_DATA10,                  // 数据位10
                GC9503V_PIN_NUM_DATA11,                  // 数据位11
                GC9503V_PIN_NUM_DATA12,                  // 数据位12
                GC9503V_PIN_NUM_DATA13,                  // 数据位13
                GC9503V_PIN_NUM_DATA14,                  // 数据位14
                GC9503V_PIN_NUM_DATA15,                  // 数据位15
            },
            .flags= {
                .fb_in_psram = true,                     // 在PSRAM中分配帧缓冲区
            }
        };
    
        ESP_LOGI(TAG, "Initialize RGB LCD panel");
    
        // GC9503厂商特定配置
        // gc9503_vendor_config_t: GC9503 LCD控制器的厂商特定参数
        gc9503_vendor_config_t vendor_config = {
            .rgb_config = &rgb_config,               // RGB接口配置指针（连接到ESP32-S3的RGB LCD控制器）
            .flags = {
                .mirror_by_cmd = 0,                  // 不通过SPI命令进行镜像翻转（由RGB控制器处理）
                .auto_del_panel_io = 1,              // 面板删除时自动清理panel_io资源
            },
        };
        // LCD面板设备配置
        // esp_lcd_panel_dev_config_t: LCD面板设备的通用配置参数
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = -1,                    // 复位引脚（-1表示不使用硬件复位，采用软件复位）
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // RGB像素元素顺序（RGB或BGR）
            // .bits_per_pixel = 16,                 // 每像素位数（原始为16位RGB565）
            // 添加EV开发板支持 - 18位每像素（RGB666格式，提供更好的色彩表现）
            .bits_per_pixel = 18,
            .vendor_config = &vendor_config,         // 厂商特定配置（GC9503相关参数）
        };
        // 创建GC9503面板驱动实例
        // esp_lcd_new_panel_gc9503: 创建GC9503 LCD控制器的面板驱动对象
        // 该函数会分配内存并初始化GC9503面板结构，设置操作函数指针（reset、init、mirror等）
        (esp_lcd_new_panel_gc9503(panel_io, &panel_config, &panel_handle));
        
        // 执行LCD面板硬件/软件复位操作
        // esp_lcd_panel_reset: 调用面板驱动的reset函数指针，执行以下操作：
        // 1. 如果配置了reset_gpio_num >= 0：执行硬件复位
        //    - 将复位引脚设为有效电平（通常为低电平）
        //    - 延时10ms保持复位状态
        //    - 将复位引脚设为无效电平（通常为高电平）  
        //    - 延时120ms等待芯片启动完成
        // 2. 如果未配置复位引脚但有panel_io：执行软件复位
        //    - 通过SPI发送LCD_CMD_SWRESET(0x01)软件复位命令
        //    - 延时120ms等待复位完成
        // 注意：ESP-S3-LCD-EV板在config中设置reset_gpio_num = -1，所以使用软件复位
        (esp_lcd_panel_reset(panel_handle));
        
        // 初始化LCD面板显示参数
        // esp_lcd_panel_init: 调用面板驱动的init函数指针，执行LCD控制器初始化序列：
        // 1. 发送厂商特定的初始化命令序列到GC9503控制器
        // 2. 配置显示模式、颜色格式、时序参数等寄存器
        // 3. 设置RGB接口参数以匹配ESP32-S3的RGB LCD控制器
        // 4. 启用显示输出，使LCD面板开始正常工作
        (esp_lcd_panel_init(panel_handle));

        // 创建RGB LCD显示器对象
        display_ = new RgbLcdDisplay(panel_io, panel_handle,
                                  DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                  DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                  {
                                      .text_font = &font_puhui_30_4,    // 文本字体
                                      .icon_font = &font_awesome_30_4,  // 图标字体
                                      .emoji_font = font_emoji_64_init(), // 表情字体
                                  });
    }
    // 初始化音频编解码器I2C总线
    void InitializeCodecI2c() {
        // 初始化I2C外设
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,                      // I2C端口号
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,      // SDA数据引脚
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,      // SCL时钟引脚
            .clk_source = I2C_CLK_SRC_DEFAULT,          // 默认时钟源
            .glitch_ignore_cnt = 7,                     // 毛刺忽略计数
            .intr_priority = 0,                         // 中断优先级
            .trans_queue_depth = 0,                     // 传输队列深度
            .flags = {
                .enable_internal_pullup = 1,            // 使能内部上拉电阻
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));

        // 添加EV开发板LCD放大器支持
        // 初始化TCA9554 IO扩展器
        esp_io_expander_new_i2c_tca9554(codec_i2c_bus_, 0x20, &expander);
        /* 设置功率放大器引脚，默认设置为使能 */
        esp_io_expander_set_dir(expander, BSP_POWER_AMP_IO, IO_EXPANDER_OUTPUT);  // 设置为输出
        esp_io_expander_set_level(expander, BSP_POWER_AMP_IO, true);              // 设置为高电平（使能）

    }

    // 初始化按键
    void InitializeButtons() {
        // 按键点击事件：重置WiFi配置
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // 如果设备正在启动且WiFi未连接，则重置WiFi配置
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
        });
        // 按键按下事件：开始语音监听
        boot_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        // 按键释放事件：停止语音监听
        boot_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
    }

public:
    // 构造函数：初始化ESP-S3-LCD-EV开发板
    ESP_S3_LCD_EV_Board() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeCodecI2c();              // 初始化音频编解码器I2C
        InitializeButtons();               // 初始化按键
        InitializeRGB_GC9503V_Display();   // 初始化RGB显示屏
    }

    // 获取音频编解码器 - ES7210用作音频采集，ES8311用作音频播放
    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            codec_i2c_bus_,                  // I2C总线句柄
            AUDIO_INPUT_SAMPLE_RATE,         // 音频输入采样率
            AUDIO_OUTPUT_SAMPLE_RATE,        // 音频输出采样率
            AUDIO_I2S_GPIO_MCLK,            // I2S主时钟引脚
            AUDIO_I2S_GPIO_BCLK,            // I2S位时钟引脚
            AUDIO_I2S_GPIO_WS,              // I2S字选择引脚
            AUDIO_I2S_GPIO_DOUT,            // I2S数据输出引脚
            AUDIO_I2S_GPIO_DIN,             // I2S数据输入引脚
            GPIO_NUM_NC,                     // 未连接的GPIO
            AUDIO_CODEC_ES8311_ADDR,        // ES8311音频编解码器I2C地址
            AUDIO_CODEC_ES7210_ADDR,        // ES7210音频ADC I2C地址
            true);                           // 使能扬声器
        return &audio_codec;
    }



    // 获取显示器对象
    virtual Display* GetDisplay() override {
        return display_;
    }
    
    // 获取LED对象 - 添加彩灯显示状态
    // 如果亮度太暗可以去更改默认亮度值 DEFAULT_BRIGHTNESS 在led的single_led.cc中
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);  // 使用内置LED GPIO
        return &led;
    }


};

// 声明并注册ESP-S3-LCD-EV开发板
DECLARE_BOARD(ESP_S3_LCD_EV_Board);
