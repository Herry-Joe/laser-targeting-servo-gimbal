/**
 * @file    ssd1306.c
 * @brief   SSD1306 (128x64) OLED 驱动实现 — I2C 接口 (STM32 HAL)
 *
 * 接线 (本工程已重映射 I2C1 到 PB8/PB9):
 *   OLED VCC → 3.3V
 *   OLED GND → GND
 *   OLED SCL → PB8
 *   OLED SDA → PB9
 *   OLED 地址 0x3C (SA0 接地)
 *
 * 字体: 标准 5x7 ASCII 字模 (列优先, 每字符 5 字节, bit0=顶行)
 */
#include "ssd1306.h"

/* ========================== 显存缓冲 ========================== */
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_PAGES];

/* 已发送到屏幕的显存影子 — 脏页更新用: 只重传变化的页, 既省 I2C 带宽又避免整屏重刷闪屏 */
static uint8_t SSD1306_Sent[SSD1306_WIDTH * SSD1306_PAGES];

/* ========================== 内部状态 ========================== */
static uint8_t            ssd1306_ready = 0;

/* SSD1306 常见 I2C 地址: 0x3C (SA0=GND) 或 0x3D (SA0=VCC) */
#define SSD1306_ADDR_3C    ((uint16_t)(0x3C << 1))   /* 0x78 */
#define SSD1306_ADDR_3D    ((uint16_t)(0x3D << 1))   /* 0x7A */
static uint16_t           ssd1306_detected_addr = 0; /* 实际探测到的地址 */

/* ========================== 字模 (5x7) ========================== */
/* 每个字符 5 字节, 每字节 = 一列 (左→右), bit0=顶行, bit6=底行 */
static const uint8_t font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // (space) 32
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // ! 33
    {0x00, 0x07, 0x00, 0x07, 0x00}, // " 34
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // # 35
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $ 36
    {0x23, 0x13, 0x08, 0x64, 0x62}, // % 37
    {0x36, 0x49, 0x55, 0x22, 0x50}, // & 38
    {0x00, 0x05, 0x03, 0x00, 0x00}, // ' 39
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // ( 40
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ) 41
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // * 42
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // + 43
    {0x00, 0x50, 0x30, 0x00, 0x00}, // , 44
    {0x08, 0x08, 0x08, 0x08, 0x08}, // - 45
    {0x00, 0x60, 0x60, 0x00, 0x00}, // . 46
    {0x20, 0x10, 0x08, 0x04, 0x02}, // / 47
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0 48
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1 49
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2 50
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3 51
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4 52
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5 53
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6 54
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7 55
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8 56
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9 57
    {0x00, 0x36, 0x36, 0x00, 0x00}, // : 58
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ; 59
    {0x08, 0x14, 0x22, 0x41, 0x00}, // < 60
    {0x14, 0x14, 0x14, 0x14, 0x14}, // = 61
    {0x00, 0x41, 0x22, 0x14, 0x08}, // > 62
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ? 63
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @ 64
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A 65
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B 66
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C 67
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // D 68
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E 69
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F 70
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G 71
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H 72
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I 73
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J 74
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K 75
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L 76
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M 77
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N 78
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O 79
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P 80
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q 81
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R 82
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S 83
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T 84
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U 85
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V 86
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W 87
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X 88
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y 89
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z 90
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [ 91
    {0x02, 0x04, 0x08, 0x10, 0x20}, // \ 92
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ] 93
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^ 94
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _ 95
    {0x00, 0x01, 0x02, 0x04, 0x00}, // ` 96
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a 97
    {0x7F, 0x44, 0x44, 0x44, 0x38}, // b 98
    {0x38, 0x44, 0x44, 0x44, 0x44}, // c 99
    {0x38, 0x44, 0x44, 0x44, 0x7F}, // d 100
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e 101
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f 102
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g 103
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h 104
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i 105
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j 106
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k 107
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l 108
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m 109
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n 110
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o 111
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p 112
    {0x08, 0x14, 0x14, 0x14, 0x7C}, // q 113
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r 114
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s 115
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t 116
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u 117
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v 118
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w 119
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x 120
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y 121
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z 122
    {0x00, 0x08, 0x36, 0x41, 0x00}, // { 123
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // | 124
    {0x00, 0x41, 0x36, 0x08, 0x00}, // } 125
    {0x10, 0x08, 0x08, 0x10, 0x08}, // ~ 126
};

/* ========================== 软件 I2C (位敲, PB8=SCL, PB9=SDA) ==========================
 * 改用软件 I2C 是因为硬件 I2C1 重映射到 PB8/PB9 有兼容性问题,
 * 而参考用户其他项目, 软件 I2C 在此硬件上已验证可稳定工作。
 * 时序: ~50kHz, 比 100kHz 更可靠, 且 OLED 刷新率不受限 (10fps 足够)。
 * ================================================================= */

/* PB8=SCL, PB9=SDA 均为开漏输出 + 外部上拉 (模块内部通常自带 ~10kΩ) */
#define SW_SCL_PORT   GPIOB
#define SW_SCL_PIN    GPIO_PIN_8
#define SW_SDA_PORT   GPIOB
#define SW_SDA_PIN    GPIO_PIN_9

static void sw_delay(void)
{
    for (volatile uint32_t i = 0; i < 20; i++) {}  /* ~1.5µs @ 72MHz → I2C≈250kHz(在规格内) */
}

static void sw_scl_high(void) { HAL_GPIO_WritePin(SW_SCL_PORT, SW_SCL_PIN, GPIO_PIN_SET); }
static void sw_scl_low(void)  { HAL_GPIO_WritePin(SW_SCL_PORT, SW_SCL_PIN, GPIO_PIN_RESET); }
static void sw_sda_high(void) { HAL_GPIO_WritePin(SW_SDA_PORT, SW_SDA_PIN, GPIO_PIN_SET); }
static void sw_sda_low(void)  { HAL_GPIO_WritePin(SW_SDA_PORT, SW_SDA_PIN, GPIO_PIN_RESET); }

/** @brief 初始化 PB8/PB9 为开漏输出 (软件 I2C) */
static void sw_i2c_init_pins(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = SW_SCL_PIN | SW_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    sw_scl_high();
    sw_sda_high();
}

/** @brief I2C START 条件: SDA 在 SCL 高时由高→低 */
static void sw_i2c_start(void)
{
    sw_sda_high();  sw_delay();
    sw_scl_high();  sw_delay();
    sw_sda_low();   sw_delay();
    sw_scl_low();   sw_delay();
}

/** @brief I2C STOP 条件: SDA 在 SCL 高时由低→高 */
static void sw_i2c_stop(void)
{
    sw_sda_low();   sw_delay();
    sw_scl_high();  sw_delay();
    sw_sda_high();  sw_delay();
}

/**
 * @brief 写一个字节并读取 ACK
 * @return 0=ACK received, 1=NACK/无应答
 */
static uint8_t sw_i2c_write_byte(uint8_t data)
{
    /* 发送 8 个数据位 (MSB first) */
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) sw_sda_high(); else sw_sda_low();
        sw_delay();
        sw_scl_high();  sw_delay();     /* SCL high → 从机采样 */
        sw_scl_low();   sw_delay();
        data <<= 1;
    }

    /* 第 9 个时钟: 主机释放 SDA, 读取从机 ACK */
    sw_sda_high();      /* 释放 SDA (开漏输出写 1 = 高阻) */
    sw_delay();
    sw_scl_high();  sw_delay();
    uint8_t ack = HAL_GPIO_ReadPin(SW_SDA_PORT, SW_SDA_PIN);
    sw_scl_low();   sw_delay();
    return (ack == GPIO_PIN_RESET) ? 0 : 1;  /* 0=ACK, 1=NACK */
}

/* ========================== OLED 通信 (基于软件 I2C) ========================== */

/**
 * @brief 探测指定地址的 I2C 设备
 * @return 1=设备应答, 0=无应答
 */
static uint8_t sw_i2c_probe(uint8_t addr_7bit)
{
    uint8_t addr_w = (addr_7bit << 1) & 0xFE;   /* 7 位地址左移 1 位 = 写地址 */
    sw_i2c_start();
    uint8_t nak = sw_i2c_write_byte(addr_w);
    sw_i2c_stop();
    return (nak == 0) ? 1 : 0;
}

/** @brief 写命令字节 (先发控制字节 0x00, 再发命令) */
static void ssd1306_write_cmd(uint8_t cmd)
{
    if (!ssd1306_ready || !ssd1306_detected_addr) return;
    uint8_t addr_w = (ssd1306_detected_addr << 1) & 0xFE;  /* 7 位 → 写地址 */
    sw_i2c_start();
    sw_i2c_write_byte(addr_w);          /* 器件地址 + W */
    sw_i2c_write_byte(0x00);            /* 控制字节: 命令流 */
    sw_i2c_write_byte(cmd);             /* 命令 */
    sw_i2c_stop();
}

/** @brief 写数据块 (先发控制字节 0x40, 再发数据) */
static void ssd1306_write_data(const uint8_t *data, uint16_t len)
{
    if (!ssd1306_ready || !ssd1306_detected_addr || !data) return;
    uint8_t addr_w = (ssd1306_detected_addr << 1) & 0xFE;
    sw_i2c_start();
    sw_i2c_write_byte(addr_w);          /* 器件地址 + W */
    sw_i2c_write_byte(0x40);            /* 控制字节: 数据流 */
    for (uint16_t i = 0; i < len; i++) {
        sw_i2c_write_byte(data[i]);
    }
    sw_i2c_stop();
}

/* ========================== 初始化 ========================== */

/* 前向声明 (供 Init 调用; 实现在 UpdateScreen 前) */
static void SSD1306_DoRefresh(void);

/* ===== 底层初始化命令序列 (不含探测/GPIO, 供 Init/Recovery 共用) ===== */
static void SSD1306_WriteInitCommands(void)
{
    ssd1306_write_cmd(0xAE);  HAL_Delay(2);
    ssd1306_write_cmd(0x20);  HAL_Delay(2);
    ssd1306_write_cmd(0x00);  HAL_Delay(2);
    ssd1306_write_cmd(0x21);  HAL_Delay(2);
    ssd1306_write_cmd(0x00);  HAL_Delay(2);
    ssd1306_write_cmd(0x7F);  HAL_Delay(2);
    ssd1306_write_cmd(0x22);  HAL_Delay(2);
    ssd1306_write_cmd(0x00);  HAL_Delay(2);
    ssd1306_write_cmd(0x07);  HAL_Delay(2);
    ssd1306_write_cmd(0xB0);  HAL_Delay(2);
    ssd1306_write_cmd(0xC8);  HAL_Delay(2);
    ssd1306_write_cmd(0x00);  HAL_Delay(2);
    ssd1306_write_cmd(0x10);  HAL_Delay(2);
    ssd1306_write_cmd(0x40);  HAL_Delay(2);
    ssd1306_write_cmd(0x81);  HAL_Delay(2);
    ssd1306_write_cmd(0xCF);  HAL_Delay(2);
    ssd1306_write_cmd(0xA1);  HAL_Delay(2);
    ssd1306_write_cmd(0xA6);  HAL_Delay(2);
    ssd1306_write_cmd(0xA8);  HAL_Delay(2);
    ssd1306_write_cmd(0x3F);  HAL_Delay(2);
    ssd1306_write_cmd(0xD3);  HAL_Delay(2);
    ssd1306_write_cmd(0x00);  HAL_Delay(2);
    ssd1306_write_cmd(0xD5);  HAL_Delay(2);
    ssd1306_write_cmd(0x80);  HAL_Delay(2);
    ssd1306_write_cmd(0xD9);  HAL_Delay(2);
    ssd1306_write_cmd(0xF1);  HAL_Delay(2);
    ssd1306_write_cmd(0xDA);  HAL_Delay(2);
    ssd1306_write_cmd(0x12);  HAL_Delay(2);
    ssd1306_write_cmd(0xDB);  HAL_Delay(2);
    ssd1306_write_cmd(0x40);  HAL_Delay(2);
    ssd1306_write_cmd(0x8D);  HAL_Delay(2);
    ssd1306_write_cmd(0x14);  HAL_Delay(10);
    ssd1306_write_cmd(0xAF);  HAL_Delay(10);
}

uint8_t SSD1306_Init(void)
{
    ssd1306_ready = 0;
    ssd1306_detected_addr = 0;

    sw_i2c_init_pins();
    HAL_Delay(50);

    /* ---- 依次探测 0x3C 和 0x3D ---- */
    static const uint8_t probe_addrs[2] = { 0x3C, 0x3D };
    for (int i = 0; i < 2; i++) {
        if (sw_i2c_probe(probe_addrs[i])) {
            ssd1306_detected_addr = probe_addrs[i];
            break;
        }
    }
    if (!ssd1306_detected_addr) return 0;

    ssd1306_ready = 1;          /* 先标记就绪, 否则 write_cmd 会跳过 */
    HAL_Delay(10);
    SSD1306_WriteInitCommands();

    memset(SSD1306_Sent, 0xFF, sizeof(SSD1306_Sent));
    SSD1306_Clear();
    SSD1306_DoRefresh();        /* 首刷全部页面 (跳过探测, 刚 init 肯定在线) */
    return 1;
}

uint8_t SSD1306_IsReady(void)
{
    return ssd1306_ready;
}

/* ========================== 绘图 ========================== */

void SSD1306_Clear(void)
{
    memset(SSD1306_Buffer, 0x00, sizeof(SSD1306_Buffer));
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    uint16_t idx = (y / 8) * SSD1306_WIDTH + x;
    uint8_t  bit = (uint8_t)(y % 8);

    if (color) {
        SSD1306_Buffer[idx] |= (1 << bit);
    } else {
        SSD1306_Buffer[idx] &= ~(1 << bit);
    }
}

void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t color)
{
    /* 仅支持可打印 ASCII, 其余用 '?' 代替 */
    if (ch < 32 || ch > 126) ch = '?';

    const uint8_t *glyph = font5x7[ch - 32];

    for (uint8_t col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                SSD1306_DrawPixel(x + col, y + row, color);
            }
        }
    }
}

void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    if (!str) return;
    uint8_t cx = x;
    while (*str) {
        SSD1306_DrawChar(cx, y, *str, color);
        cx += 6;                 /* 5px 字形 + 1px 间距 */
        str++;
    }
}

void SSD1306_DrawHLine(uint8_t x0, uint8_t x1, uint8_t y, uint8_t color)
{
    if (x1 < x0) { uint8_t t = x0; x0 = x1; x1 = t; }
    for (uint8_t x = x0; x <= x1; x++) {
        if (x < SSD1306_WIDTH && y < SSD1306_HEIGHT)
            SSD1306_DrawPixel(x, y, color);
    }
}

void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    for (uint8_t j = 0; j < h; j++) {
        for (uint8_t i = 0; i < w; i++) {
            uint8_t px = (uint8_t)(x + i);
            uint8_t py = (uint8_t)(y + j);
            if (px < SSD1306_WIDTH && py < SSD1306_HEIGHT)
                SSD1306_DrawPixel(px, py, color);
        }
    }
}

/* ========================== 刷新 ========================== */

/* ===== 内部刷新: 仅脏页刷屏, 不含探测/重初始化 (供 Init + UpdateScreen 共用) ===== */
static void SSD1306_DoRefresh(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        const uint8_t *src  = &SSD1306_Buffer[page * SSD1306_WIDTH];
        uint8_t       *sent = &SSD1306_Sent[page * SSD1306_WIDTH];
        if (memcmp(src, sent, SSD1306_WIDTH) != 0) {
            ssd1306_write_cmd(0xB0 + page);
            ssd1306_write_cmd(0x00);
            ssd1306_write_cmd(0x10);
            ssd1306_write_data(src, SSD1306_WIDTH);
            memcpy(sent, src, SSD1306_WIDTH);
        }
    }
}

void SSD1306_UpdateScreen(void)
{
    if (!ssd1306_ready) return;

    /* 关停步进中断 — 任何 I2C 操作(探测/重初始化/刷屏)都不被步进 ISR 打断 */
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_DisableIRQ(TIM3_IRQn);

    /* 健康检查: 28BYJ-48 步进切换时电流冲击可能耦合到 I2C 线(PB8/PB9),
     * 导致 SSD1306 内部状态机紊乱(表现: 屏幕不断重启/花屏/无法稳定显示坐标).
     * 探测无应答则自动重做全套初始化, 无需外部干预。 */
    if (!sw_i2c_probe(ssd1306_detected_addr)) {
        SSD1306_WriteInitCommands();          /* 全序列重初始化 */
        memset(SSD1306_Sent, 0xFF, sizeof(SSD1306_Sent));
        SSD1306_Clear();
        /* 继续执行下方的 DoRefresh, 把清除后的缓冲刷到 OLED */
    }

    SSD1306_DoRefresh();

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
}
