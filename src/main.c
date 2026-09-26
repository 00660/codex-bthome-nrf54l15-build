/*
 * nRF54L15 / E73-2G4M08S1F  BTHome 传感器固件（NTC + IP5328 + OTA）
 * ---------------------------------------------------------------------
 * 传感器
 *   1. 100k NTC 分压测温（P1.10 供电 / P1.11 AIN4 采样）
 *   2. IP5328 移动电源 SOC 的 I2C 数据（电池电压/电流/功率/充电状态/电量）
 *      —— 软件 bit-bang I2C 挂在 P1.13 / P1.14，启动时自动侦测哪个是 SCL
 *
 * 引脚（E73 模组脚 → nRF54L15）
 *   5  → P1.13   IP5328 I2C（SCL 或 SDA，运行时自动判定）
 *   6  → P1.14   IP5328 I2C（另一根）
 *   7  → P1.04   IP5328 INT/RSET 状态输入（高 = 主板醒着）
 *   8  → P1.02   IP5328 KEY 网络（NFC1，overlay 里已关 NFC）
 *   10 → P1.10   NTC 分压供电
 *   11 → P1.11   NTC ADC (AIN4)
 *
 * 运行策略（双唤醒 + 轮询 OTA 窗口）
 *   唤醒源 1：定时，每 10 分钟一轮（原机制，保留）
 *   唤醒源 2：模组第 8 脚 KEY 网络下降沿，按下按键立刻醒一轮
 *   每轮流程：采样 → 可连接广播 120 秒（OTA 窗口）→ 停止广播 → 回去睡
 *   广播期间若被连上，一直等到断开才停止广播，所以 OTA 不会被睡眠打断
 *   上电后第一轮也走同样流程，保证刷完固件还能连上验证或重刷
 */

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/app_version.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(ntc_thl, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ---------------- BTHome ---------------- */

#define BTHOME_UUID_LE_0 0xD2
#define BTHOME_UUID_LE_1 0xFC
#define BTHOME_DEVICE_INFO_V2 0x40
#define BTHOME_ID_BATTERY 0x01
#define BTHOME_ID_TEMPERATURE 0x02
#define BTHOME_ID_VOLTAGE 0x0C
#define BTHOME_ID_CURRENT_SIGNED 0x5D
#define BTHOME_ID_FIRMWARE_VERSION 0xF2

/*
 * BTHome 要求 object id 按数值从小到大排列，接收端碰到不认识的 id
 * 就直接停止解析后面的内容。所以顺序必须是 01 < 02 < 0C < 5D < F2。
 */
#define BTHOME_BATTERY_OFFSET 4U
#define BTHOME_TEMP_OFFSET 6U
#define BTHOME_VOLTAGE_OFFSET 9U
#define BTHOME_CURRENT_OFFSET 12U
#define BTHOME_VERSION_OFFSET 15U

/* ---------------- NTC ---------------- */

#define NTC_REF_OHMS 100000U
#define NTC_SAMPLE_COUNT 8U
#define NTC_SETTLE_TIME K_MSEC(50)
#define NTC_SAMPLE_INTERVAL K_MSEC(2)
#define NTC_SUPPLY_FALLBACK_MV 3300U
#define NTC_ADC_FULL_SCALE_MV 3600U
#define NTC_ADC_MAX_RAW ((1U << 12) - 1U)

#define BATTERY_FULL_MV 3000U
#define BATTERY_EMPTY_MV 2200U

/* ---------------- 运行周期 ---------------- */

/*
 * 双唤醒：
 *   1. 定时唤醒 —— 每 SAMPLE_INTERVAL 采样一次（原来的机制，保留）
 *   2. 按键唤醒 —— 模组第 8 脚（IP5328 的 KEY 网络）下降沿，立刻醒一次
 * 两者谁先来就用谁，醒来后都是同一套流程：采样 → 开 OTA 窗口 → 回去睡。
 */
#define SAMPLE_INTERVAL K_MINUTES(10)
#define OTA_WINDOW_SECONDS 120U
#define KEY_DEBOUNCE_MS 30

/*
 * 功能测试模式：不休眠，一直保持可连接广播，数据每 5 秒刷一次，
 * 这样随时都能连上去看数据，不用等 10 分钟窗口。
 *
 * 运行期可以改：手机端把 "Sleep enable" 特征写成 1 就切回正常的
 * 10 分钟周期，写 0 又回到常醒。量产前把 STAY_AWAKE_DEFAULT 改成 0。
 */
#define STAY_AWAKE_DEFAULT 1
#define TEST_SAMPLE_INTERVAL K_SECONDS(5)

/* ---------------- IP5328 ---------------- */

/* 模组脚号 → P1 引脚号 */
#define IP5328_PIN_M5 13U  /* 模组第 5 脚 = P1.13 */
#define IP5328_PIN_M6 14U  /* 模组第 6 脚 = P1.14 */
#define IP5328_PIN_M7 4U   /* 模组第 7 脚 = P1.04 = INT/RSET */
#define IP5328_PIN_M8 2U   /* 模组第 8 脚 = P1.02 = KEY 网络（NFC1） */

/* 备用的第二根 NFC 脚，要和 P1.02 保持同电平以免产生漏电流 */
#define IP5328_PIN_NFC2 3U /* 模组第 13 脚 = P1.03 = NFC2 */

#define IP5328_ADDR7 0x75U /* 写 0xEA / 读 0xEB */

#define IP5328_BIND_UNKNOWN 0xFFU
#define IP5328_BIND_A 0U /* 模组 5 脚 = SCL，6 脚 = SDA */
#define IP5328_BIND_B 1U /* 模组 6 脚 = SCL，5 脚 = SDA */

/* 软件 I2C 半周期，约 100kHz 上下，够用且对中断抖动不敏感 */
#define IP5328_BIT_DELAY_US 2U

#define IP5328_REPORT_LEN 16U

/*
 * 总线诊断结果，只读。IP5328 读不通时靠它远程判断卡在哪一步：
 * 是线没上拉、主板没醒、还是从机根本不应答。
 */
#define IP5328_DIAG_LEN 24U

#define IP5328_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0200, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define IP5328_REPORT_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0201, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define IP5328_DIAG_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0202, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)

/* 版本号特征，只读，手机端直接看字符串，不用去解广播 */
#define APP_INFO_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0300, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define APP_VERSION_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0301, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define APP_SLEEP_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0302, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)

/* ---------------- GPIO 测试开关 ---------------- */

#define GPIO_SWITCH_COUNT 22U

#define GPIO_SWITCH_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0000, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define GPIO_SWITCH_UUID_VAL(index) \
	BT_UUID_128_ENCODE((0x6F6B0100U + (index)), 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)

/*
 * 已从列表移除、被 IP5328 占用的脚：
 *   P1.02（模组 8 脚，KEY）、P1.03（NFC2，跟随 P1.02 电平）、
 *   P1.04（模组 7 脚，INT）、P1.13 / P1.14（模组 5/6 脚，I2C）
 * 其余引脚的 UUID 编号保持不变，手机端原来的配置不会错位。
 */
#define GPIO_SWITCH_LIST(X) \
	X(0, gpio0, 0, "P0.00") \
	X(1, gpio0, 1, "P0.01") \
	X(2, gpio0, 2, "P0.02") \
	X(3, gpio0, 3, "P0.03") \
	X(4, gpio0, 4, "P0.04") \
	X(8, gpio1, 5, "P1.05") \
	X(9, gpio1, 6, "P1.06") \
	X(10, gpio1, 7, "P1.07") \
	X(11, gpio1, 8, "P1.08") \
	X(12, gpio1, 9, "P1.09") \
	X(13, gpio1, 12, "P1.12") \
	X(16, gpio2, 0, "P2.00") \
	X(17, gpio2, 1, "P2.01") \
	X(18, gpio2, 2, "P2.02") \
	X(19, gpio2, 3, "P2.03") \
	X(20, gpio2, 4, "P2.04") \
	X(21, gpio2, 5, "P2.05") \
	X(22, gpio2, 6, "P2.06") \
	X(23, gpio2, 7, "P2.07") \
	X(24, gpio2, 8, "P2.08") \
	X(25, gpio2, 9, "P2.09") \
	X(26, gpio2, 10, "P2.10")

struct ntc_point {
	int16_t temp_x10;
	uint32_t ohms;
};

struct ntc_capture {
	int32_t temp_centi;
	uint32_t ntc_ohms;
	uint16_t adc_mv;
	uint16_t vdd_mv;
	uint16_t adc_min_mv;
	uint16_t adc_max_mv;
	uint16_t sample_count;
};

struct soc_point {
	uint16_t mv;
	uint8_t pct;
};

struct ip5328_data {
	uint8_t valid;
	uint8_t bind;
	uint8_t sys_state;
	uint8_t charge_stage;
	uint8_t charging;
	uint8_t full;
	uint8_t soc;
	int8_t error;
	uint16_t batocv_mv;
	uint16_t batvad_mv;
	int16_t bat_ma;
	uint16_t vsys_mv;
	int16_t vsys_ma;
	uint32_t power_mw;
};

struct gpio_switch {
	const struct device *port;
	gpio_pin_t pin;
	const char *name;
	uint8_t value;
};

static const struct ntc_point ntc_table[] = {
	{ -200, 1053847U },
	{ -150, 778981U },
	{ -100, 582457U },
	{ -50, 440260U },
	{ 0, 336206U },
	{ 50, 259246U },
	{ 100, 201746U },
	{ 150, 158371U },
	{ 200, 125353U },
	{ 250, 100000U },
	{ 300, 80371U },
	{ 350, 65055U },
	{ 400, 53015U },
	{ 450, 43481U },
	{ 500, 35882U },
	{ 550, 29784U },
	{ 600, 24862U },
	{ 650, 20864U },
	{ 700, 17598U },
	{ 750, 14917U },
	{ 800, 12703U },
};

/*
 * 单节高压锂电（4.35V 满充）开路电压 → 电量。
 * IP5328 没有直接给 SOC 百分比，只有 BATOCV，所以自己查表。
 */
static const struct soc_point soc_table[] = {
	{ 4350U, 100U }, { 4250U, 95U }, { 4180U, 88U }, { 4100U, 80U },
	{ 4020U, 70U },  { 3950U, 62U }, { 3880U, 53U }, { 3820U, 45U },
	{ 3760U, 36U },  { 3700U, 28U }, { 3650U, 21U }, { 3600U, 14U },
	{ 3550U, 9U },   { 3500U, 5U },  { 3400U, 2U },  { 3300U, 0U },
};

static const struct adc_dt_spec ntc_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec vdd_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct gpio_dt_spec ntc_power = GPIO_DT_SPEC_GET(DT_ALIAS(ntcpower), gpios);
static const struct device *const ip_port = DEVICE_DT_GET(DT_NODELABEL(gpio1));

#define GPIO_SWITCH_ENTRY(index, port_node, pin_number, label) \
	{ \
		.port = DEVICE_DT_GET(DT_NODELABEL(port_node)), \
		.pin = pin_number, \
		.name = label, \
	},

static struct gpio_switch gpio_switches[] = {
	GPIO_SWITCH_LIST(GPIO_SWITCH_ENTRY)
};

#define GPIO_SWITCH_UUID_DEFINE(index, port_node, pin_number, label) \
	static const struct bt_uuid_128 gpio_switch_uuid_##index = \
		BT_UUID_INIT_128(GPIO_SWITCH_UUID_VAL(index));

static const struct bt_uuid_128 gpio_switch_service_uuid =
	BT_UUID_INIT_128(GPIO_SWITCH_SERVICE_UUID_VAL);
GPIO_SWITCH_LIST(GPIO_SWITCH_UUID_DEFINE)

static const struct bt_uuid_128 ip5328_service_uuid = BT_UUID_INIT_128(IP5328_SERVICE_UUID_VAL);
static const struct bt_uuid_128 ip5328_report_uuid = BT_UUID_INIT_128(IP5328_REPORT_UUID_VAL);
static const struct bt_uuid_128 ip5328_diag_uuid = BT_UUID_INIT_128(IP5328_DIAG_UUID_VAL);
static const struct bt_uuid_128 app_info_service_uuid = BT_UUID_INIT_128(APP_INFO_SERVICE_UUID_VAL);
static const struct bt_uuid_128 app_version_uuid = BT_UUID_INIT_128(APP_VERSION_UUID_VAL);
static const struct bt_uuid_128 app_sleep_uuid = BT_UUID_INIT_128(APP_SLEEP_UUID_VAL);

BUILD_ASSERT(ARRAY_SIZE(gpio_switches) == GPIO_SWITCH_COUNT);

static bool connected;
static bool vdd_adc_ready;
static int16_t adc_sample_buffer[2];

/* 1 = 允许休眠（正常 10 分钟周期），0 = 测试模式常醒 */
static bool sleep_enabled = !STAY_AWAKE_DEFAULT;

static uint32_t ip_scl_pin;
static uint32_t ip_sda_pin;
static uint8_t ip_bind = IP5328_BIND_UNKNOWN;

/* 第 8 脚（KEY 网络）按键唤醒 */
K_SEM_DEFINE(wake_sem, 0, 1);
static struct gpio_callback key_cb;
static volatile uint32_t key_wake_count;

/*
 * BTHome service data，18 字节：
 *   D2 FC         BTHome UUID，小端
 *   40            BTHome v2，未加密
 *   01 BB         battery，uint8，%
 *   02 TT TT      temperature，sint16，0.01 °C
 *   0C VV VV      voltage，uint16，0.001 V
 *   5D CC CC      current (signed)，sint16，0.001 A，充正放负
 *   F2 PP MM JJ   firmware version，patch/minor/major
 */
static uint8_t bthome_service_data[] = {
	BTHOME_UUID_LE_0,
	BTHOME_UUID_LE_1,
	BTHOME_DEVICE_INFO_V2,
	BTHOME_ID_BATTERY,
	0x00,
	BTHOME_ID_TEMPERATURE,
	0x00,
	0x00,
	BTHOME_ID_VOLTAGE,
	0x00,
	0x00,
	BTHOME_ID_CURRENT_SIGNED,
	0x00,
	0x00,
	BTHOME_ID_FIRMWARE_VERSION,
	APP_PATCHLEVEL,
	APP_VERSION_MINOR,
	APP_VERSION_MAJOR,
};

BUILD_ASSERT(sizeof(bthome_service_data) == BTHOME_VERSION_OFFSET + 3U);

/* IP5328 全量数据，给 GATT 只读特征值用 */
static uint8_t ip5328_report[IP5328_REPORT_LEN];

/*
 * 广播包只有 31 字节，加了 current 之后 BTHome service data 变成 18 字节，
 * 再塞完整设备名就超了。所以设备名挪到 scan response 里，两边都放得下：
 *   ad = flags(3) + service data(20) = 23 字节
 *   sd = name(9) + 128bit UUID(18)   = 27 字节
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_SVC_DATA16, bthome_service_data, sizeof(bthome_service_data)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0x84, 0xAA, 0x60, 0x74, 0x52, 0x8A, 0x8B, 0x86,
		      0xD3, 0x4C, 0xB7, 0x1D, 0x1D, 0xDC, 0x53, 0x8D),
};

/* ============================================================
 *  IP5328 软件 I2C
 * ============================================================ */

/* 软件 I2C 半周期，约 100kHz 上下，够用且对中断抖动不敏感 */
static uint32_t ip_bit_delay_us = IP5328_BIT_DELAY_US;

static inline void ip_dly(void)
{
	k_busy_wait(ip_bit_delay_us);
}

/*
 * 诊断时可以切到慢速。从机如果嫌快不应答，慢下来就能通，
 * 这样能区分"线没接好"和"时序太快"两种完全不同的故障。
 * 三级：2µs（~150kHz）/ 20µs（~25kHz）/ 100µs（~5kHz）。
 */
static void ip_i2c_set_delay(uint32_t us)
{
	ip_bit_delay_us = us;
}

#define IP5328_DELAY_FAST 2U
#define IP5328_DELAY_MID 20U
#define IP5328_DELAY_SLOW 100U

static inline void ip_scl_low(void)
{
	(void)gpio_pin_configure(ip_port, ip_scl_pin, GPIO_OUTPUT_LOW);
}

static inline void ip_scl_rel(void)
{
	(void)gpio_pin_configure(ip_port, ip_scl_pin, GPIO_INPUT | GPIO_PULL_UP);
}

static inline void ip_sda_low(void)
{
	(void)gpio_pin_configure(ip_port, ip_sda_pin, GPIO_OUTPUT_LOW);
}

static inline void ip_sda_rel(void)
{
	(void)gpio_pin_configure(ip_port, ip_sda_pin, GPIO_INPUT | GPIO_PULL_UP);
}

static inline int ip_sda_get(void)
{
	return gpio_pin_get(ip_port, ip_sda_pin);
}

static void ip_i2c_bind(uint32_t scl, uint32_t sda)
{
	ip_scl_pin = scl;
	ip_sda_pin = sda;
	ip_scl_rel();
	ip_sda_rel();
}

static void ip_i2c_start(void)
{
	ip_sda_rel();
	ip_dly();
	ip_scl_rel();
	ip_dly();
	ip_sda_low();
	ip_dly();
	ip_scl_low();
	ip_dly();
}

static void ip_i2c_stop(void)
{
	ip_sda_low();
	ip_dly();
	ip_scl_rel();
	ip_dly();
	ip_sda_rel();
	ip_dly();
}

/*
 * 9 个时钟 + STOP，把卡在半个字节里、正拉着 SDA 不放的从机踢出来。
 * 从机如果死在读数据中间，不发这个它就一直不应答。
 */
static void ip_i2c_bus_recover(void)
{
	ip_sda_rel();
	ip_dly();

	for (int i = 0; i < 9; i++) {
		ip_scl_low();
		ip_dly();
		ip_scl_rel();
		ip_dly();
	}

	ip_i2c_stop();
}

/* 返回 0 = 收到 ACK */
static int ip_i2c_write_byte(uint8_t b)
{
	int nak;

	for (int i = 0; i < 8; i++) {
		if (b & 0x80U) {
			ip_sda_rel();
		} else {
			ip_sda_low();
		}
		ip_dly();
		ip_scl_rel();
		ip_dly();
		ip_scl_low();
		ip_dly();
		b = (uint8_t)(b << 1);
	}

	ip_sda_rel();
	ip_dly();
	ip_scl_rel();
	ip_dly();
	nak = ip_sda_get();
	ip_scl_low();
	ip_dly();

	return nak;
}

static uint8_t ip_i2c_read_byte(int ack)
{
	uint8_t b = 0;

	ip_sda_rel();
	ip_dly();
	for (int i = 0; i < 8; i++) {
		ip_scl_rel();
		ip_dly();
		b = (uint8_t)((b << 1) | (ip_sda_get() ? 1U : 0U));
		ip_scl_low();
		ip_dly();
	}

	if (ack) {
		ip_sda_low();
	} else {
		ip_sda_rel();
	}
	ip_dly();
	ip_scl_rel();
	ip_dly();
	ip_scl_low();
	ip_dly();
	ip_sda_rel();
	ip_dly();

	return b;
}

static int ip_i2c_probe(uint8_t addr7)
{
	int nak;

	ip_i2c_start();
	nak = ip_i2c_write_byte((uint8_t)((addr7 << 1) | 0U));
	ip_i2c_stop();

	return nak;
}

static int ip_i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *out)
{
	ip_i2c_start();
	if (ip_i2c_write_byte((uint8_t)((addr7 << 1) | 0U))) {
		ip_i2c_stop();
		return -1;
	}
	if (ip_i2c_write_byte(reg)) {
		ip_i2c_stop();
		return -2;
	}
	ip_i2c_start();
	if (ip_i2c_write_byte((uint8_t)((addr7 << 1) | 1U))) {
		ip_i2c_stop();
		return -3;
	}
	*out = ip_i2c_read_byte(0);
	ip_i2c_stop();

	return 0;
}

/* IP5328 的 16bit 量都是小端地址：低字节地址在前 */
static int ip_i2c_read_reg16(uint8_t addr7, uint8_t addr_lo, uint8_t addr_hi, uint32_t *out)
{
	uint8_t lo;
	uint8_t hi;

	if (ip_i2c_read_reg(addr7, addr_lo, &lo) != 0) {
		return -1;
	}
	if (ip_i2c_read_reg(addr7, addr_hi, &hi) != 0) {
		return -1;
	}
	*out = ((uint32_t)hi << 8) | lo;

	return 0;
}

/*
 * 自动判定模组第 5/6 脚谁是 SCL。
 * 用户不确定焊的是 5=SCL/6=SDA 还是反的，所以两个组合都试一遍，
 * 谁收到 0xEA 的 ACK 就用谁，之后固定不再重试。
 */
static int ip5328_ensure_bind(void)
{
	if (ip_bind != IP5328_BIND_UNKNOWN) {
		return 0;
	}

	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	k_msleep(2);
	if (ip_i2c_probe(IP5328_ADDR7) == 0) {
		ip_bind = IP5328_BIND_A;
		return 0;
	}

	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	k_msleep(2);
	if (ip_i2c_probe(IP5328_ADDR7) == 0) {
		ip_bind = IP5328_BIND_B;
		return 0;
	}

	/* 都不通：保持待定，下一轮再试 */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	return -ENODEV;
}

static uint8_t soc_from_mv(uint16_t mv)
{
	const size_t last = ARRAY_SIZE(soc_table) - 1U;

	if (mv >= soc_table[0].mv) {
		return soc_table[0].pct;
	}
	if (mv <= soc_table[last].mv) {
		return soc_table[last].pct;
	}

	for (size_t i = 0; i < last; i++) {
		uint16_t hi_mv = soc_table[i].mv;
		uint16_t lo_mv = soc_table[i + 1].mv;

		if (mv <= hi_mv && mv >= lo_mv) {
			uint16_t span = hi_mv - lo_mv;
			uint16_t offset = hi_mv - mv;
			int32_t hi_pct = soc_table[i].pct;
			int32_t lo_pct = soc_table[i + 1].pct;
			int32_t pct = hi_pct + ((lo_pct - hi_pct) * (int32_t)offset + (int32_t)span / 2) /
						       (int32_t)span;

			return (uint8_t)CLAMP(pct, 0, 100);
		}
	}

	return 0U;
}

static int ip5328_read_registers(struct ip5328_data *d)
{
	uint8_t v;
	uint32_t raw;

	if (ip_i2c_read_reg(IP5328_ADDR7, 0xD1, &v) != 0) {
		return -EIO;
	}
	d->sys_state = v & 0x07U;
	d->charging = (v >> 4) & 0x01U;
	d->full = (v >> 6) & 0x01U;

	if (ip_i2c_read_reg(IP5328_ADDR7, 0xD7, &v) == 0) {
		d->charge_stage = v & 0x07U;
	}

	/* 0x7B:0x7A BATOCV，补偿内阻与滤波后的开路电压 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x7A, 0x7B, &raw) == 0) {
		d->batocv_mv = (uint16_t)(((uint64_t)raw * 26855U) / 100000U + 2600U);
		d->soc = soc_from_mv(d->batocv_mv);
	}

	/* 0x65:0x64 电池真实端电压 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x64, 0x65, &raw) == 0) {
		d->batvad_mv = (uint16_t)(((uint64_t)raw * 26855U) / 100000U + 2600U);
	}

	/* 0x67:0x66 电池端电流，补码，充电为正 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x66, 0x67, &raw) == 0) {
		d->bat_ma = (int16_t)(((int64_t)(int16_t)raw * 127883) / 100000);
	}

	/* 0x69:0x68 VSYS 电压 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x68, 0x69, &raw) == 0) {
		d->vsys_mv = (uint16_t)(((uint64_t)raw * 161133U) / 100000U + 15600U);
	}

	/* 0x6B:0x6A VSYS 电流 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x6A, 0x6B, &raw) == 0) {
		d->vsys_ma = (int16_t)(((int64_t)(int16_t)raw * 63940) / 100000);
	}

	/* 0x7D:0x7C 输入/输出功率 */
	if (ip_i2c_read_reg16(IP5328_ADDR7, 0x7C, 0x7D, &raw) == 0) {
		d->power_mw = (uint32_t)(((uint64_t)raw * 844U) / 100U);
	}

	return 0;
}

/*
 * 模组第 8 脚 = P1.02 接的是 IP5328 的 KEY 网络。
 * 按键按下时 KEY 被拉到地，用这个下降沿唤醒 MCU。
 *
 * 引脚配成纯高阻输入，不开内部上拉：KEY 网络在 IP5328 内部本来就有上拉，
 * 再叠一个 MCU 内部上拉只会白耗 ~240µA，把深睡电流整个毁掉。
 */
static void key_pressed_cb(const struct device *port, struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	key_wake_count++;
	k_sem_give(&wake_sem);
}

static int configure_key_wakeup(void)
{
	int ret;

	ret = gpio_pin_configure(ip_port, IP5328_PIN_M8, GPIO_INPUT);
	if (ret) {
		LOG_ERR("KEY pin configure failed: %d", ret);
		return ret;
	}

	gpio_init_callback(&key_cb, key_pressed_cb, BIT(IP5328_PIN_M8));

	ret = gpio_add_callback(ip_port, &key_cb);
	if (ret) {
		LOG_ERR("KEY callback add failed: %d", ret);
		return ret;
	}

	/* 引脚没有 ACTIVE_LOW 标志，所以 edge-to-inactive 就是下降沿 */
	ret = gpio_pin_interrupt_configure(ip_port, IP5328_PIN_M8, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret) {
		LOG_ERR("KEY interrupt configure failed: %d", ret);
	}

	return ret;
}

static int ip5328_int_level(void)
{
	if (!device_is_ready(ip_port)) {
		return -1;
	}

	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);

	return gpio_pin_get(ip_port, IP5328_PIN_M7);
}

/* ============================================================
 *  I2C 总线诊断
 * ============================================================ */

static uint8_t ip5328_diag[IP5328_DIAG_LEN];

static int ip_pin_level(uint32_t pin, gpio_flags_t flags)
{
	(void)gpio_pin_configure(ip_port, pin, GPIO_INPUT | flags);
	k_busy_wait(20);

	return gpio_pin_get(ip_port, pin) ? 1 : 0;
}

/*
 * 把线驱动到低，再放开成高阻，然后立刻读。
 * 线上有外部上拉（或对端在推高）就会马上回到高；什么都没接就停在低。
 * 这是判断"上拉到底在不在"最直接的办法，内部上下拉都太弱，分不清。
 */
static int ip_line_has_pullup(uint32_t pin)
{
	(void)gpio_pin_configure(ip_port, pin, GPIO_OUTPUT_LOW);
	k_busy_wait(50);
	(void)gpio_pin_configure(ip_port, pin, GPIO_INPUT);
	k_busy_wait(50);

	return gpio_pin_get(ip_port, pin) ? 1 : 0;
}

/*
 * 布局（24 字节）：
 *   [0]     标志：bit0 已跑过，bit1 组合A 2µs ACK，bit2 组合B 2µs ACK，
 *                bit3 总线恢复后 SDA 仍被拉低（从机卡住总线）
 *   [1]     INT/RSET(P1.04) 电平，1 = 主板醒着
 *   [2..3]  SCL/SDA 纯高阻电平
 *   [4..5]  SCL/SDA 内部上拉电平
 *   [6..7]  SCL/SDA 内部下拉电平
 *   [8..9]  SCL/SDA 释放后恢复电平，1 = 有外部上拉
 *   [10..11] 组合A/B 2µs   probe 0xEA，0 = 收到 ACK
 *   [12..13] 组合A/B 20µs  probe 0xEA，0 = 收到 ACK
 *   [14..15] 组合A/B 100µs probe 0xEA，0 = 收到 ACK
 *   [16]    组合A 全地址扫描命中数（2µs）
 *   [17..19] 组合A 前 3 个命中地址（7bit）
 *   [20]    组合B 全地址扫描命中数（2µs）
 *   [21]    组合B 首个命中地址
 *   [22]    组合A 全地址扫描命中数（100µs）
 *   [23]    组合A 慢速首个命中地址
 */
static void ip5328_diag_run(void)
{
	uint8_t hits_a[3] = { 0 };
	uint8_t first_b = 0U;
	uint8_t first_slow = 0U;
	uint8_t n_a = 0U;
	uint8_t n_b = 0U;
	uint8_t n_slow = 0U;

	ip5328_diag[1] = (uint8_t)(ip5328_int_level() > 0 ? 1U : 0U);

	/* 先量静态电平，探测本身会扰动总线 */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip5328_diag[2] = (uint8_t)ip_pin_level(IP5328_PIN_M5, 0);
	ip5328_diag[3] = (uint8_t)ip_pin_level(IP5328_PIN_M6, 0);
	ip5328_diag[4] = (uint8_t)ip_pin_level(IP5328_PIN_M5, GPIO_PULL_UP);
	ip5328_diag[5] = (uint8_t)ip_pin_level(IP5328_PIN_M6, GPIO_PULL_UP);
	ip5328_diag[6] = (uint8_t)ip_pin_level(IP5328_PIN_M5, GPIO_PULL_DOWN);
	ip5328_diag[7] = (uint8_t)ip_pin_level(IP5328_PIN_M6, GPIO_PULL_DOWN);
	ip5328_diag[8] = (uint8_t)ip_line_has_pullup(IP5328_PIN_M5);
	ip5328_diag[9] = (uint8_t)ip_line_has_pullup(IP5328_PIN_M6);

	/* 三级速率各试两个引脚组合，每次先做一次总线恢复 */
	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[10] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);
	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	ip_i2c_bus_recover();
	ip5328_diag[11] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	ip_i2c_set_delay(IP5328_DELAY_MID);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[12] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);
	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	ip_i2c_bus_recover();
	ip5328_diag[13] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	ip_i2c_set_delay(IP5328_DELAY_SLOW);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[14] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);
	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	ip_i2c_bus_recover();
	ip5328_diag[15] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	/* 全地址扫一遍：万一从机地址不是 0x75，也能发现 */
	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			if (n_a < ARRAY_SIZE(hits_a)) {
				hits_a[n_a] = a;
			}
			n_a++;
		}
	}
	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			if (n_b == 0U) {
				first_b = a;
			}
			n_b++;
		}
	}

	/* 慢速再扫一遍，万一是从机嫌快 */
	ip_i2c_set_delay(IP5328_DELAY_SLOW);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			if (n_slow == 0U) {
				first_slow = a;
			}
			n_slow++;
		}
	}

	ip5328_diag[0] = (uint8_t)(1U | (ip5328_diag[10] == 0 ? 0x02U : 0U) |
				   (ip5328_diag[11] == 0 ? 0x04U : 0U));
	ip5328_diag[16] = n_a;
	ip5328_diag[17] = hits_a[0];
	ip5328_diag[18] = hits_a[1];
	ip5328_diag[19] = hits_a[2];
	ip5328_diag[20] = n_b;
	ip5328_diag[21] = first_b;
	ip5328_diag[22] = n_slow;
	ip5328_diag[23] = first_slow;

	/* 哪个速率能通就固定用哪个，免得真正读数据时又失败 */
	if (ip5328_diag[10] != 0 && ip5328_diag[11] != 0 &&
	    (ip5328_diag[12] == 0 || ip5328_diag[13] == 0)) {
		ip_i2c_set_delay(IP5328_DELAY_MID);
	} else if (ip5328_diag[10] != 0 && ip5328_diag[11] != 0 &&
		   ip5328_diag[14] != 0 && ip5328_diag[15] != 0 && n_slow > 0U) {
		ip_i2c_set_delay(IP5328_DELAY_SLOW);
	} else {
		ip_i2c_set_delay(IP5328_DELAY_FAST);
	}

	/* 收摊前看看总线有没有被从机拉死 */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip_sda_rel();
	k_busy_wait(50);
	if (!ip_sda_get()) {
		ip5328_diag[0] |= 0x08U;
	}

	LOG_INF("diag int=%u idle=%u%u pup=%u%u pdn=%u%u rec=%u%u "
		"nak=%u%u/%u%u/%u%u hits=%u,%u,%u stuck=%u",
		ip5328_diag[1], ip5328_diag[2], ip5328_diag[3], ip5328_diag[4],
		ip5328_diag[5], ip5328_diag[6], ip5328_diag[7], ip5328_diag[8],
		ip5328_diag[9], ip5328_diag[10], ip5328_diag[11], ip5328_diag[12],
		ip5328_diag[13], ip5328_diag[14], ip5328_diag[15], n_a, n_b, n_slow,
		(ip5328_diag[0] >> 3) & 1U);
}

static int ip5328_sample(struct ip5328_data *d)
{
	int ret;

	*d = (struct ip5328_data){
		.bind = ip_bind,
	};

	ret = ip5328_ensure_bind();
	if (ret) {
		d->error = (int8_t)ret;
		return ret;
	}

	d->bind = ip_bind;

	/*
	 * 不管是被定时唤醒还是按键唤醒，IP5328 那边要么本来就醒着，
	 * 要么被同一次按键一起叫醒，所以这里直接读一次即可。
	 */
	if (ip5328_read_registers(d) == 0) {
		d->valid = 1U;
		return 0;
	}

	d->error = -EIO;
	return -EIO;
}

static void ip5328_encode_report(const struct ip5328_data *d)
{
	ip5328_report[0] = d->valid ? (uint8_t)(1U + d->bind) : 0U;
	ip5328_report[1] = (uint8_t)((d->sys_state & 0x07U) | (d->charging ? 0x10U : 0U) |
				     (d->full ? 0x40U : 0U));
	ip5328_report[2] = (uint8_t)(d->charge_stage & 0x07U);
	ip5328_report[3] = d->soc;
	sys_put_le16(d->batocv_mv, &ip5328_report[4]);
	sys_put_le16(d->batvad_mv, &ip5328_report[6]);
	sys_put_le16((uint16_t)(int16_t)d->bat_ma, &ip5328_report[8]);
	sys_put_le16(d->vsys_mv, &ip5328_report[10]);
	sys_put_le16((uint16_t)(int16_t)d->vsys_ma, &ip5328_report[12]);
	sys_put_le16((uint16_t)MIN(d->power_mw, 0xFFFFU), &ip5328_report[14]);
}

static int configure_ip5328_io(void)
{
	if (!device_is_ready(ip_port)) {
		LOG_ERR("gpio1 is not ready");
		return -ENODEV;
	}

	/* I2C 两脚先放开，外部 4.7k 上拉到 IP5328 的 VREG */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);

	/* INT：主板醒着为高，平时下拉成确定电平 */
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);

	/* 第 8 脚的 KEY 中断在 configure_key_wakeup() 里单独配 */

	/*
	 * P1.03 是 NFC2，和 P1.02 一样是 NFC 脚。两个 NFC 脚被驱动到不同电平
	 * 会有额外漏电流，所以也配成纯高阻，跟 P1.02 一样不驱动、不加上拉。
	 */
	(void)gpio_pin_configure(ip_port, IP5328_PIN_NFC2, GPIO_INPUT);

	return 0;
}

/* ============================================================
 *  NTC
 * ============================================================ */

static int32_t clamp_s16(int32_t value)
{
	return CLAMP(value, INT16_MIN, INT16_MAX);
}

static void put_s16_le(uint8_t *dst, int32_t value)
{
	sys_put_le16((uint16_t)(int16_t)clamp_s16(value), dst);
}

static uint8_t battery_percent_from_mv(uint16_t mv)
{
	if (mv >= BATTERY_FULL_MV) {
		return 100U;
	}

	if (mv <= BATTERY_EMPTY_MV) {
		return 0U;
	}

	return (uint8_t)(((uint32_t)(mv - BATTERY_EMPTY_MV) * 100U +
			  (BATTERY_FULL_MV - BATTERY_EMPTY_MV) / 2U) /
			 (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
}

static void ntc_power_off(void)
{
	int ret = gpio_pin_configure_dt(&ntc_power, GPIO_DISCONNECTED);

	if (ret) {
		(void)gpio_pin_configure_dt(&ntc_power, GPIO_INPUT);
	}
}

static int configure_io(void)
{
	if (!gpio_is_ready_dt(&ntc_power)) {
		LOG_ERR("NTC power GPIO device is not ready");
		return -ENODEV;
	}

	ntc_power_off();
	return 0;
}

static int configure_adc(void)
{
	int ret;

	if (!adc_is_ready_dt(&ntc_adc)) {
		LOG_ERR("NTC ADC device is not ready");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&ntc_adc);
	if (ret) {
		LOG_ERR("NTC ADC setup failed: %d", ret);
		return ret;
	}

	vdd_adc_ready = false;
	if (!adc_is_ready_dt(&vdd_adc)) {
		LOG_WRN("VDD ADC device is not ready, using %umV fallback", NTC_SUPPLY_FALLBACK_MV);
		return 0;
	}

	ret = adc_channel_setup_dt(&vdd_adc);
	if (ret) {
		LOG_WRN("VDD ADC setup failed: %d, using %umV fallback", ret, NTC_SUPPLY_FALLBACK_MV);
		return 0;
	}

	vdd_adc_ready = true;
	return 0;
}

static int read_adc_mv(const struct adc_dt_spec *channel, int32_t *mv)
{
	struct adc_sequence sequence = { 0 };
	int32_t value;
	int ret;

	adc_sequence_init_dt(channel, &sequence);
	sequence.buffer = adc_sample_buffer;
	sequence.buffer_size = sizeof(adc_sample_buffer);

	ret = adc_read_dt(channel, &sequence);
	if (ret) {
		return ret;
	}

	value = adc_sample_buffer[0];
	ret = adc_raw_to_millivolts_dt(channel, &value);
	if (ret) {
		value = (int32_t)(((int64_t)adc_sample_buffer[0] * NTC_ADC_FULL_SCALE_MV +
				  NTC_ADC_MAX_RAW / 2U) /
				  NTC_ADC_MAX_RAW);
	}

	*mv = value;
	return 0;
}

static int32_t ntc_ohms_to_centi(uint32_t ohms)
{
	if (ohms >= ntc_table[0].ohms) {
		return ntc_table[0].temp_x10 * 10;
	}

	if (ohms <= ntc_table[ARRAY_SIZE(ntc_table) - 1].ohms) {
		return ntc_table[ARRAY_SIZE(ntc_table) - 1].temp_x10 * 10;
	}

	for (size_t i = 0; i + 1 < ARRAY_SIZE(ntc_table); i++) {
		uint32_t r0 = ntc_table[i].ohms;
		uint32_t r1 = ntc_table[i + 1].ohms;

		if (ohms <= r0 && ohms >= r1) {
			int32_t t0 = ntc_table[i].temp_x10;
			int32_t t1 = ntc_table[i + 1].temp_x10;
			uint32_t span = r0 - r1;
			uint32_t offset = r0 - ohms;
			int32_t temp_x10 = t0 + (int32_t)(((int64_t)(t1 - t0) * offset + span / 2U) / span);

			return temp_x10 * 10;
		}
	}

	return 0;
}

static uint32_t ntc_resistance_ohms(uint32_t adc_mv, uint32_t vdd_mv)
{
	uint32_t denominator;

	if (adc_mv == 0 || adc_mv >= vdd_mv) {
		return 0;
	}

	denominator = vdd_mv - adc_mv;
	return (uint32_t)(((uint64_t)NTC_REF_OHMS * adc_mv + denominator / 2U) / denominator);
}

static int sample_ntc(struct ntc_capture *capture)
{
	uint32_t adc_sum = 0;
	uint32_t vdd_sum = 0;
	uint16_t adc_min = UINT16_MAX;
	uint16_t adc_max = 0;
	int32_t adc_mv;
	int32_t vdd_mv;
	int last_error = -EIO;
	int ret;

	*capture = (struct ntc_capture){ 0 };

	ret = gpio_pin_configure_dt(&ntc_power, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("NTC power on failed: %d", ret);
		return ret;
	}

	k_sleep(NTC_SETTLE_TIME);

	for (size_t i = 0; i < NTC_SAMPLE_COUNT; i++) {
		ret = read_adc_mv(&ntc_adc, &adc_mv);
		if (ret) {
			LOG_WRN("NTC ADC read failed: %d", ret);
			last_error = ret;
			continue;
		}

		if (vdd_adc_ready) {
			ret = read_adc_mv(&vdd_adc, &vdd_mv);
			if (ret) {
				LOG_WRN("VDD ADC read failed: %d, using %umV fallback", ret,
					NTC_SUPPLY_FALLBACK_MV);
				vdd_mv = NTC_SUPPLY_FALLBACK_MV;
			}
		} else {
			vdd_mv = NTC_SUPPLY_FALLBACK_MV;
		}

		adc_mv = CLAMP(adc_mv, 0, UINT16_MAX);
		vdd_mv = CLAMP(vdd_mv, 0, UINT16_MAX);
		adc_sum += (uint32_t)adc_mv;
		vdd_sum += (uint32_t)vdd_mv;
		adc_min = MIN(adc_min, (uint16_t)adc_mv);
		adc_max = MAX(adc_max, (uint16_t)adc_mv);
		capture->sample_count++;
		k_sleep(NTC_SAMPLE_INTERVAL);
	}

	ntc_power_off();

	if (capture->sample_count == 0) {
		return last_error;
	}

	capture->adc_mv = (uint16_t)(adc_sum / capture->sample_count);
	capture->vdd_mv = (uint16_t)(vdd_sum / capture->sample_count);
	capture->adc_min_mv = adc_min;
	capture->adc_max_mv = adc_max;
	capture->ntc_ohms = ntc_resistance_ohms(capture->adc_mv, capture->vdd_mv);
	capture->temp_centi = ntc_ohms_to_centi(capture->ntc_ohms);

	return 0;
}

static void set_error_capture(struct ntc_capture *capture, int error)
{
	uint32_t code = error < 0 ? (uint32_t)-error : (uint32_t)error;

	*capture = (struct ntc_capture){
		.temp_centi = INT16_MIN,
		.ntc_ohms = 0xE00000U | MIN(code, 0xFFFFU),
	};
}

static void encode_sensors(const struct ntc_capture *ntc, const struct ip5328_data *ip,
			   int int_level)
{
	uint8_t soc;
	uint16_t volt_mv;

	if (ip->valid) {
		soc = ip->soc;
		volt_mv = ip->batocv_mv;
	} else {
		soc = battery_percent_from_mv(ntc->vdd_mv);
		volt_mv = ntc->vdd_mv;
	}

	bthome_service_data[BTHOME_BATTERY_OFFSET] = soc;
	put_s16_le(&bthome_service_data[BTHOME_TEMP_OFFSET], ntc->temp_centi);
	sys_put_le16(volt_mv, &bthome_service_data[BTHOME_VOLTAGE_OFFSET]);
	bthome_service_data[BTHOME_VERSION_OFFSET] = APP_PATCHLEVEL;
	bthome_service_data[BTHOME_VERSION_OFFSET + 1U] = APP_VERSION_MINOR;
	bthome_service_data[BTHOME_VERSION_OFFSET + 2U] = APP_VERSION_MAJOR;
	put_s16_le(&bthome_service_data[BTHOME_CURRENT_OFFSET], ip->valid ? ip->bat_ma : 0);

	ip5328_encode_report(ip);

	LOG_INF("temp=%d.%02dC ntc=%uohm adc=%umV vdd=%umV samples=%u", ntc->temp_centi / 100,
		abs(ntc->temp_centi % 100), ntc->ntc_ohms, ntc->adc_mv, ntc->vdd_mv,
		ntc->sample_count);
	LOG_INF("ip5328 valid=%u bind=%u err=%d st=%u chg=%u full=%u stage=%u soc=%u "
		"ocv=%umV vad=%umV i=%dmA vsys=%umV isys=%dmA p=%umW int=%d",
		ip->valid, ip->bind, ip->error, ip->sys_state, ip->charging, ip->full,
		ip->charge_stage, ip->soc, ip->batocv_mv, ip->batvad_mv, ip->bat_ma, ip->vsys_mv,
		ip->vsys_ma, ip->power_mw, int_level);
}

/* 每次采样后刷新 BTHome 广播数据和 IP5328 的 GATT 报告，顺带读一次 INT 电平 */
static void publish_sensors(const struct ntc_capture *ntc, const struct ip5328_data *ip)
{
	encode_sensors(ntc, ip, ip5328_int_level());
}

/* ============================================================
 *  GATT
 * ============================================================ */

static ssize_t read_gpio_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	const struct gpio_switch *gpio = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &gpio->value, sizeof(gpio->value));
}

static ssize_t write_gpio_switch(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct gpio_switch *gpio = attr->user_data;
	uint8_t value;
	int ret;

	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	value = *(const uint8_t *)buf;
	if (value > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	if (!device_is_ready(gpio->port)) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	ret = gpio_pin_configure(gpio->port, gpio->pin,
				 value != 0U ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
	if (ret) {
		LOG_WRN("%s configure failed: %d", gpio->name, ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	gpio->value = value;
	LOG_INF("%s=%u", gpio->name, gpio->value);
	return len;
}

static ssize_t read_ip5328_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, ip5328_report,
				 sizeof(ip5328_report));
}

static ssize_t read_ip5328_diag(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, ip5328_diag,
				 sizeof(ip5328_diag));
}

/*
 * 版本串在编译期就拼好，和 BTHome 广播里的 0xF2 对象同源（都来自 VERSION 文件），
 * 不用运行时初始化，也不会出现两处版本号不一致。
 */
#define APP_VER_STR_(x) #x
#define APP_VER_STR(x) APP_VER_STR_(x)
static const char app_version_str[] = APP_VER_STR(APP_VERSION_MAJOR) "."
				     APP_VER_STR(APP_VERSION_MINOR) "."
				     APP_VER_STR(APP_PATCHLEVEL);

static ssize_t read_app_version(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, app_version_str,
				 strlen(app_version_str));
}

/* 1 = 允许休眠，0 = 常醒测试模式。随时可写，测完切回休眠不用重新刷机。 */
static ssize_t read_sleep_enable(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	uint8_t value = sleep_enabled ? 1U : 0U;

	ARG_UNUSED(attr);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

static ssize_t write_sleep_enable(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t value;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	value = *(const uint8_t *)buf;
	if (value > 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	sleep_enabled = value != 0U;
	LOG_INF("sleep_enabled=%u", sleep_enabled);

	return len;
}

#define GPIO_SWITCH_GATT_ENTRY(index, port_node, pin_number, label) \
	BT_GATT_CHARACTERISTIC(&gpio_switch_uuid_##index.uuid, \
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, \
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, \
			       read_gpio_switch, write_gpio_switch, &gpio_switches[index]), \
	BT_GATT_CUD(label, BT_GATT_PERM_READ),

BT_GATT_SERVICE_DEFINE(gpio_switch_service,
	BT_GATT_PRIMARY_SERVICE(&gpio_switch_service_uuid),
	GPIO_SWITCH_LIST(GPIO_SWITCH_GATT_ENTRY)
);

/*
 * IP5328 全量数据（16 字节，小端）：
 *   [0]    1 = 有效，低 2 bit 是引脚组合（1 = 5脚SCL/6脚SDA，2 = 反过来）
 *   [1]    bit0~2 系统状态，bit4 充电中，bit6 已充满
 *   [2]    bit0~2 充电阶段（0 IDLE/1 涓流/2 恒流/3 恒压/4 停充检测/5 充满/6 超时）
 *   [3]    电量 %
 *   [4:6]  BATOCV  开路电压 mV
 *   [6:8]  BATVAD  端电压 mV
 *   [8:10] BATIAD  电池电流 mA（有符号，充正放负）
 *   [10:12] VSYS 电压 mV
 *   [12:14] VSYS 电流 mA（有符号）
 *   [14:16] 功率 mW
 *
 * 诊断特征（20 字节，见 ip5328_diag_run 里的布局说明）用来排查读不通的原因：
 * 上拉在不在、主板醒没醒、probe 有没有 ACK、地址对不对。
 */
BT_GATT_SERVICE_DEFINE(ip5328_service,
	BT_GATT_PRIMARY_SERVICE(&ip5328_service_uuid),
	BT_GATT_CHARACTERISTIC(&ip5328_report_uuid.uuid, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_ip5328_report, NULL, NULL),
	BT_GATT_CUD("IP5328 report", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(&ip5328_diag_uuid.uuid, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_ip5328_diag, NULL, NULL),
	BT_GATT_CUD("I2C diag", BT_GATT_PERM_READ),
);

BT_GATT_SERVICE_DEFINE(app_info_service,
	BT_GATT_PRIMARY_SERVICE(&app_info_service_uuid),
	BT_GATT_CHARACTERISTIC(&app_version_uuid.uuid, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_app_version, NULL, NULL),
	BT_GATT_CUD("Version", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(&app_sleep_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_sleep_enable, write_sleep_enable, NULL),
	BT_GATT_CUD("Sleep enable", BT_GATT_PERM_READ),
);

static void configure_gpio_switches(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(gpio_switches); i++) {
		struct gpio_switch *gpio = &gpio_switches[i];
		int ret;

		gpio->value = 0U;
		if (!device_is_ready(gpio->port)) {
			LOG_WRN("%s GPIO device is not ready", gpio->name);
			continue;
		}

		ret = gpio_pin_configure(gpio->port, gpio->pin, GPIO_DISCONNECTED);
		if (ret) {
			LOG_WRN("%s disconnect failed: %d", gpio->name, ret);
		}
	}
}

/* ============================================================
 *  广播与睡眠
 * ============================================================ */

/*
 * 广播始终保持可连接：OTA 客户端是"轮询扫描 + 撞上窗口就连"，
 * 所以任何一个广播窗口都必须允许连接，不能只在 OTA 窗口期才可连。
 */
static int start_advertising(void)
{
	int ret;

	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret == -EALREADY) {
		ret = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	}

	if (ret) {
		LOG_ERR("advertising start/update failed: %d", ret);
		return ret;
	}

	LOG_INF("advertising as %s", DEVICE_NAME);
	return 0;
}

/*
 * 广播 duration 之后停止。中间如果有人连上（OTA），就等它断开再停。
 * 这一段就是"轮询唤醒窗口"：设备只在窗口内可被扫到并连接。
 */
static void advertise_then_stop(k_timeout_t duration)
{
	int ret = start_advertising();
	uint32_t waited_s = 0U;

	if (ret) {
		return;
	}

	k_sleep(duration);

	/*
	 * 有人连着就等它断开再收摊，但最多再等 10 分钟 ——
	 * 万一 OTA 客户端异常没断开，不能把设备永远挂在这儿。
	 */
	while (connected && waited_s < 600U) {
		k_sleep(K_SECONDS(1));
		waited_s++;
	}

	if (connected) {
		LOG_WRN("client still connected, dropping the window");
	}

	(void)bt_le_adv_stop();
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("connection failed: 0x%02x", err);
		return;
	}

	connected = true;
	LOG_INF("connected");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	connected = false;
	LOG_INF("disconnected: 0x%02x", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

/*
 * 运行模型（双唤醒）：
 *   上电 → 采样 → 开 OTA 窗口 → 睡
 *   每 10 分钟定时醒 → 采样 → 开 OTA 窗口 → 睡
 *   第 8 脚 KEY 按下 → 立刻醒 → 采样 → 开 OTA 窗口 → 睡
 *
 * k_sem_take 带超时正好同时实现两种唤醒：超时到点自己醒，
 * 或者被 KEY 中断提前 give 掉。醒着的这段时间就是 OTA 窗口，
 * 电脑端一直轮询扫描，撞上窗口就能连上刷机。
 */
int main(void)
{
	struct ntc_capture capture;
	struct ip5328_data ip;
	bool sensor_ready = false;
	int ret;

	set_error_capture(&capture, -EAGAIN);
	ip = (struct ip5328_data){
		.bind = IP5328_BIND_UNKNOWN,
	};
	publish_sensors(&capture, &ip);

	configure_gpio_switches();

	ret = configure_ip5328_io();
	if (ret) {
		LOG_WRN("IP5328 IO init failed: %d", ret);
	}

	ret = configure_key_wakeup();
	if (ret) {
		LOG_ERR("KEY wakeup init failed: %d", ret);
	}

	ret = configure_io();
	if (ret) {
		LOG_WRN("NTC GPIO init deferred: %d", ret);
		set_error_capture(&capture, ret);
		publish_sensors(&capture, &ip);
	} else {
		ret = configure_adc();
		if (ret) {
			LOG_WRN("NTC ADC init deferred: %d", ret);
			set_error_capture(&capture, ret);
			publish_sensors(&capture, &ip);
		} else {
			sensor_ready = true;
		}
	}

	/*
	 * 初始化全都过了才确认镜像：万一新固件在初始化阶段就崩，
	 * MCUboot 下次启动会把它回滚掉，不会把设备卡死。
	 */
	ret = boot_write_img_confirmed();
	if (ret) {
		LOG_WRN("image confirm failed: %d", ret);
	}

	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("Bluetooth init failed: %d", ret);
		return ret;
	}

	while (true) {
		if (!sensor_ready) {
			ret = configure_io();
			if (!ret) {
				ret = configure_adc();
			}

			if (ret) {
				LOG_WRN("NTC init retry failed: %d", ret);
				set_error_capture(&capture, ret);
			} else {
				sensor_ready = true;
			}
		}

		if (sensor_ready) {
			ret = sample_ntc(&capture);
			if (ret) {
				LOG_WRN("NTC sample failed: %d", ret);
				set_error_capture(&capture, ret);
			}
		}

		/*
		 * 每轮都重跑一次诊断：按键唤醒也会走到这里，
		 * 所以按一下充电宝的键就能拿到一份最新的总线状态。
		 */
		ip5328_diag_run();

		ret = ip5328_sample(&ip);
		if (ret) {
			LOG_WRN("IP5328 sample failed: %d (bind=%u)", ret, ip_bind);
		}
		publish_sensors(&capture, &ip);

		if (sleep_enabled) {
			/* OTA 窗口：这段时间可连接广播，电脑端轮询到就能刷机 */
			advertise_then_stop(K_SECONDS(OTA_WINDOW_SECONDS));

			LOG_INF("idle, wait up to %d min or KEY (count=%u)", 10,
				key_wake_count);
			(void)k_sem_take(&wake_sem, SAMPLE_INTERVAL);
		} else {
			/*
			 * 测试模式：广播不收，一直保持可连接，数据每 5 秒刷一次，
			 * 按一下键也能立刻刷。想回正常休眠就往 "Sleep enable" 写 1。
			 */
			(void)start_advertising();
			(void)k_sem_take(&wake_sem, TEST_SAMPLE_INTERVAL);
		}

		/* 按键机械抖动，等电平稳定后再采样 */
		k_msleep(KEY_DEBOUNCE_MS);
	}

	return 0;
}
