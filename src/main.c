/*
 * nRF54L15 / E73-2G4M08S1F  BTHome 传感器固件（NTC + 电池 ADC + OTA）
 * ---------------------------------------------------------------------
 * 传感器（纯 ADC 路线，不用 I2C）
 *   1. 100k NTC 分压测温（模组 2 脚供电 / 模组 3 脚 = P1.11 AIN4 采样）
 *   2. 电池电压（模组 4 脚 = P1.12 AIN5，外部 1M+1M 分压）
 *   3. 充电器输入 VBUS（模组 6 脚 = P1.14 AIN7，外部 1M+100k 分压）
 *      —— 快充已砍掉，输入只有普通 5V。6 脚反算回输入端的电压
 *         > 3.6V 且 < 5V 才算插着充电器（没插 ≈3.59V，插着 ≈4.60V）
 *   4. 充电状态：VBUS 为主，电池电压斜率兜底
 *
 * 对外协议（两套共存，见 prj.conf）：
 *   1. BTHome v2 广播 —— 电量 / 温度 / 充电状态 / 电池电压 / 版本号，不用连
 *   2. 标准 GATT —— Battery Service 0x180F（电量 0x2A19），
 *      Device Information 0x180A（厂商 / 型号 / 固件版本）；
 *      不懂 BTHome 的通用客户端连上就能读
 *
 * IP5328 的 I2C 那套（模组 5/6/7 脚）代码还在，但默认关掉了 ——
 * 见 IP5328_I2C_ENABLE。关掉之后开机不用等 30 秒静默期、不跑总线诊断，
 * P1.13 / P1.04 都保持高阻（P1.14 现在给 VBUS 用）。
 *
 * 引脚（E73 模组脚 → nRF54L15）
 *   2  → P1.10   NTC 分压供电（只在采样时给电）
 *   3  → P1.11   电池/NTC ADC (AIN4)
 *   4  → P1.12   电池电压 ADC (AIN5)，外部 1M+1M 分压
 *   6  → P1.14   充电器输入 VBUS ADC (AIN7)，外部 1M+100k 分压
 *   8  → P1.02   IP5328 KEY 网络（NFC1，overlay 里已关 NFC）—— 按键唤醒
 *
 * 运行策略（双唤醒 + 轮询 OTA 窗口）
 *   唤醒源 1：定时，每 10 分钟一轮（原机制，保留）
 *   唤醒源 2：模组第 8 脚 KEY 网络下降沿，按下按键立刻醒一轮
 *   每轮流程：采样 → 可连接广播 120 秒（OTA 窗口）→ 停止广播 → 回去睡
 *   广播期间若被连上，一直等到断开才停止广播，所以 OTA 不会被睡眠打断
 *   ★ 两种"不休眠"：插着充电器（拔掉立刻回睡眠）、温度突变爬升
 *     （直到温度突变跌落才回睡眠）
 *   ★ 睡着期间每 2 秒查一次充电器、每 30 秒采一次温度，命中立刻醒
 *   ★ 充电中不休眠：插着充电器时不睡，一直可连接、数据每 5 秒刷一次
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
#include <zephyr/bluetooth/services/bas.h>
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
#define BTHOME_ID_CHARGING 0x16
#define BTHOME_ID_VOLTAGE 0x4A
#define BTHOME_ID_FIRMWARE_VERSION 0xF2

/*
 * ★ 这一段的 id 是从 bthome-ble 的 MEAS_TYPES 表逐个核对过的，别再凭印象写。
 *
 * 权威来源（改 id 之前先查这两个）：
 *   https://raw.githubusercontent.com/Bluetooth-Devices/bthome-ble/main/src/bthome_ble/const.py
 *   https://raw.githubusercontent.com/Bluetooth-Devices/bthome-ble/main/src/bthome_ble/parser.py
 *
 * 踩过的坑（0.31.0 及之前的错误认知）：
 *   - 以为 0x0E 是 generic voltage → **其实是 PM10（颗粒物浓度）**。
 *     塞了个 4928 进去，HA 就建了个"PM10 = 4928 μg/m³"的实体，莫名其妙。
 *   - 以为 0x15 是 charging     → **其实是 Battery（电池状态）**。
 *     HA 里显示成"电池"二进制传感器，语义也是错的。
 *   - 以为 BTHome 只有 0x0C 一个电压对象 → **其实有两个**：0x0C 和 0x4A。
 *     （查表时找到一个就收手了，这是偷懒。要把所有含 VOLTAGE 的条目都列出来。）
 *
 * 真相 —— **整个 BTHome 只有这两个电压对象**，都是 device_class = voltage：
 *   0x0C  voltage  factor 0.001V  ← 精度高（能到 1mV），量程 0~65.535V
 *   0x4A  voltage  factor 0.1V    ← 精度低（只到 100mV），量程 0~6553.5V
 *
 * ★ **0x0C 已经不用了** —— 充电输入电压不广播了，只留 0x4A 报电池电压。
 *   判"插没插充电器"看 0x16 charging 那一位就够，不必再建一个电压实体。
 *
 * ⚠️ 注意 id 顺序：0x16(22) < 0x4A(74)，
 *    所以**充电状态在电池电压前面**。见下面的下标对照表。
 *
 * charging 是 **0x16**（device_class = BATTERY_CHARGING）。
 *
 * BTHome 要求 object id 按数值从小到大排列，接收端碰到不认识的 id
 * 就直接停止解析后面的内容。所以顺序必须是 01 < 02 < 0C < 16 < 4A < F2。
 *
 * 电流对象（0x5D）已经摘掉 —— 这块板子拿去给别的设备供电，不需要采电流。
 *
 * 0x16 = charging（布尔）。值 1 = 充电中 / 0 = 没充电。下面用它表达
 * "插着充电器"：charge_state 是 CHARGING 或 FULL 就报 1，其余报 0。
 * 想要 IDLE / DISCHARGING / FULL 四态细分的话，这个标准对象做不到，
 * 得去读 GATT 报告特征值（6f6b0201-…）的 [10:12]。
 *
 * 0xF2 = firmware version（3 字节，小端 patch/minor/major）。
 * 它**不会建实体**，但解析器会拿它调 set_device_sw_version()，
 * 所以在 HA 的设备详情页"固件版本"那一栏能看到 —— 不是白放的。
 *
 * ★ BTHome 协议没有给实体指定名字的字段，名字是接收端按 device_class
 *   拼的，固件侧改不了。要改名字只能在 HA 里手动改。
 */
/*
 * 各字段的字节偏移。语义统一：**指向该字段的「值」的第一个字节**，
 * 不包括前面那个 object id 字节。
 *
 * 对照（下标从 0 开始）：
 *   0  1  2      : D2 FC 40          BTHome UUID + v2 标识
 *   3  4         : 01 BB             battery
 *   5  6  7      : 02 TT TT          temperature
 *   8  9         : 16 CC             charging（1 = 插着充电器 / 0 = 没插）
 *   10 11 12     : 4A BB BB          voltage（电池，0.1V）
 *   13 14 15 16  : F2 PP MM JJ       firmware version
 *
 * 所以 VERSION_OFFSET = 14（PP 的下标），数组总长 = 14 + 3 = 17。
 */
#define BTHOME_BATTERY_OFFSET 4U
#define BTHOME_TEMP_OFFSET 6U
#define BTHOME_CHARGING_OFFSET 9U
#define BTHOME_VOLTAGE_OFFSET 11U
#define BTHOME_VERSION_OFFSET 14U

/*
 * 0x4A 的 factor 是 0.1V，所以填进去的是**十分之一伏**的整数。
 * 例：4.1V → 填 41。**千万别拿它直接填 mV**，会显示成 410.0V，差 10 倍。
 */
#define BATTERY_VOLTAGE_DECIVOLTS_DIV 100U

/* ---------------- NTC ---------------- */

#define NTC_REF_OHMS 100000U
#define NTC_SAMPLE_COUNT 8U
#define NTC_SETTLE_TIME K_MSEC(50)
#define NTC_SAMPLE_INTERVAL K_MSEC(2)
/*
 * 模组供电（LDO 输出）的兜底值。实测这块板子是 3320mV（VDD 通道读出来的
 * 就是这个数，万用表也对得上），所以兜底改成 3320，比之前猜的 3300 准。
 *
 * ★ 这个值只在 VDD 通道读失败时才用得上（正常情况下一直读实测值）。
 *   供电接的是 3.3V LDO，满载也就掉几十 mV，所以 3320 是个安全估计。
 */
#define NTC_SUPPLY_FALLBACK_MV 3320U
#define NTC_ADC_FULL_SCALE_MV 3600U
#define NTC_ADC_MAX_RAW ((1U << 12) - 1U)

/*
 * 电池电压采样（模组第 4 脚 = P1.12 = AIN5）。
 *
 * ADC 用内部 0.9V 参考 + 1/4 增益 → 满量程 3.6V，超过 3.6V 就削顶。
 * 锂电池满电 4.2V，所以外面必须分压。默认按 1:1（R1=R2）算，
 * 4.2V → 2.1V，离 3.6V 满量程还有余量。
 *
 *   BAT ──[R1]──┬── 模组第 4 脚
 *               └──[R2]── GND
 *
 * 分压比 = (R1 + R2) / R2。R1=R2 时就是 2。
 * 换别的阻值只改这两个宏，不要改代码逻辑。
 */
#define BAT_ADC_DIVIDER_NUM 2U
#define BAT_ADC_DIVIDER_DEN 1U

/*
 * 只认落在这个区间的读数。分压没接时模组第 4 脚是悬空的，
 * ADC 会读到乱七八糟的值，光看"大于 0"会把悬空当成真电压上报。
 * 单节锂电正常范围 2.5~4.4V，放宽到 1.5~5.0V。
 */
#define BAT_ADC_MIN_VALID_MV 1500U
#define BAT_ADC_MAX_VALID_MV 5000U

/* ---------------- 充电状态（纯 ADC 推断） ---------------- */

/*
 * 没有 I2C 就读不到 IP5328 的充电标志，只能看电池电压往哪边走。
 *
 * 关键：不能拿两个单点相减。实测电池读数在 4124~4140 mV 之间跳，
 * 峰峰值 16mV（±8mV），这个噪声自己就能把 8mV 的阈值顶穿，
 * 于是"没在充电"也被判成 CHARGING。
 *
 * 所以改成【窗口平均值比窗口平均值】：窗口内每个采样点都累加，
 * 到点了取平均再相减。5 分钟按 5 秒一次能攒 60 个点，噪声从 ±6mV
 * 压到 ±0.8mV 左右，阈值就能降到 5mV 而不会被噪声误触发。
 *
 * 阈值为什么是 5mV：锂电池在 3.7~4.0V 那段曲线很平，恒流充电每分钟
 * 只涨 1~2mV，5 分钟真实涨 5~10mV。低于 5mV 分不出来，高了又抓不住。
 *
 * 第一个窗口只用来打基准（这时候状态还是 UNKNOWN），所以开机后要满
 * 两个窗口才出第一个判断：常醒测试模式约 10 分钟，正常周期则是两轮。
 */
#define BAT_TREND_WINDOW_MS (5U * 60U * 1000U)
#define BAT_TREND_MIN_MV 5

/*
 * ★★ 满电电压 —— 按**这块板子实际能充到多少**来设，别照电芯型号猜。
 *
 * ★ 板主 2026-09-27 实测确认：这块板子的充电器**最高只充到 4.2V**
 *   （把充电设定电阻改成 120k 也上不去 4.35V），所以它是一颗普通 4.2V 电芯。
 *
 * 之前按"高压电芯 4.35V"把阈值抬到 4300mV，那是错的：
 *   4.2V 电芯永远充不到 4300，charge_state 会**永远停在 CHARGING**，
 *   满电永远判不出来；soc_table[] 用 4.35V 的表也会让同一电压下的电量恒偏低
 *   （3880mV 在 4.35V 表里只给 53%，4.2V 电芯实际约 62%）。
 *
 * 4150mV = 4.2V 满充减去一点 ADC 余量，再配合下面的 !moving（电压不再涨）判满。
 *
 */
#define BAT_FULL_MV 4150U

/*
 * 拔掉充电器后等这么久，才把电池电压当成 OCV（开路电压）来用。
 * 刚拔线时电芯还挂着表面电荷，读数偏高几十 mV，等一会儿才准。
 */
#define BAT_OCV_SETTLE_MS (60 * 1000)

/*
 * 判满还要求"电压不再涨"（见 bat_update_charge_state 里的 !moving）。
 * 因为恒流充到 4.2V 时电压还在爬，只有进恒压阶段、电流掉下来，
 * 电压才会真正钉住不动 —— 那才是真满。
 */

#define BAT_STATE_UNKNOWN 0U     /* 还没攒够一个比较窗口 */
#define BAT_STATE_IDLE 1U        /* 待机 */
#define BAT_STATE_CHARGING 2U    /* 充电中 */
#define BAT_STATE_DISCHARGING 3U /* 放电中 */
#define BAT_STATE_FULL 4U        /* 已充满 */

/* ---------------- 电池检测（脱线 / 越界） ---------------- */

/*
 * 把"电池电压这一次读数为什么不可信"分类记下来，别都糊成一个 0。
 *
 * 以前只有一个 bat_mv：够不着区间就不写，留在 0，广播里电量跟着掉 0%，
 * 但看不出到底是没接电池、还是采样通道坏了。
 */
#define BAT_DETECT_UNKNOWN 0U      /* 一个样本都没采到：ADC 通道有问题 */
#define BAT_DETECT_OK 1U           /* 检测正常，bat_mv 有效 */
#define BAT_DETECT_DISCONNECTED 2U /* 分压脚没电平：电池没接 / 分压断了 */
#define BAT_DETECT_OVERRANGE 3U    /* 电压高得离谱：基准漂了 / 脚短路 */

/* ---------------- 充电器输入 VBUS（充电状态的可靠来源） ---------------- */

/*
 * 模组第 6 脚 = P1.14 = AIN7，外部 1M+100k 分压（约 11:1）。
 *
 *   充电输入 ──[1MΩ]──┬── 模组第 6 脚 (P1.14/AIN7)
 *                     └──[100kΩ]── GND      （再并一颗电容到 GND）
 *
 * 实测这一脚就两个状态：
 *
 *   插着充电器 ≈ 418mV
 *   没插       ≈ 327mV
 *
 * ★ 判定规则：**只看第 6 脚的原始 mV**，带 20mV 死区防抖：
 *
 *      ≥ 375mV → 插着充电器
 *      ≤ 355mV → 拔了
 *      中间    → 保持上一次的结论（噪声不翻状态）
 *
 *   不反算、不套分压比 —— 分压比写错了也不影响"插没插"这个判断。
 *
 * ⚠️ 以前是拿"反算回输入端的电压"卡 (3.6V, 5V) 开区间，而 327mV × 11 =
 *    3597mV 正好压在那条 3600mV 线上，**余量只剩 3mV**（不到 4 个 ADC 码）。
 *    ADC 抖一下、或者分压比写大一点，拔了充电器照样显示"充电中"，
 *    插拔反馈时好时坏、设备死活不进休眠，根子都在这儿。
 */
#define VBUS_ADC_ENABLE 1
#define VBUS_PIN_PRESENT_MV 375U /* 6 脚 ≥ 这个值 → 插着充电器 */
#define VBUS_PIN_ABSENT_MV 355U  /* 6 脚 ≤ 这个值 → 拔了；中间保持原状态 */

/*
 * 分压比，用来把 ADC 读到的分压值**反算回输入端真实电压**，
 * 好让广播里能显示"充电输入 5.13V"这种可读的东西。
 *
 *   VIN = 分压读数 × (1M + 100k) / 100k = 读数 × 11
 *
 * 实测：插着充电器读 418mV → 418 × 11 = 4598mV。
 *
 * 注意：这个反算值精度有限 —— 1M 和 100k 本身有 1% 误差，合起来约 2%，
 * 在 5V 上有 ±100mV 的不确定度。所以它适合"看个大概"，不能当万用表。
 * 只有判为"插着充电器"时才反算。
 */
#define VBUS_DIVIDER_NUM 11U
#define VBUS_DIVIDER_DEN 1U

/* ---------------- 电池电流：已取消 ---------------- */

/*
 * 这块板子拿去给别的设备供电，不需要采电流，所以整条差分电流通道删掉了。
 * 模组第 6 脚（P1.14 / AIN7）现在归 VBUS 用，也没有第二只脚能凑成差分对。
 * 原来那套接线（采样电阻用 IP5328 的 24 脚 VSP / 25 脚 VSN，两端各
 * 1M+1M 分压到 AIN6/AIN7）记在 README 里，真要恢复照着接就行。
 */

/* 电池电压 → 电量百分比走的是下面 soc_table[] 那条放电曲线，不再用两点直线 */

/* ---------------- 运行周期 ---------------- */

/*
 * 双唤醒：
 *   1. 定时唤醒 —— 每 SAMPLE_INTERVAL 采样一次（原来的机制，保留）
 *   2. 按键唤醒 —— 模组第 8 脚（IP5328 的 KEY 网络）下降沿，立刻醒一次
 * 两者谁先来就用谁，醒来后都是同一套流程：采样 → 开 OTA 窗口 → 回去睡。
 */
#define SAMPLE_INTERVAL K_MINUTES(10)
#define SAMPLE_INTERVAL_MS (10 * 60 * 1000)
#define OTA_WINDOW_SECONDS 120U
#define KEY_DEBOUNCE_MS 30

/*
 * 休眠期间的轮询（拔掉充电器后才用得上）：
 *   每 SLEEP_POLL_MS（1 秒）只读一次第 6 脚 —— 插拔立刻发现，不等 10 分钟。
 *   每 SLEEP_TEMP_TICKS 次轮询（1s × 15 = 15s）采一次完整数据判温度突变。
 * 单通道 ADC 读一次不到 1ms，平均功耗远低于常醒广播，可以放心轮。
 */
#define SLEEP_POLL_MS K_SECONDS(1)
#define SLEEP_TEMP_TICKS 15U

/*
 * 温度突变阈值（0.01°C）：一次采样比上次涨 2.00°C 以上 → 立刻转常醒；
 * 一次采样比上次跌 2.00°C 以上 → 立刻回去睡。
 */
#define TEMP_RISE_WAKE_CENTI 200
#define TEMP_FALL_SLEEP_CENTI 200
/*
 * 爬升之后"常醒"保持多久（每次新的爬升续期）。
 *
 * ★ 以前这里是个**永久锁存**：爬升之后必须来一次单次 -2°C 的跳变才解锁，
 *   而环境温度只会慢慢降（每次采样差 0.1°C 量级），根本凑不出来 ——
 *   设备被一次采样噪声顶上去就再也不会休眠了。改成限时保持就没事了。
 */
#define TEMP_RISE_HOLD_MS (10U * 60U * 1000U)

/*
 * 功能测试模式：不休眠，一直保持可连接广播，数据每 5 秒刷一次，
 * 这样随时都能连上去看数据，不用等 10 分钟窗口。
 *
 * 运行期可以改：手机端把 "Sleep enable" 特征写成 1 就切回正常的
 * 10 分钟周期，写 0 又回到常醒。量产前把 STAY_AWAKE_DEFAULT 改成 0。
 */
#define STAY_AWAKE_DEFAULT 1
#define TEST_SAMPLE_INTERVAL K_SECONDS(5)

/*
 * ================== IP5328 I2C：默认关闭 ==================
 *
 * 板子上那三根 I2C 线（模组 5/6/7 脚）已经拆掉，改走纯 ADC 了。
 * 置 0 就完全不碰 I2C：
 *   - 不探测、不跑总线诊断、不按键
 *   - 开机不用再等 30 秒静默期
 *   - P1.13 / P1.04 保持高阻（P1.14 现在归 VBUS 用）
 *
 * 代码全都留着，哪天把线接回去（并且确认上拉挂在 IP5328 第 27 脚 VREG 上），
 * 把这里改成 1 就恢复。
 */
#define IP5328_I2C_ENABLE 0

/*
 * 开机后先安静这么久，一个字节都不碰 I2C 两脚。
 * IP5328 是在【上电那一刻】检测这两脚为高才进 I2C 模式的，
 * 我们一上电就探测、拉低、扫描，会把它的检测过程搅掉 ——
 * 它一旦没进模式，之后怎么读都不会应答。
 * I2C 关掉时不需要这个静默期。
 */
#define IP5328_QUIET_BOOT_MS (IP5328_I2C_ENABLE ? 30000 : 0)

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
#define IP5328_BIND_A 0U /* 模组 5 脚 = SCL，6 脚 = SDA（接线已定死） */

/* 软件 I2C 半周期，约 100kHz 上下，够用且对中断抖动不敏感 */
#define IP5328_BIT_DELAY_US 2U

#define IP5328_REPORT_LEN 16U

/*
 * IP5328 报告特征第 0 字节的含义：
 *   0x00 = I2C 没通，而且模组第 4 脚的分压也没接（整包全 0）
 *   0x01 = I2C 通了，绑定组合 A
 *   0xFE = I2C 没通，但 [4..5] 里是模组 ADC 实测的电池电压
 */
#define IP5328_REPORT_OK_BASE 0x01U
#define IP5328_REPORT_ADC_FALLBACK 0xFEU

/*
 * 总线诊断结果，只读。IP5328 读不通时靠它远程判断卡在哪一步：
 * 是线没上拉、主板没醒、还是从机根本不应答。
 */
#define IP5328_DIAG_LEN 34U

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

/*
 * 电压校准特征（uint16 小端，单位"千分比"，可读可写）：
 *   6F6B0303 = 电池电压分压比 ×1000（默认 2000 = ×2.000）
 *   6F6B0304 = 充电输入分压比 ×1000（默认 11000 = ×11.000）
 *
 * 目的：分压电阻实际值、ADC 绝对标定、走线压降都会让"按理论比值算"的
 * 电压偏几个百分点。拿万用表量真实电压，把比值写对就行，不用为了调一个
 * 数重新编译 + OTA 一遍。掉电不保存，调准了再回来改成默认值。
 */
#define APP_BAT_DIV_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0303, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
#define APP_VBUS_DIV_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0304, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)
/*
 * 6F6B0305 = 第 6 脚原始 mV（只读，uint16 小端）
 * 就是把"判插没插"用的那个数直接摊开看：插着 ≈418mV / 没插 ≈327mV。
 * 阈值要不要动（VBUS_PIN_PRESENT_MV / VBUS_PIN_ABSENT_MV），看它就够了。
 */
#define APP_VBUS_PIN_UUID_VAL \
	BT_UUID_128_ENCODE(0x6F6B0305, 0x8C9A, 0x4CC4, 0xA848, 0x16B7E44F5415)

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
	/* 模组第 4 脚 (P1.12/AIN5) 分压后测到的电池电压，已换算回 BAT 端 */
	uint16_t bat_mv;
	uint16_t bat_raw_mv;
	/*
	 * 拿来算电量（SOC）的电压。
	 * 没插充电器时 = bat_mv；充电中 = 上次拔线后的读数（OCV）。
	 * 原因：充电时电池分压点被顶到充电电压（CV 阶段的 4.2V），
	 * 那是充电器给的，不是电芯自己的电压，拿它算电量会虚高。
	 */
	uint16_t bat_soc_mv;
	/* 模组第 6 脚 (P1.14/AIN7) 的原始 mV；没插充电器时为 0（插没插就看它） */
	uint16_t vbus_mv;
	/* 上一个值反算回输入端的真实电压（mV）。没插充电器时为 0 */
	uint16_t vbus_in_mv;
	/* 充电状态，见 BAT_STATE_* */
	uint8_t charge_state;
	/*
	 * 电池检测结果的分类，见 BAT_DETECT_*。
	 *
	 * 只靠 bat_mv 是不是 0 分不清"真没接"和"读取全失败"，
	 * 所以把原因记下来：脱线/短路/电压越界都各是各的。
	 */
	uint8_t bat_detect;
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
 * 单节普通锂电（4.2V 满充）开路电压 → 电量。
 * IP5328 没有直接给 SOC 百分比，只有 BATOCV，所以自己查表。
 */
static const struct soc_point soc_table[] = {
	{ 4200U, 100U }, { 4150U, 95U }, { 4100U, 90U }, { 4050U, 84U },
	{ 4000U, 77U },  { 3950U, 70U }, { 3900U, 62U }, { 3850U, 54U },
	{ 3800U, 45U },  { 3750U, 37U }, { 3700U, 28U }, { 3650U, 21U },
	{ 3600U, 14U },  { 3550U, 9U },  { 3500U, 5U },  { 3400U, 2U },
	{ 3300U, 0U },
};

static const struct adc_dt_spec ntc_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec vdd_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec bat_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
static const struct adc_dt_spec vbus_adc = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);
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
static const struct bt_uuid_128 app_bat_div_uuid = BT_UUID_INIT_128(APP_BAT_DIV_UUID_VAL);
static const struct bt_uuid_128 app_vbus_div_uuid = BT_UUID_INIT_128(APP_VBUS_DIV_UUID_VAL);
static const struct bt_uuid_128 app_vbus_pin_uuid = BT_UUID_INIT_128(APP_VBUS_PIN_UUID_VAL);

BUILD_ASSERT(ARRAY_SIZE(gpio_switches) == GPIO_SWITCH_COUNT);

static bool connected;
static bool vdd_adc_ready;
static bool bat_adc_ready;
static bool vbus_adc_ready;
/* 第 6 脚判出来的当前结论（滞回状态）：true = 插着充电器 */
static bool vbus_pin_present;
/* 第 6 脚最近一次读到的原始 mV，只读特征 0305 给手机端看，也方便对着实测调阈值 */
static uint16_t vbus_pin_mv_now;
static int16_t adc_sample_buffer[2];

/* 电池电压(mV) → 电量百分比，查 soc_table[] 放电曲线。实现在下面 */
static uint8_t soc_from_mv(uint16_t mv);

/*
 * 充电状态推断：一个窗口内把每个采样点都累加，到点取平均再和上一个
 * 窗口的平均值比。单点比单点会被 ±8mV 的噪声顶穿，见上面的注释。
 */
static uint32_t bat_trend_sum;
static uint16_t bat_trend_count;
static uint16_t bat_trend_avg;   /* 上一个窗口的平均值 */
static int64_t bat_trend_ms;     /* 上一个窗口结束的时刻 */
static bool bat_trend_valid;     /* 是否已经打过基准 */
static uint8_t bat_charge_state = BAT_STATE_UNKNOWN;
/* 上一次看到的 VBUS 状态，-1 = 还没看过。用来在插拔瞬间立刻出结论 */
static int8_t bat_last_vbus = -1;

/*
 * 电芯开路电压（OCV）：只在**没插充电器**时更新，用来算电量。
 * 拔线后表面电荷要几秒到几分钟才散掉，所以拔线瞬间起先等
 * BAT_OCV_SETTLE_MS 再开始采信读数。
 */
static uint16_t bat_ocv_mv;
static bool bat_ocv_valid;
static int64_t bat_ocv_not_before;

/*
 * 温度突变爬升 → 常醒一段时间（TEMP_RISE_HOLD_MS，每次新爬升续期），
 * 出现一次突变跌落就立刻清掉。睡眠期间每 30 秒采一次温度来喂它。
 */
static int64_t temp_rise_until;
static bool temp_prev_valid;
static int16_t temp_prev_centi;

/* 1 = 允许休眠（正常 10 分钟周期），0 = 测试模式常醒 */
static bool sleep_enabled = !STAY_AWAKE_DEFAULT;

/*
 * 运行期电压校准：分压比的千分比形式。
 *   2000  = ×2.000（电池 1M+1M）
 *   11000 = ×11.000（充电输入 1M+100k）
 * 通过下面的 0303 / 0304 特征改写，掉电重置。
 */
static uint16_t bat_div_permille = (BAT_ADC_DIVIDER_NUM * 1000U) / BAT_ADC_DIVIDER_DEN;
static uint16_t vbus_div_permille = (VBUS_DIVIDER_NUM * 1000U) / VBUS_DIVIDER_DEN;

static uint32_t ip_scl_pin;
static uint32_t ip_sda_pin;
static uint8_t ip_bind = IP5328_BIND_UNKNOWN;

static int ip5328_key_press(uint32_t ms);

/* 第 8 脚（KEY 网络）按键唤醒 */
K_SEM_DEFINE(wake_sem, 0, 1);
static struct gpio_callback key_cb;
static volatile uint32_t key_wake_count;

/*
 * 诊断只在开机后第一次、以及按键唤醒时跑。
 * 平时（定时/5 秒循环）不碰总线 —— 频繁探测会把 IP5328 搅得进不了 I2C 模式。
 */
static bool diag_ran;
static uint32_t diag_key_count;
/* 本次诊断是不是被第 8 脚按键唤醒触发的（上电那次不算） */
static bool diag_from_key;

/*
 * BTHome service data，17 字节：
 *   D2 FC         BTHome UUID，小端
 *   40            BTHome v2，未加密
 *   01 BB         battery，uint8，%
 *   02 TT TT      temperature，sint16，0.01 °C
 *   16 CC         charging，uint8，1 = 插着充电器 / 0 = 没插
 *   4A BB BB      voltage，uint16，0.1 V（电池电压）
 *   F2 PP MM JJ   firmware version，patch/minor/major
 *
 * object id 必须升序（01 < 02 < 16 < 4A < F2），充电状态夹在温度和
 * 电池电压中间。
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
	BTHOME_ID_CHARGING,
	0x00,
	BTHOME_ID_VOLTAGE,
	0x00,
	0x00,
	BTHOME_ID_FIRMWARE_VERSION,
	APP_PATCHLEVEL,
	APP_VERSION_MINOR,
	APP_VERSION_MAJOR,
};

/*
 * 数组总长 = 版本号第一个值字节的下标 + 3（PP / MM / JJ 三个字节）。
 * 这个 assert 是防呆的：加字段时漏一个占位就会在这里编译失败，
 * 而不是等到设备上广播解析不出来才发现。
 */
BUILD_ASSERT(sizeof(bthome_service_data) == BTHOME_VERSION_OFFSET + 3U);

/*
 * 广播包不能超过 31 字节，算一遍留个底：
 *   ad = flags(3) + service data(2+17) + uuid16(2+2) = 26 字节  ✓
 * 以后加字段时注意别把这行撑爆（超了 bt_le_adv_start 会返回 -EINVAL）。
 */
BUILD_ASSERT(sizeof(bthome_service_data) + 9U <= 31U);

/* IP5328 全量数据，给 GATT 只读特征值用 */
static uint8_t ip5328_report[IP5328_REPORT_LEN];

/*
 * 广播包只有 31 字节，塞不下完整设备名，所以设备名挪到 scan response 里：
 *   ad = flags(3) + service data(2+17) = 22 字节  (上限 31)
 *   sd = name(9) + 128bit UUID(18)     = 27 字节  (上限 31)
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_SVC_DATA16, bthome_service_data, sizeof(bthome_service_data)),
	/*
	 * 标准 Battery Service 的 16bit UUID（0x180F 小端）——
	 * 广播里同时给出 BTHome 和标准电量服务，两套协议共存：
	 * 懂 BTHome 的直接读广播，不懂的把它当普通电量设备连上读 0x2A19。
	 */
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0F, 0x18),
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
 * 接线顺序已定死，不再自动判定：
 *   模组第 5 脚 = P1.13 = SCL
 *   模组第 6 脚 = P1.14 = SDA
 * 所以这里只 bind 这一个组合，不做"两个组合轮着试"。
 */
static int ip5328_ensure_bind(void)
{
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);

	if (ip_bind != IP5328_BIND_UNKNOWN) {
		return 0;
	}

	k_msleep(2);
	if (ip_i2c_probe(IP5328_ADDR7) == 0) {
		ip_bind = IP5328_BIND_A;
		return 0;
	}

	/*
	 * 不应答。IP5328 待机时整颗都可能停了 —— 手册第 20 页写着
	 * "I2C 模式下 IP5328P 关机时 RSET 为低电平，开机时为高电平"，
	 * 而诊断读到的 RSET 正好是低。所以先替用户按一下键把升压叫起来。
	 */
	ip5328_key_press(150);
	ip_i2c_bus_recover();
	if (ip_i2c_probe(IP5328_ADDR7) == 0) {
		ip_bind = IP5328_BIND_A;
		return 0;
	}

	return -ENODEV;
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
 * 【必须开内部上拉】手册从头到尾没说 KEY 有内部上拉，图 3 的接法是
 *   KEY ──[R]──[按键]── GND
 *   KEY ── WLED(照明 LED，阳极在 KEY、阴极在 GND)
 * 那个 LED 会把 KEY 钳在 ~1.8V，低于 nRF54L15 在 3.1V 供电下的判决门限
 * （约 2.17V）—— 结果是不按键和按下都读成低，边沿永远不来，
 * 第 8 脚唤醒等于没接。开内部上拉把静止电平抬到门限以上才行。
 * （ESP32-C3 那边同样是必须开内部上拉才能唤醒，现象一致。）
 *
 * 代价：nRF 内部上拉约 13k，静止时白耗 ~240µA。量产如果在意这个数，
 * 可以在 KEY 网络对 3.1V 挂一颗 100k 外部上拉，然后把这里改回纯高阻。
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

	ret = gpio_pin_configure(ip_port, IP5328_PIN_M8, GPIO_INPUT | GPIO_PULL_UP);
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

/*
 * 替用户按一下充电宝的按键。
 *
 * IP5328P 平时在待机态，手册里写得很明白：「在 I2C 模式下，IP5328P 关机时
 * RSET 为低电平，开机时 RSET 为高电平」。待机时它整颗都可能不响应 I2C，
 * 得先短按 KEY（>60ms）把升压输出叫起来。
 *
 * 模组第 8 脚就挂在 KEY 网络上，直接驱动到低就等于按下按键。
 * 拉低期间先关掉 KEY 中断，免得自己触发一次"按键唤醒"。
 *
 * 返回放开 50ms 后、并且把内部上拉打开之后，第 8 脚的电平：
 *   读到高 → 内部上拉能压过 KEY 网络上的负载，按键按下一定能产生下降沿，
 *            第 8 脚唤醒可用；
 *   仍是低 → KEY 网络被 WLED 照明 LED（或别的下拉）钳得太死，
 *            内部上拉抬不起来，按键唤醒就废了 —— 这时候得在 KEY 网络
 *            对 3.1V 挂一颗 100k 外部上拉，或者干脆换唤醒源。
 */
static int ip5328_key_press(uint32_t ms)
{
	int level;

	if (!device_is_ready(ip_port)) {
		return -1;
	}

	(void)gpio_pin_interrupt_configure(ip_port, IP5328_PIN_M8, GPIO_INT_DISABLE);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M8, GPIO_OUTPUT_LOW);
	k_msleep(ms);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M8, GPIO_INPUT | GPIO_PULL_UP);
	k_msleep(50);

	level = gpio_pin_get(ip_port, IP5328_PIN_M8);

	(void)gpio_pin_interrupt_configure(ip_port, IP5328_PIN_M8,
					   GPIO_INT_EDGE_TO_INACTIVE);

	return level;
}

/*
 * 把两根线当纯输入挂一会儿，数它们自己跳变了几次。
 *
 * 这一格是用来区分两种"完全不应答"的：
 *   有跳变 → 这两个脚上另有主机在跑 I2C（板上有别的 MCU 在轮询 IP5328，
 *            或者接错到了 IP5328 的 I2C1 主机口 L1/L2），
 *            模组插进去只是第二个主机，谁都不理谁；
 *   没跳变 → 线上真的什么都没发生，是 IP5328 没进从机模式。
 *
 * 挂满约 1 秒：板上的轮询周期可能是几百毫秒，只听 20ms 很容易漏掉。
 */
static void ip_listen(uint8_t *scl_edges, uint8_t *sda_edges)
{
	uint32_t n_scl = 0U;
	uint32_t n_sda = 0U;
	int last_scl;
	int last_sda;

	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M5, GPIO_INPUT);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M6, GPIO_INPUT);

	last_scl = gpio_pin_get(ip_port, IP5328_PIN_M5);
	last_sda = gpio_pin_get(ip_port, IP5328_PIN_M6);

	for (uint32_t i = 0U; i < 500000U; i++) {
		int scl = gpio_pin_get(ip_port, IP5328_PIN_M5);
		int sda = gpio_pin_get(ip_port, IP5328_PIN_M6);

		if (scl != last_scl) {
			n_scl++;
			last_scl = scl;
		}
		if (sda != last_sda) {
			n_sda++;
			last_sda = sda;
		}
		k_busy_wait(1);
	}

	*scl_edges = (uint8_t)MIN(n_scl, 255U);
	*sda_edges = (uint8_t)MIN(n_sda, 255U);
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
 * 给一根线做两次测试，用来判断它到底有没有接在东西上。
 *
 *   驱动到低再放开        —— 线上有外部上拉就会马上回高
 *   驱动到高再放开，等 1ms —— 悬空的脚靠引脚电容把电平撑住，还是高；
 *                            接在对地阻抗上的脚会在这段时间内泄干净，掉到低
 *
 * 两个结果合起来：
 *   低 / 高 → 悬空，这根线什么都没接
 *   高 / 高 → 接在一个有上拉的网络上
 *   低 / 低 → 接在对地阻抗上（一颗电阻，或者芯片在推低）
 */
static void ip_line_probe(uint32_t pin, uint8_t *low_release, uint8_t *high_release)
{
	(void)gpio_pin_configure(ip_port, pin, GPIO_OUTPUT_LOW);
	k_busy_wait(50);
	(void)gpio_pin_configure(ip_port, pin, GPIO_INPUT);
	k_busy_wait(50);
	*low_release = (uint8_t)(gpio_pin_get(ip_port, pin) ? 1U : 0U);

	(void)gpio_pin_configure(ip_port, pin, GPIO_OUTPUT_HIGH);
	k_busy_wait(50);
	(void)gpio_pin_configure(ip_port, pin, GPIO_INPUT);
	k_msleep(1);
	*high_release = (uint8_t)(gpio_pin_get(ip_port, pin) ? 1U : 0U);
}

/*
 * 布局（34 字节）。
 * 接线顺序已定死：模组 5 脚(P1.13) = SCL，6 脚(P1.14) = SDA，不再试别的组合。
 *
 *   [0]     标志：bit0 已跑过，bit1 2µs ACK，bit2 20µs ACK，bit3 100µs ACK，
 *                bit4 总线恢复后 SDA 仍被拉低（从机卡住总线），
 *                bit5 本次诊断是【第 8 脚按键唤醒】触发的（上电那次是 0）
 *                —— 按一下实体键再读，bit5 变 1 就证明按键唤醒真的通了
 *   [1]     INT/RSET(P1.04) 电平，1 = 主板醒着
 *   [2..3]  SCL/SDA 纯高阻电平
 *   [4..5]  SCL/SDA 内部上拉电平
 *   [6..7]  SCL/SDA 内部下拉电平
 *   [8..9]  SCL/SDA 释放后恢复电平，1 = 有外部上拉
 *   [10]    2µs   probe 0x75，0 = 收到 ACK
 *   [11]    20µs  probe 0x75，0 = 收到 ACK
 *   [12]    100µs probe 0x75，0 = 收到 ACK
 *   [13]    快速全地址扫描命中数
 *   [14]    快速首个命中地址（7bit）
 *   [15]    慢速全地址扫描命中数
 *   [16]    慢速首个命中地址（7bit）
 *   [17]    INT(P1.04) 纯高阻时的电平
 *   [18]    INT(P1.04) 加内部上拉时的电平
 *   [19]    按 KEY 之前，约 1 秒内 SCL 自己跳变了几次（饱和 255）
 *   [20]    按 KEY 之前，约 1 秒内 SDA 自己跳变了几次（饱和 255）
 *   [21]    按 KEY 之后 probe 0x75 的结果，0 = 收到 ACK，0xFF = 没测
 *   [22]    按 KEY 之后 INT(P1.04) 加内部上拉时的电平
 *   [23]    按 KEY 之后快速全地址扫描命中数
 *   [24]    按 KEY 放开 50ms 后第 8 脚的电平（此时内部上拉已打开），
 *           1 = 上拉能压过 KEY 网络的负载，按键唤醒可用；0 = 被钳死，唤醒废
 *   [25]    [26] 第 8 脚：驱动低放开 / 驱动高放开 1ms 后的电平
 *   [27]    [28] 第 7 脚（INT）：同上
 *   [29]    第 13 脚（NFC2，本该悬空）：驱动低放开后的电平
 *   [30]    反向组合（6 脚 = SCL，5 脚 = SDA）2µs probe 0x75，0 = 收到 ACK
 *   [31]    反向组合全地址扫描命中数
 *   [32]    长按 KEY 10 秒复位 IP5328 之后，probe 0x75 的结果，0 = 收到 ACK
 *   [33]    复位之后快速全地址扫描命中数
 *
 *   [32]/[33] 是最后手段。手册第 18 页：「超长按 10s 可复位整个系统」——
 *   这是唯一不用拔电池就能让 IP5328 重跑一遍"上电检测 DMB/DPB 电平"的软件办法。
 *   如果它当初就是因为上电那一刻电平不对才没进 I2C 模式，这一下能救回来。
 *
 *   [30]/[31] 只报结果，不参与主流程 —— 主流程一律按 5=SCL / 6=SDA 走。
 *   加它是因为：用户是从 USB 口背面的丝印接的线，而 USB 的 DM/DP 和
 *   IP5328 的 SCL/SDA 是交叉的（手册第 3 页：DMB = 快充识别 DM 兼 I2C2
 *   的 SCK，DPB = 快充识别 DP 兼 SDA），照着 USB 丝印接很容易反过来。
 *
 *   [25]~[29] 用来分开"没上拉"和"悬空"：
 *   低/高 = 悬空，什么都没接；高/高 = 接在有上拉的网络上；低/低 = 接在对地阻抗上。
 *
 *   [18] 判断"芯片进没进 I2C 模式"：
 *   没进模式时 RSET(21) 只是对地的内阻设定电阻，内部上拉压不过 → 低；
 *   进了模式后它是芯片的 INT 输出（待机高阻 / 工作高电平）→ 高。
 *   手册第 20 页：I2C 模式下 IP5328P 关机时 RSET 为低、开机时为高。
 *
 *   [19]/[20] 判断"线上有没有别的主机在跑"：
 *   有跳变说明接错到了 IP5328 的 I2C1 主机口（L1/L2），不是从机口 DMB/DPB。
 *
 *   [21]~[23] 判断"是不是芯片在待机、按一下键就能救回来"。
 */
static void ip5328_diag_run(void)
{
	uint8_t first_fast = 0U;
	uint8_t first_slow = 0U;
	uint8_t n_fast = 0U;
	uint8_t n_slow = 0U;
	uint8_t unused_high = 0U;

	/*
	 * 纯 ADC 模式：I2C 线拆了，诊断没意义。
	 * 整包填 0xFF 当"未测"标记，免得读的人把全 0 误当成"测过但什么也没发生"。
	 */
	if (!IP5328_I2C_ENABLE) {
		memset(ip5328_diag, 0xFF, sizeof(ip5328_diag));
		return;
	}

	/*
	 * 第一件事：别碰总线，先听。
	 * 后面所有探测都会往线上打时钟，只有现在能听到"线上本来在发生什么"。
	 */
	ip_listen(&ip5328_diag[19], &ip5328_diag[20]);

	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip5328_diag[1] = (uint8_t)(ip5328_int_level() > 0 ? 1U : 0U);

	/* 先量静态电平，探测本身会扰动总线 */
	ip5328_diag[2] = (uint8_t)ip_pin_level(IP5328_PIN_M5, 0);
	ip5328_diag[3] = (uint8_t)ip_pin_level(IP5328_PIN_M6, 0);
	ip5328_diag[4] = (uint8_t)ip_pin_level(IP5328_PIN_M5, GPIO_PULL_UP);
	ip5328_diag[5] = (uint8_t)ip_pin_level(IP5328_PIN_M6, GPIO_PULL_UP);
	ip5328_diag[6] = (uint8_t)ip_pin_level(IP5328_PIN_M5, GPIO_PULL_DOWN);
	ip5328_diag[7] = (uint8_t)ip_pin_level(IP5328_PIN_M6, GPIO_PULL_DOWN);
	ip5328_diag[8] = (uint8_t)ip_line_has_pullup(IP5328_PIN_M5);
	ip5328_diag[9] = (uint8_t)ip_line_has_pullup(IP5328_PIN_M6);

	/* 三级速率各试一次，每次先做一次总线恢复 */
	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[10] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	ip_i2c_set_delay(IP5328_DELAY_MID);
	ip_i2c_bus_recover();
	ip5328_diag[11] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	ip_i2c_set_delay(IP5328_DELAY_SLOW);
	ip_i2c_bus_recover();
	ip5328_diag[12] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	/* 全地址扫一遍：万一从机地址不是 0x75，也能发现 */
	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bus_recover();
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			if (n_fast == 0U) {
				first_fast = a;
			}
			n_fast++;
		}
	}

	/* 慢速再扫一遍，万一是从机嫌快 */
	ip_i2c_set_delay(IP5328_DELAY_SLOW);
	ip_i2c_bus_recover();
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			if (n_slow == 0U) {
				first_slow = a;
			}
			n_slow++;
		}
	}

	/* 哪个速率能通就固定用哪个，免得真正读数据时又失败 */
	if (ip5328_diag[10] != 0 && ip5328_diag[11] == 0) {
		ip_i2c_set_delay(IP5328_DELAY_MID);
	} else if (ip5328_diag[10] != 0 && ip5328_diag[11] != 0 &&
		   ip5328_diag[12] == 0) {
		ip_i2c_set_delay(IP5328_DELAY_SLOW);
	} else {
		ip_i2c_set_delay(IP5328_DELAY_FAST);
	}

	/* 收摊前看看总线有没有被从机拉死 */
	ip_i2c_bus_recover();
	ip_sda_rel();
	k_busy_wait(50);

	ip5328_diag[0] = (uint8_t)(1U | (ip5328_diag[10] == 0 ? 0x02U : 0U) |
				   (ip5328_diag[11] == 0 ? 0x04U : 0U) |
				   (ip5328_diag[12] == 0 ? 0x08U : 0U) |
				   (ip_sda_get() ? 0U : 0x10U) |
				   (diag_from_key ? 0x20U : 0U));
	ip5328_diag[13] = n_fast;
	ip5328_diag[14] = first_fast;
	ip5328_diag[15] = n_slow;
	ip5328_diag[16] = first_slow;

	/*
	 * RSET/INT 的两种读法：
	 *   纯高阻 —— 芯片在推高就高，是对地电阻就低
	 *   加内部上拉 —— 能压过对地电阻就读高，压不过就读低
	 * 两者合起来能判断"芯片到底进没进 I2C 模式"。
	 */
	ip5328_diag[17] = (uint8_t)ip_pin_level(IP5328_PIN_M7, 0);
	ip5328_diag[18] = (uint8_t)ip_pin_level(IP5328_PIN_M7, GPIO_PULL_UP);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);

	/*
	 * 以上都是"静默状态下"的结果。现在替用户按一下键，把 IP5328 从
	 * 待机叫起来，再测一遍同样的东西 —— 如果待机就是原因，这一遍会通。
	 */
	ip5328_diag[24] = (uint8_t)(ip5328_key_press(150) > 0 ? 1U : 0U);
	k_msleep(300);

	ip5328_diag[22] = (uint8_t)ip_pin_level(IP5328_PIN_M7, GPIO_PULL_UP);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);

	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[21] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	n_fast = 0U;
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			n_fast++;
		}
	}
	ip5328_diag[23] = n_fast;

	/*
	 * 交叉验证一下 SCL/SDA 有没有接反。
	 * 只探一次、只报结果，主流程用的顺序不变（还是 5=SCL / 6=SDA）。
	 */
	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M6, IP5328_PIN_M5);
	ip_i2c_bus_recover();
	ip5328_diag[30] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	n_fast = 0U;
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			n_fast++;
		}
	}
	ip5328_diag[31] = n_fast;

	/* 回到用户给的顺序，下面还原引脚也按这个来 */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);

	/*
	 * 最后一组：挨个确认这几根线到底有没有接在东西上。
	 * [24] 只说"8 脚放开后没上拉"，但"没上拉"和"悬空"是两回事，
	 * 再驱动到高放开一次就能分开。
	 */
	ip_line_probe(IP5328_PIN_M8, &ip5328_diag[25], &ip5328_diag[26]);
	ip_line_probe(IP5328_PIN_M7, &ip5328_diag[27], &ip5328_diag[28]);
	ip_line_probe(IP5328_PIN_NFC2, &ip5328_diag[29], &unused_high);

	/* 三根线各自还原：8 脚按键输入（带内部上拉），7 脚 INT 输入，13 脚高阻 */
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M8, GPIO_INPUT | GPIO_PULL_UP);
	(void)gpio_pin_interrupt_configure(ip_port, IP5328_PIN_M8,
					   GPIO_INT_EDGE_TO_INACTIVE);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);
	(void)gpio_pin_configure(ip_port, IP5328_PIN_NFC2, GPIO_INPUT);

	/*
	 * 最后手段：替用户长按 10 秒，把 IP5328 整个复位一次。
	 *
	 * 手册第 18 页：「超长按 10s 可复位整个系统」。芯片只在【自己上电那一刻】
	 * 检测 DMB/DPB 电平来决定进不进 I2C 模式，一旦错过就再也进不去 ——
	 * 而按键复位是唯一不用拔电池就能让它重跑一遍这个检测的软件办法。
	 *
	 * 拉低 10s + 恢复 50ms + 等 3s 让芯片重启完，然后重新探一遍。
	 * 如果模组本身也从 VREG 取电，这一下会把自己也重启 —— 那也没关系，
	 * 重启后固件会重新跑一遍诊断。
	 *
	 * 前面隔 1.2s 再按：手册里「1s 内连续两次短按会强制关机」，
	 * 上面刚按过一次 150ms，离太近可能被算成双击。
	 */
	k_msleep(1200);
	(void)ip5328_key_press(10000);
	k_msleep(3000);

	ip_i2c_set_delay(IP5328_DELAY_FAST);
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);
	ip_i2c_bus_recover();
	ip5328_diag[32] = (uint8_t)ip_i2c_probe(IP5328_ADDR7);

	n_fast = 0U;
	for (uint8_t a = 0x08U; a <= 0x77U; a++) {
		if (ip_i2c_probe(a) == 0) {
			n_fast++;
		}
	}
	ip5328_diag[33] = n_fast;

	LOG_INF("diag int=%u/%u/%u idle=%u%u pup=%u%u pdn=%u%u rec=%u%u "
		"nak=%u/%u/%u hits=%u,%u slow=%u,%u edges=%u/%u "
		"afterkey=%u int=%u hits=%u keynet=%u "
		"m8=%u/%u m7=%u/%u nfc2=%u rev=%u hits=%u hold=%u hits=%u",
		ip5328_diag[1], ip5328_diag[17], ip5328_diag[18], ip5328_diag[2],
		ip5328_diag[3], ip5328_diag[4], ip5328_diag[5], ip5328_diag[6],
		ip5328_diag[7], ip5328_diag[8], ip5328_diag[9], ip5328_diag[10],
		ip5328_diag[11], ip5328_diag[12], ip5328_diag[13], ip5328_diag[14],
		ip5328_diag[15], ip5328_diag[16], ip5328_diag[19],
		ip5328_diag[20], ip5328_diag[21], ip5328_diag[22],
		ip5328_diag[23], ip5328_diag[24], ip5328_diag[25],
		ip5328_diag[26], ip5328_diag[27], ip5328_diag[28],
		ip5328_diag[29], ip5328_diag[30], ip5328_diag[31],
		ip5328_diag[32], ip5328_diag[33]);
}

static int ip5328_sample(struct ip5328_data *d)
{
	int ret;

	*d = (struct ip5328_data){
		.bind = ip_bind,
	};

	/*
	 * 纯 ADC 模式：I2C 线已经拆了，直接返回失败。
	 * d->valid 保持 0，上层就会用电池 ADC 的数据。
	 */
	if (!IP5328_I2C_ENABLE) {
		d->error = -ENOTSUP;
		return -ENOTSUP;
	}

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

static const char *bat_detect_name(uint8_t detect)
{
	switch (detect) {
	case BAT_DETECT_OK:
		return "OK";
	case BAT_DETECT_DISCONNECTED:
		return "DISCONNECTED";
	case BAT_DETECT_OVERRANGE:
		return "OVERRANGE";
	default:
		return "UNKNOWN";
	}
}

static void ip5328_encode_report(const struct ip5328_data *d, const struct ntc_capture *ntc)
{	if (!d->valid) {
		/*
		 * 纯 ADC 模式（I2C 关了，或者读失败）：这一格改报 ADC 实测的数据，
		 * 方便现场验收，不用看串口日志。
		 *
		 *   [0]     0xFE = 本包是 ADC 数据
		 *   [1]     bit0~2 充电状态，bit3 电池脱线，bit4 充电中，
		 *           bit5 电压越界，bit6 已充满（沿用原来的位语义）
		 *   [2]     bit0~2 同上，方便只读一个字节的人
		 *   [3]     电量 %
		 *   [4:6]   电池电压 mV（分压换算回 BAT 端）
		 *   [6:8]   分压后、换算前的原始 mV（1:1 分压时是电池电压的一半）
		 *   [8:10]  充电器输入 VBUS 分压后的 mV，0 = 没插
		 *   [10:12] 充电状态（和 [1] bit0~2 同值）
		 *   [12:14] 模组供电 VDD mV（NTC 分压基准，实测约 3320）
		 *   [14:16] NTC 原始 ADC mV（配合 [12:14] 能反推 NTC 阻值）
		 *
		 * ★ [12:16] 原本是"保留恒 0"，现在改成放 VDD 和 NTC ADC ——
		 *   温度不准的时候，看这两个值就能判断是 NTC 电路的问题还是
		 *   换算的问题，不用接串口。
		 */
		memset(ip5328_report, 0, sizeof(ip5328_report));
		ip5328_report[0] = IP5328_REPORT_ADC_FALLBACK;
		ip5328_report[1] = (uint8_t)((ntc->charge_state & 0x07U) |
					     (ntc->bat_detect == BAT_DETECT_DISCONNECTED ? 0x08U : 0U) |
					     (ntc->charge_state == BAT_STATE_CHARGING ? 0x10U : 0U) |
					     (ntc->bat_detect == BAT_DETECT_OVERRANGE ? 0x20U : 0U) |
					     (ntc->charge_state == BAT_STATE_FULL ? 0x40U : 0U));
		ip5328_report[2] = (uint8_t)(ntc->charge_state & 0x07U);
		/* 报出去的是 OCV（充电中 = 上次拔线后的读数），不是被顶高的那一下 */
		uint16_t soc_mv = ntc->bat_soc_mv > 0U ? ntc->bat_soc_mv : ntc->bat_mv;

		ip5328_report[3] = soc_from_mv(soc_mv);
		sys_put_le16(soc_mv, &ip5328_report[4]);
		sys_put_le16(ntc->bat_raw_mv, &ip5328_report[6]);
		sys_put_le16(ntc->vbus_mv, &ip5328_report[8]);
		sys_put_le16(ntc->charge_state, &ip5328_report[10]);
		sys_put_le16(ntc->vdd_mv, &ip5328_report[12]);
		sys_put_le16(ntc->adc_mv, &ip5328_report[14]);
		return;
	}

	ip5328_report[0] = (uint8_t)(IP5328_REPORT_OK_BASE + d->bind);
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

	if (!IP5328_I2C_ENABLE) {
		/*
		 * 纯 ADC 模式：I2C 三根线（模组 5/6/7 脚）已经拆掉。
		 * 这三脚全部配成纯高阻，不驱动、不加上拉，免得悬空脚互相漏电。
		 * P1.02（模组 8 脚，KEY 唤醒）在 configure_key_wakeup() 里配，那个还要用。
		 */
		(void)gpio_pin_configure(ip_port, IP5328_PIN_M5, GPIO_INPUT);
		(void)gpio_pin_configure(ip_port, IP5328_PIN_M6, GPIO_INPUT);
		(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT);
		(void)gpio_pin_configure(ip_port, IP5328_PIN_NFC2, GPIO_INPUT);
		return 0;
	}

	/* I2C 两脚先放开，外部 3.3k 上拉到 IP5328 的 VREG */
	ip_i2c_bind(IP5328_PIN_M5, IP5328_PIN_M6);

	/* INT：主板醒着为高，平时下拉成确定电平 */
	(void)gpio_pin_configure(ip_port, IP5328_PIN_M7, GPIO_INPUT | GPIO_PULL_DOWN);

	/* 第 8 脚的 KEY 中断在 configure_key_wakeup() 里单独配 */

	/*
	 * P1.03 是 NFC2。NFC 已经整个 disable，两个 NFC 脚之间没有漏电通路，
	 * 而且这一脚板子上是空着的，所以保持纯高阻就行（诊断里当空白对照）。
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

/*
 * 电池电压 → 电量百分比。
 *
 * 用 soc_table[] 这条锂电池放电曲线（4.2V 满充 → 3.3V 空）线性插值，
 * 而不是拿两点做直线。
 *
 * 为什么必须查表：锂电池的放电曲线是**中间很平、两头很陡**的。
 * 4.0~3.7V 这一段占了大概 60% 的容量，但电压只差 300mV。用两点直线的话，
 * 这个区间里电压动一点，百分比就跳一大截，读数毫无意义。
 *
 * 另外要清楚：**光靠电压永远推不出准确电量**。同样的 3.8V，空载静置
 * 可能是 45%，带载放电时可能是 25%。所以这里给的是"静置开压估计值"，
 * 只有在电池静置、没有大电流时才比较可信。要精确得靠库仑计。
 *
 * 之前那个实现是 mv>=3000 就直接返回 100，而实测电池 4130mV，
 * 所以永远报 100% —— 现在改成查表，4130mV 大约落到 83%。
 *
 * IP5328 I2C 那条路（d->batocv_mv）和 ADC 兜底这条路（ADC 测到的 bat_mv）
 * 共用这一个函数，保证两条路给出的百分比口径一致。
 */
static uint8_t soc_from_mv(uint16_t mv)
{
	const size_t last = ARRAY_SIZE(soc_table) - 1U;

	if (mv >= soc_table[0].mv) {
		return soc_table[0].pct;
	}
	if (mv <= soc_table[last].mv) {
		return soc_table[last].pct;
	}

	/* 表是按电压降序排的，找到 mv 落在哪两个点之间 */
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

	/* 理论上到不了这里 */
	return 0U;
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

	/*
	 * 电池电压那一路是"能用就更好"：模组第 4 脚没接分压时读数会贴着 0，
	 * 所以这里失败只告警，不影响 NTC 和 I2C。
	 */
	bat_adc_ready = false;
	if (adc_is_ready_dt(&bat_adc) && adc_channel_setup_dt(&bat_adc) == 0) {
		bat_adc_ready = true;
	} else {
		LOG_WRN("Battery ADC (模组 4 脚 / P1.12 / AIN5) 不可用，电池电压只能靠 IP5328");
	}

	/* 充电器输入 VBUS —— 充电状态的主要依据，见 VBUS_ADC_ENABLE */
	vbus_adc_ready = false;
	if (VBUS_ADC_ENABLE && adc_is_ready_dt(&vbus_adc) && adc_channel_setup_dt(&vbus_adc) == 0) {
		vbus_adc_ready = true;
	} else if (VBUS_ADC_ENABLE) {
		LOG_WRN("VBUS ADC (模组 6 脚 / P1.14 / AIN7) 不可用，充电器插入检测关闭");
	}

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

/*
 * 判断充电状态。
 *
 * 两种信息源，VBUS 优先：
 *
 *   ① 模组第 6 脚 VBUS 有没有电 —— 直接、可靠、不猜。有电就是插着充电器。
 *   ② 电池电压往哪边走 —— 兜底。窗口内每点累加，到点取平均再和上个窗口比。
 *
 * 只看 ② 是不靠谱的：实测电池读数在 4124~4140 之间跳（峰峰值 16mV），
 * 而真实充电 5 分钟才涨 5~10mV，信噪比接近 1；电池接近满时电压被钉在
 * 4.2V，斜率更是直接变成 0。所以 ② 只在没接 ① 的时候当兜底。
 *
 * 组合判断：
 *   插着充电器              → CHARGING，除非电压已到 BAT_FULL_MV 又不动 → FULL
 *   没插 + 电压在跌          → DISCHARGING
 *   没插 + 其它              → IDLE
 *
 * ★ 没插充电器时**永远不判 FULL** —— FULL 的含义是"插着且已充满"。
 *   电池本来就 4.2V，拔线后如果还判 FULL，而广播里 CHARGING 和 FULL 都
 *   报 charging=1，看起来就是"拔了充电器还一直在充电"。这是修掉的那个 bug。
 *
 * 第一个窗口只用来打基准，所以第一次判断要等两个窗口。
 */
static void bat_update_charge_state(uint16_t mv, bool vbus_present)
{
	int64_t now;
	uint16_t avg;

	if (mv == 0U) {
		return; /* 分压没接，不猜 */
	}

	now = k_uptime_get();

	/*
	 * VBUS 一变化就立刻出结论，不等窗口。
	 * 插上/拔掉充电器是个瞬时事件，让人等 5 分钟才看到状态变化没道理。
	 * 同时把趋势窗口重置 —— 插拔瞬间电压会跳，那段数据不能进斜率比较。
	 *
	 * ★ 但"插上就判 FULL"这个逻辑删掉了。
	 * 之前是 `mv >= BAT_FULL_MV ? FULL : CHARGING` —— 只要插上时电压过了阈值
	 * 就立刻报"已充满"，这正是"一插上就说满"的元凶之一。
	 * 电压高不代表充满（充电器把电压顶上去也会高），必须等它稳住不动才算。
	 * 插上后一律先报 CHARGING，让后面的窗口逻辑去判满。
	 */
	if ((int8_t)vbus_present != bat_last_vbus) {
		bat_last_vbus = (int8_t)vbus_present;

		if (vbus_present) {
			/* 插上充电器：先当"充电中"。真满了由窗口逻辑（电压不再涨）判定 */
			bat_charge_state = BAT_STATE_CHARGING;
		} else {
			/*
			 * 拔掉充电器：一律 IDLE —— 绝不能用 mv >= BAT_FULL_MV 判 FULL。
			 * 那块电池本来就充到了 4.2V，一拔线就会满足这个条件，
			 * 而广播把 FULL 也当成"插着充电器"，于是拔了线还显示充电中。
			 */
			bat_charge_state = BAT_STATE_IDLE;
			/* 刚拔线，表面电荷还没散，先别急着把读数当 OCV */
			bat_ocv_not_before = k_uptime_get() + BAT_OCV_SETTLE_MS;
		}

		LOG_INF("charge state -> %u (vbus=%u, 立即判定)", bat_charge_state, vbus_present);

		bat_trend_avg = mv;
		bat_trend_ms = now;
		bat_trend_valid = true;
		bat_trend_sum = 0U;
		bat_trend_count = 0U;
		return;
	}

	bat_trend_sum += mv;
	bat_trend_count++;

	if (now - bat_trend_ms < (int64_t)BAT_TREND_WINDOW_MS) {
		return; /* 窗口还没到，继续攒 */
	}

	avg = (uint16_t)(bat_trend_sum / bat_trend_count);

	if (!bat_trend_valid) {
		/* 第一个窗口只打基准，状态保持 UNKNOWN */
		bat_trend_valid = true;
	} else {
		int delta = (int)avg - (int)bat_trend_avg;
		bool moving = (delta >= BAT_TREND_MIN_MV || delta <= -BAT_TREND_MIN_MV);

		if (vbus_present) {
			/* 插着充电器就是在充电，除非已经满了又不动 */
			bat_charge_state = (avg >= BAT_FULL_MV && !moving) ? BAT_STATE_FULL
									  : BAT_STATE_CHARGING;
		} else if (delta <= -BAT_TREND_MIN_MV) {
			bat_charge_state = BAT_STATE_DISCHARGING;
		} else {
			/* 没插充电器：只有 DISCHARGING / IDLE 两种，永远不判 FULL */
			bat_charge_state = BAT_STATE_IDLE;
		}

		LOG_INF("charge state -> %u (vbus=%u avg %u -> %u mV, delta=%d, %u samples)",
			bat_charge_state, vbus_present, bat_trend_avg, avg, delta,
			bat_trend_count);
	}

	bat_trend_avg = avg;
	bat_trend_ms = now;
	bat_trend_sum = 0U;
	bat_trend_count = 0U;
}

/*
 * 第 6 脚（充电输入分压点）的原始 mV → 插没插充电器。
 *
 * 带 20mV 死区：≥375mV 判"插着"，≤355mV 判"拔了"，中间保持上一次的结论。
 * 实测两个状态是 418mV / 327mV，离阈值都有 30mV 以上，而这条线上的噪声
 * 只有 1~2mV —— 不会像以前那条只剩 3mV 余量的线一样乱翻。
 *
 * 唯一出口：sample_ntc() 的采样和睡眠轮询都走它，状态只有一份，
 * 不会出现"采样说插着、轮询说没插"这种自相矛盾。
 *
 * 返回 true = 插着充电器。
 */
static bool vbus_pin_update(uint16_t pin_mv)
{
	vbus_pin_mv_now = pin_mv;

	if (pin_mv >= VBUS_PIN_PRESENT_MV) {
		vbus_pin_present = true;
	} else if (pin_mv <= VBUS_PIN_ABSENT_MV) {
		vbus_pin_present = false;
	}

	return vbus_pin_present;
}

static int sample_ntc(struct ntc_capture *capture)
{
	uint32_t adc_sum = 0;
	uint32_t vdd_sum = 0;
	uint32_t vbus_sum = 0;
	uint16_t vbus_count = 0;
	uint16_t adc_min = UINT16_MAX;
	uint16_t adc_max = 0;
	int32_t adc_mv;
	int32_t vdd_mv;
	int32_t bat_mv;
	int32_t vbus_mv;
	int last_error = -EIO;
	int ret;

	/*
	 * ★ 电池电压单独存全部样本，最后取中值（不用平均）。
	 *
	 * 为什么要这样：实测抓到一个 bug —— 正常读数 4116mV，偶尔某一次掉到
	 * 3168mV（掉了 950mV，raw 从 2059 变成 1584，比例 0.77）。
	 * 8 次求平均时这一个疙瘩就能把结果从 4116 拉到 3168，
	 * soc_from_mv(3168) 直接算出 0% —— 电量显示瞬间归零，很吓人。
	 *
	 * 中值（排序取中间那个）对少数离群点天然免疫：只要异常次数不到一半，
	 * 结果完全不受影响。8 个样本用插入排序，代码几行就够。
	 *
	 * VBUS 不用这套 —— 它就一条阈值线，插着/没插差着 90mV，取平均就够稳。
	 */
	uint16_t bat_samples[NTC_SAMPLE_COUNT];
	uint16_t bat_count = 0;
	uint32_t bat_raw = 0;

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

		if (bat_adc_ready) {
			ret = read_adc_mv(&bat_adc, &bat_mv);
			if (ret == 0 && bat_count < NTC_SAMPLE_COUNT) {
				bat_samples[bat_count++] =
					(uint16_t)CLAMP(bat_mv, 0, UINT16_MAX);
			}
		}

		if (vbus_adc_ready) {
			ret = read_adc_mv(&vbus_adc, &vbus_mv);
			if (ret == 0) {
				vbus_sum += (uint16_t)CLAMP(vbus_mv, 0, UINT16_MAX);
				vbus_count++;
			}
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

	/*
	 * ★ 电池电压：先插入排序，再取中值（不是平均）。
	 *
	 * 原因见函数开头 bat_samples[] 的说明 —— 偶发一次读数掉 950mV，
	 * 求平均会把电量从 83% 直接拉到 0%。取中值就完全不受影响。
	 */
	if (bat_count > 0U) {
		/* 插入排序：8 个元素，最坏 28 次比较，比调 qsort 轻 */
		for (uint16_t i = 1; i < bat_count; i++) {
			uint16_t key = bat_samples[i];
			int32_t j = (int32_t)i - 1;

			while (j >= 0 && bat_samples[j] > key) {
				bat_samples[j + 1] = bat_samples[j];
				j--;
			}
			bat_samples[j + 1] = key;
		}

		/* 取中值。偶数个取中间两个的平均（这里是 (n-1)/2 和 n/2） */
		bat_raw = (uint32_t)((bat_samples[(bat_count - 1U) / 2U] +
				      bat_samples[bat_count / 2U]) /
				     2U);
	}

	capture->adc_mv = (uint16_t)(adc_sum / capture->sample_count);
	capture->vdd_mv = (uint16_t)(vdd_sum / capture->sample_count);
	capture->adc_min_mv = adc_min;
	capture->adc_max_mv = adc_max;
	capture->ntc_ohms = ntc_resistance_ohms(capture->adc_mv, capture->vdd_mv);
	capture->temp_centi = ntc_ohms_to_centi(capture->ntc_ohms);

	/*
	 * ★ 电池检测：不只看电压合不合理，还要把"为什么不合理"记下来。
	 *
	 * 分压后的电压换算回 BAT 端：(adc × NUM) / DEN。
	 * 换算完再判区间 —— 悬空脚读出来的值大概率落不进来。
	 *
	 * 判据（BAT_DETECT_*，见宏定义）：
	 *   ① 一次样本都没采到（bat_count == 0）
	 *      → ADC 通道本身坏了，不是电池的问题，报 UNKNOWN
	 *   ② 中值电压换回 BAT 端后 < BAT_ADC_MIN_VALID_MV（1500mV）
	 *      → 分压脚没有有效电平：没接电池、或者分压电阻/走线断了
	 *        报 DISCONNECTED，bat_mv 置 0
	 *   ③ 电压 > BAT_ADC_MAX_VALID_MV（5000mV）
	 *      → 单节锂电不可能到 5V 以上，多半是基准漂了或采样脚短路到 VBUS
	 *        报 OVERRANGE，bat_mv 置 0（宁可不显示也不显示错的）
	 *   ④ 落在区间内 → OK，bat_mv 用实测值
	 */
	if (bat_count == 0U) {
		capture->bat_detect = BAT_DETECT_UNKNOWN;
	} else if (bat_raw > 0U) {
		/* 分压比走运行期可调的千分比（0303 特征），调准后就是默认值 */
		uint32_t bat = (uint32_t)(((uint64_t)bat_raw * bat_div_permille) / 1000U);

		capture->bat_raw_mv = (uint16_t)MIN(bat_raw, 0xFFFFU);
		if (bat >= BAT_ADC_MIN_VALID_MV && bat <= BAT_ADC_MAX_VALID_MV) {
			capture->bat_mv = (uint16_t)bat;
			capture->bat_detect = BAT_DETECT_OK;
		} else if (bat > BAT_ADC_MAX_VALID_MV) {
			capture->bat_detect = BAT_DETECT_OVERRANGE;
		} else {
			capture->bat_detect = BAT_DETECT_DISCONNECTED;
		}
	} else {
		capture->bat_detect = BAT_DETECT_DISCONNECTED;
	}

	/*
	 * 充电器输入：**只看第 6 脚原始 mV**（阈值见 VBUS_PIN_PRESENT_MV）。
	 * 插着才写 vbus_mv —— 充电状态就是按"vbus_mv 是不是 0"判的。
	 */
	if (vbus_count > 0U) {
		uint16_t avg = (uint16_t)(vbus_sum / vbus_count);

		if (vbus_pin_update(avg)) {
			capture->vbus_mv = avg;
			/* 反算值只留给日志/调试看（分压比走 0304 特征可调） */
			capture->vbus_in_mv = (uint16_t)MIN(
				(uint32_t)(((uint64_t)avg * vbus_div_permille) / 1000U),
				UINT16_MAX);
		}
	}

	/*
	 * ★ 充电时电池分压点会被顶到充电电压（恒压阶段就是 4.2V），那不是电芯电压 ——
	 *   只有拔掉充电器、并且等表面电荷散掉之后的读数才是真正的电池电压。
	 *   所以：没插充电器且过了结算时间才刷新 OCV；充电中沿用上次的 OCV。
	 *   这样电量不会因为"充电器把电压顶高"而瞬间盐到 100%。
	 */
	if (capture->vbus_mv == 0U && capture->bat_detect == BAT_DETECT_OK &&
	    capture->bat_mv > 0U && k_uptime_get() >= bat_ocv_not_before) {
		bat_ocv_mv = capture->bat_mv;
		bat_ocv_valid = true;
	}

	capture->bat_soc_mv = bat_ocv_valid ? bat_ocv_mv : capture->bat_mv;

	bat_update_charge_state(capture->bat_mv, capture->vbus_mv != 0U);
	capture->charge_state = bat_charge_state;

	return 0;
}

/*
 * 睡眠期间快速查一下有没有插充电器：只读 VBUS 那一路（4 次取平均），
 * 不做 NTC、不碰电池、不动广播。
 *
 * 判定和 sample_ntc() 共用 vbus_pin_update()（同一份滞回状态），
 * 所以一个插拔边沿只会被认到一次，不会出现两边结论相反。
 */
static bool vbus_present_now(void)
{
	uint32_t sum = 0U;
	uint16_t count = 0U;
	int32_t mv;

	if (!vbus_adc_ready) {
		return vbus_pin_present;
	}

	for (int i = 0; i < 4; i++) {
		if (read_adc_mv(&vbus_adc, &mv) == 0) {
			sum += (uint16_t)CLAMP(mv, 0, UINT16_MAX);
			count++;
		}
	}

	if (count == 0U) {
		/* 读失败就沿用上一次的结论，别把状态翻成"没插" */
		return vbus_pin_present;
	}

	return vbus_pin_update((uint16_t)(sum / count));
}

/*
 * 温度趋势：一次采样涨 TEMP_RISE_WAKE_CENTI 以上 → 常醒 TEMP_RISE_HOLD_MS
 * （每次新爬升续期）；一次采样跌 TEMP_FALL_SLEEP_CENTI 以上 → 立刻清掉回睡。
 * 采样失败时 temp_centi 是 INT16_MIN，直接跳过，别污染趋势。
 */
static void temp_trend_update(int16_t centi)
{
	if (centi <= -30000) {
		return;
	}

	if (temp_prev_valid) {
		int32_t delta = (int32_t)centi - (int32_t)temp_prev_centi;

		if (delta >= TEMP_RISE_WAKE_CENTI) {
			temp_rise_until = k_uptime_get() + TEMP_RISE_HOLD_MS;
		} else if (delta <= -TEMP_FALL_SLEEP_CENTI) {
			temp_rise_until = 0;
		}
	}

	temp_prev_centi = centi;
	temp_prev_valid = true;
}

/* 温度突变爬升的"常醒"是否还在有效期内 */
static bool temp_rising_active(void)
{
	return k_uptime_get() < temp_rise_until;
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
	} else if (ntc->bat_soc_mv > 0U || ntc->bat_mv > 0U) {
		/*
		 * I2C 不通，用 ADC 实测的电压。
		 * ★ 用 bat_soc_mv：充电中它会退回上次拔线后的 OCV，
		 *   免得把充电器顶上去的 4.2V 当成电芯电压，电量虚高。
		 */
		volt_mv = ntc->bat_soc_mv > 0U ? ntc->bat_soc_mv : ntc->bat_mv;
		soc = soc_from_mv(volt_mv);
	} else {
		/* 分压也没接，只能拿模组自己的供电电压充数（不是电池电压） */
		soc = soc_from_mv(ntc->vdd_mv);
		volt_mv = ntc->vdd_mv;
	}

	bthome_service_data[BTHOME_BATTERY_OFFSET] = soc;
	put_s16_le(&bthome_service_data[BTHOME_TEMP_OFFSET], ntc->temp_centi);
	/*
	 * 0x16 charging：这是布尔量，只能表达"有没有在充电"。
	 * CHARGING（正在充）和 FULL（插着但已充满）都算"插着充电器"报 1，
	 * 其余（IDLE 待机 / DISCHARGING 放电 / UNKNOWN 没攒够窗口）报 0。
	 */
	bthome_service_data[BTHOME_CHARGING_OFFSET] =
		(ntc->charge_state == BAT_STATE_CHARGING || ntc->charge_state == BAT_STATE_FULL)
			? 1U
			: 0U;
	/*
	 * 0x4A voltage：电池电压。
	 * ★ factor 是 0.1V，填的是**十分之一伏**：4120mV → 41（HA 显示 4.1V）。
	 * 电池电压只看趋势，0.1V 分辨率就够。填 mV 会显示成 412.0V，差十倍。
	 */
	sys_put_le16((uint16_t)(volt_mv / BATTERY_VOLTAGE_DECIVOLTS_DIV),
		     &bthome_service_data[BTHOME_VOLTAGE_OFFSET]);
	bthome_service_data[BTHOME_VERSION_OFFSET] = APP_PATCHLEVEL;
	bthome_service_data[BTHOME_VERSION_OFFSET + 1U] = APP_VERSION_MINOR;
	bthome_service_data[BTHOME_VERSION_OFFSET + 2U] = APP_VERSION_MAJOR;

	/*
	 * 标准 Battery Service（0x180F / Battery Level 0x2A19）——
	 * 和 BTHome 广播同时存在：任何通用 BLE 客户端（包括 HA 的通用集成，
	 * 它根本不懂 BTHome）都能直接读出电量。
	 */
	(void)bt_bas_set_battery_level(MIN(soc, 100U));

	ip5328_encode_report(ip, ntc);

	LOG_INF("temp=%d.%02dC ntc=%uohm adc=%umV vdd=%umV samples=%u", ntc->temp_centi / 100,
		abs(ntc->temp_centi % 100), ntc->ntc_ohms, ntc->adc_mv, ntc->vdd_mv,
		ntc->sample_count);
	LOG_INF("batadc raw=%umV -> bat=%umV (soc 用 %umV) soc=%u detect=%s", ntc->bat_raw_mv,
		ntc->bat_mv, ntc->bat_soc_mv, soc, bat_detect_name(ntc->bat_detect));
	LOG_INF("vbus=%umV(%umV) charge_state=%u  0x4A 填 %u(0.1V档,=%umV)",
		ntc->vbus_mv, ntc->vbus_in_mv, ntc->charge_state,
		ntc->bat_mv / BATTERY_VOLTAGE_DECIVOLTS_DIV, ntc->bat_mv);
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

static ssize_t read_voltage_cal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	const uint16_t *ratio = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, ratio, sizeof(*ratio));
}

static ssize_t write_voltage_cal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	uint16_t *ratio = attr->user_data;
	uint16_t value;

	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	value = sys_get_le16((const uint8_t *)buf);
	/* 上限 60000：×60 已经远超任何想得出来的分压比，再大基本是写错了 */
	if (value > 60000U) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	*ratio = value;
	LOG_INF("voltage ratio -> %u permille", value);

	return len;
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

/* 第 6 脚原始 mV（只读）。判插拔就靠这个数，所以给它一个能直接读的窗口 */
static ssize_t read_vbus_pin(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &vbus_pin_mv_now,
				 sizeof(vbus_pin_mv_now));
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
	BT_GATT_CHARACTERISTIC(&app_bat_div_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_voltage_cal, write_voltage_cal, &bat_div_permille),
	BT_GATT_CUD("Battery divider x1000", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(&app_vbus_div_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_voltage_cal, write_voltage_cal, &vbus_div_permille),
	BT_GATT_CUD("VBUS divider x1000", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(&app_vbus_pin_uuid.uuid, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_vbus_pin, NULL, NULL),
	BT_GATT_CUD("Pin6 raw mV", BT_GATT_PERM_READ),
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
 *
 * 两种参数：
 *   ADV_FAST —— 20ms 一条、可连接（BT_LE_ADV_CONN_FAST_2）：
 *               常醒 / 充电中 / 开机后 / OTA 窗口用，随时连得上，最费电。
 *   ADV_SLOW —— 1 秒一条、仍然可连接：休眠期间用。HA 那边电量/充电状态
 *               一直是新鲜的（插拔也立刻刷新），也不会因为"3 分钟没广播"
 *               被标成不可用；平均电流只有快播的 2%，该睡照睡。
 */
enum adv_mode {
	ADV_OFF,
	ADV_FAST,
	ADV_SLOW,
};

static enum adv_mode adv_current = ADV_OFF;

/*
 * ★★ 两套广播参数必须定义在【文件作用域】，不能写在函数里再返回地址 ★★
 *
 * BT_LE_ADV_CONN_FAST_2 / BT_LE_ADV_PARAM 展开出来的是**复合字面量**：
 * 写在函数内部时，它的生命周期只到该函数返回为止。0.40.0 曾经把它写在
 * adv_param_for() 里、再把地址返回出去，于是 bt_le_adv_start() 拿到的是一块
 * 已经失效的栈内存（未定义行为）：参数是垃圾值时它返回 -EINVAL，而调用点
 * 是 (void)start_advertising() —— 返回值被丢掉。
 * 症状就是最坏的那一种：设备活着、采样正常、特征读得到，但**一个字都不广播**
 * → 扫不到 → 连不上 → 再也 OTA 不进去（只能有线刷）。
 * 放文件作用域，生命周期 = 整个程序，取地址永远安全。
 */
static const struct bt_le_adv_param *const adv_fast_param = BT_LE_ADV_CONN_FAST_2;
/* 1 秒一条（0x0640 = 1600 × 0.625ms）、可连接、无超时 */
static const struct bt_le_adv_param *const adv_slow_param =
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, 0x0640, 0x0640, NULL);

static const struct bt_le_adv_param *adv_param_for(enum adv_mode mode)
{
	return mode == ADV_FAST ? adv_fast_param : adv_slow_param;
}

/*
 * 切到指定广播模式（同一个模式就只把 payload 重发一遍）。
 *
 * 广播数据是原地改的（bthome_service_data[]），所以每刷新一次值就必须再调
 * 一次 start/update，新数据才能真的发出去。
 * ★ 同一个模式时不能只调 bt_le_adv_update_data：有人连上来时控制器会自动
 *   停掉广播，只有再 start 一次才能恢复可连接（以前那条老路就是这么写的）。
 */
static int adv_apply(enum adv_mode mode)
{
	int ret;

	if (mode == adv_current) {
		if (mode == ADV_OFF) {
			return 0;
		}

		ret = bt_le_adv_start(adv_param_for(mode), ad, ARRAY_SIZE(ad), sd,
				      ARRAY_SIZE(sd));
		if (ret == -EALREADY) {
			ret = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		}
	} else {
		if (adv_current != ADV_OFF) {
			(void)bt_le_adv_stop();
			adv_current = ADV_OFF;
		}

		if (mode == ADV_OFF) {
			return 0;
		}

		ret = bt_le_adv_start(adv_param_for(mode), ad, ARRAY_SIZE(ad), sd,
				      ARRAY_SIZE(sd));
	}

	if (ret) {
		/*
		 * 失败不留状态：清掉 adv_current，下一次调用就走"先 stop 再 start"
		 * 的完整路径重试，而不是在同一模式上打转。
		 * 广播开不起来是致命的（扫不到 = 连不上 = 再也 OTA 不了），
		 * 所以这里必须能自愈，不能一失败就永久静默。
		 */
		LOG_ERR("advertising (mode %u) failed: %d", (unsigned int)mode, ret);
		adv_current = ADV_OFF;
		return ret;
	}

	if (adv_current != mode) {
		adv_current = mode;
		LOG_INF("advertising as %s (%s)", DEVICE_NAME,
			mode == ADV_FAST ? "fast 20ms" : "slow 1s");
	}

	return 0;
}

/* 常醒 / 充电中 / OTA 窗口：20ms 快速可连接广播 */
static int start_advertising(void)
{
	return adv_apply(ADV_FAST);
}

/*
 * 广播 duration 之后停止。中间如果有人连上（OTA），就等它断开再停。
 * 这一段就是"轮询唤醒窗口"：设备只在窗口内可被扫到并连接。
 */
static void advertise_then_stop(k_timeout_t duration)
{
	int ret = adv_apply(ADV_FAST);
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

	(void)adv_apply(ADV_OFF);
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

	/*
	 * ★ 开播之前先采一次真实的 ADC。
	 *
	 * 之前这里是直接 start_advertising()，广播 payload 用的是 set_error_capture()
	 * 留下的默认值：bat_mv=0 → soc=0%、charge_state=0 → charging=0。
	 * 结果开机后第一个广播帧永远是"电量 0%、没插充电器"，要等下一轮采样
	 * （静默期 + 第一轮循环，十几秒）才变成真值。手机 App 一开机扫到那一帧，
	 * 就会把设备记成"电量 0%"，看着像坏了一样。
	 *
	 * ADC 采样不受下面 I2C 静默期影响（静默期只管 I2C 两脚），所以放心提前采。
	 */
	if (sensor_ready) {
		ret = sample_ntc(&capture);
		if (ret) {
			LOG_WRN("开机首次采样失败: %d", ret);
			set_error_capture(&capture, ret);
		}
	}
	publish_sensors(&capture, &ip);

	/*
	 * 开机后先安静一会儿：这期间照常广播（可以连上来刷机、读特征），
	 * 但一个字节都不碰 I2C 两脚。
	 *
	 * IP5328 是在【上电那一刻】检测这两脚为高才进 I2C 模式的。
	 * 我们以前一上电就探测、拉低、全地址扫描，很可能正好把它的检测过程搅掉，
	 * 它一旦没进模式，之后怎么读都不会应答。
	 */
	/*
	 * ★ 开播失败不能就这么算了。广播是设备的唯一入口（HA 读数据、电脑 OTA
	 *   都靠它），不广播 = 这台设备从此再也进不去（只能有线刷）。
	 *   所以这里失败就隔一秒重试，仍失败也留日志；main 循环里还会继续重试。
	 */
	for (int attempt = 0; attempt < 5; attempt++) {
		if (start_advertising() == 0) {
			break;
		}
		LOG_ERR("广播启动失败，第 %d 次重试", attempt + 1);
		k_sleep(K_SECONDS(1));
	}

	k_sleep(K_MSEC(IP5328_QUIET_BOOT_MS));

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
			/* 喂温度趋势：突变爬升 → 常醒，突变跌落 → 回睡 */
			temp_trend_update(capture.temp_centi);
		}

		/*
		 * 只在开机后第一次、以及按键唤醒时重跑诊断。
		 * 按一下充电宝的键就能拿到一份最新的总线状态。
		 */
		if (!diag_ran || key_wake_count != diag_key_count) {
			/*
			 * 不是上电后第一次、而是按键把它叫起来的 → 打上 bit5 标记。
			 * 这样按一下实体键再读诊断，就能确认第 8 脚唤醒真的通了：
			 * [0] 的 bit5 从 0 变 1 就是证据。
			 */
			diag_from_key = diag_ran;
			ip5328_diag_run();
			diag_key_count = key_wake_count;
			diag_ran = true;
		}

		ret = ip5328_sample(&ip);
		if (ret) {
			LOG_WRN("IP5328 sample failed: %d (bind=%u)", ret, ip_bind);
		}
		publish_sensors(&capture, &ip);

		/*
		 * 什么时候不休眠：
		 *   1. 充电中（CHARGING / FULL）—— 插着充电器就不睡（反正有外电），
		 *      数据 5 秒一刷，拔掉立刻回休眠周期
		 *   2. 温度突变爬升后的 10 分钟内（TEMP_RISE_HOLD_MS）
		 * 充电状态只看第 6 脚电压（vbus_mv），插拔一变化就立刻出结论。
		 */
		bool charging = (bat_charge_state == BAT_STATE_CHARGING ||
				 bat_charge_state == BAT_STATE_FULL);

		if (!sleep_enabled || charging || temp_rising_active()) {
			/*
			 * 常醒：20ms 快速可连接广播不收，数据每 5 秒刷一次，
			 * 按一下键也能立刻刷。想回正常休眠就往 "Sleep enable" 写 1
			 * （充电中 / 温度爬升时写了也不睡，条件消失才生效）。
			 */
			(void)start_advertising();
			(void)k_sem_take(&wake_sem, TEST_SAMPLE_INTERVAL);
		} else {
			/* OTA 窗口：20ms 快速可连接广播，电脑端轮询到就能刷机 */
			advertise_then_stop(K_SECONDS(OTA_WINDOW_SECONDS));

			/*
			 * ★ 平时（休眠期间）**不静音**：切成 1 秒一条的慢速广播。
			 *   以前这里一停广播就是 8 分钟，HA 那边电量/充电状态全冻住，
			 *   3 分钟收不到还直接标成"不可用" —— 插拔反馈"不实时"一半是
			 *   这么来的。慢速广播平均只有几 µA，该睡照睡。
			 */
			(void)adv_apply(ADV_SLOW);

			LOG_INF("idle, wait up to %d min or KEY (count=%u)", 10,
				key_wake_count);

			/*
			 * 睡着期间也要能"马上"发现插拔 / 温度突变：
			 *   每 SLEEP_POLL_MS（1 秒）读一次第 6 脚 —— 电压一变就立刻完整采样
			 *   + 刷新广播；插上就 break 出去进常醒分支，拔掉当场改报未充电。
			 *   每 SLEEP_TEMP_TICKS 次（15 秒）采一次完整数据：喂温度趋势，
			 *   顺带刷新广播（HA 那边不会长时间收不到数据）。
			 *   按键唤醒同样 break。
			 */
			int64_t deadline = k_uptime_get() + (int64_t)SAMPLE_INTERVAL_MS;
			uint32_t tick = 0U;

			while (k_uptime_get() < deadline) {
				bool present_before = vbus_pin_present;

				if (k_sem_take(&wake_sem, SLEEP_POLL_MS) == 0) {
					break;
				}

				if (vbus_present_now() != present_before) {
					if (sample_ntc(&capture) == 0) {
						temp_trend_update(capture.temp_centi);
					}
					publish_sensors(&capture, &ip);
					(void)adv_apply(ADV_SLOW);

					if (vbus_pin_present) {
						LOG_INF("插上充电器 → 立刻醒");
						break;
					}

					LOG_INF("拔掉充电器 → 立刻改报未充电");
				}

				if (++tick >= SLEEP_TEMP_TICKS) {
					struct ntc_capture tmp;

					tick = 0U;
					if (sample_ntc(&tmp) == 0) {
						temp_trend_update(tmp.temp_centi);
						publish_sensors(&tmp, &ip);
						(void)adv_apply(ADV_SLOW);

						if (temp_rising_active()) {
							LOG_INF("温度突变爬升 → 立刻醒");
							break;
						}
					}
				}
			}
		}

		/* 按键机械抖动，等电平稳定后再采样 */
		k_msleep(KEY_DEBOUNCE_MS);
	}

	return 0;
}
