#ifndef CAN_SLAVE_H
#define CAN_SLAVE_H

#include "CanProtocol.h"
#include "MotorUserControl.h"

/************************ 扩展帧从机配置 ************************/
typedef struct {
    uint8_t sys_num;     // 系统编号
    uint8_t main_cat;    // 设备主类别
    uint8_t sub_cat;     // 设备副类别
    uint8_t dev_num;     // 设备编号
} ExtSlaveConfig_t;

/************************ 通信超时定义 ************************/
#define EXT_CMD_TIMEOUT_MS      100   // 指令报文通信超时(ms)

/************************ 接口 ************************/
void ext_slave_init(const ExtSlaveConfig_t *config);
void ext_slave_process_rx(uint32_t ext_id, const uint8_t *dat, uint8_t len);
void ext_slave_periodic_send(void);
void ext_set_fault(FaultLevel_e level, uint8_t cod);
void ext_clear_fault(void);

/* ==================== 【调试用】协议层计数打印 ====================
 * 【限制】内部使用 printf（阻塞），只能在主循环里调用，
 *         绝不能放进 CAN 中断或 1ms 任务（TickCycle_1ms）里。
 */
void ext_slave_debug_print(void);

/* ==================== 【调试】三个探针变量（定义在 CanSlave.c）====================
 *   g_dbg_filtId  : 最近一个被"地址不匹配/帧号=0"过滤掉的帧 ID
 *   g_dbg_cmdMsgNo: CMD 帧里 msg_num≠1 的个数（会被静默忽略）
 *   g_dbg_lastCtrl: 最近一次真正进入指令处理的 ctrl 字节
 */
extern uint32_t xdata g_dbg_filtId;
extern uint16_t xdata g_dbg_cmdMsgNo;
extern uint8_t  xdata g_dbg_lastCtrl;

/* ==================== 【新增】配置落盘延迟执行 ====================
 * 【限制】内部写 EEPROM（96 字节 = 2 次页写，约 50ms 阻塞），
 *         只能在主循环里调用，绝不能放进 CAN 中断或 1ms 任务（TickCycle_1ms）里。
 * 用法：1ms 任务收到"写配置"报文时只置标志（CanSlave.c 的 g_cfgSaveReq），
 *       由 main.c 大循环调用本函数真正落盘。
 */
void ext_slave_cfg_save_pending(void);

#endif // CAN_SLAVE_H
