/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
    本程序是基于DWin的4寸480*480温控器C语言例子程序，去除了不需要的按键检测、RTC等
不需要的命令、函数。
   配置如下：
     1. UART2 作为log的输出，用于监控程序的状态
     2. UART4 作为ModBus的通讯串口，处理发送和接收命令，
     3. 其他为迪文的DGUS基本配置，
     功能如下：
     1. 实现Modbus RTU主站命令，03读取命令，06写寄存器命令，控制、显示modbus从站状态
     2. 实现UI的显示，控制，状态的更新，

     说明：程序可自由传播，但请仔细阅读后再应用，对引起的任何问题无任何的责任。使用
过程中，如果有疑问，可以加V(181 1266 9427)共同探讨。
******************************************************************************/
#include "modbus.h"
#include "sys.h"
#include "crc16.h"
#include "uart.h"
#include "ui.h"

void Modbus_RX_Reset(void);
// void Modbus_TX_Reset(void);
void Modbus_Write_Register06H(modbosCmd_t *CmdNow, u16 value);
void Modbus_Read_Register(modbosCmd_t *CmdNow);
void modbus_process_command(u8 *pstr, u16 strlen);

u8 modbus_rx_count = 0;                 //接收到的字符串的长度
u8 modbus_rx_flag  = 0;                 //接收到的字符串的标志，为1表示有收到数据
u8 modbus_rx_buf[UART_RX_BUF_MAX_LEN];  //接收到的字符串的内容

// extern process_struct process_flag;  //命令状态标志
extern u32 data SysTick;        //每隔1ms+1
u32 uart_rx_check_tick    = 0;  //检查串口是否接收结束
u8 modbus_rx_count_before = 0;  //接收串口的数据

u32 modbus_tx_process_tick = 0;  // modbus发送命令的时间间隔

const modbosCmd_t modbusCmdlib[] = {
    // en         id         fun    len  timeout   mod    modP     VP  slaveAddr feedback
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE00, 0xa020, 0x004b, 0x00ff},  // P-DI0
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE00, 0xa021, 0x01f6, 0x00ff},  //系统状态字 数字输入
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE00, 0xa023, 0x01f9, 0x00ff},  //告警状态字0 1
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE00, 0xa025, 0x01ff, 0x00ff},  //温湿度
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE00, 0xa027, 0x0064, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE00, 0xa028, 0x006d, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE00, 0xa029, 0x0072, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PARA, 0xa08a, 0xa02a, 0x006e, 0x00ff},  // waterOut key
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PARA, 0xa08b, 0xa02b, 0x006f, 0x00ff},  // waterOut key
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE11, 0xab20, 0x0201, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE11, 0xab21, 0x0203, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE11, 0xab22, 0x0202, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PAGE, PAGE11, 0xab23, 0x020a, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE12, 0xac20, 0x00ab, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x09, 0xc8, MODE_PAGE, PAGE12, 0xac21, 0x01f6, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE12, 0xac2a, 0x0206, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x04, 0xc8, MODE_PAGE, PAGE12, 0xac2b, 0x0213, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE13, 0xad20, 0x01f6, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE13, 0xad21, 0x01fb, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE13, 0xad22, 0x0217, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE13, 0xad23, 0x021a, 0x00ff},  //
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x05, 0xc8, MODE_PANP, 0xae00, 0xae20, 0x0064, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xae00, 0xae25, 0x0094, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xae00, 0xae26, 0x00a1, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xae00, 0xae28, 0x00b6, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae80, 0xae20, 0x0064, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae81, 0xae21, 0x0065, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae82, 0xae22, 0x0066, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae83, 0xae23, 0x0067, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae84, 0xae24, 0x0068, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae85, 0xae25, 0x0094, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae86, 0xae26, 0x00a1, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae87, 0xae27, 0x00a2, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae88, 0xae28, 0x00b6, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xae89, 0xae29, 0x00b7, PAGE14},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xaf00, 0xaf20, 0x0048, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xaf00, 0xaf21, 0x004b, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xaf00, 0xaf22, 0x0069, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xaf00, 0xaf24, 0x0072, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xaf00, 0xaf25, 0x0096, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xaf00, 0xaf28, 0x009e, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf80, 0xaf20, 0x0048, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf81, 0xaf21, 0x004b, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf82, 0xaf22, 0x0069, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf83, 0xaf23, 0x006a, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf84, 0xaf24, 0x0072, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf85, 0xaf25, 0x0096, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf86, 0xaf26, 0x0097, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf87, 0xaf27, 0x0098, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf88, 0xaf28, 0x009e, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf89, 0xaf29, 0x009f, PAGE15},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb020, 0x0071, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb021, 0x007f, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb022, 0x0081, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb000, 0xb023, 0x009b, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb025, 0x00a0, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb000, 0xb026, 0x00b9, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb028, 0x00bd, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb000, 0xb029, 0x00c2, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb080, 0xb020, 0x0071, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb081, 0xb021, 0x007f, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb082, 0xb022, 0x0081, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb083, 0xb023, 0x009b, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb084, 0xb024, 0x009c, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb085, 0xb025, 0x00a0, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb086, 0xb026, 0x00b9, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb087, 0xb027, 0x00ba, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb088, 0xb028, 0x00bd, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb089, 0xb029, 0x00c2, PAGE16},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb100, 0xb120, 0x006b, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb100, 0xb122, 0x0074, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb100, 0xb124, 0x0077, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb100, 0xb125, 0x00a7, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb100, 0xb127, 0x00aa, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xb100, 0xb128, 0x00ac, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb180, 0xb120, 0x006b, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb181, 0xb121, 0x006c, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb182, 0xb122, 0x0074, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb183, 0xb123, 0x0075, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb184, 0xb124, 0x0077, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb185, 0xb125, 0x00a7, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb186, 0xb126, 0x00a8, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb187, 0xb127, 0x00aa, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb188, 0xb128, 0x00ac, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb189, 0xb129, 0x00ad, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb18a, 0xb12a, 0x00ae, PAGE17},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb380, 0xb320, 0x0082, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE19, 0xb321, 0x0208, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE19, 0xb323, 0x0211, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE19, 0xb324, 0x020d, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x08, 0xc8, MODE_PANP, 0xb400, 0xb420, 0x008b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb480, 0xb420, 0x008b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb481, 0xb421, 0x008c, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb482, 0xb422, 0x008d, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb483, 0xb423, 0x008e, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb484, 0xb424, 0x008f, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb485, 0xb425, 0x0090, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb486, 0xb426, 0x0091, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb487, 0xb427, 0x0092, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb500, 0xb520, 0x0072, PAGE21},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb580, 0xb520, 0x0072, PAGE21},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb581, 0xb521, 0x0083, PAGE21},
};
modbosCmd_t modbusCmdNow = {0};
u8 CmdIndex              = 0;

const dataCheckCmd_t dataCheckLib[] = {
    // en     page    data    back    flag
    {BUS_EN, PAGE14, 0xae20, 0xae50, 0xae80},  //
    {BUS_EN, PAGE14, 0xae21, 0xae51, 0xae81},  //
    {BUS_EN, PAGE14, 0xae22, 0xae52, 0xae82},  //
    {BUS_EN, PAGE14, 0xae23, 0xae53, 0xae83},  //
    {BUS_EN, PAGE14, 0xae24, 0xae54, 0xae84},  //
    {BUS_EN, PAGE14, 0xae25, 0xae55, 0xae85},  //
    {BUS_EN, PAGE14, 0xae26, 0xae56, 0xae86},  //
    {BUS_EN, PAGE14, 0xae27, 0xae57, 0xae87},  //
    {BUS_EN, PAGE14, 0xae28, 0xae58, 0xae88},  //
    {BUS_EN, PAGE14, 0xae29, 0xae59, 0xae89},  //
    {BUS_EN, PAGE15, 0xaf20, 0xaf50, 0xaf80},  //
    {BUS_EN, PAGE15, 0xaf21, 0xaf51, 0xaf81},  //
    {BUS_EN, PAGE15, 0xaf22, 0xaf52, 0xaf82},  //
    {BUS_EN, PAGE15, 0xaf23, 0xaf53, 0xaf83},  //
    {BUS_EN, PAGE15, 0xaf24, 0xaf54, 0xaf84},  //
    {BUS_EN, PAGE15, 0xaf25, 0xaf55, 0xaf85},  //
    {BUS_EN, PAGE15, 0xaf26, 0xaf56, 0xaf86},  //
    {BUS_EN, PAGE15, 0xaf27, 0xaf57, 0xaf87},  //
    {BUS_EN, PAGE15, 0xaf28, 0xaf58, 0xaf88},  //
    {BUS_EN, PAGE15, 0xaf29, 0xaf59, 0xaf89},  //
    {BUS_EN, PAGE16, 0xb020, 0xb050, 0xb080},  //
    {BUS_EN, PAGE16, 0xb021, 0xb051, 0xb081},  //
    {BUS_EN, PAGE16, 0xb022, 0xb052, 0xb082},  //
    {BUS_EN, PAGE16, 0xb023, 0xb053, 0xb083},  //
    {BUS_EN, PAGE16, 0xb024, 0xb054, 0xb084},  //
    {BUS_EN, PAGE16, 0xb025, 0xb055, 0xb085},  //
    {BUS_EN, PAGE16, 0xb026, 0xb056, 0xb086},  //
    {BUS_EN, PAGE16, 0xb027, 0xb057, 0xb087},  //
    {BUS_EN, PAGE16, 0xb028, 0xb058, 0xb088},  //
    {BUS_EN, PAGE16, 0xb029, 0xb059, 0xb089},  //
    {BUS_EN, PAGE17, 0xb120, 0xb150, 0xb180},  //
    {BUS_EN, PAGE17, 0xb121, 0xb151, 0xb181},  //
    {BUS_EN, PAGE17, 0xb122, 0xb152, 0xb182},  //
    {BUS_EN, PAGE17, 0xb123, 0xb153, 0xb183},  //
    {BUS_EN, PAGE17, 0xb124, 0xb154, 0xb184},  //
    {BUS_EN, PAGE17, 0xb125, 0xb155, 0xb185},  //
    {BUS_EN, PAGE17, 0xb126, 0xb156, 0xb186},  //
    {BUS_EN, PAGE17, 0xb127, 0xb157, 0xb187},  //
    {BUS_EN, PAGE17, 0xb128, 0xb158, 0xb188},  //
    {BUS_EN, PAGE17, 0xb129, 0xb159, 0xb189},  //
    {BUS_EN, PAGE17, 0xb12a, 0xb15a, 0xb18a},  //
    {BUS_EN, PAGE20, 0xb420, 0xb450, 0xb480},  //
    {BUS_EN, PAGE20, 0xb421, 0xb451, 0xb481},  //
    {BUS_EN, PAGE20, 0xb422, 0xb452, 0xb482},  //
    {BUS_EN, PAGE20, 0xb423, 0xb453, 0xb483},  //
    {BUS_EN, PAGE20, 0xb424, 0xb454, 0xb484},  //
    {BUS_EN, PAGE20, 0xb425, 0xb455, 0xb485},  //
    {BUS_EN, PAGE20, 0xb426, 0xb456, 0xb486},  //
    {BUS_EN, PAGE20, 0xb427, 0xb457, 0xb487},  //
    {BUS_EN, PAGE21, 0xb520, 0xb550, 0xb580},  //
};

_TKS_FLAGA_type modbusFlag = {0};
/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
 ******************************************************************************
modbus 命令解析处理程序，实现：
1. 03H的回送命令解析
2. 06H的回送命令解析，如果回送命令正确，则停止UI的触发发送命令
******************************************************************************/

void modbus_process_command(u8 *pstr, u16 strlen)
{
    u16 num;
    u16 crc_data;
    u16 len;

    // printf("Modbus string:");
    for (num = 0; num < strlen; num++)
    {
        // printf("%02X ", (u16)(*(pstr + num)));
    }
    // printf(",length:%d\r\n", strlen);

    if (strlen < 5)
    {
        return;
    }
    num = 0;
    do
    {
        if ((*(pstr + num)) == SLAVE_ID)
        {
            switch (*(pstr + num + 1))  //判读下一个字节是modbus的哪个命令
            {
                case BUS_FUN_03H:
                    len = *(pstr + num + 2);
                    if ((len + num + 5) > strlen)  //长度超过最大长度
                    {
                        num = strlen;  //非modbus命令
                        break;
                    }
                    crc_data = crc16table(pstr + num, 3 + len);
                    // printf("num:%d,len:%d,crc data:%02X,%02X,", num, len, (u16)((crc_data >> 8) &
                    // 0xFF),(u16)(crc_data & 0xFF));
                    if ((*(pstr + num + len + 3) != ((crc_data >> 8) & 0xFF)) ||
                        (*(pstr + num + len + 4) != (crc_data & 0xFF)))  // CRC
                    {
                        break;
                    }
                    WriteDGUS(modbusCmdNow.VPAddr, (pstr + 3), *(pstr + 2));
                    memset(&modbusCmdNow, 0, sizeof(modbosCmd_t));
                    num       = len + 5;
                    cmdRxFlag = 1;
                    break;
                case BUS_FUN_06H:
                    if ((num + 8) > strlen)
                    {
                        num = strlen;  //非modbus命令
                        break;
                    }
                    crc_data = crc16table(pstr + num, 6);
                    if ((*(pstr + num + 6) != ((crc_data >> 8) & 0xFF)) ||
                        (*(pstr + num + 7) != (crc_data & 0xFF)))  // CRC
                    {
                        break;
                    }
                    num += 8;
                    memset(&modbusCmdNow, 0, sizeof(modbosCmd_t));
                    cmdRxFlag = 1;
                    break;
                default:
                    break;
            }
        }
        num++;
    } while (num < (strlen - 5));  // addre,command,data,crch,crcl,至少需要有5个字节
}
/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
 ******************************************************************************
modbus 发送和接收任务处理程序，实现：
1. 监控串口接收，当判断接收结束后，调用处理函数，
2. 监控UI的触发命令，当有检测到发送命令时，发送modbus写命令
3. 每隔1秒钟触发一次查询modbus寄存器状态的命令
******************************************************************************/
void Modbus_Process_Task(void)
{
    modbosCmd_t *cmdTemp_t = NULL;
    if (modbus_rx_flag == 1)  //接收数据
    {
        if (modbus_rx_count > modbus_rx_count_before)
        {
            modbus_rx_count_before = modbus_rx_count;
            uart_rx_check_tick     = 0;
        }
        else if (modbus_rx_count == modbus_rx_count_before)
        {
            if (uart_rx_check_tick > 0)
            {
                if ((SysTick - uart_rx_check_tick) > RX_CHECK_TICK_TIME)
                {
                    modbus_process_command(modbus_rx_buf, modbus_rx_count);
                    Modbus_RX_Reset();
                }
            }
            else
            {
                uart_rx_check_tick = SysTick;
            }
        }
    }

    if (cmdTxFlag)
    {
        if ((cmdRxFlag) || ((SysTick - modbus_tx_process_tick) >= modbusCmdNow.timeout))
        {
            if (cmdRxFlag)
                CmdIndex++;
            goto processCMDLib;
        }
        return;
    }

    if ((SysTick - modbus_tx_process_tick) < MODBUS_SEND_TIME_PERIOD)  //间隔固定时间后再处理UI的设置命令，
    {
        return;
    }
processCMDLib:
    if (CmdIndex == 0)
        checkChange();
    modbus_tx_process_tick = SysTick;
    cmdRxFlag              = 0;
    cmdTxFlag              = 0;
    getCmd(&CmdIndex);
    if (CmdIndex < CMD_NUMBER)
    {
        memcpy(&modbusCmdNow, &modbusCmdlib[CmdIndex], sizeof(modbosCmd_t));
        if (modbusCmdNow.funCode == BUS_FUN_03H)
        {
            Modbus_Read_Register(&modbusCmdNow);
            cmdTxFlag = 1;
        }
        else if (modbusCmdNow.funCode == BUS_FUN_06H)
        {
            u16 value;
            ReadDGUS(modbusCmdNow.VPAddr, (u8 *)(&value), 2);
            Modbus_Write_Register06H(&modbusCmdNow, value);
            cmdTxFlag = 1;
        }
    }
    else
    {
        CmdIndex = 0;
    }
}
// modbus 03H 读取寄存器
void Modbus_Read_Register(modbosCmd_t *CmdNow)
{
    u16 crc_data;
    u8 len;
    u8 modbus_tx_buf[20];

    len                  = 0;
    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_03H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = (CmdNow->length >> 8) & 0xFF;  // register number
    modbus_tx_buf[len++] = CmdNow->length & 0xFF;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
    Uart2SendStr(modbus_tx_buf, len);
}

// modbus 06H 发送
void Modbus_Write_Register06H(modbosCmd_t *CmdNow, u16 value)
{
    u16 crc_data;
    u8 len;
    u8 modbus_tx_buf[20];

    len                  = 0;
    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_06H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = (value >> 8) & 0xFF;  // register value
    modbus_tx_buf[len++] = value & 0xFF;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
    Uart2SendStr(modbus_tx_buf, len);
}  // modbus 06H 发送
void Modbus_Write_Register10H(modbosCmd_t *CmdNow, u16 value)
{
    u16 crc_data;
    u8 len;
    u8 modbus_tx_buf[20];

    len                  = 0;
    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_10H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = CmdNow->length * 2;
    modbus_tx_buf[len++] = (value >> 8) & 0xFF;  // register value
    modbus_tx_buf[len++] = value & 0xFF;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
    Uart2SendStr(modbus_tx_buf, len);
}
//清除modbus RX的相关参数
void Modbus_RX_Reset(void)
{
    modbus_rx_count = 0;
    modbus_rx_flag  = 0;
    memset(modbus_rx_buf, '\0', UART_RX_BUF_MAX_LEN);
    modbus_rx_count_before = 0;
    uart_rx_check_tick     = 0;
}
//初始化modbus 相关参数
void Modbus_UART_Init(void)
{
    //	Modbus_TX_Reset();
    Modbus_RX_Reset();
    modbus_tx_process_tick = 0;  //初始化 0
}

void getCmd(u8 *index)
{
    u8 i;
    for (i = *index; i < CMD_NUMBER; i++)
    {
        if ((modbusCmdlib[i].modbusEn != BUS_EN) || (modbusCmdlib[i].length == 0))
        {
            continue;
        }
        if (modbusCmdlib[i].mode == MODE_ALWA)
        {
            goto getCmdExit;
        }
        else if (modbusCmdlib[i].mode == MODE_PAGE)
        {
            if (picNow == modbusCmdlib[i].modePara)
            {
                goto getCmdExit;
            }
            continue;
        }
        else if (modbusCmdlib[i].mode == MODE_PARA)
        {
            u16 paraTemp;
            ReadDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
            if ((paraTemp & 0xff) == 0x5a)
            {
                if (i < CMD_NUMBER - 1)
                {
                    if ((modbusCmdlib[i + 1].mode == MODE_PARA) &&
                        (modbusCmdlib[i].modePara == modbusCmdlib[i + 1].modePara))
                    {
                        goto getCmdExit;
                    }
                }
                paraTemp = 0;
                WriteDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
                goto getCmdExit;
            }
            continue;
        }
        else if (modbusCmdlib[i].mode == MODE_PANP)
        {
            u16 paraTemp;
            if (modbusCmdlib[i].feedback != picNow)
            {
                continue;
            }
            ReadDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
            if ((paraTemp & 0xff) == 0x5a)
            {
                if (i < CMD_NUMBER - 1)
                {
                    if ((modbusCmdlib[i + 1].mode == MODE_PANP) &&
                        (modbusCmdlib[i].modePara == modbusCmdlib[i + 1].modePara))
                    {
                        goto getCmdExit;
                    }
                }
                paraTemp = 0;
                WriteDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
                goto getCmdExit;
            }
            continue;
        }
    }
getCmdExit:
    *index = i;
}

void checkChange(void)
{
    u16 cache[20] = {0};
    u16 i;
    for (i = 0; i < CHECK_NUMBER; i++)
    {
        if (dataCheckLib[i].page != picNow)
            continue;
        ReadDGUS(dataCheckLib[i].dataAddr, (u8 *)&cache[0], 2);
        ReadDGUS(dataCheckLib[i].backAddr, (u8 *)&cache[1], 2);
        if (cache[0] != cache[1])
        {
            WriteDGUS(dataCheckLib[i].backAddr, (u8 *)&cache[0], 2);
            cache[2] = 0x5a;
            WriteDGUS(dataCheckLib[i].flagAddr, (u8 *)&cache[2], 2);
        }
    }
}

void forcedOutputHnadle(void)
{
    u16 cache[7] = {0};
    ReadDGUS(0xc7a0, (u8 *)cache, 12);
    cache[7] = 0x00;
    cache[7] |= ((cache[0] & 1) << 0x00);
    cache[7] |= ((cache[1] & 1) << 0x01);
    cache[7] |= ((cache[2] & 1) << 0x02);
    cache[7] |= ((cache[3] & 1) << 0x06);
    cache[7] |= ((cache[4] & 1) << 0x0c);
    cache[7] |= ((cache[5] & 1) << 0x0d);
    WriteDGUS(0xc722, (u8 *)&cache[7], 2);
    cache[7] = 0x005a;
    WriteDGUS(0xc782, (u8 *)&cache[7], 2);
}
