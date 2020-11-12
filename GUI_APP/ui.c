/**
 * @file ui.c
 * @brief
 * @author  xiaowine (xiaowine@sina.cn)
 * @version 01.00
 * @date    2020-10-12
 *
 * @copyright Copyright (c) {2020}  xiaowine
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-10-12 <td>1.0     <td>wangh     <td>内容
 * </table>
 * ******************************************************************
 * *                   .::::
 * *                 .::::::::
 * *                ::::::::::
 * *             ..:::::::::::
 * *          '::::::::::::
 * *            .:::::::::
 * *       '::::::::::::::..        女神助攻,流量冲天
 * *            ..::::::::::::.     永不宕机,代码无bug
 * *          ``:::::::::::::::
 * *           ::::``:::::::::'        .:::
 * *          ::::'   ':::::'       .::::::::
 * *        .::::'      ::::     .:::::::'::::
 * *       .:::'       :::::  .:::::::::' ':::::
 * *      .::'        :::::.:::::::::'      ':::::
 * *     .::'         ::::::::::::::'         ``::::
 * * ...:::           ::::::::::::'              ``::
 * *```` ':.          ':::::::::'                  ::::.
 * *                   '.:::::'                    ':'````.
 * ******************************************************************
 */

#include "ui.h"
#include "timer.h"
u16 picNow = 0;
/**
 * @brief ui task
 */
void ui(void)
{
    if (MS1msFlag)
    {
        ReadDGUS(0x0014, (u8*)(&picNow), 2);
    }
    if (MS500msFlag)
    {
        if (picNow == 0x002c)
        {
            u16 cache[4];
            ReadDGUS(0xcca0, (u8*)&cache[3], 2);
            cache[0] = ((cache[3] >> 4) & 0x0f00);
            cache[0] |= ((cache[3] >> 7) & 0x1f);
            cache[1] = ((cache[3] & 0x007f) << 8);
            cache[2] = SOFTWARE_VER;
            WriteDGUS(0xcc20, (u8*)cache, 6);
        }
        if (picNow == PAGE00)
        {
        }
        /**
         * @brief   standby
         */
        {
            static u16 timerCounter = 0;
            u16 cache;
            ReadDGUS(0x0016, (u8*)&cache, 2);
            if (cache != 0)
            {
                cache = 0;
                WriteDGUS(0x0016, (u8*)&cache, 2);
                timerCounter = 0;
            }
            else
            {
                if (timerCounter < STANGBYTIME)
                    timerCounter++;
                else if (timerCounter == STANGBYTIME)
                {
                    timerCounter++;
                    if (picNow != 0)
                        JumpPage(0);
                    //用户等级
                }
            }
        }
    }
}
/**
 * @brief  jump tu id page
 * @param  pageId page od
 */
void JumpPage(uint16_t pageId)
{
    uint8_t temp[4] = {0x5A, 0x01, 0, 0};
    temp[2]         = (uint8_t)(pageId >> 8);
    temp[3]         = pageId;
    WriteDGUS(0x0084, temp, sizeof(temp));
    // do
    // {
    //     DelayMs(5);
    //     ReadDGUS(DHW_SPAGE, temp, 1);
    // } while (temp[0] != 0);
}
