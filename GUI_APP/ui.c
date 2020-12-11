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
u16 picNow                 = 0;
static u16 childLockCount  = 0;
const u16 factorySetData[] = {0x0069, 0x005a, 0x003c, 0x0015};

void uiPage00Opt(void);

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
        if (picNow == PAGE00)
        {
            uiPage00Opt();
        }
        if (picNow == PAGE24)
        {
            u16 cache[4];
            ReadDGUS(0xb820, (u8*)&cache[0], 2);
            if ((cache[0] != 0) && (cache[0] <= 4))
            {
                WriteDGUS(0xb824, (u8*)&factorySetData[cache[0] - 1], 2);
                cache[0] = 0x005a;
                WriteDGUS(0xb884, (u8*)&cache[0], 2);
                cache[0] = 0;
                WriteDGUS(0xb820, (u8*)&cache[0], 2);
            }
        }
        if (picNow == PAGE25)
        {
            u16 cache[4];
            ReadDGUS(0xb923, (u8*)&cache[3], 2);
            cache[0] = ((cache[3] >> 4) & 0x0f00);
            cache[0] |= ((cache[3] >> 7) & 0x1f);
            cache[1] = ((cache[3] & 0x007f) << 8);
            cache[2] = SOFTWARE_VER;
            WriteDGUS(0xb924, (u8*)cache, 6);
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
        /**
         * @brief   childLock
         */
        {
            if (childLockCount > 1)
                childLockCount--;
            else if (childLockCount == 1)
            {
                u16 cache = 0;
                childLockCount--;
                cache = 0;
                WriteDGUS(0xa02c, (u8*)&cache, 2);
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
/**
 * @brief
 */
void uiPage00Opt(void)
{
    u16 cache[10];
    u16 alarmCache[6] = {0};
    u8 i, j, k = 0;
    ReadDGUS(0xa020, (u8*)&cache[0], 10);
    //水位状态
    if (((cache[2] & (1 << 3))) == 0)  //到达低水位
    {
        cache[5] = 1;

        if ((cache[0] & 0x40) != 0)  //饮水箱4浮球
        {
            if (((cache[2] & (1 << 6))) == 0)  //到达中水位
            {
                cache[5] = 2;
                if (((cache[2] & (1 << 4))) == 0)  //到达满水位
                {
                    cache[5] = 3;
                }
                else
                {
                    cache[5] = 2;
                }
            }
            else
            {
                cache[5] = 1;
            }
        }
        else
        {
            if (((cache[2] & (1 << 4))) == 0)  //到达满水位
            {
                cache[5] = 3;
            }
            else
            {
                cache[5] = 1;
            }
        }
    }
    else  //缺水
    {
        cache[5] = 0;
    }
    WriteDGUS(0xa0a0, (u8*)&cache[5], 2);

    //滤芯状态
    if (cache[3] & (1 << 14))
    {
        if (cache[3] & (1 << 15))
        {
            cache[5] = 6;
        }
        else
        {
            cache[5] = 1;
        }
    }
    else if (cache[3] & (1 << 15))
    {
        cache[5] = 2;
    }
    else if (cache[4] & (1 << 0))
    {
        cache[5] = 3;
    }
    else if (cache[4] & (1 << 1))
    {
        cache[5] = 4;
    }
    else if (cache[4] & (1 << 2))
    {
        cache[5] = 5;
    }
    else
    {
        cache[5] = 0;
    }
    WriteDGUS(0xa0a1, (u8*)&cache[5], 2);

    cache[3] &= 0x1FFF;
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 16; j++)
        {
            if (cache[3 + i] & (1 << j))
            {
                alarmCache[k++] = j + 1 + (16 * i);
            }
            if (k >= 6)
                break;
        }
        if (k >= 6)
            break;
    }
    WriteDGUS(0xa0a2, (u8*)&alarmCache[0], 10);

    ReadDGUS(0xa027, (u8*)&cache[6], 6);

    //出水状态
    //  if (cache[7] == 1)  //长按出水
    {
        cache[9] = 0;
    }
    //  else if (cache[7] == 0)  //点动出水
    {
        if ((cache[1] & (1 << 2)) == 0)  //	if ("S-系统状态字/Out_Water" == 0)
        {
            cache[9] = 1;
        }
        else
        {
            cache[9] = 2;
        }
    }
    if (cache[2] & (1 << 3))  // if (("S-数字输入/DI3") != 0)
    {
        if (cache[6] == 0)
        {
            cache[9] = 4;
        }
        else
        {
            cache[9] = 3;
        }
    }
    //贮存状态
    if (cache[8] == 1)  //"P-外接水源"
    {
        cache[9] = 5;
    }
    else if (cache[8] == 2)  //"P-外接水源"
    {
        cache[9] = 6;
    }
    else if (cache[8] == 3)  //"P-外接水源"
    {
        cache[9] = 7;
    }
    else if (cache[8] == 4)  //"P-外接水源"
    {
        cache[9] = 8;
    }
    WriteDGUS(0xa0a7, (u8*)&cache[9], 2);

    ReadDGUS(0xa02a, (u8*)&cache[6], 2);
    if (cache[1] & (1 << 2))  //	if ("S-系统状态字/Out_Water" == 0)
    {
        cache[7] = cache[6];
    }
    else
    {
        cache[7] = 0;
    }
    WriteDGUS(0xa02d, (u8*)&cache[7], 2);
}

/**
 * @brief
 */
void heatLockHandle(void)
{
    u16 cache      = 0;
    childLockCount = 11;
    cache          = 1;
    WriteDGUS(0xa02c, (u8*)&cache, 2);
}