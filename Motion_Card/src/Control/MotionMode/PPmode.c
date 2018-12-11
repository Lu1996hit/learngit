/*
 *        File: PPmode.c
 *     Version: 2.0
 * Description: 点位运动
 *              在点位运动模式下，各轴可以独立设置目标位置、目标速度、加速度、减速度、起跳
 *              速度、平滑时间等运动参数，能够独立运动或停止。
 *
 *              可设置参数：    目标位置
 *                              目标速度（全程匀速段速度）   这个值区分正负
 *                              加速度
 *                              减速度
 *                              起跳速度            //只有当前速度为0的时候，起跳速度才有效
 *                              平滑时间
 *              在已有的点位运动的基础上增加了平滑化处理、修改了结构体部分内容
 *
 *  Modified on: 2018年12月10日
 *       Author: Joye、 Lu
 *        Email: chenchenjoye@sina.com
 */

#include "math.h"
#include "system.h"
#include <Kernel/interpolate.h>
#include "taskComm.h"
#include "PPmode.h"
#include "JOGmode.h"            //有几个函数是和Jog模式一起公用的

/*
 * 用来存储点位运动和JOG运动的参数
 */

PP_JOG_KERNEL pjkernel[AXISNUM];

/*
 * 初始化PP JOG运动的参数。
 */
void Init_PP_JOG_Kernel()    // KERNEL 结构体用以记录轴的运行状态
{                                   //而 PP_JOG_KERNEL 结构体细化到了两种运动模式具体的状态规划
    int axis;
    PP_JOG_KERNEL *pPJ = pjkernel;
    for(axis = 0; axis < AXISNUM; axis++)
    {
        pPJ->objVel = 5;         // 目标速度
        pPJ->acc = 0.01;         // 加速度
        pPJ->dec = 0.01;         // 减速度
        pPJ->startVel = 2;       // 起跳速度
        pPJ->endVel = 0;         // 设定速度
        pPJ->smoothTime = 20;    // 平滑时间
        pPJ->smooth = 0.2;       // 平滑系数
        pPJ ++;
    }
}

/*
 * 设置指定轴为点位运动模式
 * 1.先检查当前轴是否处在可规划的模式
 * 2.如果是IDLE模式下，可以设置为其他的模式
 */
static ERROR_CODE PrfTrap()
{
    int axis;
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            if(kernel[axis].kersta == MOTORS_STA_IDLE)  //检查当前轴是否处于空闲状态（当前运行状态）
                kernel[axis].axsta = MOTORS_STA_PPMODE;
            else
                return RTN_ERROR;
        }
    }
    return RTN_SUCC;
}

/*
 * 设置点位运动模式下的运动参数
 */
static ERROR_CODE SetTrapPrm()
{
    int axis;
    KernelPrm *pKer;
    PP_JOG_KERNEL *pPJ;   // 用于传送参数的临时变量
    unsigned int *p = &cmd.prm[3];
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)    // 检查是否为指定轴
        {
            pKer = &kernel[axis];     // 利用指针对轴的参数结构体（全局变量）进行操作
            if(pKer->axsta != MOTORS_STA_PPMODE)   // 当前轴不是处于点位运动模式下，则设置运动参数指令无效
                return RTN_ERROR;

            pPJ = &pjkernel[axis];
            memcpy(&pPJ->acc,p,2);  // memcpy(void *dest, const void *src, int size)
            p += 2;
            memcpy(&pPJ->dec,p,2);
            p += 2;
            memcpy(&pPJ->startVel,p,2);
            p += 2;
            memcpy(&pPJ->smoothTime,p,1);
            p += 1;
            pKer->flag = 1;
        }
    }
    return RTN_SUCC;
}

/*
 * 读取点动运动模式下的运动参数
 */
static ERROR_CODE GetTrapPrm()
{
    int axis;
    PP_JOG_KERNEL *pPJ;
    unsigned int *p = cmd.buf;
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            if(kernel[axis].axsta != MOTORS_STA_PPMODE) //  检查当前轴是否被指定为点动运动模式
                return RTN_ERROR;

            /*———按照加速度、减速度、起跳速度、平滑时间的顺序逐个拷贝数据———*/
            pPJ = &pjkernel[axis];
            memcpy(p,&pPJ->acc,2);
            p += 2;
            memcpy(p,&pPJ->dec,2);
            p += 2;
            memcpy(p,&pPJ->startVel,2);       // 传过来的仅仅是起跳速度的大小、并无方向信息
            p += 2;
            memcpy(p,&pPJ->smoothTime,1);
            p += 1;
            cmd.buflen += 7;
        }
    }
    return RTN_SUCC;
}

/*
 * 设定目标位置
 */
static ERROR_CODE SetPos()
{
    int axis;
    KernelPrm *pKer;
    PP_JOG_KERNEL *pPJ;
    unsigned int *p = &cmd.prm[3];
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            pKer = &kernel[axis];
            if(pKer->axsta != MOTORS_STA_PPMODE)
                return RTN_ERROR;

            pPJ = &pjkernel[axis];
            memcpy(&pPJ->objPos,p,2);
            p += 2;
            pKer->flag = 1;                    // 需要重新规划伺服电机的运动
        }
    }
    return RTN_SUCC;
}

/*
 * 读取目标位置
 */
static ERROR_CODE GetPos()
{
    int axis;
    PP_JOG_KERNEL *pPJ;
    unsigned int *p = cmd.buf;
    for(axis = 0; axis < AXISNUM; axis ++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            if(kernel[axis].axsta != MOTORS_STA_PPMODE)
                return RTN_ERROR;
            pPJ = &pjkernel[axis];
            memcpy(p,&pPJ->objPos,2);                    // 目标位置是否存在有正有负的情况？？
            p += 2;
            cmd.buflen += 2;
        }
    }
    return RTN_SUCC;
}

/*
 * 共用函数
 * 与Jog模式公用一个函数
 * 设置目标速度
 */
static ERROR_CODE SetVel()
{
    int axis;
    KernelPrm *pKer;
    PP_JOG_KERNEL *pPJ;
    unsigned int *p = &cmd.prm[3];
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            pKer = &kernel[axis];
            if(pKer->axsta != MOTORS_STA_PPMODE && pKer->axsta != MOTORS_STA_JOGMODE)
                return RTN_ERROR;
            pPJ = &pjkernel[axis];
            memcpy(&pPJ->objVel,p,2);     // 设置的目标速度是有速度方向信息的矢量
            p += 2;
            pKer->flag = 1;                // 需要重新规划伺服电机的运动
        }
    }
    return RTN_SUCC;
}

/*
 * 共用函数
 * 与Jog模式公共用一个函数
 * 读取目标速度
 */
static ERROR_CODE GetVel()
{
    int axis;
    PP_JOG_KERNEL *pPJ;
    unsigned int *p = cmd.buf;
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)
        {
            if(kernel[axis].axsta != MOTORS_STA_PPMODE && kernel[axis].axsta != MOTORS_STA_JOGMODE)
                return RTN_ERROR;
            pPJ = &pjkernel[axis];
            memcpy(p,&pPJ->objVel,2);
            p += 2;
            cmd.buflen += 2;
        }
    }
    return RTN_SUCC;
}

/*
 * 规划点动运动模式
 */
double cal_pos[7];                      // 加加速、匀加速、减加速、匀速、加减速、匀减速、减减速运动七段走过的路程
    // 调试，结束后仍作为临时变量
ERROR_CODE Prep_PPmode(int axis)
{
    KernelPrm *pKer = &kernel[axis];        // 电机内核结构体
    PP_JOG_KERNEL *pPJ = &pjkernel[axis];   // 点位运动和JOG运动模式下的共用结构体
    long set_pos;
    double objVel;                          //目标速度，这是一个正值

    if(pKer->flag == 0)                     //是否需要重新规划 1：需要 0：不需要
        return RTN_SUCC;


    /*————————— 确定运动方向、得到目标速度的绝对值 —————————————*/
    if( pPJ->objPos > pKer->nowPos )            //正向运动
    {
        set_pos = pPJ->objPos - pKer->nowPos;
        pKer->dir = 1;                      // 运动方向标志位置位
        if(pPJ->objVel < pPJ->startVel || pPJ->objVel < pPJ->endVel)
            return RTN_ERROR;               // 检查速度参数是否合理
        objVel = pPJ->objVel;               // 正向点位运动时，目标速度应当大于起跳速度和设定速度
    }
    else if( pPJ->objPos < pKer->nowPos )       // 反向运动
    {
        set_pos =  pKer->nowPos - pPJ->objPos;
        pKer->dir = 0;                      // 运动方向标志位复位
        if( -pPJ->objVel < pPJ->startVel || -pPJ->objVel < pPJ->endVel ) // 这里的起跳速度是一个仅仅表示速度大小的正数endVel一定为0
            return RTN_ERROR;               // 类比反向点位运动，其运动曲线应该为倒梯形
        objVel = -pPJ->objVel;
    }
    else
        return RTN_ERROR;                   //目标位置等于当前位置，出错退出

    double t0,t1,t2;                        // 匀加速时间、匀速时间和匀减速时间
    double v[4];                            // 各个变加速运动阶段开始前或结束后的速度
    double a,b,c;
    /*
     * 生成起始速度
     * 判断是衔接上一次的速度还是选择从起跳速度开始启动
     */
    if(pKer->nowVel == 0)                  // 起跳速度只有在实际起始速度为 0 时才有效
    {
        if(pKer->dir == 1)
        {
            pPJ->realstartVel = pPJ->startVel; // 起跳速度根据位置信息确定正负性
        }
        else
        {
            pPJ->realstartVel = - pPJ->startVel;
        }

    }
    else
        pPJ->realstartVel = pKer->nowVel;

    /* ————————— 平滑化梯形速度曲线规划（此时仅考虑正向运动）————————*/

    v[0] = pPJ->startVel + pPJ->smoothTime * pPJ->acc / 2;     // 加加速阶段结束后的速度
    cal_pos[0] = pPJ->startVel * pPJ->smoothTime + pPJ->smoothTime * pPJ->smoothTime * pPJ->acc / 6;
                                                               // 加加速阶段走过的路程
    v[1] = objVel - pPJ->smoothTime * pPJ->acc / 2;            // 减加速阶段开始前速度
    v[2] = objVel - pPJ->smoothTime * pPJ->dec / 2;            // 加减速阶段结束后的速度
    v[3] = pPJ->endVel + pPJ->smoothTime * pPJ->dec / 2;       // 减减速阶段开始前的速度

    if( v[1]<v[0] || v[3]>v[2] )  return RTN_ERROR;
    t0 = (v[1] - v[0])/ pPJ->acc;
    t2 = (v[2] - v[3])/ pPJ->dec;

    cal_pos[1] = (v[1] + v[0]) * t0 / 2;                           // 匀加速走过的路程
    cal_pos[4] = v[2] * pPJ->smoothTime + pPJ->smoothTime * pPJ->smoothTime * pPJ->dec/3;
                                                                   // 加减速走过的路程
    cal_pos[2] = v[1] * pPJ->smoothTime + pPJ->smoothTime * pPJ->smoothTime * pPJ->acc/3;
                                                                   // 减加速走过的路程
    cal_pos[5] = (v[3] + v[2]) * t2 / 2;                           // 匀减速走过的路程
    cal_pos[6] = pPJ->endVel * pPJ->smoothTime + pPJ->smoothTime * pPJ->smoothTime * pPJ->dec / 6;
                                                                   // 减减速阶段走过的路程
    cal_pos[3] = set_pos - cal_pos[0] - cal_pos[1] - cal_pos[2] - cal_pos[4] - cal_pos[5] - cal_pos[6];
                                                                   // 匀速走过的路程
    if(cal_pos[3] >= 0)                                            // 速度曲线可以构成一个完整的梯形
    {
        t1 = cal_pos[3] / objVel;
    //  pPJ->downVel = objVel;                                     // 减速时的转折速度
    }
    else                                                          // 平滑化曲线的任务转化为平滑化三角形
    {
        /*—————— 通过求解一元二次方程确定三角形平滑化后的最大速度 ————————*/
        a = 1 / ( 2 * pPJ->dec ) + 1 / ( 2 * pPJ->acc );
        b = pPJ->smoothTime;
        c = cal_pos[0] + cal_pos[6] - set_pos - pPJ->smoothTime * pPJ->smoothTime * (pPJ->acc+pPJ->dec)/6 +
                (( pPJ->smoothTime * pPJ->acc / 2 )*( pPJ->smoothTime * pPJ->acc / 2 ) - v[0]*v[0])/(2 * pPJ->acc)
                        +(( pPJ->smoothTime * pPJ->dec / 2 )* ( pPJ->smoothTime * pPJ->dec / 2 ) - v[3]*v[3])/(2 * pPJ->dec);
        objVel = ( -b + sqrt(b * b - 4 * a * c) ) /( 2 * a);  // 更新平滑化后可以达到的最大速度
        v[1] = objVel - pPJ->smoothTime * pPJ->dec / 2;       // 由 最大速度 平滑过渡至 v1
        v[2] = objVel - pPJ->smoothTime * pPJ->dec / 2;       // 由 v2 平滑过渡至终值速度
        t0 = (v[1] - v[0])/pPJ->acc;                          // 匀加速过程用时
        t2 = (v[2] - v[3])/pPJ->dec;                          // 匀减速过程用时
        t1 = 0;
        if( v[1]<v[0] || v[3]> v[2] ) return RTN_ERROR;       // 路程过短或平滑时间过长返回报错
        if(pKer->dir == 1)
        {
            pPJ->objVel = objVel;
        }
        else
        {
            pPJ->objVel = -objVel;
        }
    }

    pPJ->t[0] = Approximate(t0 / INTERRUPT_TIM);
    pPJ->t[1] = Approximate(t1 / INTERRUPT_TIM);
    pPJ->t[2] = Approximate(t2 / INTERRUPT_TIM);              // 0.1ms 中断，故在此时间均乘以系数 10

    pPJ->turnVel[0] = v[0];
    pPJ->turnVel[1] = v[1];
    pPJ->turnVel[2] = v[2];
    pPJ->turnVel[3] = v[3];                                   // 各个加速度转折点处对应的速度值


    //pPJ->smoothTime = pPJ->smoothTime / INTERRUPT_TIM ;  // 同步时间轴,以下所有计数变量均为中断时间0.1ms的整数倍
    pKer->lastVel = pPJ->realstartVel;
    /*——————— 电机的规划位置还需要考虑其实际的行进方向 —————————*/

    // 原程序是根据梯形（三角形）速度曲线在速度转折点处位置信息进行分段控制的
     /* 点位运动程序优化后，不再分段式控制，以前程序和变量也就不再被使用到
    pPJ->pos[0] = pKer->nowPos;                   // 记录起始位置
    if(pKer->dir == 1)
    {
        pPJ->pos[1] = pPJ->pos[0] + pos0;
        pPJ->pos[2] = pPJ->pos[1] + pos1;
    }
    else
    {
        pPJ->pos[1] = pPJ->pos[0] - pos0;
        pPJ->pos[2] = pPJ->pos[1] - pos1;
    } ------------------------------------------------------------------------*/
    pKer->flag = 0;                               // 不需要重新规划路线
    pKer->count = 0;                              // 时间计数器清零
    // pKer->step = 0;
    //pKer->aimPos = pPJ->objPos;                 // 当前运行状态下的目标位置
    return RTN_SUCC;
}

/*
 * 开启点位运动
 */
static ERROR_CODE Open_PPmode(KernelPrm *pKer)
{
    pKer->kersta = MOTORS_STA_PPMODE;
    pKer->step = 0;
    pKer->count = 0;
    return RTN_SUCC;
}

/*
 * 开启Jog运动
 */
static ERROR_CODE Open_JOGmode(KernelPrm *pKer)
{
    pKer->kersta = MOTORS_STA_JOGMODE;
    pKer->step = 0;
    pKer->count = 0;
    return RTN_SUCC;
}

/*
 * 强制停止点位运动
 */
//static ERROR_CODE Stop_PP_JOG_mode(KernelPrm *pKer)
//{
//  pKer->state =  MOTORS_STA_IDLE;
//  pKer->flag = 1;
//  pKer->step = 0;
//  pKer->count = 0;
//  pKer->nowVel = 0;
//  pKer->nowacc = 0;
//  return RTN_ERROR;
//}

/*
 * 启动或停止点位运动或Jog运动、暂时不控制停止运动
 * 注意！！！
 * 需要控制点位运动或者Jog运动的开与关
 */
static ERROR_CODE Update()
{
    ERROR_CODE rtn;
    int axis;
//  KernelPrm *pKer;
    for(axis = 0; axis < AXISNUM; axis++)
    {
        if((cmd.mark >> axis) & 0x01)           // 确定对应电机轴
        {
            if(kernel[axis].axsta == MOTORS_STA_PPMODE)    // 点位运动模式
            {
                rtn = Prep_PPmode(axis);
                if(rtn == RTN_ERROR)
                    return rtn;
                else
                    Open_PPmode(&kernel[axis]);
            }
            else if(kernel[axis].axsta == MOTORS_STA_JOGMODE)   // Jog 运动模式
            {
                rtn = Prep_JOGmode(axis);
                if(rtn == RTN_ERROR)
                    return rtn;
                else
                    Open_JOGmode(&kernel[axis]);
            }
        }
//      else                                    //关闭状态      暂时不使用
//      {
//          pKer = &kernel[axis];
//          if(pKer->state == MOTORS_STA_PPMODE || pKer->state == MOTORS_STA_JOGMODE)
//              Stop_PP_JOG_mode(&kernel[axis]);
//      }
    }
    return rtn;
}

/*
 * 解码指令
 */
void Decouple_PPmode()
{
    switch(cmd.type)
    {
        case CMD_PP_MODE:           cmd.rtn = PrfTrap();                    break;
        case CMD_PP_SETPRM:         cmd.rtn = SetTrapPrm();                 break;
        case CMD_PP_GETPRM:         cmd.rtn = GetTrapPrm();                 break;
        case CMD_PP_SETPOS:         cmd.rtn = SetPos();                     break;
        case CMD_PP_GETPOS:         cmd.rtn = GetPos();                     break;
        case CMD_SETVEL:            cmd.rtn = SetVel();                     break;
        case CMD_GETVEL:            cmd.rtn = GetVel();                     break;
        case CMD_UPDATE:            cmd.rtn = Update();                     break;
        default:                    cmd.rtn = RTN_INVALID_COMMAND;          break;
    }
}

/*
 * 点动模式运行函数 在中断中运行
 * 策略：梯形速度曲线，三段式规划，可以使用smoothTime的值来修改加速度的过程,使速度曲线平滑化
 */

void Run_PPmode(int axis)
{

    double real_time;                    // 用于记录程序实际运行时间的临时变量
    PVAT_S pvat;
    KernelPrm *pKer = &kernel[axis];
    PP_JOG_KERNEL *pPJ = &pjkernel[axis];
    long pos, stime = pPJ->smoothTime * INTERRUPT_FRE;   // stime 是为了便于统一计算的临时变量
    if(pKer->flag == 1)                    // 需要重新规划时，等待规划完成再运行控制函数
        return;
   // return;
    /*———— 点位运动模式下的运动规划，主要包含有当前速度、加速度、下一步真实位置的更新 ————
     * ———————————————— 注意考虑电机实际的运动方向————————————————
     */

    real_time = pKer->count * INTERRUPT_TIM;
    switch(pKer->step)
    {
        case 0:            // 起始变加速运动( 加速度线性增加至 pPJ->acc )
        {
            if(stime != 0)
            {
                pKer->count ++;
                if(pKer->count >= stime)        //最后一次
                {
                    pKer->count = 0;
                    pKer->step = 1;
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pPJ->realstartVel + pPJ->acc * real_time * real_time / ( 2 * pPJ->smoothTime );
                        pKer->nowacc = pPJ->acc * real_time / pPJ->smoothTime;
                    }
                    else
                    {
                        pKer->nowVel = pPJ->realstartVel - pPJ->acc * real_time * real_time / ( 2 * pPJ->smoothTime );
                        pKer->nowacc = - pPJ->acc * real_time / pPJ->smoothTime;
                    }
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 1;         //转跳下一步
            }

        }
        case 1:            // 匀加速运动过程
        {
            if(pPJ->t[0] != 0)
            {
                pKer->count ++;
                if(pKer->count >= pPJ->t[0])        //最后一次
                {
                    pKer->count = 0;
                    pKer->step = 2;
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pKer->lastVel + pPJ->acc *INTERRUPT_TIM ;
                        pKer->nowacc = pPJ->acc;
                    }
                    else
                    {
                        pKer->nowVel = pKer->lastVel - pPJ->acc *INTERRUPT_TIM ;
                        pKer->nowacc = - pPJ->acc;
                    }
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 2;         //转跳下一步
            }

        }
        case 2:            // 匀加速过渡到匀速运动过程（减加速运动过程）
        {
            if(stime != 0)
            {
                pKer->count ++ ;
                if(pKer->count >= stime)
                {
                    pKer->count = 0;
                    pKer->step = 3;         //转跳下一步
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pPJ->turnVel[1] + pPJ->acc * real_time - pPJ->acc * real_time * real_time / ( pPJ->smoothTime * 2 );
                        pKer->nowacc = pPJ->acc - pPJ->acc * real_time / pPJ->smoothTime;
                    }
                    else
                    {
                        pKer->nowVel = - pPJ->turnVel[1] - pPJ->acc * real_time + pPJ->acc * real_time * real_time / ( pPJ->smoothTime * 2 );
                        pKer->nowacc = - pPJ->acc + pPJ->acc * real_time/ pPJ->smoothTime;
                    }


                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 3;         //转跳下一步

            }

        }
        case 3:            // 匀速运动过程
        {
            if(pPJ->t[1] != 0)
            {
                pKer->count ++;
                if(pKer->count >= pPJ->t[1])
                {
                    pKer->count = 0;
                    pKer->step = 4;         //转跳下一步
                }
                else
                {
                    pKer->nowVel = pKer->lastVel;
                    pKer->nowacc = 0;
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 4;         //转跳下一步
            }
        }
        case 4:            // 匀速过渡到匀减速过程（加减速过程）
        {
            if(stime != 0)
            {
                pKer->count ++;
                if(pKer->count >= stime)
                {
                    pKer->count = 0;
                    pKer->step = 5;         //转跳下一步
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pPJ->objVel - pPJ->dec * real_time * real_time / ( 2 * pPJ->smoothTime );
                        pKer->nowacc = pPJ->dec * real_time / pPJ->smoothTime ;
                    }
                    else
                    {
                        pKer->nowVel = pPJ->objVel + pPJ->dec * real_time * real_time / ( 2 * pPJ->smoothTime );
                        pKer->nowacc = - pPJ->dec * real_time / pPJ->smoothTime;
                    }
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 5;         //转跳下一步
            }
        }
        case 5:            // 匀减速运动过程
        {
            if(pPJ->t[2] != 0)
            {
                pKer->count ++;
                if(pKer->count >= pPJ->t[2])
                {
                    pKer->count = 0;
                    pKer->step = 6;         //转跳下一步
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pKer->lastVel - pPJ->dec *INTERRUPT_TIM ;
                        pKer->nowacc = - pPJ->dec;
                    }
                    else
                    {
                        pKer->nowVel = pKer->lastVel + pPJ->dec *INTERRUPT_TIM ;
                        pKer->nowacc = pPJ->dec;
                    }
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 6;         //转跳下一步
            }

        }
        case 6:            // 匀减速过渡到停止状态 (MOTORS_STA_IDLE)
        {
            if(stime != 0)
            {
                pKer->count ++;
                if(pKer->count >= stime)
                {
                    pKer->count = 0;
                    pKer->step = 0;       // 结束
                    pKer->kersta = MOTORS_STA_IDLE;
                }
                else
                {
                    if(pKer->dir == 1)
                    {
                        pKer->nowVel = pPJ->turnVel[3] - pPJ->dec * real_time + pPJ->dec * real_time * real_time /(pPJ->smoothTime * 2);
                        pKer->nowacc = pPJ->dec - pPJ->dec * real_time/pPJ->smoothTime;
                    }
                    else
                    {
                        pKer->nowVel = - pPJ->turnVel[3] + pPJ->dec * real_time - pPJ->dec * real_time * real_time /(pPJ->smoothTime * 2);
                        pKer->nowacc = - pPJ->dec + pPJ->dec * real_time /pPJ->smoothTime;
                    }
                }
                break;
            }
            else
            {
                pKer->count = 0;
                pKer->step = 0;       // 结束
                pKer->kersta = MOTORS_STA_IDLE;
            }

        }
    }

    /*——————————————— 电机运动位置更新 ————————————————*/
    pKer->realPos = pKer->realPos + ( pKer->lastVel + pKer->nowVel ) * INTERRUPT_TIM /2;   // 此处的速度是考虑了其大小和方向的

    pKer->lastVel = pKer->nowVel;       // 更新存储的速度值

    /*——————————— 同电机的 FPGA 部分相关 —————————————*/
    pos = Approximate(pKer->realPos);
    pvat.aim_pos = pos;
    pvat.start_acc = 0;
    pvat.start_vel = (pos - pKer->nowPos)*10000;
    pvat.min_period = TIME;
    M_SetPvat(axis, &pvat);
    pKer->nowPos = pvat.aim_pos;
    return;
}
