/* 激光测距 V6 */
/* UC-OSII版本 */
/* 主控任务、测量生产数据任务、处理数据任务、AGC任务 校准调试任务 */
/* 采用消息队列通信 */
/* 使用TDC-GPX2时间测量芯片 */
/* 多阈值拟合校准 */
/* 自动增益控制(AGC) */
/* 大小目标检测切换 */
/* 优化排序，使用快速排序算法 */
/* date:2020.05.22 Emil: hdu_tangguodong@163.com */

/* 让阈值时间差跟随距离，不再单独对阈值时间差做滤波，保证阈值时间差和输出距离唯一对应 */
/* 提供最后对结果缓冲区内数据两种处理方式，平均和中值，通过宏定义ARGV选择 */
/* 最后再来转double,之前都用time_ps来处理数据，减小精度损失 */
/* 设置调试校准任务 */
/* date:2020.05.24 Emil: hdu_tangguodong@163.com */

/* 930~940 GAIN 1V/V */
/* 135米 1.6V */
/* 多阈值拟合策略: 仅使用err_time3(Vth3 - Vth1) */
/* 0<err_time3<=1120 err = 2.71 */
/* 1120<err_time3<2000  err = -3.019+0.8541*ln(-10.9372*err_time3+11430) */
/* err_time>2000  err = 0.0004*err_time3+4.0737 */
/* todo : 上升沿跳变点。峰值2V以上，err_time3 峰值2V以下，err_time1 */
/* date:2020.05.25 Emil: hdu_tangguodong@163.com */

/* 增加侦察阈值，负责探测是否有有用信号返回，当干扰脉冲大于信号脉冲时能够提高测距能力 */
/* 测试：测大目标基本稳定，测线（只测了26米以内）跳动1~2米，猜测是增益太大，干扰脉冲叠加 */
/* todo: 确定测线问题原因，必要时需减小前级增益，打开AGC_LOW功能（可能会让近距离大目标不准） */
/* date:2020.05.26 Emil: hdu_tangguodong@163.com */

/* 经测试，对于不同反射目标，阈值时间差有不同的误差，特别是近距离阈值时间差 */
/* 这个要解决有点恼火，软件上貌似无能为力，改善硬件或许是唯一的办法 */
/* date:2020.05.29 Emil: hdu_tangguodong@163.com */

/* 需要对拟合曲线不线性区域进行优化（暂时还没方法） */
/* 增加自动档位切换模式代替基于脉冲峰值的AGC模式 实测发现测得更远了 最远稳定测到1030米 */
/* date:2020.06.05 Emil: hdu_tangguodong@163.com */

/* 针对近距离截止失真后阈值时间差跳动到非线性区域导致修正误差过大，
在小于某一个距离时跳过阈值时间差落入非线性区域（1200~1500）的数据 */
/* date:2020.06.09 Emil: hdu_tangguodong@163.com */

/* 优化TDC-GPX2驱动程序，提供一次测量读取缓冲区所有数据 */
/* date:2020.06.10 Emil: hdu_tangguodong@163.com */

/* 优化BUG ,stop通道读取的所有数据，都应该减去start通道读取的第一次数据，后面再读start通道都是0 */
/* 硬件问题：stop通道输入的脉冲有负电压，超过了手册给出的-0.3V，导致有错误数据输出 */
/* 解决思路：加二极管到地，对电压进行钳位 目前电路暂且定型，以后有机会再测试修改硬件 */
/* 最重要的问题1：由于之前的接收发射坏了换了个镜头和接收发射，发现和之前的回波特性不同，
		可能增益更大，导致每套设备可能都要修改程序参数 */
/* 最重要的问题2：严重怀疑干扰脉冲是硬件本身产生的，因为换了个镜头和接收发射小板，干扰脉冲的位置从十几米左右变到了5米
		而且干扰脉冲似乎变大了 猜测不一定对 */
		/* todo: 可以改进去除干扰脉冲的方式，一次测量得到的几组数据中只有一个是正确数据， */		
/* date:2020.06.15 Emil: hdu_tangguodong@163.com */

/* 加入看门狗，避免由于错误的脉冲输入导致TDC芯片的INTERUPT状态一直处于0，无法跳出读取FIFO数据的循环，导致程序跑飞 */
/* date:2020.06.19 Emil: hdu_tangguodong@163.com */

/* 增加用户输入测量模式  是测线还是测墙 */
/* date:2020.07.06 Emil: hdu_tangguodong@163.com */




#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"

#include "spi.h"
#include "tdc_gpx2.h"
#include "laser.h"
#include "tpl0202.h"
#include "tlv5636.h"
#include "agc.h"
#include "key.h"
#include "wdg.h"

//#define DEBUG
/* 校准调试模式参数设置 */
#define STANDARD_DISTANCE 0 //测试目标标准距离
#define INC_STEP 10         //增益增加步进
#define MIN_GAIN 920       //最小增益 quantify DAC量化值
#define MAX_GAIN 1350       //最大增益
/**********************************************/
/* 宏定义模式设置 */
/* 通过注释或定义宏选择模式 */

//#define ADJ //校准调试模式 ,定义它将会启动校准模式
#define MULTI_VTH           //定义它将会把结果进行多阈值拟合校准
//#define AGC_LOW   //如果定义AGC_LOW,AGC控制会将高于CONTROL_MAX_VOLTAGE的时候减少增益，否则只会增大增益
#define AUTO_LEVEL_MOD //自动档位切换模式 不定义则为AGC模式
//#define ARGV  //是否将缓冲区结果求平均，不然取中值

/**********************************************/


/* 将浮点数转换为3个字节表示 */
/* 阈值时间差 */
typedef struct CHANGE_3CHAR_DATA
{
	/* h_data : 整数高字节 */
	/* l_data ：整数低字节 */
	/* f_data : 小数位 */
	INT8U h_data;
	INT8U l_data;
	INT8U f_data;//小数位为 小数*256表示 例如0.01*256 = 2 
	INT32U err_time;
}CH_DATA;

/* INT32U表示的数据 */
/* 阈值时间差 */
typedef struct CHANGE_INT_DATA
{
	INT32U i_data;//距离 低8位小数，其余表示整数
	INT32U err_time;
}INT_DATA;

/* 飞行时间和阈值时间差 */
typedef struct TIME_PS_DATA
{
	INT32U time_ps;
	INT32U err_time1;
	INT32U err_time2;
	INT32U err_time3;
}TIME_DATA;

//#define MAX_LEVEL 8
///* 增益档位表 */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1100,1200,1300,1400,1500,1600,1700,1800
//};
#define MAX_LEVEL 9
/* 增益档位表 */
const INT16U LEVEL[MAX_LEVEL] = 
{
	1100,1150,1200,1250,1300,1350,1400,1450,1500
};

enum {
	wall,//测墙模式
	line//测线模式
};

//#define MAX_LEVEL 1
///* 增益档位表 */
//const INT16U LEVEL[MAX_LEVEL] = 
//{
//	1300
//};
INT8U level = 0;//档位

//UCOS 任务设置
//设置开始任务优先级
#define START_TASK_PRIO 10//一共有10个优先级，开始任务的优先级最低为10
#define START_STAK_SIZE  64//设置开始任务堆栈大小
OS_STK START_TASK_STAK[START_STAK_SIZE];//开始任务堆栈

//设置主控线程优先级最高
#define MASTER_TASK_PRIO 5
#define MASTER_STAK_SIZE 256
__align(8) OS_STK MASTER_TASK_STAK[MASTER_STAK_SIZE];

//设置激光测距生产者任务
#define GEN_TASK_PRIO 6
#define GEN_STAK_SIZE 512
OS_STK GEN_TASK_STAK[GEN_STAK_SIZE];

//设置数据处理消费者任务
#define HANDLE_TASK_PRIO 7
#define HANDLE_STAK_SIZE 128
__align(8) OS_STK HANDLE_TASK_STAK[HANDLE_STAK_SIZE];//使用printf打印浮点数到串口，需要任务堆栈8字节对齐

//设置AGC控制任务
#define AGC_TASK_PRIO 8
#define AGC_STAK_SIZE 64
OS_STK AGC_TASK_STAK[AGC_STAK_SIZE];

#ifdef ADJ
//设置调试校准任务 
#define ADJ_TASK_PRIO 4
#define ADJ_STAK_SIZE 64
__align(8) OS_STK ADJ_TASK_STAK[ADJ_STAK_SIZE];
#endif


void start_task(void *pdata);
void master_task(void *pdata);
void gen_task(void *pdata);
void handle_task(void *pdata);
void agc_task(void *pdata);
void adj_task(void *pdata);

#define DATA_GROUP_SIZE 8 //每一组接收数据长度
#define QSTART_SIZE 256    //消息队列指针数组缓存区大小
void *Qstart[QSTART_SIZE];//消息队列指针数组
OS_EVENT *q_msg;//消息队列
#define ANS_BUF_SIZE 10   //输出结果缓冲区大小
//INT_DATA ans_buf[ANS_BUF_SIZE];//输出结果缓冲区
TIME_DATA ans_buf[ANS_BUF_SIZE];//输出结果缓冲区
//INT32U err_buf[ANS_BUF_SIZE];//输出阈值时间差缓冲区
INT8U ans_ite = 0;//当前写入位置
INT8U ans_cnt = 0;//缓冲区中结果数量

/* 位操作宏 */
#define setbit(x,y) (x)|=(1<<(y))
#define clrbit(x,y) (x)&=~(1<<(y))
#define reversebit(x,y) (x)^=(1<<(y))
#define getbit(x,y) ((x)>>(y)&1)

/* 初始化多阈值参数 */
#define QUANTIFY_INIT 1200  //初始化增益值 DAC
#define START_THREAD 80			
#define STOP_THREAD1 60
#define STOP_THREAD2 80
#define STOP_THREAD3 80

/* 数据筛选校准参数 */
#define GAIN_TH 1200      //增益限值
#define DISTANCE_TH 21    //距离限制
#define TIME_PS_TH 100000 //15米的时间 
#define MAX_TIME 10000000 //最大飞行时间 1500米的时间 ps
#define MIN_TIME 0        //最小飞行时间  
#define DETECT_MIN_TIME 300000 //探测阈值最小飞行时间 45m
#define MAX_ERR_TIME 10000 //最大阈值时间差
#define MIN_ERR_TIME 0    //最小阈值时间差

/* AGC 变量 */
unsigned char AGC_EN;
unsigned short quantify = QUANTIFY_INIT;//DAC 量化值
INT32U detect_val = 0;//侦察阈值检测到的值 time_ps
/* AGC Control parameter     */
#define CONTROL_MAX_VOLTAGE 3
#define FLOW_RANGE 0.2
#define CONTROL_DIFF_TIME 2000
#define TIME_FLOW_RANGE 200

#define MAX_NO_DATA_CNT 4

const int refclk_divisions = 125000;//8M晶振  ,TDC参考时钟

unsigned char outmode = 0;//数据输出模式 0:输出到照相机的格式  1:打印到串口调试字符格式
unsigned char measure_mode = wall;//测量模式，测线模式，测墙模式

void out_mode_set(void);
double ch_data_dou(CH_DATA* pch_data);
CH_DATA dou_ch_data(double* d_data,INT32U err_time);
INT32U ch_data_u32(CH_DATA* pch_data);
double u32_dou(INT32U data);
INT32U dou_u32(double* d_data);
void quick_sort_int_data(INT_DATA* arr,INT32U size);//快速排序
void quick_sort_time_data(TIME_DATA* arr,INT32U size);
void print_ch_data(CH_DATA* ch_data);
void print_int_data(INT32U int_data);
double my_log(double a);
void agc_control(OS_CPU_SR cpu_sr);
void create_time_data(TIME_DATA* ptime_data,INT32U time_ps,INT32U err_time1,INT32U err_time2,INT32U err_time3);
double multi_thread_adj(double *org,unsigned int *time);


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	delay_init();
	uart_init(9600);
	out_mode_set();
	tpl0202_init();
	KEY_Init();
	tlv5636_init();
	agc_init();
	adc_init();
	laser_init();//激光发射初始化
	tdc_init();//时间测量芯片初始化
	tdc_config();//时间测量芯片配置完成
	write_wa(STOP_THREAD1,&TPL1_CS,&TPL1_CLK,&TPL1_D);//set voltage thread of stop1 pulse
	write_wb(START_THREAD,&TPL1_CS,&TPL1_CLK,&TPL1_D);//set voltage thread of start pulse
	write_wa(STOP_THREAD2,&TPL2_CS,&TPL2_CLK,&TPL2_D);//set voltage thread of stop2 pulse
	write_wb(STOP_THREAD3,&TPL2_CS,&TPL2_CLK,&TPL2_D);//set voltage thread of stop3 pulse
	setRefValue(REF1);//set tlv5636 ref voltage mode
	setDacValueBin(quantify);//初始化DAC输出电压,默认增益
	OSInit();//初始化RTOS 启动启动线程
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STAK[START_STAK_SIZE-1],START_TASK_PRIO);
	OSStart();
}

void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;//采用方法3开关中断需要的中间变量
	/* 开机保持到峰值 */
	PEAK_CONTROL = 1;
	delay_ms(1);
	PEAK_CONTROL = 0;
	AGC_EN=1;//开启AGC
	pdata = pdata;//pdata没用时防止waring
	q_msg = OSQCreate((void**)&Qstart[0],QSTART_SIZE);//创建消息队列
	
	IWDG_Init(4,1000);//设置看门狗 1.6s 
	
	OS_ENTER_CRITICAL();//进入临界区，关中断
	#ifdef ADJ  //调试校准模式
	OSTaskCreate(gen_task,(void*)0,(OS_STK*)&GEN_TASK_STAK[GEN_STAK_SIZE-1],GEN_TASK_PRIO);
	OSTaskCreate(adj_task,(void*)0,(OS_STK*)&ADJ_TASK_STAK[ADJ_STAK_SIZE-1],ADJ_TASK_PRIO);
	#else
	OSTaskCreate(master_task,(void*)0,(OS_STK*)&MASTER_TASK_STAK[MASTER_STAK_SIZE-1],MASTER_TASK_PRIO);
	OSTaskCreate(gen_task,(void*)0,(OS_STK*)&GEN_TASK_STAK[GEN_STAK_SIZE-1],GEN_TASK_PRIO);
	OSTaskCreate(handle_task,(void*)0,(OS_STK*)&HANDLE_TASK_STAK[HANDLE_STAK_SIZE-1],HANDLE_TASK_PRIO);
	OSTaskCreate(agc_task,(void*)0,(OS_STK*)&AGC_TASK_STAK[AGC_STAK_SIZE-1],AGC_TASK_PRIO);
	#endif

	OSTaskSuspend(START_TASK_PRIO);//挂起起始任务
	OS_EXIT_CRITICAL();//退出临界区，开中断
}

#define coe 1.4
/* 主任务 */
double d_distance = 0;
void master_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	INT32U time_ps = 0;
	INT32U err_time = 0;
	INT8U recive_flag = 0;//有数据标记
	//double d_distance = 0;
	CH_DATA ch_distance;

	#ifdef AUTO_LEVEL_MOD
/**************************auto level mode*********************************/
	INT8U level_stable=0;
	INT8U no_data_cnt=0;
	double org_data;
	AGC_EN = 0;
	
	OSTaskSuspend(AGC_TASK_PRIO);//挂起AGC任务
	level = 0;//初始化档位
	OS_ENTER_CRITICAL();
	quantify = LEVEL[level];//设置档位对应增益
	setDacValueBin(quantify);
	OS_EXIT_CRITICAL();
	delay_ms(500);//上电等待500ms出数据
	
	while(1)
	{
		if(outmode==1)
		{
			OS_ENTER_CRITICAL();
			printf("q:%d\n",quantify);
			OS_EXIT_CRITICAL();
		}
		/* 获得数据 */
		OS_ENTER_CRITICAL();
		if(ans_cnt!=0)//结果缓冲区里有数据
		{
			#ifdef ARGV //结果求平均
			//将缓冲区里的数据求平均
			//printf("ans_buf :");
			for(master_i=0;master_i<ans_cnt;master_i++)
			{
				d_sum += ans_buf[master_i].time_ps;
				e_sum += ans_buf[master_i].err_time;
				//print_int_data(ans_buf[master_i].i_data);
			}
			//printf("\n");
			//distance = d_sum/ans_cnt;
			time_ps = d_sum/ans_cnt;
			err_time = e_sum/ans_cnt;
			#else //结果求中值
			quick_sort_time_data(ans_buf,ans_cnt);
			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps = ans_buf[ans_cnt/2].time_ps;
			err_time = ans_buf[ans_cnt/2].err_time3;
			#endif
			if(time_ps<TIME_PS_TH&&err_time>2200)//数据挑选，阈值时间差，距离
			{				
				recive_flag = 0;
			}
			else
				recive_flag = 1;
			ans_cnt = 0;
			ans_ite = 0;			
			#ifdef ARGV
			e_sum = 0;
			d_sum = 0;
			#endif
		}
		else//缓冲区没有数据，标记为0
		{
			//printf("de:%.2f\n",(1.5*(double)detect_val)/10000);
			//if(detect_val>100000)
				recive_flag = 0;
			//else
			//{
			//	recive_flag = 1;
				//time_ps = detect_val;
				//detect_val = 0xFFFFFFFF;
				//printf("de:%.2f\n",(1.5*(double)time_ps)/10000);
			//}
		}
		OS_EXIT_CRITICAL();
		if(recive_flag)//缓冲区有数据
		{
			//有数据，档位固定
			if(level_stable==0)
			{
				level_stable = 1;
			}
			no_data_cnt = 0;//一旦有数据，无数据计数清零
			/* time_ps转距离double */
			d_distance = (1.5*(double)time_ps)/10000;
			org_data = d_distance;
			//d_distance = u32_dou(distance);//转化为double
			if(outmode==1)
			{
				OS_ENTER_CRITICAL();
				printf("ORG:%.2f %d\n",org_data,err_time);
				OS_EXIT_CRITICAL();
			}
			
			//通过阈值拟合校准
			#ifdef MULTI_VTH
			switch(measure_mode)
			{
				case wall://测墙模式
					if(org_data<50)
					{	
							d_distance = d_distance-coe;
					}
					else
					{
						d_distance = multi_thread_adj(&org_data,&err_time);
					}
					break;
				case line://测线模式
					d_distance = multi_thread_adj(&org_data,&err_time);
					break;
				default:break;
			}			
			#endif
			if(outmode == 1)//串口助手调试输出格式
			{
				OS_ENTER_CRITICAL();
				#ifdef MULTI_VTH
				//printf("No:%.2f\n",org_data);
				printf("MV:");
				#else
				printf("No:");
				#endif
				printf("%.2f ",d_distance);
				//printf(" e:%d ",err_time);
				//printf("q:%d\n",quantify);
				OS_EXIT_CRITICAL();
			}
			else//按照照相机要求的格式输出数据
			{
				OS_ENTER_CRITICAL();
				ch_distance = dou_ch_data(&d_distance,err_time);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.h_data);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.l_data);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.f_data);
				OS_EXIT_CRITICAL();
			}
		}
		else//没有数据
		{
			if(outmode==0)
			{
				OS_ENTER_CRITICAL();
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				OS_EXIT_CRITICAL();
			}
			//档位切换
			if(level_stable==0)
			{
				if(level<MAX_LEVEL-1)
				{
					++level;
				}
				else
				{
					level_stable = 1;
				}
				OS_ENTER_CRITICAL();
				quantify = LEVEL[level];
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
			}
			else//档位已固定得情况下没有数据累计到一定次数后，重新设置档位
			{
				if(no_data_cnt<MAX_NO_DATA_CNT)
				{
					++no_data_cnt;
				}
				else
				{
					level_stable = 0;
					no_data_cnt = 0;
					level = 0;
					OS_ENTER_CRITICAL();
					quantify = LEVEL[level];
					setDacValueBin(quantify);
					OS_EXIT_CRITICAL();
				}
				
			}
		}
		delay_ms(500);
	}
/**************************************************************************/	
	#else	//AGC mode	
/**************************AGC mode****************************************/
	//INT32U distance = 0;
	INT16U old_q = 0;
	INT8U master_cnt=0;//master task循环计数 
	INT8U res_cnt = 0;//在远近目标检测周期中的结果计数
	INT8U detect_cnt = 0;//检测阈值在远近目标检测周期中的计数
	#ifdef ARGV
	INT32U e_sum = 0;//阈值时间差求和
	INT8U master_i;
	long d_sum = 0;//距离求和
	#endif
	AGC_EN = 1;
	while(1)
	{
		if(outmode==1)
		{
			OS_ENTER_CRITICAL();
			printf("q:%d\n",quantify);
			OS_EXIT_CRITICAL();
		}
			
		
		/* 远近目标切换判断 */
		/* 因为增益过大会导致近距离数据失效，因此必须检测用户是否由远目标时的高增益切换到了近目标，导致一直没有数据 */
		/* 若2秒没有数据输出，抽出0.5秒切换至初始增益检查是否是由远目标切换至近目标 */
		if(master_cnt==4)//2秒测量到达,最多响应时间不超过4秒
		{
			if(res_cnt == 0&&detect_cnt==0)//没有结果
			{
				if(AGC_EN==1)
				{
					AGC_EN = 0;
					OSTaskSuspend(AGC_TASK_PRIO);//挂起AGC任务
					if(outmode==1)
					{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
					}					
				}
				old_q = quantify;//保存之前的值，如果关闭AGC后还没数据，说明是没有探测到目标或目标太远，再恢复之前的增益值
				OS_ENTER_CRITICAL();
				quantify = QUANTIFY_INIT;
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
			}
			master_cnt = 0;
			res_cnt = 0;
			detect_cnt = 0;
		}
		else if(master_cnt==1&&old_q!=0)
		{
			if(ans_cnt==0&&detect_cnt==0)//不是切换到近目标了
			{
				quantify = old_q;//恢复增益值
				setDacValueBin(quantify);
				AGC_EN = 1;
				OSTaskResume(AGC_TASK_PRIO);//恢复AGC任务
				if(outmode==1)
				{
					OS_ENTER_CRITICAL();
					printf("AGC resume\n");
					OS_EXIT_CRITICAL();
				}
			}
			old_q = 0;
		}
		
		/* 获得数据 */
		OS_ENTER_CRITICAL();
		if(ans_cnt!=0)//结果缓冲区里有数据
		{
			#ifdef ARGV //结果求平均
			//将缓冲区里的数据求平均
			//printf("ans_buf :");
			for(master_i=0;master_i<ans_cnt;master_i++)
			{
				d_sum += ans_buf[master_i].time_ps;
				e_sum += ans_buf[master_i].err_time;
				//print_int_data(ans_buf[master_i].i_data);
			}
			//printf("\n");
			//distance = d_sum/ans_cnt;
			time_ps = d_sum/ans_cnt;
			err_time = e_sum/ans_cnt;
			#else //结果求中值
			quick_sort_time_data(ans_buf,ans_cnt);
			//distance = ans_buf[ans_cnt/2].i_data;
			time_ps = ans_buf[ans_cnt/2].time_ps;
			err_time = ans_buf[ans_cnt/2].err_time3;
			#endif
			if(time_ps<150000&&err_time>2200)//当结果少，说明难测，而距离又近，当作噪声去掉（宁错杀，不放过）
				recive_flag = 0;
			else
				recive_flag = 1;
			ans_cnt = 0;
			ans_ite = 0;			
			#ifdef ARGV
			e_sum = 0;
			d_sum = 0;
			#endif
		}
		else//缓冲区没有数据，标记为0
		{
			recive_flag = 0;
		}
		OS_EXIT_CRITICAL();
				
		/* 校准后输出结果 */
		if(recive_flag)
		{
			res_cnt++;//数据计数加1
			/* time_ps转距离double */
			d_distance = (1.5*(double)time_ps)/10000;
			//d_distance = u32_dou(distance);//转化为double
			//通过阈值拟合校准
			#ifdef MULTI_VTH			
			d_distance = multi_thread_adj(&d_distance,&time_ps);
			#endif
			if(outmode == 1)//串口助手调试输出格式
			{
				OS_ENTER_CRITICAL();
				#ifdef MULTI_VTH
				//printf("MV:");
				#else
				//printf("No:");
				#endif
				//printf("%.2f ",d_distance);
				//printf(" e:%d ",err_time);
				//printf("q:%d\n",quantify);
				OS_EXIT_CRITICAL();
			}
			else//按照照相机要求的格式输出数据
			{
				OS_ENTER_CRITICAL();
				ch_distance = dou_ch_data(&d_distance,err_time);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.h_data);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.l_data);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,ch_distance.f_data);
				OS_EXIT_CRITICAL();
			}
			
			/* 判断AGC是否开启 */
			if(d_distance>21.0)
			{
				if(AGC_EN==0)
				{
					AGC_EN = 1;
					quantify = QUANTIFY_INIT;//重启AGC任务前初始化增益
					OSTaskResume(AGC_TASK_PRIO);//恢复AGC任务
					if(outmode==1)
					{
						OS_ENTER_CRITICAL();
						printf("AGC task resume1\n");
						OS_EXIT_CRITICAL();
					}						
				}			
			}
			else
			{
				if(AGC_EN==1)
				{
					AGC_EN = 0;
					OSTaskSuspend(AGC_TASK_PRIO);//挂起AGC任务
					quantify = QUANTIFY_INIT;
					setDacValueBin(quantify);//重置增益
					if(outmode==1)
					{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend\n");
						OS_EXIT_CRITICAL();
					}
						
				}
			}
			
		}
		else//没有数据输出 //0xFF 0xFF 0xFF
		{
			if(detect_val!=0)//探测阈值探测到信号脉冲//此时AGC由于干扰脉冲大于信号脉冲，增益上不去，挂起AGC任务
			{
				detect_cnt++;
				if(AGC_EN==1)
				{
					AGC_EN = 0;
					OSTaskSuspend(AGC_TASK_PRIO);//挂起AGC任务
					if(outmode==1)
					{
						OS_ENTER_CRITICAL();
						printf("AGC task suspend--\n");
						OS_EXIT_CRITICAL();
					}
				}
				OS_ENTER_CRITICAL();
				detect_val = 0;
				if(old_q!=0)
				{
					quantify = old_q+100;
					old_q = quantify;
				}
				else
				{
					quantify += 100;//手动增加100增益
				}				
				setDacValueBin(quantify);//重置增益
				OS_EXIT_CRITICAL();
			}
			else
			{
				//没数据输出，开启AGC拉大增益
				if(AGC_EN==0)
				{
					AGC_EN = 1;
					quantify = QUANTIFY_INIT;//重启AGC任务前初始化增益
					OSTaskResume(AGC_TASK_PRIO);//恢复AGC任务
					if(outmode==1)
					{
						OS_ENTER_CRITICAL();
						printf("AGC task resume2\n");
						OS_EXIT_CRITICAL();	
					}
				}
			}
			if(outmode==0)
			{
				OS_ENTER_CRITICAL();
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==0);
				USART_SendData(USART1,0xFF);
				OS_EXIT_CRITICAL();
			}
			
		}
		
		/* 防止回波过少导致峰值保持不住，集中发送连续脉冲充电 */
		if(AGC_EN)
		{
			int i;
			PEAK_CONTROL = 1;//放电
			i=0;
			PEAK_CONTROL = 0;//峰值保持
			for(i=0;i<50;i++)//充电到峰值,快速发射多个脉冲
			{
				laser_plus();
				delay_ms(1);
			}
		}
		
		delay_ms(500);//每隔0.5秒查询结果并输出
		master_cnt++;
	}
	#endif
/*************************************************************************/
}
#define MEASURE_DATA_ARR_SIZE 30
/* 生产者任务，获取飞行时间，阈值时间差 */
void gen_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	/* TDC-GPX2 测量结果 */
	result measure_data_arr[MEASURE_DATA_ARR_SIZE];
	INT8U res_cnt;
	INT8U i_gen;
	INT32U time_ps;//飞行时间 ps
	INT32U time_ps_detect;//探测阈值测量数据
	int err_time1,err_time2,err_time3;//阈值时间差
	long start_res;
	long start_index;
	//double distance;//未处理距离
	//CH_DATA org_data[DATA_GROUP_SIZE];//数据缓冲数组，当填充满时通过消息队列发送给消费者任务
	TIME_DATA org_data[DATA_GROUP_SIZE];//数据缓冲数组，当填充满时通过消息队列发送给消费者任务
	//CH_DATA ch_data;
	TIME_DATA time_data;
	unsigned int org_data_ite = 0;
	INT8U data_vaild;
	detect_val = 0xFFFFFFFF;
	//test 
	//OSTaskSuspend(GEN_TASK_PRIO);
	
	while(1)
	{
		//tdc_measure(&measure_data);//测量一次
		OS_ENTER_CRITICAL();
		res_cnt = tdc_measure_group(measure_data_arr);
		//printf("test3\n");
		OS_EXIT_CRITICAL();
		IWDG_Feed();//喂狗 防止TDC芯片跑飞
		start_res = measure_data_arr[0].stopresult[0];
		start_index = measure_data_arr[0].reference_index[0];
		for(i_gen=0;i_gen<res_cnt;i_gen++)
		{
			//测量阈值飞行时间计算
			time_ps = measure_data_arr[i_gen].stopresult[1] - start_res+\
								(measure_data_arr[i_gen].reference_index[1]-start_index)*refclk_divisions;
			//探测阈值飞行时间
			time_ps_detect = measure_data_arr[i_gen].stopresult[2] - start_res+\
								(measure_data_arr[i_gen].reference_index[2]-start_index)*refclk_divisions;
			//vth1--vth3阈值时间差计算
			err_time3 = measure_data_arr[i_gen].stopresult[3] - measure_data_arr[i_gen].stopresult[1]+\
								(measure_data_arr[i_gen].reference_index[3]-measure_data_arr[i_gen].reference_index[1])*refclk_divisions;
			//OS_ENTER_CRITICAL();
			//printf("i_gen:%d,d:%.2f,err_time:%d\n",i_gen,(1.5*(double)time_ps)/10000,err_time3);
			//OS_EXIT_CRITICAL();
			
			/*  有效值筛选 */
			//数据有效筛选,排除掉对空噪值，通过增益和距离筛选
			//if((quantify>GAIN_TH)&&(distance<DISTANCE_TH))
			#ifdef ADJ
			if((1.5*(double)time_ps)/10000 >= STANDARD_DISTANCE)
				data_vaild = 1;
			else
				data_vaild = 0;
			
			#else
			switch (measure_mode)
			{
				case wall://测墙模式数据筛选
					if((quantify>GAIN_TH)&&(time_ps<TIME_PS_TH))//增益大，距离近的去掉,干掉干扰脉冲
					{
							data_vaild = 0;
					}
					else if(err_time3>MAX_ERR_TIME||err_time3<MIN_ERR_TIME)//阈值时间差有误的去掉
					{
						//if(time_ps<200000&&err_time3>2000)//判断是测墙模式下近距离测了线  30米，时间差2000
						//{
						//	measure_mode = line;//切换至测线模式
						//}
						data_vaild = 0;		
					}
					else
					{
						data_vaild = 1;
					}
					break;
				case line://测线模式数据筛选
					if((quantify>GAIN_TH)&&(time_ps<TIME_PS_TH)&&err_time3>3000)//增益大，距离近的，阈值时间差大的去掉
					{
							data_vaild = 0;
					}
					else if(err_time3>MAX_ERR_TIME||err_time3<2000)//阈值时间差有误的去掉，测线模式只取反射弱没饱和的回波
					{
						//if(err_time3<1100)//判断是在测线模式下测了墙
						//{
						//	measure_mode = wall;//切换至测墙模式
						//}
						data_vaild = 0;						
					}
					else
					{
						data_vaild = 1;
					}
					break;
			}
			
			#endif
			if(time_ps_detect>0&&time_ps_detect<10000000&&quantify<=GAIN_TH)
			{
				OS_ENTER_CRITICAL();
				detect_val = time_ps_detect;
				OS_EXIT_CRITICAL();
			}
			if(time_ps>MAX_TIME||time_ps<=MIN_TIME||data_vaild==0)//当前没有数据
			{
				continue;
			}
						
			create_time_data(&time_data,time_ps,err_time1,err_time2,err_time3);
			org_data[org_data_ite++] = time_data;//装载到数据缓冲区			
			if(org_data_ite==DATA_GROUP_SIZE)//数据缓冲数组填充满
			{
				org_data_ite = 0;
				OSQPost(q_msg,org_data);//发送消息队列
			}					
		}
		delay_ms(10);
	}
}

/* 消费者处理任务，中值滤波 */
void handle_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	TIME_DATA *res;
	INT8U err;
	TIME_DATA time_data[DATA_GROUP_SIZE];//待处理数据缓冲区
	int handle_i;
	INT8U mid_pos = DATA_GROUP_SIZE/2;
	INT8U ans_pos = 0; 
	
	while(1)
	{
		res = OSQPend(q_msg,0,&err);//响应消息队列
		if(err!=OS_ERR_NONE)
		{
			#ifdef DEBUG
			printf("OSQPend err %d",err);
			#endif
		}
		else
		{			
			for(handle_i=0;handle_i<DATA_GROUP_SIZE;handle_i++)
			{
				time_data[handle_i] = res[handle_i];
			}
			
			quick_sort_time_data(time_data,DATA_GROUP_SIZE);//快速排序
			switch(measure_mode)
			{
				case wall:
					ans_pos = mid_pos;//结果取中值
					break;
				case line:
					ans_pos = 0;//结果取
					break;
				default:break;
			}
			
			/* 将取得的中值加入ans_buf结果缓冲区 */
			OS_ENTER_CRITICAL();//ans_buf  ans_cnt  ans_ite 为临界资源
			if(ans_cnt<ANS_BUF_SIZE)
			{
				ans_cnt++;//更新缓冲区结果数
			}
			if(ans_ite==ANS_BUF_SIZE-1)//如果缓冲区已满，覆盖最开始的数据
			{
				ans_buf[ans_ite] = time_data[ans_pos];
				ans_ite = 0;
			}
			else
			{						
				ans_buf[ans_ite++] = time_data[ans_pos];	
			}
			OS_EXIT_CRITICAL();			
		}
		delay_ms(50);
	}
}

/* AGC任务 */
void agc_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	while(1)
	{
		agc_control(cpu_sr);
		delay_ms(5);
	}
}

#ifdef ADJ
/* 调试校准任务，进行多阈值拟合曲线 */
/* 通过逐渐增大增益，获取不同阈值时间差下的距离误差 */
/* 输出打印阈值时间差和对应的距离误差 */
void adj_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	TIME_DATA *res;
	INT8U err;
	TIME_DATA time_data[DATA_GROUP_SIZE];//待处理数据缓冲区
	int handle_i;
	INT32U mid_pos = DATA_GROUP_SIZE/2;
	double distance;
	double err_distance;
	double standare_distance = STANDARD_DISTANCE;

	quantify = MIN_GAIN;
	OS_ENTER_CRITICAL();
	setDacValueBin(quantify);//初始最低增益	
	printf("standard distance set: %.2f\n",standare_distance);
	OS_EXIT_CRITICAL();
	while(1)
	{
		res = OSQPend(q_msg,0,&err);//响应消息队列
		if(err!=OS_ERR_NONE)
		{
			#ifdef DEBUG
			OS_ENTER_CRITICAL();
			printf("OSQPend err %d",err);
			OS_EXIT_CRITICAL();
			#endif
		}
		else
		{			
			for(handle_i=0;handle_i<DATA_GROUP_SIZE;handle_i++)
			{
				time_data[handle_i] = res[handle_i];
			}
			
			/* 中值滤波 */
			quick_sort_time_data(time_data,DATA_GROUP_SIZE);//快速排序
			/* 将中值转化为距离 */
			distance = (1.5*(double)time_data[mid_pos].time_ps)/10000;
			err_distance = distance - STANDARD_DISTANCE;//距离误差
			OS_ENTER_CRITICAL();
			printf("%.2f,%d,,%d,%d,%d\n",err_distance,time_data[mid_pos].err_time1,\
							time_data[mid_pos].err_time2,time_data[mid_pos].err_time3,quantify);//打印到串口
			OS_EXIT_CRITICAL();
		}
		quantify += INC_STEP;
		if(quantify > MAX_GAIN)//结束了，挂起任务
		{
			OS_ENTER_CRITICAL();
			printf("ADJ end\n");
			OS_EXIT_CRITICAL();
			OSTaskSuspend(GEN_TASK_PRIO);
			OSTaskSuspend(ADJ_TASK_PRIO);
		}
		else
		{
			OS_ENTER_CRITICAL();
			setDacValueBin(quantify);//扩大增益
			OS_EXIT_CRITICAL();
		}			
		delay_ms(50);
	}
}

#endif

/* 设置数据输出格式 */
/* outmode为1时输出字符串格式 */
/* outmode为0时输出字节流格式 */
/* MODE为0时测墙模式 */
/* MODE为1时测线模式 */
void out_mode_set()
{
	/* 输出模式设定 */
	outmode = 0;
		if(MODE==1)
		{
			measure_mode = line;
		}
		else
		{
			measure_mode = wall;
		}
}

/* CH_DATA转double */
double ch_data_dou(CH_DATA* pch_data)
{
	double ans;
	INT16U i_temp = 0;
	ans = (double)pch_data->f_data / 256;
	i_temp = ((INT16U)(pch_data->h_data)<<8) + pch_data->l_data;
	return ans+(double)i_temp;
}

/* double转CH_DATA */
CH_DATA dou_ch_data(double* d_data,INT32U err_time)
{
	CH_DATA ch_data;
	INT32U i_temp;
	i_temp = *d_data;//获取整数位
	ch_data.h_data = (i_temp>>8)&0xFF;
	ch_data.l_data = i_temp&0xFF;
	ch_data.f_data = (INT8U)((*d_data-i_temp)*256);
	ch_data.err_time = err_time;
	return ch_data;
}

/* CH_DATA转u32 */
INT32U ch_data_u32(CH_DATA* pch_data)
{
	INT32U ans = 0;
	ans += pch_data->h_data;
	ans = ans<<8;
	ans += pch_data->l_data;
	ans = ans<<8;
	ans += pch_data->f_data;
	return ans;
}

/* u32转double */
double u32_dou(INT32U data)
{
	double ans;
	ans = (double)(data&0xFF) / 256;//取出小数
	ans = ans+(data>>8);//加上整数位
	return ans;
}

/* double转u32 */
INT32U dou_u32(double* d_data)
{
	INT32U ans;
	CH_DATA temp;
	temp = dou_ch_data(d_data,0);
	ans = ch_data_u32(&temp);
	return ans;
}

/* 将CH_DATA格式的数据打印成浮点数显示 */
/* 显示小数点后两位 */
void print_ch_data(CH_DATA* ch_data)
{
	printf("%d.%02d ",((INT16U)ch_data->h_data<<8)+ch_data->l_data,ch_data->f_data*100/256);
}
/* 将U32格式的数据打印成浮点数显示(3个字节合并) */
/* 显示小数点后两位 */
void print_int_data(INT32U int_data)
{
	double res;
	res = u32_dou(int_data);
	//printf("%d.%.2f ",int_data>>8,((double)(int_data&0xFF))/256);
	printf("%.2f ",res);
}

/* 快速排序算法 */
int partiton_int_data(INT_DATA* arr,int low,int high)
{
	int pivotkey;
	INT_DATA d_temp;
	pivotkey = arr[low].i_data;
	while(low<high)
	{
		while(low<high&&arr[high].i_data>=pivotkey)
			high--;
		d_temp = arr[low];
		arr[low] = arr[high];
		arr[high] = d_temp;//swap(arr,low,high);
		while(low<high&&arr[low].i_data<=pivotkey)
			low++;
		d_temp = arr[low];
		arr[low] = arr[high];
		arr[high] = d_temp;//swap(arr,low,high);
		
	}
	
	return low;
}
void q_sort_int_data(INT_DATA* arr,int low,int high)
{
	int pivot;
	if(low<high)
	{
		pivot = partiton_int_data(arr,low,high);
		q_sort_int_data(arr,low,pivot-1);
		q_sort_int_data(arr,pivot+1,high);
	}
}
void quick_sort_int_data(INT_DATA* arr,INT32U size)
{
	q_sort_int_data(arr,0,size-1);
}


/* 快速排序算法 */
int partiton_time_data(TIME_DATA* arr,int low,int high)
{
	int pivotkey;
	TIME_DATA d_temp;
	pivotkey = arr[low].time_ps;
	while(low<high)
	{
		while(low<high&&arr[high].time_ps>=pivotkey)
			high--;
		d_temp = arr[low];
		arr[low] = arr[high];
		arr[high] = d_temp;//swap(arr,low,high);
		while(low<high&&arr[low].time_ps<=pivotkey)
			low++;
		d_temp = arr[low];
		arr[low] = arr[high];
		arr[high] = d_temp;//swap(arr,low,high);
		
	}
	
	return low;
}
void q_sort_time_data(TIME_DATA* arr,int low,int high)
{
	int pivot;
	if(low<high)
	{
		pivot = partiton_time_data(arr,low,high);
		q_sort_time_data(arr,low,pivot-1);
		q_sort_time_data(arr,pivot+1,high);
	}
}
void quick_sort_time_data(TIME_DATA* arr,INT32U size)
{
	q_sort_time_data(arr,0,size-1);
}


/*********对数函数Ln(x)**********/
double my_log(double a)
{
	int N = 100000;
	long k,nk;
	double x,xx,y;
	if(a<0)
	{
		a = -a;
	}
	x = (a-1)/(a+1);
	xx = x*x;
	nk = 2*N+1;
	y = 1.0/nk;
	for(k=N;k>0;k--)
	{
		nk = nk -2;
		y = 1.0/nk+xx*y;
	}
	return 2.0*x*y;
}

/*******************************/
/*********AGC 控制函数**********/
/*********基于峰值电压**********/
void agc_control(OS_CPU_SR cpu_sr)
{
		static double ADC_Value;//ADC value
		static double err;//增益控制差值
		/* Get ADC_Value */
			ADC_Value = (double)ADC_ConvertedValue/4096*3.3;
				/* AGC Control Code */		
				/* 增量控制 */
			if(ADC_Value<CONTROL_MAX_VOLTAGE)
			{
				err = CONTROL_MAX_VOLTAGE - ADC_Value;
			}
			else if(ADC_Value>=CONTROL_MAX_VOLTAGE)
			{
				err = ADC_Value - CONTROL_MAX_VOLTAGE;
			}
			
			if(ADC_Value<CONTROL_MAX_VOLTAGE-FLOW_RANGE)
			{
				if(quantify>2000)
					quantify = 2000;
				else if(err>1.0)
				{
					quantify+=10;
				}
				else if(err>0.8)
				{
					quantify+=8;
				}
				else if(err>0.6)
				{
					quantify+=4;
				}
				else if(err>0.4)
				{
					quantify+=2;
				}
				else 
				{
					quantify+=1;
				}
				OS_ENTER_CRITICAL();
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
			}
			#ifdef AGC_LOW
			else if(ADC_Value>CONTROL_MAX_VOLTAGE+FLOW_RANGE)
			{				
				if(quantify<=10)
					quantify = 10;
				else if(err>1.0)
					quantify-=10;
				else if(err>0.8)
					quantify-=8;
				else if(err>0.6)
					quantify-=4;
				else if(err>0.4)
					quantify-=2;
				else 
					quantify-=1; 
				OS_ENTER_CRITICAL();
				setDacValueBin(quantify);
				OS_EXIT_CRITICAL();
			}
			#endif

}

/* 构建TIME_DATA */
void create_time_data(TIME_DATA* ptime_data,INT32U time_ps,INT32U err_time1,INT32U err_time2,INT32U err_time3)
{
	ptime_data->time_ps = time_ps;
	ptime_data->err_time1 = err_time1;
	ptime_data->err_time2 = err_time2;
	ptime_data->err_time3 = err_time3;
}

/* 多阈值数据拟合 */
double multi_thread_adj(double *org,unsigned int *time)
{
	double ans;
	if(*time>1100&&*time<2000)
	{
		ans = *org-0.8541*my_log(-10.9372*(*time)+11430)+3.019-coe+2.2;
	}
	else if((*time)>=2000)
	{
		ans = *org-0.0004*(*time)-4.0737-coe+2.2;
	}
	else
	{
		ans = *org-coe;
	}
	return ans;
}

