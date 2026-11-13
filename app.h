/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  app.h
** 创   建   人:  林依刚
** 版        本:  V1.0
** 最后修改日期:  2008-5-11
** 描        述:  31号机控制软件应用程序头文件,包含程序相关数据结构的定义，外部变量与外部函数的声明
** 操作系统版本： μC/OS-II V2.76
********************************************************************************************************/
#ifndef APP_H
#define APP_H  

#include "bsp.h"
#include "bd1.h"
#include "app_cfg.h"
#include "cyc_buf.h"



     
/*************************************** 软件运行相关数据结构声明 **************************************/
//--VHF module's frequence parameter struct
typedef struct vhf_ch_para_type {
    INT8U       m_ChNbr;                // 参数对应的频道号
    INT8U       m_TxA;                  // Preset divide ratio of binary 7-bit swallow counter (0 <= A <= 127)
    INT8U       m_TxN;                  // Preset divide ratio of binary 11-bit programmable counter (5 to 2,047)
    INT8U       m_RxA;
    INT8U       m_RxN;
} VHF_CH_PARA_TYPE; 

//--DSC alarm message struct
typedef struct dsc_alm_msg_type {       
    INT16U      format;                 // 格式符(format),112表示遇险呼叫
                                        // 呼叫对象的识别，对于遇险呼叫没有收信方地址项(Address)
                                        // 呼叫的类别，对于遇险呼叫没有信息类别项(Category)
    INT16U      ship_id[5];             // 自识别，呼叫台的自我表示船名代码
    INT16U      msg_distress;           // 电文1，表示遇险性质=0x70
    INT16U      msg_pos[5];             // 电文2，表示船舶遇险的地理位置，10位十进制
    INT16U      msg_time[2];            // 电文3，遇险时间，表示给出遇险坐标的时间，4位十进制
    INT16U      msg_term;               // 电文4，在VHF波段仅适用无线电话，功能代码"126"=0x7E
    INT16U      eos;                    // End of Sequence,使用功能代码"127"=0x7F
    INT16U      ecc;
} DSC_ALM_MSG_TYPE;
	             
//--北斗报警位置信息数据格式 
typedef struct bd_alm_pos_type{            
    BD1S_DWXX_TIME  time;               // 定位时间
    INT8U           quad;               // 所在象限
    BD1S_DWXX_POS   lon;                // 经度
    BD1S_DWXX_POS   lat;                // 纬度
} ALM_MSG_POS_TYPE;

//--北斗报警信息格式
typedef struct bd_alm_msg_type{
	INT8U       msg_header[2];          // 自定义信息报头，在此只有遇险报一种，代码(0xA3 0x3E) 汉字(0xA3 0xC1)
    INT8U       msg_instr[4];           // 自定义代码指令类型
    INT8U       msg_ctr;                // 发送信息计数器
    INT8U       msg_flg;	            // 信息标志
               
    INT8U       distress_ship_id[5];    // 遇险船名代码
    INT8U       distress_kind;          // 遇险性质
    INT8U       distress_time[8];       // 遇险时间，年（2字节）、月、日、时、分、秒、0.01秒 
    ALM_MSG_POS_TYPE    distress_pos;   // 遇险位置信息
} ALM_MSG_TYPE;

//--与舱内终端通信数据格式
typedef struct term_comm_type{          
    INT8U       header;                 // 帧头
    INT8U       len;                    // 帧长
    INT8U       type;                   // 类型
    INT8U       msg[251];               // 消息内容
    INT8U       checksum;               // 校验和
} TERM_COMM_TYPE;

//--舱内终端初始化信息格式
typedef struct sys_init_type{	        
    INT8U       ship_id[5];             // 船名代码
    INT8U       distress_kind;          // 遇险性质
    INT8U       user_id[3];	            // 用户ID号
    INT8U       bank_id[3];             // 岸基ID号
    INT8U       interval[2];            // 用户级别 Time scale of star module
    INT8U       term_time[8];           // 终端当前系统时间：年（2字节）、月、日、时、分、秒、0.01秒
} SYS_INIT_TYPE;

//--软件运行标志位数据结构
typedef struct sys_flag_type {          
    unsigned    m_WaveValid   :4;       // 卫星有效波束个数（0~6）
        
    unsigned    m_LampMode    :4;       // 01:10S两次; 10: 亮1S，停1S;
                                        // 11:常亮;    100:20S两次;    其它:关闭闪光灯
    unsigned    m_ChCur       :8;       // VHF模块当前频道号
	
	unsigned    m_DscAlmCtr   :8;       // DSC报警次数计数器(1~255)
	
	unsigned    m_BdAlmCtr    :8;       // 卫星报警次数计数器(1~255)
		          	                                
    unsigned    m_SigDetectEn :1;       //0:除能系统服务任务中卫星信号周期性检测
                                        //1:使能系统服务任务中卫星信号周期性检测
	  	                                  	
    unsigned    m_EmrgPos     :1;       //0:未进行过紧急定位申请
                                        //1:已进行过紧急定位申请	
 
    unsigned    m_ExistPos    :1;       //0:未收到卫星定位信息
                                        //1:已收到卫星定位信息
    	   
    unsigned    m_SysTime     :1;       //0:未收到终端系统时间信息 
                                        //1:已收到终端系统时间信息
    	                                              
    unsigned    m_ManPos      :1;       //0:未收到终端人工位置信息
                                        //1:已收到终端人工位置信息
        
    unsigned    m_9GAck       :1;       //0:9G模块未应答 Response state
                                        //1:9G模块已应答

    unsigned    m_9GTest      :1;       //0:9G模块正常模式
                                        //1:9G模块自检测试中
                                        
    unsigned    m_NatAlm      :1;       //0:本地报警未启动
                                        //1:本地报警已启动
   
    unsigned    m_PassAlm     :1;       //0:被动报警未启动
                                        //1:被动报警已启动
                                                                                                                       
    unsigned    m_WlMode      :1;       //0:浮标处于默认的有缆工作模式
                                        //1:浮标处于无缆工作模式 wireless mode
                                        
    unsigned    m_PreWlMode   :1;       //0:浮标处于默认的有缆工作模式
                                        //1:浮标处于有缆工作（无缆预备）模式，舱内供电消失后将进入无缆工作模式
                                                                              
    unsigned    m_Sweep       :1;       //0:非扫频状态
                                        //1:扫频状态
                                                                             
    unsigned    m_WorkMode    :8;       //工作模式
                                        //35:运行35#机程序
                                        //37:运行37#机程序                                    
                                                                             	                                            	                                    
} SYS_FLAG_TYPE;    


/********************************** EXTERN GLOBAL MACRO DECLARATION ************************************/		
#define MAX(x,y)     ((x) >= (y) ? (x) : (y))

		    
/********************************** EXTERN GLOBAL VARIABLES DECLARATION ********************************/ 
extern volatile SYS_FLAG_TYPE  g_Flg;   // Declare the software running flags.
                    

/********************************** EXTERN FUNCTION PROTOTYPES *****************************************/
void  CommInit(void);
INT8U DscAlarmProc(void);

void  StarSigDetect(void);
void  StarAlarmProc(void);
INT8U StarCommRece(void);
INT8U StarCommSend(void);
void  ToneSweep(void);
void  MikePttChk(void);


INT8U TermCommRece(void);
INT8U TermCommSend(INT8U type,INT8U *pmsg,INT8U msg_len);

void  NGRespDis(void);
void  NGRespEn(void);
void  VhfPwrDis(void);
void  VhfPwrEn(void);

#endif
