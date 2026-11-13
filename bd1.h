/********************************************************************************************************
**                            (c) Copyright  苏州市江海通讯发展实业有限公司
**                                      All Rights Reserved                                        
**
**---------------------------- 文件信息 -----------------------------------------------------------------
** 文   件   名:  bd1.h
** 创   建   人:  林依刚
** 版        本:  V1.0
** 最后修改日期:  2008-5-11
** 描        述:  本文件为22号机示位标软件北斗卫星通信头文件。在本文件中，定义了《北斗一号用户机数据接口要求》
                  相关数据结构、宏定义。
                  BD1_PROTOCOL_VERSION    定义了协议的版本号，目前支持V2.1\3.0\4.0;更改版本只需修改该宏并
                  重新编译程序即可。 
********************************************************************************************************/
#ifndef BD1_H
#define BD1_H

 
/*************************************** "北斗一号"数据接口要求相关宏、数据结构定义 ********************/
#define BD1_PROTOCOL_VERSION              4.0     // "北斗一号"用户机通信协议版本,本软件支持2.1、3.0、4.0协议

#define BD1_FR_HEADER                    0x24     // 北斗数据帧报头 "$"
          
#if BD1_PROTOCOL_VERSION >= 3.0
#define BD1_TXSQ_FIXED_LEN                 18     // 卫星协议V3.0、4.0 通信申请中固定位置数据的长度
#else 
#define BD1_TXSQ_FIXED_LEN                 17     // 卫星协议V2.1 通信申请中固定位置数据的长度
#endif

#define BD1_XTZJ_TOTAL_LEN   sizeof(BD1S_XTZJ)    // 系统自检信息总长度为:0x0D
#define BD1_GLJC_TOTAL_LEN   sizeof(BD1S_GLJC)    // 功率检测信息V2.1长度为:0x0D; V3.0、4.0长度为:0x0C 

#define BD1_MAX_MSG_LEN                   242     // 北斗一号电文内容最大长度（字节数）1920bits + 16bits
#define BD1_RX_BUF_SIZE                   512     // 北斗卫星接收缓冲区的大小（字节数）

#define BD1_FR_NO_ERR                    0x00     // 卫星数据帧校验正确
#define BD1_FR_CHK_ERR                   0x01     // 卫星数据帧校验错误
#define BD1_FR_UNDEF                     0x02     // 卫星数据帧校验正确，但属于未定义帧，不进行处理
#define BD1_FR_TX_FAIL                   0x03     // 卫星数据帧入站失败

       
/*************************************** "北斗一号"用户机与外设之间信息传输格式 ************************/
typedef struct bd1s_dwxx_time{      
    INT8U       hour;                   // 定位时刻的小时位数据，起始值为0，单位1小时
    INT8U       min;                    // 定位时刻的分位数据，起始值为0，单位1分钟
    INT8U       sec;                    // 定位时刻的秒位数据，起始值为0，单位1秒
    INT8U       centi_sec;              // 定位时刻的秒小数位数据，起始值为0，单位0.01秒
} BD1S_DWXX_TIME, *BD1PS_DWXX_TIME;

typedef struct bd1s_dwxx_pos{  
    INT8U       deg;                    // 大地经度(longitude)纬度(Latitude)数据，单位1度
    INT8U       ct;                     // 大地经度(longitude)纬度(Latitude)数据，单位1角分
    INT8U       sec;                    // 大地经度(longitude)纬度(Latitude)数据，单位1角秒
    INT8U       deci_sec;               // 大地经度(longitude)纬度(Latitude)数据，单位0.1角秒
} BD1S_DWXX_POS, *BD1PS_DWXX_POS;

typedef struct bd1s_dwxx_hgt{
    INT8U       hgt[2];                 // 用户位置的大地高程数据
    INT8U       hgt_exceptional[2];     // 用户位置的大地高程数据异常值
} BD1S_DWXX_HGT, *BD1PS_DWXX_HGT;

typedef struct bd1s_txhz_rtn{
    INT8U       post_hour;              // 发信时间（小时）
    INT8U       post_min;               // 发信时间（分钟）
    INT8U       rtn_hour;               // 回执时间（小时）
    INT8U       rtn_min;                // 回执时间（分钟）
} BD1S_TXHZ_RTN, *BD1PS_TXHZ_RTN;

//--------------------------------- "北斗一号"外设至用户机信息传输格式 ----------------------------------
#if BD1_PROTOCOL_VERSION >= 4.0
typedef struct bd1s_icjc{           //--$ICJC(IC检测)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址，固定填"0"
	              
    INT8U       fr_nbr;                 // 帧号,"0"表示本机用户地址
    INT8U       checksum;               // 校验和
} BD1S_ICJC, *BD1PS_ICJC;
 
typedef struct bd1s_xtzj{           // $XTZJ(系统自检)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	              
    INT8U       check_freq[2];          // 自检频度,单位1秒,填"0"表示单次输出
    INT8U       checksum;               // 校验和
} BD1S_XTZJ, *BD1PS_XTZJ;

#else
typedef struct bd1s_gljc{           //--$GLJC(功率检测)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址

#if BD1_PROTOCOL_VERSION == 2.1          
    INT8U       cur_check;              // 当前状况检测："0x00"表示输出当前功率状况"0x01"表示停止输出
#endif	              
    INT8U       out_freq;               // 输出频度,单位1分钟，填"0"表示单次输出
    INT8U       checksum;               // 校验和
} BD1S_GLJC, *BD1PS_GLJC;
#endif

typedef struct bd1s_dwsq{           //--$DWSQ(定位申请)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
    	              
    INT8U       type;                   // 信息类别	
#if BD1_PROTOCOL_VERSION >= 4.0              
    INT8U       hgt_antenna[4];         // 高程数据和天线高Antenna
    INT8U       air_pressure[4];        // 气压数据
    INT8U       in_freq[2];             // 入站频度，设定连续定位频度，单位1秒，填"0"表示单次定位
#else
    INT8U       hgt_antenna[3];         // 高程数据和天线高Antenna
    INT8U       known_hgt[2];           // 已知高程
    INT8U       in_freq[2];             // 入站频度，设定连续定位频度，单位1秒，填"0"表示单次定位
#endif        	              
    INT8U       checksum;               // 校验和
} BD1S_DWSQ, *BD1PS_DWSQ;
                     
typedef struct bd1s_txsq{           //--$TXSQ(通信申请),固定长度随版本而不同（17或18）电文内容可变
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	              
    INT8U       type;                   // 信息类别
    INT8U       rece_id[3];             // 收信方地址
    INT8U       msg_len[2];             // 电文长度
#if BD1_PROTOCOL_VERSION >= 4.0    
    INT8U       is_resp;                // 是否应答
#endif
                      
    INT8U       msg[BD1_MAX_MSG_LEN];   // 电文内容
    INT8U       checksum;               // 校验和
} BD1S_TXSQ, *BD1PS_TXSQ;

typedef struct bd1s_sjsc{           //--$SJSC(时间输出)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
    	              
    INT8U       out_freq[2];            // 输出频度,单位1秒，填"0"表示单次输出
    INT8U       checksum;               // 校验和
} BD1S_SJSC, *BD1PS_SJSC;	  

typedef struct bd1s_jjzh{           //--$JJZH(紧急自毁)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
    	              
    INT8U       key[4];                 // 密码
    INT8U       checksum;               // 校验和
} BD1S_JJZH, *BD1PS_JJZH;	             


//--------------------------------- "北斗一号"用户机至外设信息传输格式 -----------------------------------
#if BD1_PROTOCOL_VERSION >= 4.0 
typedef struct bd1s_icxx{           //--$ICXX(IC信息)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	                           
    INT8U       fr_nbr;                 // 帧号
    INT8U       univ_id[3];             // 通播ID(universal ID)
    INT8U       user_char;              // 用户特征
    INT8U       serve_freq[2];          // 服务频度  单位：秒
    INT8U       comm_scale;             // 通信等级
    INT8U       encrypt;                // 加密标志
    INT8U       sub_users[2];           // 下属用户总数
    INT8U       checksum;               // 校验和	
} BD1S_ICXX, *BD1PS_ICXX;  

typedef struct bd1s_zjxx{           //--$ZJXX(自检信息) 
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	                           
    INT8U       ic_st;                  // IC状态
    INT8U       hw_st;                  // 硬件状态
    INT8U       bat_st;                 // 电池电量
    INT8U       input_st;               // 入站状态                   
    INT8U       wave_pwr[10];           // 信息功率               
    INT8U       checksum;               // 校验和	
} BD1S_ZJXX, *BD1PS_ZJXX;
 
#else   
typedef struct bd1s_glzk{           //--$GLZK(功率状况)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	               
    INT8U       wave_pwr[6];            // 功率1~6之功率状况信息              
    INT8U       checksum;               // 校验和	                    
} BD1S_GLZK, *BD1PS_GLZK;
#endif
	
typedef struct bd1s_dwxx{           //--$DWXX(定位信息)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	              
    INT8U       type;                   // 信息类别
    INT8U       query_id[3];            // 查询地址
    BD1S_DWXX_TIME  time;               // 定位时间
    BD1S_DWXX_POS   lon;                // 经度
    BD1S_DWXX_POS   lat;                // 纬度
    BD1S_DWXX_HGT   hgt;                // 高程数据
    INT8U       checksum;               // 校验和
} BD1S_DWXX, *BD1PS_DWXX;
	                    
typedef struct bd1s_txxx{           //--$TXXX(通信信息),固定长度为：20；电文内容可变
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	              
    INT8U       type;                   // 信息类别
    INT8U       post_id[3];             // 发信方地址
    INT8U       post_time[2];           // 发信时间
    INT8U       msg_len[2];             // 电文长度
    INT8U       msg[BD1_MAX_MSG_LEN];   // 电文内容
    INT8U       crc_flg;                // CRC标志
    INT8U       checksum;               // 校验和
} BD1S_TXXX, *BD1PS_TXXX;

typedef struct bd1s_sjxx{           //--$SJXX(时间信息)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	              
    INT8U       bd1_time[7];            // 北斗时间（年（2字节）、月、日、时、分、秒）
    INT8U       checksum;               // 校验和
} BD1S_SJXX, *BD1PS_SJXX;

typedef struct bd1s_zhqr{           //--$ZHQR(自毁确认)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
    	              
    INT8U       zh_flag[4];             // 自毁标志
    INT8U       checksum;               // 校验和
} BD1S_ZHQR, *BD1PS_ZHQR;	 

typedef struct bd1s_txhz{           //--$TXHZ(通信回执)
    INT8U       name[5];                // 指令名称  
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	                           
    INT8U       rece_id[3];             // 收信方地址
    INT8U       rtn_nbr;                // 回执个数
    BD1S_TXHZ_RTN   rtn_data[5]; 
    INT8U       checksum;               // 校验和
} BD1S_TXHZ, *BD1PS_TXHZ;


typedef struct bd1S_fkxx_type{      //--$FKXX(反馈信息)
    INT8U       name[5];                // 指令名称
    INT8U       len[2];                 // 长度
    INT8U       user_id[3];             // 用户地址
	                           
    INT8U       fk_flag;                // 反馈标志
    INT8U       fk_info[4];             // 附加信息
    INT8U       checksum;               // 校验和
} BD1S_FKXX, BD1PS_FKXX;


typedef struct bd_ZQXX_type{
	INT8U 		name[5];
	INT8U		len[2];
	INT8U       checksum;
}BD1S_ZQXX;

typedef struct bd_PDTZ_type{
	INT8U 		name[5];
	INT8U		len[2];
	INT8U		dep;
	INT8U		deg;
	INT8U       checksum;
}BD1S_PDTZ;

typedef struct bd_KJBG_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		checksum;
}BD1S_KJBG;
typedef struct bd_GJBG_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		checksum;
}BD1S_GJBG;

typedef struct bd_JMFK_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		JMST;
	INT8U		checksum;
}BD1S_JMFK;
typedef struct bd_SQCG_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		JMST;
	INT8U		checksum;
}BD1S_SQCG;

typedef struct bd_SXCX_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		JMST;
	INT8U		checksum;
}BD1S_SXCX;

typedef struct bd_SXXX_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U		date[12];       //年月日，连续三个
	INT8U		checksum;
}BD1S_SXXX;

typedef struct bd_QPXH_type{
	INT8U      name[5];
	INT8U	   len[2];
	INT8U	   flag;
	INT8U      checksum;
}BD1S_QPXH;
typedef struct bd_FXTX_type{
	INT8U		name[5];
	INT8U		len[2];
	INT8U       alarm;
	INT8U		freq;
	INT8U       checksum;
}BD1S_FXTX;



#endif
