#include "QR_code.h"
#include "AIHMI.h"
#include "cmsis_os.h"
#include "radar_startup.h"
uint16_t openmv_shun_xu1;
uint16_t openmv_shun_xu2;
uint16_t openmv_shun_xu3;
uint16_t openmv_shun_xu4;


uint8_t A1, B1, C1;

uint8_t A2, B2, C2;

uint8_t A3, B3, C3;

uint8_t A4, B4, C4;

uint16_t A=1,B=2,C=3;         

volatile uint8_t E = 1;
volatile uint16_t ErWeiMaData[4];
int erweima[16]={0};
uint8_t ShunXu_Decode(uint16_t shunxu,
                      uint8_t max_num,
                      uint8_t *A,
                      uint8_t *B,
                      uint8_t *C)
{
    *A = shunxu / 100;
    *B = (shunxu / 10) % 10;
    *C = shunxu % 10;

    /* 数字必须在 1 ~ max_num，且三个数字不能重复 */
    if((*A < 1) || (*A > max_num) ||
       (*B < 1) || (*B > max_num) ||
       (*C < 1) || (*C > max_num) ||
       (*A == *B) || (*A == *C) || (*B == *C))
    {
        return 0;   // 数据错误
    }

    return 1;       // 数据正确
}
void USART2_IRQHandler_uaer()
{
	  static uint8_t dataFlag=0;
	  static uint8_t y=0;
		if(dataFlag==0)
		{
			dataFlag=1;
		}
		if(dataFlag==1)
		{
			erweima[y]=RXbuf2;
			if(y<15)y++;
			else
			{
				dataFlag=0;
				y=0;
				if(erweima[3]==0x2b)
				{
					ErWeiMaData[0]=100*(erweima[0]-0x30)+10*(erweima[1]-0x30)+(erweima[2]-0x30);
					ErWeiMaData[1]=100*(erweima[4]-0x30)+10*(erweima[5]-0x30)+(erweima[6]-0x30);
					ErWeiMaData[2]=100*(erweima[8]-0x30)+10*(erweima[9]-0x30)+(erweima[10]-0x30);
					ErWeiMaData[3]=100*(erweima[12]-0x30)+10*(erweima[13]-0x30)+(erweima[14]-0x30);
					E=0;
				}
			}
		}
}

 void QRcode_print_1(void)
{
    while(E)
    {
        osDelay(5);
    }

    /* 保存四组顺序码 */
    openmv_shun_xu1 = ErWeiMaData[0];  // 范围 1~6
    openmv_shun_xu2 = ErWeiMaData[1];  // 范围 1~3
    openmv_shun_xu3 = ErWeiMaData[2];  // 范围 1~6
    openmv_shun_xu4 = ErWeiMaData[3];  // 范围 1~3

    /* 第一、三组：1~6；第二、四组：1~3 */
    if(ShunXu_Decode(openmv_shun_xu1, 6, &A1, &B1, &C1) == 0)
        return;

    if(ShunXu_Decode(openmv_shun_xu2, 3, &A2, &B2, &C2) == 0)
        return;

    if(ShunXu_Decode(openmv_shun_xu3, 6, &A3, &B3, &C3) == 0)
        return;

    if(ShunXu_Decode(openmv_shun_xu4, 3, &A4, &B4, &C4) == 0)
        return;

    AIHMI_printf("page0.t0.txt=\"%d\"\xff\xff\xff", openmv_shun_xu1);
    osDelay(5);

    AIHMI_printf("page0.t2.txt=\"%d\"\xff\xff\xff", openmv_shun_xu2);
    osDelay(5);

    AIHMI_printf("page0.t3.txt=\"%d\"\xff\xff\xff", openmv_shun_xu3);
    osDelay(5);

    AIHMI_printf("page0.t4.txt=\"%d\"\xff\xff\xff", openmv_shun_xu4);
    osDelay(5);
	
    AIHMI_printf("page0.t0.txt=\"%d\"\xff\xff\xff", openmv_shun_xu1);
    osDelay(5);

    AIHMI_printf("page0.t2.txt=\"%d\"\xff\xff\xff", openmv_shun_xu2);
    osDelay(5);

    AIHMI_printf("page0.t3.txt=\"%d\"\xff\xff\xff", openmv_shun_xu3);
    osDelay(5);

    AIHMI_printf("page0.t4.txt=\"%d\"\xff\xff\xff", openmv_shun_xu4);
    osDelay(5);
		
		AIHMI_printf("page0.t9.txt=\"%d\"\xff\xff\xff", g_radar_startup.grid_count);
    osDelay(5);

    AIHMI_printf("page0.t1.txt=\"%d\"\xff\xff\xff", g_radar_startup.grid_ids[0]);
    osDelay(5);

    AIHMI_printf("page0.t6.txt=\"%d\"\xff\xff\xff", g_radar_startup.grid_ids[1]);
    osDelay(5);
}
