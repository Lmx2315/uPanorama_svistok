#include "main.h"

typedef struct  reg_ADF4351  // объявляю структуру 
{
	u32 R[6];
	
	u16 FRAC							:12;
	u16 INT								:16;
	u8  PHASE_ADJUST					:1;
	u8  PRESCALER						:1;
	u16 PHASE							:12;
	u16 MOD								:12;
	u8  LNALSM							:2;	//LOW NOISE AND LOW SPUR MODES 
	u8  MUXOUT							:3;
	u8  REF_DOUBLER						:1;
	u8  RDIV2							:1;
	u16 R_COUNTER						:10;
	u8  DOUB_BUFFER						:1; //DOUBLE BUFFER
	u8  CP_CURRENT						:4; //CHARGE PUMP CURRENT SETTING
	u8 	LDF								:1;
	u8  LDP								:1;
	u8  PD_POLARITY						:1;
	u8  POWER_DOWN						:1;
	u8  CP_THREESTATE					:1;
	u8  COUNTER_RESET					:1;
	u8  BAND_SELECT_CLOCK_MODE			:1;
	u8 	ABP								:1;
	u8  CHARGE_CANCEL					:1;
	u8  CSR								:1;
	u8  CLK_DIV_MODE					:2;
	u16 CLOCK_DIV_VALUE					:12;
	
	u8  FEEDBACK_SELECT					:1;
	u8  RF_DIVIDER_SELECT				:3;
	u8  BAND_SELECT_CLOCK_DIVIDER_VALUE :8;
	u8  VCO_POWER_DOWN					:1;
	u8  MTLD							:1;
	u8  AUX_OUTPUT_SELECT				:1;
	u8  AUX_OUTPUT_ENABLE				:1;
	u8  AUX_OUTPUT_POWER				:2;
	u8  RF_OUTPUT_ENABLE				:1;
	u8  OUTPUT_POWER					:2;
	
	u8  LD_PIN_MODE						:2;
		
} reg_ADF4351;


reg_ADF4351 pll1;


void INIT_PLL (reg_ADF4351 *pll)
{
	pll->FRAC							=0;// 0     -    4095
	pll->INT							=3000;// (65536 > INT >22 ) если 4/5 и больше 75 если 8/9!!!
	pll->PHASE_ADJUST					=0;//0 - OFF
	pll->PRESCALER						=0;//0 - 4/5  |  1 - 8/9
	pll->PHASE							=1;//(RECOMMENDED)
	pll->MOD							=2;	
	pll->LNALSM							=0;//	0 - low noise mode    3 - low spur mode           //LOW NOISE AND LOW SPUR MODES 
	pll->MUXOUT							=6;//3 - R counter output, было - 2, 6 - Digital lock detect
	pll->REF_DOUBLER					=0;
	pll->RDIV2							=1;
	pll->R_COUNTER						=50;
	pll->DOUB_BUFFER					=0;
	pll->CP_CURRENT						=15;//15 - 5 mA,7-2.5mA
	pll->LDF							=1;//1 - INT N
	pll->LDP							=0;//0 - 10 ns
	pll->PD_POLARITY					=1;//0 - negative 1 -positive
	pll->POWER_DOWN						=0;//0 - disable PWRDN
	pll->CP_THREESTATE					=0;// 1 - ENABLE 3-state, 0 - disable
	pll->COUNTER_RESET					=0;
	pll->BAND_SELECT_CLOCK_MODE			=0;// 0 - low
	pll->ABP							=1;// 1 - 3 ns, INT-N
	pll->CHARGE_CANCEL					=0;// 0 - disable
	pll->CSR							=0;// 0 - disable
	pll->CLK_DIV_MODE					=0;// 0 - disable, 1-activate fast lock
	pll->CLOCK_DIV_VALUE				=150;
	pll->FEEDBACK_SELECT				=1;// 1 - fundamental
	pll->RF_DIVIDER_SELECT				=0;// RF divider :1 
	pll->BAND_SELECT_CLOCK_DIVIDER_VALUE=8;//Надо менять взависимости от частоты сравнения!!!! (fref/0.125)
	pll->VCO_POWER_DOWN					=0;// 0 - VCO power up
	pll->MTLD							=0;// 0 - mute disable
	pll->AUX_OUTPUT_SELECT				=0;//
	pll->AUX_OUTPUT_ENABLE				=0;//0 - disable (port B)
	pll->AUX_OUTPUT_POWER				=0;//0 - -4DBm
	pll->RF_OUTPUT_ENABLE				=1;//1 - enable
	pll->OUTPUT_POWER					=3;//3 - +5DBm
	pll->LD_PIN_MODE					=1;//1 - LOCK DETECT DIGITAL	
	

}

void init_array_pll (reg_ADF4351 *pll)
{
		//-----------------------------------
	// DEFRAG
	
	pll->R[0]=(pll->INT <<15)+
			  (pll->FRAC<< 3)+
			  (0);
			  
	pll->R[1]=(pll->PHASE_ADJUST 	<<28)+
			  (pll->PRESCALER		<<27)+
			  (pll->PHASE			<<15)+
			  (pll->MOD				<< 3)+
			  (1);
			  
	pll->R[2]=(pll->LNALSM		 	<<29)+
			  (pll->MUXOUT			<<26)+
			  (pll->REF_DOUBLER		<<25)+
			  (pll->RDIV2			<<24)+
			  (pll->R_COUNTER		<<14)+
			  (pll->DOUB_BUFFER		<<13)+
			  (pll->CP_CURRENT		<< 9)+
			  (pll->LDF				<< 8)+
			  (pll->LDP				<< 7)+
			  (pll->PD_POLARITY		<< 6)+
			  (pll->POWER_DOWN		<< 5)+
			  (pll->CP_THREESTATE	<< 4)+
			  (pll->COUNTER_RESET	<< 3)+
			  (2);
			  
	pll->R[3]=(pll->BAND_SELECT_CLOCK_MODE 	<<23)+
			  (pll->ABP						<<22)+
			  (pll->CHARGE_CANCEL			<<21)+
			  (pll->CSR						<<18)+
			  (pll->CLK_DIV_MODE			<<15)+
			  (pll->CLOCK_DIV_VALUE			<< 3)+
			  (3);
			  
	pll->R[4]=(pll->FEEDBACK_SELECT		 			<<23)+
			  (pll->RF_DIVIDER_SELECT				<<20)+
			  (pll->BAND_SELECT_CLOCK_DIVIDER_VALUE	<<12)+
			  (pll->VCO_POWER_DOWN					<<11)+
			  (pll->MTLD							<<10)+
			  (pll->AUX_OUTPUT_SELECT				<< 9)+
			  (pll->AUX_OUTPUT_ENABLE				<< 8)+
			  (pll->AUX_OUTPUT_POWER				<< 6)+
			  (pll->RF_OUTPUT_ENABLE				<< 5)+
			  (pll->OUTPUT_POWER					<< 3)+
			  (4);
			  
	
	pll->R[5]=(pll->LD_PIN_MODE			 	<<22)+
			  (3							<<19)+	
			  (5);		  
	
	//----------------
	/*
	pll->R[0]=0x5DC0000;
	pll->R[1]=0x8011;
	pll->R[2]=0x190C9F42;
	pll->R[3]=0x4004B3;
	pll->R[4]=0x80803C;
	pll->R[5]=0x580005;
*/
	
}

void ADF4351_prog (u32 freq)
{
	int i  =0;
	u32 PFD=1;//MHz
	u32 INT=0;
	u32 R  =0;
	
	INT=freq/PFD;	
	
	INIT_PLL 		(&pll1); //инициализируем структуру
	
	pll1.INT=INT;
	
	init_array_pll	(&pll1); //записываем транспортный массив
	
	ADF_LE(0);
	SPI3_CS_MK(1);//включение микрухи ADF
	for (i=5;i>-1;i--) 
	{
		un_out("R[",i);
		 x_out("]:",pll1.R[i]);
		spi3send32(pll1.R[i]);//записываем регистры ADF
		ADF_LE(1);//сигнал записи
		Delay(1);
		ADF_LE(0);
	}


}





















