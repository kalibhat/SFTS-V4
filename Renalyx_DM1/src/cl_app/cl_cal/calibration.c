/*
 * calibration.c
 *
 * Created: 4/19/2016 11:11:12 AM
 *  Author: renalyx
 */ 
#include "asf.h"
#include "cl_types.h"
#include "calibration.h"
#include "Platform/Service/sv_interface.h"

volatile float pressure_final_apt;
volatile float pressure_final_vpt=0,pressure_final_ps1=0,pressure_final_ps2=0,pressure_final_ps3=0;
volatile float temprature_final_value_1=0,temprature_final_value_2=0,temprature_final_value_3=0;
volatile float cond_final_cs3=0;
volatile Cl_Uint32Type hep_speed;
Cl_Uint32Type hep_fblower , hep_fbupper;
volatile Cl_Uint32Type uf_final =0;
extern Cl_Uint8Type sv_cntrl_setpumpspeed(sv_pumptype sv_pump_id,Cl_Uint32Type speed);

volatile temptre Temp1[117]= {0,0};
volatile temptre Temp2[117]= {0,0};
volatile temptre Temp3[117]= {0,0};

volatile lut apt1[30]={0,0};
volatile lut vpt1[30]={0,0};
volatile lut ps_1[50]={0,0};
volatile lut ps_2[50]={0,0};
volatile lut ps_3[50]={0,0};
//volatile hep hp[50]={0,0};
volatile conduc cond[50]={0,0};
uint32_t fed_value=0;



/*                 commented on 11.05.2017 for getting calibration table from nvram

typedef struct
{
	int16_t pressure;
	Cl_Uint16Type millivolt;
}lut;

temptre Temp1[150]=
{
	{384,2300},{395,2350},{406,2400},{417,2450},{428,2500},{439,2550},{450,2600},{461,2650},{472,2700},{483,2750},{494,2800},{505,2850},{516,2900},{527,2950},{538,3000},{549,3050},{560,3100},{571,3150},{582,3200},{593,3250},
	{604,3300},{615,3350},{626,3400},{638,3450},{649,3500},{660,3550},{669,3600},{679,3650},{689,3700},{700,3750},{710,3800},{721,3850},{732,3900},{743,3950},{754,4000},{765,4050},{776,4100},{787,4150},{798,4200},{809,4250},
	{820,4300},{831,4350},{842,4400},{853,4450},{864,4500},{875,4550},{886,4600},{897,4650},{908,4700},{919,4750},{930,4800},{941,4850},{952,4900},{963,4950},
	{974,5000},{985,5050},{996,5100},{1007,5150},{1018,5200},{1029,5250},{1040,5300},{1051,5350},{1062,5400},{1073,5450},{1084,5500},{1095,5550},{1106,5600},{1117,5650},
	{1128,5700},{1139,5750},{1150,5800},{1161,5850},{1172,5900},{1183,5950},{1194,6000},{1205,6050},{1216,6100},{1227,6150},{1238,6200},{1249,6250},{1260,6300},{1271,6350},
	{1282,6400},{1293,6450},{1304,6500},{1315,6550},{1326,6600},{1337,6650},{1348,6700},{1359,6750},{1370,6800},{1381,6850},{1392,6900},{1403,6950},{1414,7000},{1425,7050},
	{1436,7100},{1447,7150},{1458,7200},{1469,7250},{1480,7300},{1491,7350},{1502,7400},{1513,7450},{1524,7500},{1535,7550},{1546,7600},{1557,7650},{1568,7700},{1579,7750},
	{1590,7800},{1601,7850},{1612,7900},{1623,7950},{1634,8000},{1645,8050},{1656,8100}
};
temptre Temp2[150]=
{
	{384,2300},{395,2350},{406,2400},{417,2450},{428,2500},{439,2550},{450,2600},{461,2650},{472,2700},{483,2750},{494,2800},{505,2850},{516,2900},{527,2950},{538,3000},{549,3050},{560,3100},{571,3150},{582,3200},{593,3250},
	{604,3300},{615,3350},{626,3400},{638,3450},{649,3500},{660,3550},{669,3600},{679,3650},{689,3700},{700,3750},{710,3800},{721,3850},{732,3900},{743,3950},{754,4000},{765,4050},{776,4100},{787,4150},{798,4200},{809,4250},
	{820,4300},{831,4350},{842,4400},{853,4450},{864,4500},{875,4550},{886,4600},{897,4650},{908,4700},{919,4750},{930,4800},{941,4850},{952,4900},{963,4950},
	{974,5000},{985,5050},{996,5100},{1007,5150},{1018,5200},{1029,5250},{1040,5300},{1051,5350},{1062,5400},{1073,5450},{1084,5500},{1095,5550},{1106,5600},{1117,5650},
	{1128,5700},{1139,5750},{1150,5800},{1161,5850},{1172,5900},{1183,5950},{1194,6000},{1205,6050},{1216,6100},{1227,6150},{1238,6200},{1249,6250},{1260,6300},{1271,6350},
	{1282,6400},{1293,6450},{1304,6500},{1315,6550},{1326,6600},{1337,6650},{1348,6700},{1359,6750},{1370,6800},{1381,6850},{1392,6900},{1403,6950},{1414,7000},{1425,7050},
	{1436,7100},{1447,7150},{1458,7200},{1469,7250},{1480,7300},{1491,7350},{1502,7400},{1513,7450},{1524,7500},{1535,7550},{1546,7600},{1557,7650},{1568,7700},{1579,7750},
	{1590,7800},{1601,7850},{1612,7900},{1623,7950},{1634,8000},{1645,8050},{1656,8100}

};

temptre Temp3[150]=
{
	{384,2300},{395,2350},{406,2400},{417,2450},{428,2500},{439,2550},{450,2600},{461,2650},{472,2700},{483,2750},{494,2800},{505,2850},{516,2900},{527,2950},{538,3000},{549,3050},{560,3100},{571,3150},{582,3200},{593,3250},
	{604,3300},{615,3350},{626,3400},{638,3450},{649,3500},{660,3550},{669,3600},{679,3650},{689,3700},{700,3750},{710,3800},{721,3850},{732,3900},{743,3950},{754,4000},{765,4050},{776,4100},{787,4150},{798,4200},{809,4250},
	{820,4300},{831,4350},{842,4400},{853,4450},{864,4500},{875,4550},{886,4600},{897,4650},{908,4700},{919,4750},{930,4800},{941,4850},{952,4900},{963,4950},
	{974,5000},{985,5050},{996,5100},{1007,5150},{1018,5200},{1029,5250},{1040,5300},{1051,5350},{1062,5400},{1073,5450},{1084,5500},{1095,5550},{1106,5600},{1117,5650},
	{1128,5700},{1139,5750},{1150,5800},{1161,5850},{1172,5900},{1183,5950},{1194,6000},{1205,6050},{1216,6100},{1227,6150},{1238,6200},{1249,6250},{1260,6300},{1271,6350},
	{1282,6400},{1293,6450},{1304,6500},{1315,6550},{1326,6600},{1337,6650},{1348,6700},{1359,6750},{1370,6800},{1381,6850},{1392,6900},{1403,6950},{1414,7000},{1425,7050},
	{1436,7100},{1447,7150},{1458,7200},{1469,7250},{1480,7300},{1491,7350},{1502,7400},{1513,7450},{1524,7500},{1535,7550},{1546,7600},{1557,7650},{1568,7700},{1579,7750},
	{1590,7800},{1601,7850},{1612,7900},{1623,7950},{1634,8000},{1645,8050},{1656,8100}
};

volatile lut apt[20]=
{
	{-250,1270},{-200,1350},{-150,1430},{-100,1510},{-50,1590},{0,1670},{50,1750},{100,1840},{150,1920},{200,2000},{250,2080},{300,2160},{350,2240},{400,2320}
};

volatile lut vpt1[20]=
{
	{-250,1280},{-200,1360},{-150,1440},{-100,1520},{-50,1600},{0,1680},{50,1760},{100,1850},{150,1930},{200,2010},{250,2090},{300,2170},{350,2250},{400,2330}
};
volatile lut ps_1[50]=
{
	{-600,390},{-550,450},{-500,510},{-450,570},{-400,630},{-350,690},{-300,750},{-250,810},{-200,870},{-150,930},{-100,980},{-50,1040},{0,1100},{50,1170},{100,1245},{150,1321},{200,1396},{250,1466},{300,1540},{350,1615},{400,1690},{450,1765},{500,1840},{550,1915},{600,1990},{650,2065},{700,2140},{750,2215},{800,2290},{850,2365},{900,2440},{950,2515},{1000,2590},{1050,2665},{1100,2740},{1150,2815},{1200,2890}
};
volatile lut ps_2[50]=
{
	{-600,390},{-550,450},{-500,510},{-450,570},{-400,630},{-350,690},{-300,750},{-250,810},{-200,870},{-150,930},{-100,980},{-50,1040},{0,1100},{50,1170},{100,1245},{150,1321},{200,1396},{250,1466},{300,1540},{350,1615},{400,1690},{450,1765},{500,1840},{550,1915},{600,1990},{650,2065},{700,2140},{750,2215},{800,2290},{850,2365},{900,2440},{950,2515},{1000,2590},{1050,2665},{1100,2740},{1150,2815},{1200,2890}
};
volatile lut ps_3[50]=
{
	{-600,390},{-550,450},{-500,510},{-450,570},{-400,630},{-350,690},{-300,750},{-250,810},{-200,870},{-150,930},{-100,980},{-50,1040},{0,1100},{50,1170},{100,1245},{150,1321},{200,1396},{250,1466},{300,1540},{350,1615},{400,1690},{450,1765},{500,1840},{550,1915},{600,1990},{650,2065},{700,2140},{750,2215},{800,2290},{850,2365},{900,2440},{950,2515},{1000,2590},{1050,2665},{1100,2740},{1150,2815},{1200,2890}
};
volatile hep hp[20]=
{
	{10,25},{9,35},{8,45},{7,55},{6,65},{5,80},{4,100},{2,200},{1,350},{50,3000},{30,5500}
};
volatile conduc cond[50]=
{
	{191,1280},{370,1350},{544,1380},{759,1420},{976,1440},{1180,1480},{1404,1520},{1597,1560},{1804,1600},{1840,1620},{1980,1650},{2203,1700},{2407,1750},{2648,1800},{2824,1840},{2850,1847},{2900,1860},{2950,1872},{3000,1884},{3050,1897},{3100,1909},{3150,1921},{3200,1934}
};
*/

volatile hep hp[50]=
{
//	{10,130,16000},{9,42,15},{8,45,13},{7,55,12},{6,65,10},{5,95,9},{4,100,8},{2,530,7},{1,1060,2},{50,3000,50},{30,5500,30}
	{10,1200,30000,35000},{20,600,15000,18000},{30,400,10400,12500},{40,300,7500,8500},{50,240,6000,7000},{60,200,5000,5700 },{70,170,4200,4600},{80,150,3800,4100},{90,130,3400,3600},{100,120,3000,3300}
};

volatile lut apt2[26]=
{
	{-40,-760},{-30,-550 },{-20,-430 },{-10	,-280},{0,-210},{10,5},{20,140},{30	,280},{40,336},	{50,550},{60,692},{70,840},{80,980},{90,1115},{100,1260},{110,1396},{120,1513},{140,1747},{160,2030},{180,2381},{200,2742},{240,3190},{280,3646},{320,4300},{360,4833},{420,5680}
		
		
};

volatile lut vpt2[26]=
{
	{-40,-760},{-30,-550 },{-20,-430 },{-10	,-280},{0,-210},{10,5},{20,140},{30	,280},{40,336},	{50,550},{60,692},{70,840},{80,980},{90,1115},{100,1260},{110,1396},{120,1513},{140,1747},{160,2030},{180,2381},{200,2742},{240,3190},{280,3646},{320,4300},{360,4833},{420,5680}
};

volatile uf ufp[14] = {
	// manav04 changed
	//{40,1500},{60,1050},{70,700},{80,650},{90,600},{100,550},{110,500},{120,450},{130,400},{150,350},{170,300},{200,250},{250,200},{340,150}
		{10,1000},{20,950},{30,900},{40,850},{50,800},{60,750},{70,700},{80,650},{90,600},{100,550},{110,500},{120,450},{130,400},{150,350},{170,300},{200,250},{250,200},{340,150}
};

#if 0

{-400,-11335},{-350,-10835},{-300,-9812},{-250,-9388},{-200,-8435},{-150,-7903},{-100,-6838},{-50,-6368},{0	,-5438},{50,-4887},{100,-3997},{150,-3100},{200,-2354},{250,-1735},{300,-973},{350,-270},{400,600},{450,1343},{474,1700},{500,1800},{550,2766},{600,3600},{650,4100},{700,5100},{750,5750},{800,6500}
{850,7277}




{-400	,	-11235},
{-350	,	-10735},
{-300	,	-9691},
{-250	,	-9283},
{-200	,	-8435},
{-150	,	-7826},
{-100	,	-6811},
{-50	,	-6295},
{0	,	-5348},
{50	,	-4788},
{100	,	-3936},
{150	,	-3052},
{200	,	-2296},
{250	,	-1678},
{300	,	-971},
{350	,	-240},
{400	,	620},
{450	,	1369},
{474	,	1700},
{500	,	1817},
{550	,	2766},
{600	,	3600},
{650	,	4100},
{700	,	5100},
{750	,	5750},
{800	,	6500},






{-400	,	-11235},
{-350	,	-10735},
{-300	,	-9691},
{-250	,	-9283},
{-200	,	-8435},
{-150	,	-7822},
{-100	,	-6813},
{-50	,	-6295},
{0	,	-5370},
{50	,	-4814},
{100	,	-3958}
{150	,	-3077},
{200	,	-2354},
{250	,	-1727},
{300	,	-997},
{350	,	-240},
{400	,	620},
{450	,	1343},
{474	,	1700},
{500	,	1800},
{550	,	2743},
{600	,	3600},
{650	,	4100},
{700	,	5100},
{750	,	5750},
{800	,	6500},
{850	,	7277},
{900	,	8000},
{950	,	8800},
{1000	,	9700},
{1050	,	10700},
{1100	,	11700},
{1150	,	12800},
{1200	,	14000},
{1250	,	15200},
{1300	,	16500},
{1350	,	17800},
{1400	,	19200},
{1450	,	20700},
{1500	,	22300}
#endif

volatile lut ps_1_2[27]=
{
	
{-400,-11235},{-350,-10735},{-300,-9691},{-250,-9283},{-200,-8335},{-150,-7826},{-100,-6811},{-50,-6295},{0	,-5348},{50	,-4788},{100,-3936},{150,-3052},{200,-2296},{250,-1678},{300,-971},{350,-240},{400,620},{450,1369},{474,1700},{500,1817},{550,2766},{600,3600},{650,4100},{700,5100},{750,5750},{800,6500}

};

volatile lut ps_2_2[27]=
{
	//{-400,-11335},{-350,-10835},{-300,-9812},{-250,-9388},{-200,-8435},{-150,-7903},{-100,-6838},{-50,-6368},{0	,-5348},{50,-4887},{100,-3997},{150,-3100},{200,-2354},{250,-1735},{300,-973},{350,-270},{400,600},{450,1343},{474,1700},{500,1800},{550,2766},{600,3600},{650,4100},{700,5100},{750,5750},{800,6500}
		
	{-400,-11335},{-350,-10835},{-300,-9832},{-250,-9398},{-200,-8455},{-150,-7943},{-100,-6878},{-50,-6508},{0	,-5478},{50,-5017},{100,-4067},{150,-3290},{200,-2534},{250,-1955},{300,-1193},{350,-450},{400,400},{450,1100},{474,1470},{500,1580},{550,2536},{600,3360},{650,3900},{700,5000},{750,5650},{800,6400}
	
};


volatile lut ps_3_2[40]=
{ {-400	,	-11235},{-350	,	-10735},{-300	,	-9691},{-250	,	-9283},{-200	,	-8435},{-150	,	-7822},{-100	,	-6813},{-50	,	-6295},{0	,	-5370},{50	,	-4814},{100	,	-3958},{150	,	-3077},{200	,	-2354},{250	,	-1727},{300	,	-997},{350	,	-240},{400	,	620},{450	,	1343},{474	,	1700},{500	,	1800},{550	,	2743},{600	,	3600},{650	,	4100},{700	,	5100},{750	,	5750},{800	,	6500},{850	,	7277},{900	,	8000},{950	,	8800},{1000	,	9700},{1050	,	10700},{1100	,	11700},{1150	,	12800},{1200	,	14000},{1250	,	15200},{1300	,	16500},{1350	,	17800},{1400	,	19200},{1450	,	20700},{1500	,	22300}
};


volatile fb blood_array[20]=
{
	{50,700,50},{60,750,57},{65,750,64},{70,750,70},{75,750,74},{80,750,78},{85,750,82},{90,750,86},{95,750,90},{100,760,100},{110,780,110},{120,810,120},
	{140,810,140},{150,850,150},{200,970,200},{300,1200,290},{400,1250,350}
};

//*************************************************** till this************



void speedchange_bloodpump_feedback(uint32_t spd_change)
{
	int i;
	
	for(i=0;i<20;i++)
	{
		if(spd_change == blood_array[i].ml)
		{
			fed_value=blood_array[i].fdbck;
			//value=i;
			return fed_value;
			break;
		}
		else if((spd_change > blood_array[i].ml) && (spd_change < blood_array[i+1].ml))
		{
			fed_value = ((spd_change-blood_array[i].ml)*(blood_array[i+1].fdbck-blood_array[i].fdbck)/(blood_array[i+1].ml-blood_array[i].ml))+blood_array[i].fdbck;
			return fed_value;
			break;
		}
	}
	
}

Cl_Uint32Type hep_calibration(Cl_Uint32Type inpt_ml)
{
	int i = 0;
	float slope;
	//inpt_ml=(inpt_ml/10);
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<10;i++)
	{
		if (inpt_ml == hp[i].ml)
		{
			hep_speed=hp[i].speed;
			hep_fblower = hp[i].fblower;
			hep_fbupper = hp[i].fbupper;
			return hep_speed;
			//sv_cntrl_setpumpspeed(HEPARINPUMP,hep_speed);
			break;
		}
		else if ((inpt_ml > hp[i].ml) && (inpt_ml < hp[i+1].ml))
		{
			slope = (hp[i+1].speed-hp[i].speed)/(hp[i+1].ml-hp[i].ml);
			hep_speed = slope * (inpt_ml-hp[i].ml) + hp[i].speed;
			slope = (hp[i+1].fblower-hp[i].fblower)/(hp[i+1].ml-hp[i].ml);
			hep_fblower = slope * (inpt_ml-hp[i].ml) + hp[i].fblower;
			slope = (hp[i+1].fbupper-hp[i].fbupper)/(hp[i+1].ml-hp[i].ml);
			hep_fbupper = slope * (inpt_ml-hp[i].ml) + hp[i].fbupper;
			//sv_cntrl_setpumpspeed(HEPARINPUMP,hep_speed);
			return hep_speed;
			break;
		}
	}
	
}
void calibration_apt(int16_t sensordata)
{
	int i;
	float slope;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<26;i++)
	{
		if (sensordata == apt2[i].millivolt)
		{
			pressure_final_apt=apt2[i].pressure;
		}
		else if ((sensordata > apt2[i].millivolt) && (sensordata < apt2[i+1].millivolt))
		{
			slope = ((apt2[i+1].pressure-apt2[i].pressure)*100/(apt2[i+1].millivolt-apt2[i].millivolt));
			pressure_final_apt = (slope * (sensordata-apt2[i].millivolt))/100 + apt2[i].pressure;
		}
		else if(sensordata < apt2[0].millivolt)
		{
			pressure_final_apt = apt2[0].pressure;
		}
		else  if(sensordata > apt2[25].millivolt)
		{
			pressure_final_apt = apt2[25].pressure;
		}
		
	}
}
void calibration_vpt(int16_t sensordata)
{
	int i;
	float slope;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<26;i++)
	{
		if (sensordata == vpt2[i].millivolt)
		{
			pressure_final_vpt=vpt2[i].pressure;
			break;
		}
		else if ((sensordata > vpt2[i].millivolt) && (sensordata < vpt2[i+1].millivolt))
		{
			slope = ((vpt2[i+1].pressure-vpt2[i].pressure) * 100/(vpt2[i+1].millivolt-vpt2[i].millivolt));
			pressure_final_vpt = (slope *(sensordata-vpt2[i].millivolt))/100 + vpt2[i].pressure;
			break;
		}
		else if(sensordata < vpt2[0].millivolt)
		{
			pressure_final_vpt = vpt2[0].pressure;
		}
		else  if(sensordata > vpt2[25].millivolt)
		{
			pressure_final_vpt = vpt2[25].pressure;
		}
	}
	
}
void calibration_ps1(int16_t sensordata)
{
	int i;
	int16_t slope;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<27;i++)
	{
		if (sensordata == ps_1_2[i].millivolt)
		{
			pressure_final_ps1=ps_1_2[i].pressure;
			break;
		}
		else if ((sensordata > ps_1_2[i].millivolt) && (sensordata < ps_1_2[i+1].millivolt))
		{
			slope = ((ps_1_2[i+1].pressure-ps_1_2[i].pressure)*100/(ps_1_2[i+1].millivolt-ps_1_2[i].millivolt));
			pressure_final_ps1 = (slope * (sensordata-ps_1_2[i].millivolt))/100 + ps_1_2[i].pressure;
			break;
		}
	}
}
void calibration_ps2(int16_t sensordata)
{
	int i;
	int16_t slope;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<27;i++)
	{
		if (sensordata == ps_2_2[i].millivolt)
		{
			pressure_final_ps2=ps_2_2[i].pressure;
			break;
		}
		else if ((sensordata > ps_2_2[i].millivolt) && (sensordata < ps_2_2[i+1].millivolt))
		{
			slope = ((ps_2_2[i+1].pressure-ps_2_2[i].pressure) *100 /(ps_2_2[i+1].millivolt-ps_2_2[i].millivolt));
			pressure_final_ps2 = (slope * (sensordata-ps_2_2[i].millivolt))/100 + ps_2_2[i].pressure;
			break;
		}
	}
}
void calibration_ps3(int16_t sensordata)
{
	int i;
	int16_t slope;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<50;i++)
	{
		if (sensordata == ps_3_2[i].millivolt)
		{
			pressure_final_ps3=ps_3_2[i].pressure;
			break;
		}
		else if ((sensordata > ps_3_2[i].millivolt) && (sensordata < ps_3_2[i+1].millivolt))
		{
			slope = ((ps_3_2[i+1].pressure-ps_3_2[i].pressure)*100/(ps_3_2[i+1].millivolt-ps_3_2[i].millivolt));
			pressure_final_ps3 = ((slope * (sensordata-ps_3_2[i].millivolt))/100 + ps_3[i].pressure);
			break;
		}
	}
}

void calibration_cond(float sensordata)
{
	int i;
	float dummy_var = 0;
	dummy_var = sensordata;
	float slope=0;
	//Cl_Uint16Type tempdata=sensordata;
	for (i=0;i<50;i++)
	{
		if (dummy_var == cond[i].volts)
		{
			cond_final_cs3=cond[i].conductivity;
		}
		else if ((dummy_var > cond[i].volts) && (dummy_var < cond[i+1].volts))
		{
			slope = ((cond[i+1].conductivity-cond[i].conductivity)/(cond[i+1].volts-cond[i].volts));
			cond_final_cs3 = slope*(dummy_var-cond[i].volts) + cond[i].conductivity;
		}
		/*else
		{
			cond_final_cs3=0;
		}*/
		/*else if (sensordata < cond[0].volts )
		{
			slope = (cond[i].volts)/(cond[i].conductivity);
			cond_final_cs3 = slope*(sensordata-cond[i].volts) + cond[i].conductivity;
		}*/
	}
}


void calibrate_uf(int16_t ufrate){
	
	Cl_Uint16Type i;
	float dummy_var = ufrate;
	Cl_Uint32Type dummy_var1;
	float slope;

	
	for (i=0;i<14;i++)
		{
			if (dummy_var == ufp[i].rate)
			{
				dummy_var1=ufp[i].speed;
				uf_final=dummy_var1;
				break;
			}
			else if ((dummy_var > ufp[i].rate) && (dummy_var < ufp[i+1].rate))
			{
				//temprature_final_value =dummy_var;
				slope = (((ufp[i+1].speed - ufp[i].speed))/(ufp[i+1].rate - ufp[i].rate));
				//temprature_final_value =slope*1000;
				uf_final = ((slope*(dummy_var - ufp[i].rate)) + ufp[i].speed);
				//temprature_final_value_1 =temprature_final_value_1/100;
				//temprature_final_value =1500;
				break;
			}
			else if (dummy_var < ufp[i].rate)
			{
				uf_final = 10;
				break;
			}
			
		}
	
	
}

// void calibration_tmp(float millivolts, temp_state temp_var)
// {
// 	
// 	Cl_Uint16Type i;
// 	float dummy_var = millivolts;
// 	Cl_Uint32Type dummy_var1;
// 	float slope;
// 
// 	if (temp_var==TS1)
// 	{
// 		for (i=0;i<150;i++)
// 		{
// 			if (dummy_var == Temp1[i].volts)
// 			{
// 				dummy_var1=Temp1[i].temperature;
// 				temprature_final_value_1=dummy_var1/100;
// 				break;
// 			}
// 			else if ((dummy_var > Temp1[i].volts) && (dummy_var < Temp1[i+1].volts))
// 			{
// 				//temprature_final_value =dummy_var;
// 				slope = (((Temp1[i+1].temperature - Temp1[i].temperature)*1000)/(Temp1[i+1].volts - Temp1[i].volts));
// 				//temprature_final_value =slope*1000;
// 				temprature_final_value_1 = ((slope*(dummy_var - Temp1[i].volts))/1000 + Temp1[i].temperature);
// 				temprature_final_value_1 =temprature_final_value_1/100;
// 				//temprature_final_value =1500;
// 				break;
// 			}
// 			/*else if (dummy_var < Temp1[i].volts)
// 			{
// 				temprature_final_value_1 = (31.47*(dummy_var/1000)-9.829)*10;
// 				break;
// 			}*/
// 			
// 		}
// 		//millivolts = millivolts/1000;
// 		//temprature_final_value_1 = 1.830098345 * millivolts *millivolts +  22.62524406 *millivolts + 0.581851503  ;
// 	}
// 	else if (temp_var==TS2)
// 	{
// 		
// 		for (i=0;i<150;i++)
// 		{
// 			if (dummy_var == Temp2[i].volts)
// 			{
// 				dummy_var1=Temp2[i].temperature;
// 				temprature_final_value_2=dummy_var1/100;
// 				break;
// 			}
// 			else if ((dummy_var > Temp2[i].volts) && (dummy_var < Temp2[i+1].volts))
// 			{
// 				//temprature_final_value =dummy_var;
// 				slope = (((Temp2[i+1].temperature - Temp2[i].temperature)*1000)/(Temp2[i+1].volts - Temp2[i].volts));
// 				//temprature_final_value = slope;
// 				temprature_final_value_2 = ((slope*(dummy_var - Temp2[i].volts))/1000 + Temp2[i].temperature)/100;
// 				//temprature_final_value =1500;
// 				break;
// 			}
// 		/*	else if (dummy_var < Temp2[i].volts)
// 			{
// 				var = (29.11*(dummy_var/1000)-8.345)*10;
// 				temprature_final_value_2=var;
// 				break;
// 			}*/
// 			
// 		}
// 		//millivolts = millivolts/1000;
// 		//temprature_final_value_2 = 1.830098345 * millivolts *millivolts +  22.62524406 *millivolts + 0.581851503  ;
// 	}
// 	
// 	else if (temp_var == TS3)
// 	{
// 		for (i=0;i<150;i++)
// 		{
// 			if (dummy_var == Temp3[i].volts)
// 			{
// 				dummy_var1=Temp3[i].temperature;
// 				temprature_final_value_3=dummy_var1/100;
// 				break;
// 			}
// 			else if ((dummy_var > Temp3[i].volts) && (dummy_var < Temp3[i+1].volts))
// 			{
// 				//temprature_final_value =dummy_var;
// 				slope = (((Temp3[i+1].temperature - Temp3[i].temperature)*1000)/(Temp3[i+1].volts - Temp3[i].volts));
// 				//temprature_final_value = slope;
// 				temprature_final_value_3 = ((slope*(dummy_var - Temp3[i].volts))/1000 + Temp3[i].temperature)/100;
// 				//temprature_final_value =1500;
// 				break;
// 			}
// 		/*	else if (dummy_var < Temp3[i].volts)
// 			{
// 				var = (33.58 * (dummy_var/1000) - 16.39)*10;
// 				//var=dummy_var/100;
// 				temprature_final_value_3=var;
// 				break;
// 			}*/
// 
// 			
// 		}
// 		//millivolts = millivolts/1000;
// 		//temprature_final_value_3 = 1.830098345 * millivolts *millivolts +  22.62524406 *millivolts + 0.581851503  ;
// 		
// 	}
// 	
// }
