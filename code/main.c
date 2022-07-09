#include	"def.h"

#define USE_COS

//#define SER_RAW
//#define SER_FIL
//#define SER_PEAK
//#define SER_HBEAT
#define SER_OUT
//#define SER_SPECTRUM


#define FOSC 22118400UL
#define BGV  1190U
#define BAUDRATE 115200U

// oled flushed every 32 timer events (planned, but maybe delayed if busy)
#define TIMER_FREQ 64

#define IIC_FREQ    200000UL
#define IIC_ADDR_SEN    0xAE
#define IIC_ADDR_OLED   0x78
#define OLED_NOP 0xE3

#define HR_MIN 30
#define HR_MAX 200
#define O2_MIN 10
#define O2_MAX 99
#define BATT_MIN 3300
#define BATT_MAX 4200

#define SEN_BUFFSIZE 32

#define PROX_PA   16
#define PROX_TH   10
#define CALI_AIM  100000UL

// (65536*HR_MIN/60/SEN_RATE-6)=322, -6 is a fixed value for smallest error in whole heart beat rate scope 
#define PD_TADD  322
// (65536/(60*SEN_RATE))=11
#define PD_FADD  11

#define ST_START  0
#define ST_PROX   10
#define ST_PROX1  11
#define ST_CALI   20
#define ST_CALI1  21
#define ST_CALI2  22
#define ST_FIL    30
#define ST_FIL1   31
#define ST_FIL2   32
#define ST_WORK   50
#define ST_WORK1  51
#define ST_STABLE 80

u8  idata g_state=0;   // state defined as ST_ macro 
u8  idata g_staux=0;   // for state-machine sub-state

u32 idata g_timer=0;   // a counter, added 1 every 1/64s
u32 idata g_samps;     // how many samples processed
u32 idata g_lost;      // how many samples losted

i16 idata g_batt=0;    // battery voltage, in mV
i16 idata g_hr;        // heart beat rate per minute, fixed point, need to ">>6"
i16 idata g_o2;        // SpO2 percent, fixed point, need to ">>6"

u8  idata g_red_pa;    // sensor RED LED current 0~0xff:0~50mA
u8  idata g_ir_pa;     // sensor IR LED current 0~0xff:0~50mA
u32 idata g_cali_aim;  // aim of calibrating LED current

i32 idata g_red_dc;    // RED sample DC value 
i16 idata g_red_max;   // RED sample MAX value 
i16 idata g_red_min;   // RED sample MIN value
i32 idata g_ir_dc;     // IR sample DC value 
i16 idata g_ir_max;    // IR sample MAX value 
i16 idata g_ir_min;    // IR sample MIN value

u8  xdata g_buff[SEN_BUFFSIZE*6];
i16 xdata g_hr_i[HR_MAX-HR_MIN+1];
i16 xdata g_hr_q[HR_MAX-HR_MIN+1];

u8 code CHAR_DIGI [][64]={{  // every character is 16 COMs ( 2 pages) x 32 COLs
	0x00,0xF0,0xF8,0xF0,0x04,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x06,0x00,0x00,
	0x02,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xF4,0xF8,0xF0,0x00,0x00,0x00,0x00,
	0x00,0x0F,0x1F,0x0F,0x00,0x40,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x80,0x80,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x40,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"0",0*/ 
	},{
	0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,
	0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x02,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x02,0x02,
	0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x02,0x00,0x00,0x00,0x00,/*"1",1*/
	},{
	0x00,0xF0,0xF8,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF8,0xF8,
	0x06,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x04,0xF0,0xF8,0xF0,0x00,0x00,0x00,0x00,
	0x00,0x0F,0x1F,0x0F,0x40,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xCF,0x3F,0x3F,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x3F,0x7F,0x00,0x00,0x00,0x00,/*"2",2*/
	},{
	0x00,0xFC,0xF8,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0xFE,0xF8,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF8,0xFC,0x00,0x00,0x00,0x00,
	0x00,0x0F,0x1F,0x0F,0x20,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x67,0x5F,0x5F,
	0x60,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x20,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"3",3*/
	},{
	0x00,0x00,0x00,0x02,0x06,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xE6,0xF8,0xF8,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x80,0xC0,0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xCF,0xBF,0xBF,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,/*"4",4*/
	},{
	0x00,0xF0,0xF8,0xF0,0x04,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xE6,0xF8,0xF8,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0xFC,0xFE,0x00,0x00,0x00,0x00,
	0x00,0x7F,0x3F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x3F,0x3F,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x40,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"5",5*/
	},{
	0x00,0xF0,0xF8,0xF0,0x04,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xE6,0xFA,0xFA,
	0x06,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xF4,0xF8,0xF0,0x00,0x00,0x00,0x00,
	0x00,0x7F,0x3F,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x3F,0x3F,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x40,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"6",6*/
	},{
	0x00,0xFE,0xFC,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x3F,0x1F,0x2F,0x30,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x30,0x20,0x20,
	0x30,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x30,0x20,0x00,0x00,0x00,0x00,/*"7",7*/
	},{
	0x00,0xF0,0xF8,0xF0,0x04,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xE6,0xF8,0xF8,
	0x06,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xF4,0xF8,0xF0,0x00,0x00,0x00,0x00,
	0x00,0x0F,0x1F,0x0F,0x40,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xCF,0xBF,0xBF,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x40,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"8",8*/
	},{
	0x00,0xF0,0xF8,0xF0,0x04,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0xE6,0xF8,0xF8,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0xF8,0xFC,0x00,0x00,0x00,0x00,
	0x00,0x0F,0x1F,0x0F,0x40,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xCF,0xBF,0xBF,
	0xC0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0x40,0x0F,0x1F,0x0F,0x00,0x00,0x00,0x00,/*"9",9*/
}};

u8 code CHAR_NA [64] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0xF8,0xFC,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x07,0x07,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"-"*/
};

u8 code CHAR_SP [64] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*" "*/
};

u8 code CHAR_PCT [64] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x28,0x44,0x44,0x44,0x44,
	0x44,0x44,0x28,0xB8,0x80,0x80,0x40,0x40,0x60,0x20,0x20,0x30,0x10,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x04,0x04,0x06,0x02,0x02,
	0x03,0x01,0x01,0x0E,0x0A,0x11,0x11,0x11,0x11,0x11,0x11,0x0A,0x0E,0x00,0x00,0x00,/*"%"*/
};

u8 code CHAR_BATT [32] = {
	0x00,0xE0,0x20,0x20,0xF8,0x08,0x08,0x88,0xC8,0xE8,0x88,0xC8,0xC8,0x48,0xF8,0x00,
	0x00,0x07,0x04,0x04,0x1F,0x12,0x13,0x11,0x10,0x13,0x13,0x11,0x10,0x10,0x1F,0x00,/*"batt.bmp"*/
};

u8 code BATT_VOLT [9] = {
  0x00,0x01,0x03,0x07,0x07,0x17,0x37,0x77,0x77,
};

u8 code CHAR_NUM [] = {
	'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};

// SpO2 table, multiplied by 64
i16 code O2_LUT [] = {
  6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,
	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6397,	6396,	
	6393,	6389,	6383,	6376,	6368,	6358,	6347,	6334,	6320,	6305,	6288,	6270,
	6250,	6229,	6207,	6183,	6158,	6131,	6103,	6073,	6043,	6010,	5977,	
	5941,	5905,	5867,	5828,	5787,	5745,	5701,	5656,	5610,	5562,	5513,	
	5462,	5410,	5357,	5302,	5246,	5188,	5129,	5068,	5007,	4943,	4879,
	4812,	4745,	4676,	4606,	4534,	4461,	4386,	4310,	4233,	4154,	4074,	
	3992,	3909,	3825,	3739,	3652,	3563,	3473,	3382,	3289,	3195,	3099,
	3002,	2903,	2804,	2702,	2600,	2495,	2390,	2283,	2175,	2065,	1954,
	1841,	1727,	1612,	1495,	1377,	1257,	1136,	1014,	890,	765,	638,
	510,	380,	250,	117,	0
};

#ifdef USE_COS

i16 code COS_LUT [] = {
	256,	256,	256,	255,	255,	254,	253,	252,	251,	250,	248,	247,	245,	243,	241,	239,
	237,	234,	231,	229,	226,	223,	220,	216,	213,	209,	206,	202,	198,	194,	190,	185,
	181,	177,	172,	167,	162,	157,	152,	147,	142,	137,	132,	126,	121,	115,	109,	104,
	98,	92,	86,	80,	74,	68,	62,	56,	50,	44,	38,	31,	25,	19,	13,	6,	
	0,	-6,	-13,	-19,	-25,	-31,	-38,	-44,	-50,	-56,	-62,	-68,	-74,	-80,	-86,	-92,
	-98,	-104,	-109,	-115,	-121,	-126,	-132,	-137,	-142,	-147,	-152,	-157,	-162,	-167,	-172,	-177,
	-181,	-185,	-190,	-194,	-198,	-202,	-206,	-209,	-213,	-216,	-220,	-223,	-226,	-229,	-231,	-234,	
	-237,	-239,	-241,	-243,	-245,	-247,	-248,	-250,	-251,	-252,	-253,	-254,	-255,	-255,	-256,	-256,
	-256,	-256,	-256,	-255,	-255,	-254,	-253,	-252,	-251,	-250,	-248,	-247,	-245,	-243,	-241,	-239,
	-237,	-234,	-231,	-229,	-226,	-223,	-220,	-216,	-213,	-209,	-206,	-202,	-198,	-194,	-190,	-185,
	-181,	-177,	-172,	-167,	-162,	-157,	-152,	-147,	-142,	-137,	-132,	-126,	-121,	-115,	-109,	-104,
	-98,	-92,	-86,	-80,	-74,	-68,	-62,	-56,	-50,	-44,	-38,	-31,	-25,	-19,	-13,	-6,	
	0,	6,	13,	19,	25,	31,	38,	44,	50,	56,	62,	68,	74,	80,	86,	92,	
	98,	104,	109,	115,	121,	126,	132,	137,	142,	147,	152,	157,	162,	167,	172,	177,
	181,	185,	190,	194,	198,	202,	206,	209,	213,	216,	220,	223,	226,	229,	231,	234,
	237,	239,	241,	243,	245,	247,	248,	250,	251,	252,	253,	254,	255,	255,	256,	256
};
	
#endif

// delay in millisecond, specified by "ms"
void delay_ms(u16 ms)
{
	u8 i;
	u8 j;
	
	while (ms--){
  	i = 1+((u16)(FOSC/3000UL)>>8); 
  	j = (u8)(FOSC/3000UL);
		do {
			while (--j);
		} while (--i);		
	}		
}


void ser_send(u8 ch)
{
	SBUF=ch;
	while (!TI);
	TI=0;
}

void ser_u8 (u8 u)
{
	ser_send(CHAR_NUM[u>>4]);
	ser_send(CHAR_NUM[u&0x0f]);
}

void ser_u16 (u16 u)
{
	ser_u8(((u16s*)&u)->h);
	ser_u8(((u16s*)&u)->l);
}

void ser_u32 (u32 u)
{
	ser_u8(((u32s*)&u)->hh);
	ser_u8(((u32s*)&u)->hl);
	ser_u8(((u32s*)&u)->lh);
	ser_u8(((u32s*)&u)->ll);
}

// read ADC ch, ch==0x0f means internal Gap-Band voltage, output 0~1023
u16 adc_read(u8 ch)
{
	  ADC_CONTR = 0xC0 | (ch & 0x0f); // power on, start, select internal Gap-Band channel
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));   // wait completing
    ADC_CONTR = 0x8f;  // power on, stop, select internal Gap-Band channel 

  	return ((u16)ADC_RES << 8) | (u16)ADC_RESL; 
}

// get voltage of battery
void batt_get ()
{
	u16 t;
	u16 gb;
	
	if (!P33) return; // pulled down by intr.
	t=adc_read(0x03); // read P3.3(ADC3)
	if (!P33) return; // pulled down by intr.
			
  gb=adc_read(0x0f); // read internal Gap-Band voltage
	if (gb<32) return; // avoid fault
	
  t=(i16)(69UL*BGV*t/gb/47L);

	if (g_batt>2000) {
		g_batt += ((i16)t - g_batt)>>4; // T(0.63)=16 sample points
	}	else
		g_batt = t; 
}

void iic_wait() {
	while (!(I2CMSST & 0x40)); 
	I2CMSST &= ~0x40;
}

void iic_stop()
{
	I2CMSCR = 0x06;
	iic_wait();
}

void iic_start(u8 dev)
{
	I2CTXD = dev;                               
	I2CMSCR = 0x09;                            
	iic_wait();
}

void iic_send(u8 dat)
{
	I2CTXD = dat;                               
	I2CMSCR = 0x0A;                            
	iic_wait();
}

u8 iic_recv_ack()
{
	I2CMSCR = 0x0B;                             
	iic_wait();
	return I2CRXD;
}

u8 iic_recv_nak()
{
	I2CMSCR = 0x0C;                             
	iic_wait();
	return I2CRXD;
}

// get 1 byte from dev
u8 sen_get(u8 addr)
{
	u8 ret;

	iic_start(IIC_ADDR_SEN & 0xfe);
	iic_send(addr);
	iic_start(IIC_ADDR_SEN | 0x01);
	ret=iic_recv_nak();
	iic_stop();
	
	return ret;
}

// write a byte to sensor
void sen_put(u8 addr, u8 dat) 
{
	iic_start(IIC_ADDR_SEN & 0xfe);
	iic_send(addr);
	iic_send(dat);
	iic_stop();
}

// read bytes from sensor
void sen_read(u8 addr, u8 *p, u8 len)
{
	if (!len) return;
	
	iic_start(IIC_ADDR_SEN & 0xfe);
	iic_send(addr);
	iic_start(IIC_ADDR_SEN | 0x01);
	while (--len) {
		*(p++) = iic_recv_ack();
	}
	*p = iic_recv_nak();
	iic_stop();
}

// set sensor LED/IR current and ADC range
void sen_cali()
{
	sen_put(0x0D, g_red_pa); // RED LED current
	sen_put(0x0C, g_ir_pa);  // IR LED current
}

// send 2 commands 
void oled_cmd(u8 cmd, u8 cmd2)
{
	_ea_clr_();			
	iic_start(IIC_ADDR_OLED & 0xfe);
	iic_send(0x00);
	iic_send(cmd);
	iic_send(cmd2);	
	iic_stop();
	_ea_pop_();			
}

// set OLED RAM-writing start COL and Page
void oled_addr(u8 col, u8 page)
{
	iic_start(IIC_ADDR_OLED & 0xfe);
	iic_send(0x00);
	iic_send(0x0f & col);
	iic_send(0x10 | (col>>4));
	iic_send(0xB0 | (page & 0x07));
	iic_stop();
}

// write OLED RAM
void oled_ram(u8 *p, u8 len)
{
	if (!len) return;
	
	iic_start(IIC_ADDR_OLED & 0xfe);
	iic_send(0x40);
	do {
		iic_send(*(p++));
	} while (--len);
	iic_stop();	
}

// set OLED RAM
void oled_set(u8 dat, u8 len)
{
	if (!len) return;

	iic_start(IIC_ADDR_OLED & 0xfe);
	iic_send(0x40);
	do {
		iic_send(dat);
	} while (--len);
	iic_stop();	
}

// show a character at specified position, character is "ncol" height and "npage" width
void oled_char(u8 col, u8 page, u8 ncol, u8 npage, u8 *ch)
{
	do {
		oled_addr(col, page);
		oled_ram(ch, ncol);
		page++;
		ch+=ncol;
	} while (--npage);
}

void oled_fill(u8 col, u8 page, u8 ncol, u8 npage, u8 dat)
{
	do {
		oled_addr(col, page);
		oled_set(dat, ncol);
		page++;
	} while (--npage);
}

// show heart rate number
void oled_hr()
{
	u8 t;
	u8 hr;
	
	hr=(u8)((g_hr+32)>>6);
	
	if (g_state!=ST_STABLE) {
		oled_char(0, 0, 32, 2, CHAR_SP);
		oled_char(0, 2, 32, 2, CHAR_SP);
		oled_char(0, 4, 32, 2, CHAR_SP);
	} else if (hr<HR_MIN || hr>HR_MAX) {
		oled_char(0, 0, 32, 2, CHAR_SP);
		oled_char(0, 2, 32, 2, CHAR_SP);
		oled_char(0, 4, 32, 2, CHAR_NA);
	} else {
		t=hr/10;
		hr-=t*10;
		oled_char(0, 4, 32, 2, CHAR_DIGI[hr]);
    if (t) {
			hr=t/10;
			t-=hr*10;
			oled_char(0, 2, 32, 2, CHAR_DIGI[t]);
			oled_char(0, 0, 32, 2, hr?CHAR_DIGI[hr]:CHAR_SP);
		} else {
			oled_char(0, 2, 32, 2, CHAR_SP);
			oled_char(0, 0, 32, 2, CHAR_SP);
		}
	}
}

// show O2 percent
void oled_o2()
{
	u8 t;
	u8 o2;
	
	o2=(u8)((g_o2+32)>>6);
	
	if (g_state!=ST_STABLE) {
		oled_char(32, 2, 32, 2, CHAR_SP);
		oled_char(32, 4, 32, 2, CHAR_SP);	
	} else if (o2<O2_MIN || o2>O2_MAX) {
		oled_char(32, 2, 32, 2, CHAR_SP);
		oled_char(32, 4, 32, 2, CHAR_NA);
	} else {
		t=o2/10;
		o2-=t*10;
		oled_char(32, 4, 32, 2, CHAR_DIGI[o2]);
		oled_char(32, 2, 32, 2, t?CHAR_DIGI[t]:CHAR_SP);
	}
}

// show battery
void oled_batt()
{
	u8 t;
	u8 i;
	
	if (g_batt>BATT_MIN) 
		t=(u8)((g_batt-BATT_MIN)/((BATT_MAX-BATT_MIN+16)/32));
	else
		t=0;
	
	for (i=2;i<6;i++) {
		oled_addr(78, i);
		if (t>7) {
			oled_set(BATT_VOLT[8], 8);
			t-=8;
		} else {
			oled_set(BATT_VOLT[t], 8);
			t=0;
		}
	}
}

// process sensor sample, if this return non-zero, caller needs to clear buffer
u8 on_sample(i32 red32, i32 ir32)
{
	static idata i16 red_ac,   ir_ac;    // RED/IR filtered AC
  static idata i16 red_maxd, ir_maxd;  // RED/IR AC MAX value decayed
  static idata i16 red_mind, ir_mind;  // RED/IR AC MIN value decayed
  static bit red_maxb, ir_maxb;  // RED/IR AC MAX value reached 
  static bit red_minb, ir_minb;  // RED/IR AC MAX value reached 
  static idata i16 red_maxk, ir_maxk;  // last keeped RED/IR AC MAX value
  static idata i16 red_mink, ir_mink;  // last keeped RED/IR AC MIN value
	static i16 pd_t;               // freq(heart beat)*t(sample) value of HR_MIN, I channel, and Q channel=I channel+16384 (90 defree)
  static i16 pd_f;               // value for addition when freq. steps higher

	
	i16 hb_ac;  // heart beat DC/AC, both of RED/IR

	i32 t;
	i16 d;
	i16 xdata *pi, *pq;
	u8 i;
	i16 pdi,pdq;
	
#ifdef SER_RAW
	ser_send('R');
	ser_send(':');
	ser_u32(g_samps);
	ser_send(','); 
	ser_u32(g_lost);
	ser_send(','); 
	ser_u32(red32);
	ser_send(',');
	ser_u32(ir32);
	ser_send('\r'); 
	ser_send('\n'); 
#endif
	
	if (g_state==ST_START) {
		g_samps=0;    
		g_lost=0;
		g_state=ST_PROX;
	}
		
	// start proximity check
	if (g_state==ST_PROX) {
		g_red_pa=0;
		g_ir_pa=PROX_PA;
		sen_cali();
		g_ir_dc=0;
		g_state=ST_PROX1;
		return 1;
	}
	
	// in proximity : to calibrate 
	if (g_state==ST_PROX1) {
		g_ir_dc += (ir32-g_ir_dc)>>4; // T(0.63):0.16s
		if ((u8)(g_ir_dc>>10)<PROX_TH) // wait for enough signal
			return 0;

		g_cali_aim = CALI_AIM;
		g_state=ST_CALI;
	}
	
	// calibration start
	if (g_state==ST_CALI) {
		g_red_pa=16; 
		g_ir_pa=16;
		sen_cali();
		g_state=ST_CALI1;
		return 1;
	}

	// wait for a while for calibration to be stable
	if (g_state==ST_CALI1) {
		g_red_dc=red32; 
		g_ir_dc=ir32;
		g_staux=50;
		g_state=ST_CALI2;
		return 0;
	}
	
	if (g_state==ST_CALI2) {
		g_red_dc += (red32-g_red_dc)>>4; // F(-3db):1Hz
		g_ir_dc += (ir32-g_ir_dc)>>4;    // F(-3db):1Hz

		if (--g_staux) 
			return 0;
		
		t=g_cali_aim*(u32)g_red_pa/(g_red_dc+1000); // avoid overflow
		if (t<4L) t=4L;
		if (t>127L) t=127L; // limited for power saving
		g_red_pa=(u8)t;
		
		t=g_cali_aim*(u32)g_ir_pa/(g_ir_dc+1000); // avoid overflow
		if (t<4L) t=4L;
		if (t>127L) t=127L; // limited for power saving
		g_ir_pa=(u8)t;
		
		sen_cali();
		g_state=ST_FIL;
		return 1;			
	}
	
	// following need to check proximity
	if (red32<(g_cali_aim>>2) || ir32<(g_cali_aim>>2)) {
		g_state=ST_PROX;
		return 1;
	}	
	
	// following need to check overflow
	if (red32>262000L || ir32>262000L) { // nearly 2^18
		g_cali_aim >>=1; // try to reduce calibration-aim
		g_state=ST_CALI;
		return 1;
	}	
	
	// start to do some signal filter
	if (g_state==ST_FIL) {
		g_red_dc=red32;
		g_ir_dc=ir32;
		red_ac=0; 
		ir_ac=0;

		g_staux=50;
		g_state=ST_FIL1;
		return 0;
	}

	// peek DC/AC, with simple noise cut-off
	red32 -= g_red_dc;
	g_red_dc += red32>>6;  // F(-3db):15/minute, T(0.63):0.64s
	if (red32>32767L) red32=32767L;    // appropriate limit
	else if (red32<-32767L) red32=-32767L; // appropriate limit
	red_ac = ((i16)red32+red_ac)>>1; // F(-3db): 477/minute 
	
	ir32-=g_ir_dc;
	g_ir_dc += ir32>>6;  // F(-3db):15/minute, T(0.63):0.64s
	if (ir32>32767L) ir32=32767L;    // appropriate limit
	else if (ir32<-32767L) ir32=-32767L; // appropriate limit
	ir_ac = ((i16)ir32+ir_ac)>>1; // F(-3db): 477/minute 
	
#ifdef SER_FIL
	ser_send('F');
	ser_send(':');	
	ser_u32(g_red_dc);
	ser_send(','); 
	ser_u16(red_ac);
	ser_send(','); 
	ser_u32(g_ir_dc);
	ser_send(',');
	ser_u16(ir_ac);
	ser_send('\r'); 
	ser_send('\n'); 
#endif	
	
	// wait for filter stable, then start peek MAX/MIN of AC values
	if (g_state==ST_FIL1) {
		if (--g_staux)
			return 0;

		g_red_max=0;
		g_red_min=0;
		red_maxd=0;
		red_mind=0;
		
		g_ir_max=0;
		g_ir_min=0;
		ir_maxd=0;
		ir_mind=0;	
		
		red_maxb=0;
		red_minb=0;
		ir_maxb=0;
		ir_minb=0;
		
		red_maxk=0;
		red_mink=0;
		ir_maxk=0;
		ir_mink=0;
		
		g_staux=50; 
		g_state=ST_FIL2;
		return 0;
	}			

	// calculate new max/min, with old max/min parameters decay
	if (red_ac>red_maxd) {
		red_maxd = red_ac;
		red_maxb = 1;
	} else if (red_maxb) {
		red_maxb=0;
		red_maxk=red_maxd;
	} else
		red_maxd += (red_mink - red_maxd)>>8; // T(0.63):2.56s
	g_red_max += (red_maxk - g_red_max)>>5; // F(-3db):30/minute

	
	if (red_ac<red_mind) {
		red_mind = red_ac;
		red_minb = 1;
	} else if (red_minb) {
		red_minb=0;
		red_mink=red_mind;		
	} else
		red_mind += (red_maxk - red_mind)>>8; // T(0.63):2.56s	
	g_red_min += (red_mink - g_red_min)>>5; // F(-3db):30/minute
	
	if (ir_ac>ir_maxd) {
		ir_maxd = ir_ac;
		ir_maxb = 1;
	} else if (ir_maxb) {
		ir_maxb=0;
		ir_maxk=ir_maxd;
	} else
		ir_maxd += (ir_mink - ir_maxd)>>8; // T(0.63):2.56s
	g_ir_max += (ir_maxk - g_ir_max)>>5; // F(-3db):30/minute

	
	if (ir_ac<ir_mind) {
		ir_mind = ir_ac;
		ir_minb = 1;
	} else if (ir_minb) {
		ir_minb=0;
		ir_mink=ir_mind;		
	} else
		ir_mind += (ir_maxk - ir_mind)>>8; // T(0.63):2.56s	
	g_ir_min += (ir_mink - g_ir_min)>>5; // F(-3db):30/minute

	
#ifdef SER_PEAK
	ser_send('P');
	ser_send(':');
	ser_u32(g_red_dc);
	ser_send(',');	
	ser_u16(g_red_max);
	ser_send(',');	
	ser_u16(g_red_min);
	ser_send(',');	
	ser_u32(g_ir_dc);
	ser_send(',');	
	ser_u16(g_ir_max);
	ser_send(',');
	ser_u16(g_ir_min);
	ser_send('\r'); 
	ser_send('\n'); 
#endif	
	
	// wait for filter stable
	if (g_state==ST_FIL2) {
		if (--g_staux)
			return 0;

		g_state=ST_WORK;
	}		

	// prepare to calculate heart beat rate
	if (g_state==ST_WORK) {
			// clear I/Q value, reset wave generator
		for (i=0; i<(HR_MAX-HR_MIN+1); i++) {
			g_hr_i[i]=0;
			g_hr_q[i]=0;
		}
		pd_t=0;  
		pd_f=0;
		
		g_staux=150;
		g_state=ST_WORK1;
	}
	
	// get heart beat AC
	hb_ac=(i16)(((i32)red_ac+(i32)ir_ac)*220L/((i32)g_red_max+(i32)g_ir_max-(i32)g_red_min-(i32)g_ir_min));
	if (hb_ac>127L)
		hb_ac=127L;
	else if (hb_ac<-127L)
		hb_ac=-127L;
	
#ifdef SER_HBEAT
	ser_send('H');
	ser_send(':');	
	ser_u16(hb_ac);
	ser_send('\r'); 
	ser_send('\n'); 	
#endif
	
	// do phase-detect
	pdi=pd_t;
  pdq=pd_t+16384; // 90 degree shift
	pi=g_hr_i;
	pq=g_hr_q;

#ifdef USE_COS
  MD5=((u16s*)&hb_ac)->h;
  MD4=((u16s*)&hb_ac)->l;
#endif		

	for (i=0; i<(HR_MAX-HR_MIN+1); i++) {

#ifdef USE_COS
		d=COS_LUT[((u16s*)&pdi)->h];
		MD1=((u16s*)&d)->h;
		MD0=((u16s*)&d)->l;
 	  ARCON = 0x80; // 16bit * 16bit		
		OPCON = 1; // start
#endif

		d=*pi>>8;

#ifdef USE_COS
		while((OPCON & 0x01));		
		*pi+=(i16)(i8)MD1 - d;
#else		
		if (pdi<0)
			*pi-=hb_ac+d;
		else 
			*pi+=hb_ac-d;
#endif
		
		pi++;
		
#ifdef USE_COS
		d=COS_LUT[((u16s*)&pdq)->h];
		MD1=((u16s*)&d)->h;
		MD0=((u16s*)&d)->l;
 	  ARCON = 0x80; // 16bit * 16bit		
		OPCON = 1; // start
#endif

		d=*pq>>8;
		
#ifdef USE_COS
		while((OPCON & 0x01)); 	
		*pq+=(i16)(i8)MD1 - d;
#else		
		if (pdq<0)
			*pq-=hb_ac+d;
		else
			*pq+=hb_ac-d;
#endif
		
		pq++;
		
		pdi+=pd_f;
		pdq+=pd_f;		
	}

	pd_t+=PD_TADD;
	pd_f+=PD_FADD;

	// wait for all stable
	if (g_state==ST_WORK1) {
		if (--g_staux) 
			return 0;
		
		g_hr=0;
		g_o2=0;
		g_state=ST_STABLE;
	}		

	return 0;
}

void on_sensor()
{
	u8 n,t;
	i32 red, ir;
	u8 xdata * p;
	static bit drop=1;
	
	sen_get(0x00);                  // clear intr.
	sen_read(0x04, g_buff, 3);      // read WR,OVF,RD 
  n=(g_buff[0]-g_buff[2])&0x1f;   // num of FIFO to read
	t=g_buff[1]&0x1f;               // lost samples
	g_lost+=t;
	if (n==0 && t) n=32;
	
	while (n) {
		t=n;
		if (t>SEN_BUFFSIZE) t=SEN_BUFFSIZE;
		n-=t;
		sen_read(0x07, g_buff, t*6); // read samples
		p=g_buff;
		if (drop) 
			drop=0;
		else do {
			((u32s *)&ir)->hh=0;
			((u32s *)&ir)->hl=*p++ & 0x03;
			((u32s *)&ir)->lh=*p++;
			((u32s *)&ir)->ll=*p++;
			
			((u32s *)&red)->hh=0;
			((u32s *)&red)->hl=*p++ & 0x03;
			((u32s *)&red)->lh=*p++;
			((u32s *)&red)->ll=*p++;

			g_samps++;
			if (on_sample(red, ir)) { // need to drop current samples in buffer
				sen_put(0x06, 0x00);  // clear RD_PTR
				sen_put(0x05, 0x00);  // clear OVF_PTR
				sen_put(0x04, 0x00);  // clear WR_PTR
				sen_get(0x0);	        // clear intr.
				drop=1; // drop some old buffer data
				return;
			}
		} while (--t);
	}
}

void calc_o2() 
{
	i32 r;
	
	if (g_state!=ST_STABLE) // not stable
		return;
	
	r=((i32)g_ir_max-(i32)g_ir_min)*(g_red_dc>>6);  // at most 18+12=30 bits
	if (r<100L) return; // something wrong
	r=((i32)g_red_max-(i32)g_red_min)*(g_ir_dc>>6)/(r>>6);  // at most 18+12=30 bits
	if (r<0L || r>=sizeof(O2_LUT)) return; // something wrong
	
	if (g_o2) {
		g_o2+=(O2_LUT[r]-g_o2)>>2; // T(0.63):4 points
	} else  //first time
		g_o2=O2_LUT[r];

	if (g_o2>(O2_MAX<<6))
		g_o2=O2_MAX<<6;
}

void calc_hr() 
{
	u8  p, i;
	i16 t;
	u16 v, w;
	i16 xdata *pi, *pq;
	
	if (g_state!=ST_STABLE) // not stable
		return;
	
#ifdef SER_SPECTRUM		
	ser_send('S');
	ser_send(':');
#endif	

	p=0;
	v=0;
  pi=g_hr_i;
	pq=g_hr_q;
	for (i=0; i<(HR_MAX-HR_MIN+1); i++) {
		t=(*(pi++)>>8);
		w=t*t;
		t=(*(pq++)>>8);
		w+=t*t;
#ifdef SER_SPECTRUM		
		ser_u16(w);
		if (i<HR_MAX-HR_MIN)
			ser_send(',');
#endif
		if (w>v) {
			v=w;
			p=i;
		}
	}
	
#ifdef SER_SPECTRUM		
	ser_send('\r'); 
	ser_send('\n'); 
#endif

	if (v<100) return; // power is not enough
		
	p+=HR_MIN;
	
	if (g_hr) { // not infirst time
		g_hr-=g_hr>>2; // T(0.63):4 points
		g_hr+=(u16)p<<4;
	} else
		g_hr=(u16)p<<6;
}

void on_timer()
{
	u8 t;
	
	t=(u8)g_timer & 0x1f; // every 0.5s
	
	if (t==0) {
		batt_get();
		oled_batt();
	}	else if (t==0x06){
		calc_hr();
		oled_hr();
  }	else if (t==0x0B){
		calc_o2();
		oled_o2();
	}	else if (t==0x12) {
		on_sensor();   // for avoiding sampling lost
	} else if (t==0x18) {
		if (!P32)
			IAP_CONTR = 0x60; // reboot to download mode
	}
#ifdef SER_OUT
	else if (t==0x1C) { // send serial result
		if (g_state==ST_STABLE) {
			ser_send('O');
			ser_send(':');			
			ser_u32(g_samps);
			ser_send(','); 
			ser_u32(g_lost);
			ser_send(','); 
			ser_u16(g_batt);
			ser_send(','); 
			ser_u8((u8)((g_hr+32)>>6));
			ser_send(','); 
			ser_u8((u8)((g_o2+32)>>6));
			ser_send('\r'); 
			ser_send('\n');
		}			
	}
#endif	
}

// prog entry
void main()
{
	EA=0;  // disable intr. no need in this app
	P3M0 = 0x00;
	P3M1 = 0x08; // P3.3 is now Hign-Z, for reading battery voltage, with external pull-up
	P5M0 = 0x00;
	P5M1 = 0x00;
	
	// wait for system statble 
	delay_ms(50); 
	
	// check if need download
	if (!P32)
		IAP_CONTR = 0x60; // reboot to download mode   
	
	// init I2C
	P_SW2 = 0x90; // EAXFR (in fact, even this is set, Xdata is also accessible), SCL=>P5.4, SDA=>P5.5
	P5PU =  0x30;  // P5.4 P5.5 pull-up for I2C
	I2CMSCR = 0 ;  // no command now
	I2CMSAUX = 0;  // no auto-send
	I2CCFG = 0xc0 | (0x3f & (((FOSC/2/IIC_FREQ)-4)/2));  // enabled as master, bus speed=Fosc/2/(x*2+4) 
	I2CMSST = 0x00; // clear status	
	
	// init ADC
	ADCCFG = 0x20;  // result is right-alignment, ADCclk=SYSclk/2/1
	ADCTIM = 0x3F;  // 1(setup)+32(duty)+2(hold)+10(convert)=45 ADCclk 
	ADC_CONTR = 0x8f;  // power on, stop, select internal Gap-Band channel  

	// reset OLED
	P32=0;
	delay_ms(1); 
	P32=1;
	delay_ms(1); 
	
	// init OLED
	oled_cmd(0xAE, OLED_NOP);   // display off first
  oled_cmd(0xD3, 0x38); 			// COM shift offset, -8%64=0x38 for this 88x48 OLED
  oled_cmd(0x40, OLED_NOP);   // set RAM start line address at 0
  oled_cmd(0xA8, 0x3F); 			// set multiplex ratio, default 64MUX
  oled_cmd(0xA0, OLED_NOP);   // COL0 mapped to SEG0
  oled_cmd(0xC8, OLED_NOP);   // COM scan direction, start from COM63
  oled_cmd(0xDA, 0x02); 			// COM pin hardware configuartion, default not-alternative and no remap
	oled_cmd(0x81, 0x7F); 			// contract control, default 127
  oled_cmd(0xA4, OLED_NOP);		// entire display off
	oled_cmd(0xA6, OLED_NOP); 	// set normal  display
  oled_cmd(0x2E, OLED_NOP); 	// stop scrolling  
  oled_cmd(0xD5, 0x80); 			// set display clock divide ratio/oscillator frequency, default 
  oled_cmd(0xD9, 0x22); 			// set pre-charge period, default 2/2
  oled_cmd(0xDB, 0x20); 			// set VCOMH, default 0.77 VCC
	oled_cmd(0x8B, OLED_NOP);   // set charge pump enable, for CH1115
  oled_cmd(0x8D, 0x14); 			// set charge pump enable, for SSD1306
  oled_cmd(0x20, 0x02); 			// set address mode, page addressing mode mode, for SSD1306
	oled_fill(0, 0, 88, 6, 0);  // clear screen
	oled_char(32, 0, 32, 2, CHAR_PCT);  // OLED show O2 percent icon
	oled_char(72, 0, 16, 2, CHAR_BATT);  // OLED show Battery icon
	oled_cmd(0xAF, OLED_NOP);        // display on
	
 	// init MAX30102
	sen_put(0x09, 0x0);   // Stop
	sen_get(0x00);        // clear intr. especially PWR_RDY
	sen_get(0x01);        // clear intr.
	sen_put(0x02, 0x80);  // use AFull intr.
	sen_put(0x03, 0x00);  // not use die_temp_rdy
	sen_put(0x08, 0x3F);  // AVE=2, EN_ROLL_OVER=1, UNREAD=17
	sen_put(0x10, PROX_PA);  // Proximity mode red LED current, but this app use software proximity checking
	sen_put(0x30, PROX_TH);  // Proximity thresh, but this app use software proximity checking
 	sen_put(0x0D, 0);     // IR current 0
	sen_put(0x0C, 0);     // LED current 0
	sen_put(0x0A, 0x4B);  // ADC range 8192nA, sample rate 200(AVG=2, so output 100), bits=18
  sen_put(0x04, 0x00);  // clear WR_PTR
	sen_put(0x05, 0x00);  // clear OVF_PTR
	sen_put(0x06, 0x00);  // clear RD_PTR
	
  // get once battery voltage
	batt_get();
	
	// init default show
  oled_hr();
  oled_o2();
	oled_batt();
	
  // a timer 
  TR0 = 0;  // not start now
  TMOD &= 0xF0;  // 16bit auto-reload for T0
	AUXR &= 0x7F;  // FOSC/12
  TL0 = (u8)(65536UL-FOSC/12/TIMER_FREQ); 
  TH0 = (u8)((65536UL-FOSC/12/TIMER_FREQ)>>8); 
  ET0 = 0;  // use polling mode

  // init serial 
	PCON &= 0x3F; // default
	SCON = 0x50;  // mode 1, can recv.
	TMOD &= 0x0F; // 16bit auto-reload for T1
	TL1 = (u8)(65536 - FOSC/BAUDRATE/4);
	TH1 = (u8)((65536 - FOSC/BAUDRATE/4) >> 8);
	AUXR |= 0x40; // FOSC/1
	AUXR &= 0xDE; // using T1
	ET1 = 0; // intr. is not needed
	TR1 = 1; // start T1
	ES = 0; // not using intr.

  // start sensor 
	sen_put(0x09, 0x03);  // SpO2 mode
	
	// start timer
  TR0 = 1;

  // start state
	g_state = ST_START;
 
  // main loop
	while(1) {
		if (!P33) { // sensor intr.
			on_sensor();
		}

		if (TF0){ // timer event
			TF0=0;
			on_timer();	
			g_timer++;
		}
		
		if (RI) { // serial input
			RI = 0;
			if (SBUF=='D') 
				IAP_CONTR = 0x60; // reboot to download mode   
    }
	}
} 

