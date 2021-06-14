#include "tm4c123gh6pm.h"
#include "Nokia5110.c" 
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define SYSDIV2         7  //bus frequency is 400MHz/(SYSDIV2+1) = 400MHz/(7+1) = 50 MHz
#define RELOAD 799999 //20Hz
//#define RELOAD 159999 //100Hz
#define move (*((volatile unsigned long *) 0x4000500C)) //PORTB1-0
#define PERIOD 			25000  //(50MHz/2)/1000Hz -> /2 divider used
#define FORWARD 0x01

// External functions for interrupt control defined in startup.s
extern void DisableInterrupts(void); //Disable interrupts
extern void EnableInterrupts(void);  //Enable interrupts
extern void WaitForInterrupt(void);  //low power mode


void PWM0_0_Init(unsigned long,unsigned long); //used to initialize PWM
void PortB_Init(void);
void PortF_Init(void);//initilizes port F and arm PF4, PF0 for falling edge interrupts
void debounce(void);//10ms debounce
void PLL_Init(void);
void ADC_Init(void);
void ADC_In298(unsigned long*,unsigned long*,unsigned long*);
void SysTick_Init(void);

void control(double,double);
//void lookupTable(
//double table[13] =      {2.26,1.60,1.25,1.024,.868,0.754,0.66,0.602,0.544,0.506,0.452,0.428,0.404};
unsigned int      t[13] = {2810,1985,1560,1277,1077,938,821,747,676,627,555,533,507};//ADC table values
unsigned int dtable[13] = {  10,  15,  20,  25,  30, 35, 40, 45, 50, 55, 60, 65, 70};//distances(cm)
unsigned int  delta[13];
unsigned int* del = delta;//(delta t)/2
unsigned int lookupTable(unsigned long);
unsigned int hdL,hdR; //horizontal distance left, horizontal distance right
unsigned int pS = 0;//power setting (pot)
unsigned int dp = 4096/100;//delta power -- represents the 1% power ADC value
unsigned int sampleFlag = 0;//average of samples taken
unsigned int avgL = 0;//average of sampled values LEFT SENSOR
unsigned long avgR = 0;//average of sampled values RIGHT SENSOR
unsigned long avgP = 0;//average of sampled values POTENTIOMETER
unsigned long count = 0;
unsigned long Ain2;//PE1 -- Left Sensor
unsigned long Ain9;//PE4 -- Right Sensor
unsigned long Ain8;//PE5 -- Potentiometer
unsigned long H = PERIOD;

unsigned long *leftSpeed = NULL;
unsigned long *rightSpeed = NULL;
unsigned long LS;
unsigned long RS;
unsigned long dutyL,dutyR;
unsigned long delay;

//Ain2=PE1 Ain9=PE4 Ain8=PE5

volatile unsigned long slowIt;//slow wheel turn
volatile unsigned long speedIt;//run at power setting
volatile unsigned long superSlow;//almost crash
int main(void){ unsigned long delay;
	delta[ 0]=(t[0]-t[1])/5;delta[1]=(t[1]-t[ 2])/5;delta[ 2]=(t[ 2]-t[ 3])/5;delta[ 3]=(t[ 3]-t[ 4])/5;
	delta[ 4]=(t[4]-t[5])/5;delta[5]=(t[5]-t[ 6])/5;delta[ 6]=(t[ 6]-t[ 7])/5,delta[ 7]=(t[ 7]-t[ 8])/5;
	delta[ 8]=(t[8]-t[9])/5;delta[9]=(t[9]-t[10])/5;delta[10]=(t[10]-t[11])/5;delta[11]=(t[11]-t[12])/5;
	delta[12]=delta[11];
	
	DisableInterrupts();  //disable interrupts to allow initializations

	PLL_Init();
	SysTick_Init();
	PortF_Init();
	PortB_Init();
	PWM0_0_Init(dutyL,dutyR); //output PWM signal from PF3	
	ADC_Init();
	
	EnableInterrupts();
	
	Nokia5110_Init();
  Nokia5110_Clear();
  Nokia5110_OutString("************CECS347 LAB4************");
	Nokia5110_OutString("hdL         ");//Table DistanceValue
	Nokia5110_OutString("hdR         ");//ADC Value
	Nokia5110_OutString("pS          ");//Calibrated Distance
	
	move = FORWARD;
	volatile double a,bb,c;
	
	while(1) {
		//WaitForInterrupt();
		//dutyL = 20000; dutyR = 5000;
		//PWM0_0_Init(dutyL,dutyR);
		//PWM0_0_Init(H);

		if(sampleFlag){
			//DisableInterrupts();
			avgL = avgL/count;
			avgR = avgR/count;
			avgP = avgP/count;
			count = 0;
			sampleFlag = 0;
			Nokia5110_SetCursor(4,3);
			
			a = -3.4586; 
			bb = 36309.4743; 
			c = 0.7071;
	//		hdL = floor(a+((bb*avgL)*c));
	//		hdR = floor(a+((bb*avgR)*c));
			//hdR = (-3.4586047 + 36309.4743/avgR)*0.7071;// dR*cos(45) = hdR
			pS = avgP/dp;//power setting percentage
			control((a+(bb/avgL))*c,(a+(bb/avgR))*c); //Keil was giving issues using 'double' for some reason.
																								//Assigning values to variables first was a work-around.
			
			//Display distances on both sides along with power setting in percentage form
			Nokia5110_OutUDec((a+(bb/avgL))*c);
			Nokia5110_SetCursor(4,4);
			Nokia5110_OutUDec((a+(bb/avgR))*c);
			Nokia5110_SetCursor(4,5);
			Nokia5110_OutUDec(pS);
			avgL = 0; //value used in assigning analog value to left  wheel power setting
			avgR = 0; //value used in assigning analog value to right wheel power setting
			avgP = 0; //value used in assigning analog power setting value
			//EnableInterrupts();
		}
	}
}


void ADC_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; //activate clock for port E
	delay = SYSCTL_RCGCGPIO_R; //allow clock to stabilize
	GPIO_PORTE_DIR_R &= ~0x32; // 3) make PE1, PE4, and PE5 input
	GPIO_PORTE_AFSEL_R |= 0x32; // 4) enable alternate function on PE1, 4, 5
	GPIO_PORTE_DEN_R &= ~0x32; // 5) disable digital I/O on PE1, PE4, & PE5
	GPIO_PORTE_AMSEL_R |= 0x32; // 6) enable analog functionality on PE1,4, 5

	SYSCTL_RCGCADC_R |= 0x00000001; //activate ADC0
	SYSCTL_RCGC0_R |= 0x00010000;
	SYSCTL_RCGC0_R &= ~0x00000300;
	
	ADC0_PC_R &= ~0xF; // 8) clear max sample rate field------------------------------------------------hard fault -- clock issue?
	ADC0_PC_R |= 0x1; // configure for 125K samples/sec
	ADC0_SSPRI_R = 0x3210; // 9) Sequencer 3 is lowest priority
	ADC0_ACTSS_R &= ~0x0004; // 10) disable sample sequencer 2
	ADC0_EMUX_R &= ~0x0F00; // 11) seq2 is software trigger (event multiplexer)
	//ADC0_SSMUX2_R = 0x0892; // 12) set channels for SS2
	ADC0_SSMUX2_R = (ADC0_SSMUX2_R&0xFFFFF000) + (8<<8) + (9<<4) + 2;
	ADC0_SSCTL2_R = 0x0600; // 13) no D0 END0 IE0 TS0 D1 END1 IE1 TS1 D2 TS2, yes END2 IE2
	//ADC0_IM_R &= ~0x0004; // 14) disable SS2 interrupts
	ADC0_ACTSS_R |= 0x0004; // 15) enable sample sequencer 2
}


void ADC_In298(unsigned long *ain2,unsigned long *ain9,unsigned long *ain8){
  ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0);   // 2) wait for conversion done
  *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result
  *ain9 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result
  *ain8 = ADC0_SSFIFO2_R&0xFFF;    // 3C) read third result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}


void PLL_Init(void){
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;   //Step 0:  Allow usage of RCC2 for advanced clocking features
	SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;   //Step 1:  BYPASS PLL... clock divider not in use
	SYSCTL_RCC_R  &= ~SYSCTL_RCC_XTAL_M;    //Step 2:  clear XTAL field
  SYSCTL_RCC_R  += SYSCTL_RCC_XTAL_16MHZ; //         configure XTAL for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;//         clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;//         configure for main oscillator source
	SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;   //Step 3:  clear PWRDN2 bit to activate PLL
	SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;    //         use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) //Step 4: clear system clock divider field
                  + (SYSDIV2<<22);        //            configure for 50 MHz clock
	while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0);//Step 5: wait for the PLL to lock by polling PLLLRIS
	SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;  //Step 6: Clear BYPASS
}


void SysTick_Init(void){ //PortE
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = RELOAD;       // reload value for 50% duty cycle
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF1F)|0x00000040; // bit 23-21 for SysTick PortE, set priority to 2
  NVIC_ST_CTRL_R |= 0x00000007;  // enable with core clock and interrupts, start systick timer
}

void SysTick_Handler(void){
	NVIC_ST_CTRL_R &= ~0x00000001;// turn off SysTick to reset reload value
  NVIC_ST_RELOAD_R = RELOAD;     // reload value for high phase
	NVIC_ST_CURRENT_R = 0;
	
	ADC_In298(&Ain2,&Ain9,&Ain8);
	avgL = avgL+Ain2;
	avgR = avgR+Ain9;
	avgP = avgP+Ain8;
	++count;
	if(count == 10) sampleFlag = 1;

	NVIC_ST_CTRL_R |= 0x00000007; // turn on systick to continue
}
//////////////////////////


void GPIOPortF_Handler(void){ //called on touch of either SW1 or SW2
	debounce();
  if(GPIO_PORTF_RIS_R&0x01){  //SW2 touched 
    GPIO_PORTF_ICR_R = 0x01;  //acknowledge flag0
		H = 0;
  }
  if(GPIO_PORTF_RIS_R&0x10){  //SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  //acknowledge flag4
		H = PERIOD*pS/100;
  }

	 //PWM signal starts low then goes high when CMPB is reached (count-down mode)
}

void PortF_Init(void){
	
	SYSCTL_RCGC2_R |= 0x00000020;//turn on PortF clock
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B; //  unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;         //  allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    //(c) make PF4,0 inputs (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //    disable alt funct on PF4-0
  GPIO_PORTF_DEN_R |= 0x11;     //    enable digital I/O on PF4,0
	GPIO_PORTF_PCTL_R &= ~0x000F000F; //sw1 and sw2 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //    disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //    enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     //(d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //    PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //    PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      //(e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      //(f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF1FFFFF)|0x00200000; //(g) bits:23-21 for PORTF, set priority to 1
  NVIC_EN0_R = 0x40000000;      //(h) enable interrupt 30 in NVIC
}

void PortB_Init(void){	//Wheel Turning (PB1-0) -- Wheel Power (PB7-6)
	SYSCTL_RCGC2_R |= 0x00000002;//turn on PortB clock
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&(~0xFF000000))|0x44000000;//enable M0PWM1-0 on pins PB7-6
	GPIO_PORTB_DIR_R   |= 0x03;  //PB1-0 control wheels turning (output signals)
	GPIO_PORTB_AFSEL_R &= ~0x03; //disable alt funct on PB1-0
	GPIO_PORTB_DEN_R   |= 0x03;  //enable digital I/O on PB1-0 for wheel turning directions
	GPIO_PORTB_AFSEL_R |= 0xC0; //enable PB7-6 as alternate function output (power wheels)
	GPIO_PORTB_DEN_R |= 0xC0; //enable digital I/O on PB7-6 for PWM output to wheels (power wheels)
	GPIO_PORTB_PCTL_R  &= ~0x000000FF; //PB1-0 as GPIO
  GPIO_PORTB_AMSEL_R &= ~0x11; //disable analog functionality on PB1-0

}
	

// Initialize PB7-6 to output 8MHz PWM signal
void PWM0_0_Init(unsigned long dutyLeft,unsigned long dutyRight){//PB7-6
	SYSCTL_RCGCPWM_R |= 0X00000001; //enable PWM module 0 clock source
	delay = SYSCTL_RCGCPWM_R;
	//while((SYSCTL_PRGPIO_R&0x20)==0);
	SYSCTL_RCC_R  = (SYSCTL_RCC_R&(~0x000E0000))|0x00100000; //clear bits 17-19 of PWMDIV for /2 divisor and enable USEPWMDIV (bit 20)
	PWM0_0_CTL_R  = 0; //count-down mode
	PWM0_0_GENB_R = 0xC08; //low on LOAD, high on CMPB down
	PWM0_0_GENA_R = 0xC8;  //low on LOAD, high on CMPA down
	PWM0_0_LOAD_R = PERIOD - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPB_R = dutyLeft - 1;             // 6) count value when output rises
	PWM0_0_CMPA_R = dutyRight - 1;             // 6) count value when output rises
	PWM0_0_CTL_R |= 0x00000001; //enable PWM module 0
	PWM0_ENABLE_R |= 0x00000003;          //enable PWM0/M0PWM1-0
}

//debounce 20ms
void debounce(void){
	unsigned long i,j;
	for(i = 0; i < 2500; ++i)
		for(j=0;j<400;++j);
}

// Here is where you can finetune the turning
void control(double LL,double RR){ 
	slowIt = (H*pS/130);//slow wheel
	speedIt = (H*pS/100);//run at power setting
	superSlow = (H*pS/180);
	LS = LL; //distance from left wall
	RS = RR; //distance from right wall	

	if((RR < 15)&&(LL < 15)){
		if(LL < RR) PWM0_0_Init(superSlow,1);//closer to the left wall
		else        PWM0_0_Init(1,superSlow);//closer to right wall
	}
	else if((LL>70)&&(RR>70))
		PWM0_0_Init(1,1);
	else if(LL < 20)
		PWM0_0_Init(speedIt,superSlow-1000);
	else if(RR < 20)
		PWM0_0_Init(superSlow-1000,speedIt);
	else if(LL > RR){
		if((LL-RR) < 20)
			PWM0_0_Init(speedIt,speedIt);
		else
			PWM0_0_Init(slowIt,speedIt);
	}
	else if(RR > LL){
		if((RR-LL) < 20)
			PWM0_0_Init(speedIt,speedIt);
		else
			PWM0_0_Init(speedIt,slowIt);
	}
	else
		PWM0_0_Init(speedIt,speedIt);
}




