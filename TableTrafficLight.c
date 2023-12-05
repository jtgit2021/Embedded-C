// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts


void Port_Init(void){ volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000032;     // 1) activate clock for Port F, E and B
	delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	//port F , pf1 pf3 "dont walk" output lights
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F , hardcoded value to unlock? p684
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF3 and PF1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x0;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital I/O on PF4-0	
	//port E
  GPIO_PORTE_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port E , hardcoded value to unlock? p684
  GPIO_PORTE_CR_R = 0x07;           // allow changes to PE0-2
  GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog on PE
  GPIO_PORTE_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PFE-0
  GPIO_PORTE_DIR_R &= ~0x7;          // 5) 1 = output , PE0-2 output
  GPIO_PORTE_AFSEL_R = 0x00;        // 6) disable alt funct on PE7-0
  //GPIO_PORTE_PUR_R = 0x07;          // enable pull-up on PE0-2
  GPIO_PORTE_DEN_R |= 0x07;          // 7) enable digital I/O on PE2-0
	//port B
  GPIO_PORTB_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port B , hardcoded value to unlock? p684
  GPIO_PORTB_CR_R = 0x3F;           // allow changes to PB0-5
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog on PB
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTB_DIR_R = 0x3F;          // 5) PB0-5 output
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) disable alt funct on PB7-0
  GPIO_PORTB_PUR_R = 0x00;          // 
  GPIO_PORTB_DEN_R = 0x3F;          // 7) enable digital I/O on PB5-0	
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}

//FSM
typedef const struct State SType;

struct State{
	unsigned long PBOut;
	unsigned long PFOut;
	//add output for each port, then call ~ GPIOPORT = FSM[state].PBOut
	unsigned long Time; //delay between states?	milliseconds?
	unsigned long Next[8]; // next state for inputs
};

unsigned long Input;

 // state name
#define AllRedG			0
#define Walk				1
#define WalkFlash1a	2
#define WalkFlash1b	3
#define WalkFlash2a	4
#define WalkFlash2b	5
#define SouthStartY	6
#define SouthG			7
#define SouthStopY	8
#define WestStartY	9
#define WestG				10
#define WestStopY		11
#define AllRedW			12
#define AllRedS			13
//  algorithm -> AllRedG prioritizes Pedestian signal, AllRedW -> West, AllRedS -> South , 

// round robin -> G , S , W
SType FSM[14]={
 {0x24,0x02, 50,{ AllRedG , WestG , SouthG , SouthG , Walk , Walk , Walk , Walk }  } , // one state - {output B, out F, time, nextstate{all possible paths} }
 //walk
 {0x24,0x08, 70,{Walk, WalkFlash1a, WalkFlash1a, WalkFlash1a, Walk,WalkFlash1a,WalkFlash1a,WalkFlash1a } },
 {0x24,0x0, 50,{WalkFlash1b, WalkFlash1b, WalkFlash1b, WalkFlash1b, WalkFlash1b, WalkFlash1b, WalkFlash1b, WalkFlash1b } },
 {0x24,0x8, 50,{WalkFlash2a, WalkFlash2a, WalkFlash2a, WalkFlash2a, WalkFlash2a, WalkFlash2a, WalkFlash2a, WalkFlash2a } },
 {0x24,0x0, 50,{WalkFlash2b, WalkFlash2b, WalkFlash2b, WalkFlash2b, WalkFlash2b, WalkFlash2b, WalkFlash2b, WalkFlash2b} },
 {0x24,0x8, 50,{AllRedG ,AllRedG ,AllRedG ,AllRedG ,AllRedG ,AllRedW ,AllRedS ,AllRedS } },
 //south
 {0x22,0x2, 50,{SouthG,SouthG,SouthG,SouthG,SouthG,SouthG,SouthG,SouthG,} }, // skipped
 {0x21,0x2, 70,{SouthG,SouthStopY,SouthG,SouthStopY ,SouthStopY,SouthStopY,SouthStopY,SouthStopY} },
 {0x22,0x2, 50,{AllRedG,AllRedW,AllRedS,AllRedW,AllRedG,AllRedG,AllRedG,AllRedW} },
 //west
 {0x14,0x2, 50,{WestG,WestG,WestG,WestG, WestG,WestG,WestG,WestG} },  // skipped
 {0xC,0x2, 70,{WestG,WestG,WestStopY,WestStopY, WestStopY,WestStopY,WestStopY,WestStopY} },
 {0x14,0x2, 50,{AllRedG,AllRedW,AllRedS,AllRedS,AllRedG,AllRedG,AllRedG,AllRedG} },// change output then use it to set dir
 
 {0x24,0x02, 50,{ AllRedW , WestG , SouthG , WestG , Walk , WestG, WestG , WestG }  } ,
 {0x24,0x02, 50, { AllRedS , WestG , SouthG , SouthG , Walk , SouthG , SouthG , SouthG }  } ,
 } ;
 
 
unsigned char cState; // currentstate

void SysTick_Wait10ms(unsigned long delay);

// ***** 3. Subroutines Section *****

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	SysTick_Init();
	Port_Init();
	
	cState = 0; //Initial State
  
  EnableInterrupts();
  while(1){
     //output
		GPIO_PORTB_DATA_R = FSM[cState].PBOut;  
		GPIO_PORTF_DATA_R = FSM[cState].PFOut;  
		
		SysTick_Wait10ms(FSM[cState].Time);  
		
		Input = GPIO_PORTE_DATA_R & 0x7; 
		
		cState = FSM[cState].Next[Input]; 
  }
}

void SysTick_Wait(unsigned long delay){
	NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }  
}

void SysTick_Wait10ms(unsigned long delay){
	unsigned long i;
	for(i=0 ; i<delay ; i++){
		SysTick_Wait(800000);
	}
}

