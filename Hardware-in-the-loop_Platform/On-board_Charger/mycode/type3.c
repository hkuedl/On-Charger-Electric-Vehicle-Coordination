

#include "type3.h"

CCMRAM __IO uint16_t   V_set=0x00;
CCMRAM __IO float      Vosc;                
CCMRAM __IO float      Xn,Xn_1,Xn_2,Xn_3;   
CCMRAM __IO float      Yn,Yn_1,Yn_2,Yn_3;   
CCMRAM __IO float      Duty;								    
CCMRAM __IO uint16_t Pulse_with_type3p3z=100;
CCMRAM __IO float BUCK_PWM_PERIOD=27472.0f;

CCMRAM __IO float k1=-0.415017720023f;            
CCMRAM __IO float k2=0.37445855483f;
CCMRAM __IO float k3=0.414919275447f;
CCMRAM __IO float k4=-0.37455699940f;
CCMRAM __IO float k5=1.113014372908f;
CCMRAM __IO float k6=-0.039574719432f;
CCMRAM __IO float k7=-0.073439653475f;

CCMRAM void type_3_int(void)
{

 Xn=0;         
 Xn_1=0;       
 Xn_2=0;       
 Xn_3=0;       
	
 Yn=0;         
 Yn_1=0;       
 Yn_2=0;       
 Yn_3=0;       

 Duty=0.0f;
 
 Vosc  =3.0f;
	
}

CCMRAM void  type_3_cal(uint16_t Vout,uint16_t Vref)
{
	 Xn=(float)(Vout-Vref)*0.001f;          
	 Yn=k1*Xn+k2*Xn_1+k3*Xn_2+k4*Xn_3+k5*Yn_1+k6*Yn_2+k7*Yn_3; 

	 if(Yn>=Vosc){Yn=Vosc-0.3f;}					     
	 else if(Yn<=0.05f){Yn=0.05f;}              
	 Duty=Yn*0.333333333f;	    								 
	 
	 Xn_3=Xn_2;Xn_2=Xn_1;Xn_1=Xn;							 
	 Yn_3=Yn_2;Yn_2=Yn_1;Yn_1=Yn;	

	 Pulse_with_type3p3z=Duty*PWM_PERIOD;
	 if(Pulse_with_type3p3z<=100)Pulse_with_type3p3z=100;
	 
	 HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = Pulse_with_type3p3z;  
	 HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = Pulse_with_type3p3z>>1;  
	 
	 HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = Pulse_with_type3p3z;
	 HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP3xR = Pulse_with_type3p3z>>1;
}

void type_3_tustin(void)
{
	double R1,R2,R3,C1,C2,C3,Ts;
	double a1,a2,a3,a4,a5,k0,b1,b2,b3,b4,b5;
	double Gain_adj;

	R1=2.6*Kom;C1=10.0*pF;
	R2=10.0*Kom;C2=4.7*nF;
	R3=10.0*om;C3=33*nF;

  Ts=5.0*uS ;
	
  Gain_adj=300.0f;
	
	a1=R1*C1/Ts;
	a2=R1*C2/Ts;
	a3=R1*C3/Ts;
	a4=R2*C2/Ts;
  a5=R3*C3/Ts;
	
	b1=(a3*a4+a4*a5)*4;
	b2=(a4+a3+a5)*2;
	b3=(a1*a4+a1*a5+a2*a5)*4;
	b4=(a1*a4*a5)*8;
	b5=(a1+a2)*2;
	
	k0=(b5+b3+b4);
	k1=(-1-b2-b1)/k0;
	k2=(-3-b2+b1)/k0;
	k3=(-3+b2+b1)/k0;
	k4=(-1+b2-b1)/k0;
	k5=-(b5-b3-3*b4)/k0;
	k6=-(-b5-b3+3*b4)/k0;
	k7=-(-b5+b3-b4)/k0;
	
	k1=k1/Gain_adj;
  k2=k2/Gain_adj;
  k3=k3/Gain_adj;
  k4=k4/Gain_adj;
}

