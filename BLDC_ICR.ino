#include <avr/io.h>
#include <avr/interrupt.h>

#define average 100

volatile uint16_t T10vs1, T10vs2;           //Counts overflow
volatile uint8_t Capt1_low, Capt1_high, Capt2_low, Capt2_high;  //Timestamps
volatile uint16_t Capt1, Capt2;
volatile uint8_t Flag;                      //Capture flag
uint16_t PWM_AVG;
uint16_t data;

void sensor_read();

void setup()
{
  //Initialize 16 bit Timer 1
  noInterrupts();
  TCNT1 = 0;                //Set initial timer value to zero
  TCCR1B = 0x41;  //First capture on rising edge
  TIMSK1 =  0x20;

  TCCR1A = 0x00; // normal operation page 148 (mode0);
  
  interrupts();
  
  Serial.begin(9600);

  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
  analogWrite(5,10); //976.5625Hz, with high pulses of ~1/976.5625 x 10/256 = 40us; Connect pin 5 to INPUT_PIN and open serial monitor & you will see approximately this
  analogWrite(9,128); //490.20Hz, with high pulses of ~1/490.2 x 128/256 = ~1020us; Connect pin 9 to INPUT_PIN and open serial monitor & you will see approximately this
  /*
  PWM Notes:
  -PWM on pins 5 & 6 occurs at 976.5625Hz; see here: http://playground.arduino.cc/Main/TimerPWMCheatsheet
  -PWM on pins 9 & 10 occurs at 490.20Hz  
  */
}

void loop() 
{
  float Position;
  float UpTime;
  int EncoderPulse, FixedPulse;
  volatile uint16_t UpCount;
  volatile uint16_t DownTime;
  volatile uint8_t DutyCycle;     //dutycycle result holder
  
  while(1){
    
    PWM_AVG = 0;
    
    sensor_read();

     PWM_AVG = PWM_AVG / average;

     //Normalize
     Position = map(float(FixedPulse), 0, 4095, 0, 359);
          
         
     //send Duty Cycle value to LCD or USART
     Serial.print("Position "); Serial.println(data);
  }
}


ISR(TIMER1_CAPT_vect)
{
  if (Flag == 0){
    noInterrupts();
    Capt1 = ICR1;
    
    TCCR1B &= ~(1 << ICES1);  //change capture on falling edge
    T10vs2 = 0;               //Reset overflow
    interrupts();
  }
  
  if (Flag==1){
    Capt2 = ICR1;

    TCCR1B|=(1 << ICES1);   //save first overflow counter
    T10vs1=T10vs2;
   }
   
//increment Flag
Flag++;
}

ISR(TIMER1_OVF_vect)
{
  T10vs2++; //Increment overflow counter
}

void sensor_read()
{
  float Position;
  float UpTime;
  int EncoderPulse, FixedPulse;
  volatile uint16_t UpCount;
  volatile uint16_t DownTime;
  volatile uint8_t DutyCycle;     //dutycycle result 
  
  if (Flag==3){                //calculate duty cycle if all timestamps captured
    UpCount = Capt2 - Capt1;
    UpTime = UpCount * .0625;  //Up time in us
    EncoderPulse = UpTime * 4.119;
    FixedPulse = EncoderPulse - 16;
    data = EncoderPulse;
            
    Flag=0;       //clear flag
          
    T10vs1=0;     //clear overflow counters;
    T10vs2=0;

    TIFR1=(1<<ICF1)|(1<<TOV1);        //clear interrupt flags to avoid any pending interrupts
    TIMSK1|=(1<<ICIE1)|(1<<TOIE1);    //enable input capture and overflow interrupts
  }
}
