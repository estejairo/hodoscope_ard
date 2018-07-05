#include "Servo.h"


//Set the servo in angle 0
int Servo::init(){

 
    TCCR1A |= (1<<COM1A1)|(1<<WGM11); //Non inverted PWM
    TCCR1A &= ~(1<<WGM10); 
    TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);// Prescaler =64 Mode 14 (fast PWM)
  
    //|(1<<WGM12)
    ICR1 = 4999;  //fPWM=50Hz (Period = 20ms Standard).

    DDRB |= 0x20;   //PWM PINB5 OUTPUT
    
    //int i = lastDuty;   
    
    OCR1A = 150;

    return lastDuty;
}



void Servo::off(){
    DDRB &= ~0x20;
    PORTB |= 0x20;
    TCCR1A &= 0x00;
    TCCR1B &= 0x00;
    ICR1 = 0x00;
}

void Servo::on(){ 
    DDRB |= 0x20;
}

void Servo::test(){
    for (int i = 150;i < 600; i++){
    // 150 is 0 degree
    // 600 is 180 degree
        OCR5C = i;
        _delay_ms(100);
        lastDuty = i;
    }
    for (int i = 600;i < 150; i--){
        OCR5C = i;
        _delay_ms(100);
        lastDuty = i;
    }
}

int Servo::setAngle(int angle){

    double duty = (2.4333* angle) + 150;
    if(angle == 0) duty = 150;
    if(angle ==180) duty = 580;
    int i = lastDuty;
    if (duty < lastDuty){
        for (i; i > (int) duty; i-- ){    
            OCR1A = i;
            _delay_ms(50);
            lastDuty = i;
        }
    }
    if (duty > lastDuty){
        for (i; i < (int) duty; i++ ){    
            OCR1A = i;
            _delay_ms(50);
            lastDuty = i;
        }
    }

    return lastDuty;
}



