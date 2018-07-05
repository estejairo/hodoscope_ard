//// First Aproach for the database actualization

#define F_CPU 16000000UL
#define BAUD 57600									// baud rate for serial communication
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)
#define WBAUD 115200
#define WBAUDRATE 0x0008                            // baud rate for wifi-module communication
#define UART1_AVAILABLE bit_is_set(UCSR1A, RXC1)
#define TIMER_FREQ  F_CPU/1024.0
#define TOTAL_COUNTS  65535
#define INIT_COUNT(x)  (TOTAL_COUNTS - (x)*TIMER_FREQ)
#define MAX_GATE 4.194
#define OVERFLOW(y)  (unsigned int)((y)/MAX_GATE)

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/wdt.h>     // watchdog
#include "i2c_master.h"

#include "imudof.h"
#include "Servo.h"
#include "us.h"
#include "stepper.h"
#include "fdcr.h"



//// For timer0 and millis() function 

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)
#define NPAR 4

float new_gate = 5;
float new_azimuth = 0;
float old_azimuth = 0;
float new_distance = 20;
float old_distance = 0;
float new_polar = 0;
float angleInit = 0;
float lastAzimuth = 0;
float lastPolar = 0;
float lastDistance = 0;
int go_ahead = -1;           // Flag for avoid the interrupt coincidences 
char angleRead[10];
char distanceRead[10];
char PARNAME[NPAR][6] = {{"pa:"},{"aa:"},{"dx:"},{"gate:"}};
int PARVAL[NPAR];
bool uart_enable;

char *str;
// There exist a limit to the gate value to avoid TIMER5'S reset. TCNT5 cannot overcome 65535 in the gate time.

int gate = 5;  //   initialization value, in seconds ...
bool begin_en = true;
char msg_sys[1000];  
char msg[1000];                // my buffer to store data
char msg0[100];
//////////// Global Clock Timer0

/*

int timer0_overflow_count = 0;
unsigned long timer0_millis = 0;
unsigned char timer0_fract;

void timer0_init(){
	TCCR0B |= (1 << CS01)|(1 << CS00);
	TIMSK0 |= (1 << TOIE0);
	sei();
	TCNT0 = 0;
}
	
SIGNAL(TIMER0_OVF_vect)
{
    // copy these to local variables so they can be stored in registers
    // (volatile variables must be read from memory on every access)
    unsigned long m = timer0_millis;
    unsigned char f = timer0_fract;
 
    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
        f -= FRACT_MAX;
        m += 1;
    }
 
    timer0_fract = f;
    timer0_millis = m;
    timer0_overflow_count++;
}
    
unsigned long millis()
{
    unsigned long m;
    uint8_t oldSREG = SREG;
 
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value 
    cli();
    m = timer0_millis;
    SREG = oldSREG;    // Set the previus parameters (Status Register) 
    return m;
}

*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////      COMMUNICATION      //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Watchdog off

void WDT_off(void)
{
cli();
wdt_reset();
/* Clear WDRF in MCUSR */
MCUSR &= ~(1<<WDRF);
/* Write logical one to WDCE and WDE */
/* Keep old prescaler setting to prevent unintentional time-out
*/
WDTCSR |= (1<<WDCE) | (1<<WDE);
/* Turn off WDT */
WDTCSR = 0x00;
sei();
}

//////////////////////////////////	Serial Communication 

//Inicialization
void uart_init(void)
{
	UBRR0H = (BAUDRATE >> 8);                  //Set Baud Rate
	UBRR0L = BAUDRATE;
	UCSR0B |= (1 << TXEN0)|(1 << RXEN0);	   //Transmit and Recieve enable
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);         // Set 8-bit data transfer
    UCSR0C &= ~(1<<UMSEL01) && ~(1<<UMSEL01); // Asynchronous mode: UART 
}

//Function to send data
void uart_transmit(unsigned char data)
{
    while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
    UDR0 = data;                                   // load data in the register
}


//Function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSR0A & 1<<RXC0));                   // wait while data is being received
    return UDR0;                                   // return 8-bit data
}

//Function to clean RCX register
void uart_flush(void)
{
unsigned char dummy;
while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

//Function to send a char array
int uart_send(const char *data)
{
int i=0;
char c;
while ((c=data[i++])!='\0')
	uart_transmit(c);
	//uart_transmit('\r');          
	//uart_transmit('\n');
return 1;
}

//Function to read a char array (from UDR0 buffer)
char *uart_read()
{
int i=0;
char c;
if((UCSR0A & 1<<RXC0)){
    while (((c=uart_recieve())!='\r') && (i < 99))
    {
	    msg[i++]=c;
    }
    msg[i]='\0';
}

return msg;
}

//////////////////////////////////	Serial1 Communication (WiFi Module)

//Inicialization
void uart1_init(void)
{
	UBRR1H = (WBAUDRATE >> 8);             // Set Baud Rate
	UBRR1L = WBAUDRATE;
	UCSR1B |= (1 << TXEN1)|(1 << RXEN1);   // Transmit and Recieve enable
	UCSR1C |= (1<<UCSZ11)|(1<<UCSZ10);     // Set 8-bit data transfer
}

//Function to send data
void uart1_transmit(unsigned char data)
{
   // loop_until_bit_is_set(UCSR1A, UDRE1);       // alternative way 
   while (!( UCSR1A & (1<<UDRE1)));               // wait while register is free
   UDR1 = data;                                   // load data in the register
}

//Function to receive data

unsigned char uart1_recieve (void)
{
	//while(!(UCSR1A & 1<<RXC1));                  // alternative way
	loop_until_bit_is_set(UCSR1A, RXC1);
	return UDR1;                                   // return 8-bit data	
}

//Function to send a char array
void uart1_flush(void)
{
unsigned char dummy;
while (!( UCSR1A & (1<<RXC1)) ) dummy = UDR1;
}

//Function to send a char array
int uart1_send(const char *data)
{
int i=0;
char c;
while ((c=data[i++])!='\0')
	uart1_transmit(c);
uart1_transmit('\r');
uart1_transmit('\n');
return 1;
}

//Function to read a char array (from UDR1 buffer)   ------------->  Designed for ESP8266 responses
           // my buffer to store data
////////////Command Responses

char *uart1_read_cmd(bool type=0)
{
int i=0;
unsigned char c;
int count_ok=0;
while   (i<1000) //
{
	//if(UART1_AVAILABLE)               //alternative way
    c=uart1_recieve();
	msg_sys[i]=c;
	if ((msg_sys[i-1]=='O') && (msg_sys[i]=='K')) break;           // #! characteristic flag
	if ((msg_sys[i-4]=='E') && (msg_sys[i-3]=='R')&&(msg_sys[i-2]=='R') && (msg_sys[i-1]=='O')&& (msg_sys[i]=='R')) break;  // break when an error is announced 
	i++;
}
msg_sys[i+1]='\0';
return msg_sys;
}
//////////////Query Responses

char *uart1_read_query(bool type=0)
{
bool store_en = false;
int i=0;
unsigned char p = '\0';
unsigned char c = '\0';
int count_ok=0;
while   (i<1000) //
{
	//if(UART1_AVAILABLE)               //alternative way
    c=uart1_recieve();
	if((c=='#')&&(p=='!')){store_en = true;}
	p=c;
	if(store_en)
	{
		msg_sys[i]=c;
		if ((msg_sys[i-1]=='#') && (msg_sys[i]=='!')) break;           // #! characteristic flag
		if ((msg_sys[i-4]=='E') && (msg_sys[i-3]=='R')&&(msg_sys[i-2]=='R') && (msg_sys[i-1]=='O')&& (msg_sys[i]=='R')) break;  // break when an error is announced 
		i++;
	}
}
msg_sys[i+1]='\0';
//char  hola[10];
//sprintf(hola, "LAST I: %d\n",i);
//uart_send(hola);
return msg_sys;
}

char *uart_recieve_data(bool type=0){
bool store_en = false;
int i=0;
unsigned char p = '\0';
unsigned char c = '\0';
int count_ok=0;
while   (i<1000) //
{
	//if(UART1_AVAILABLE)               //alternative way
    c=uart_recieve();
	if((c=='#')&&(p=='!')){store_en = true;}
	p=c;
	if(store_en)
	{
		msg_sys[i]=c;
		if ((msg_sys[i-1]=='#') && (msg_sys[i]=='!')) break;           // #! characteristic flag
		if ((msg_sys[i-4]=='E') && (msg_sys[i-3]=='R')&&(msg_sys[i-2]=='R') && (msg_sys[i-1]=='O')&& (msg_sys[i]=='R')) break;  // break when an error is announced 
		i++;
	}
}
msg_sys[i+1]='\0';
//char  hola[10];
//sprintf(hola, "LAST I: %d\n",i);
//uart_send(hola);
return msg_sys;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////   COUNTERS TIMERS ////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 // Variables for frequency count

int tot_overflow;                									  // Timer3 Total Overflow counter
char data_send[59] = "GET /orsosa/store.php?word=LetMeIn!&field=freq&value=";             // Buffer to frequency value storage      
float freq;
char *serialData;
char freq_char[4];
char gate_char[4];
char azimuth_char[4];
char distance_char[4];
char polar_char[4];
// Counter5 initialization
void timer5_init()
{
	TCCR5B |= (1 << CS52)|(1 << CS51);          // External clock mode ------> On Pin 47 in Arduino ATMEGA1280
	TCNT5 = 0;                                  // coincidence counter
}



// Timer3 initialization
void timer3_init()
{
	TCCR3B |= (1 << CS32)|(1 << CS30);          							// Set 1024 prescaling
	tot_overflow = OVERFLOW(gate);              							// Set the total overflows for a given gate time
	TCNT3 = INIT_COUNT(gate-MAX_GATE*OVERFLOW(gate))*(tot_overflow==0);     // Counter value depends on overflow value
	TIMSK3 |= (1 << TOIE3);                                                 
	sei();																	// Enable global interrupts
	tot_overflow--;
}

//////// Get parameters from db
float get_par(char *p){
	char par[20];
	strcpy(par,p);
	strcat(par,":");
	char *ps = strtok(msg_sys," ");
	while(ps)
	{
		if(!strcmp(ps,par)) break;
		ps = strtok(NULL," ");
        uart_send(ps);	
    }
	ps = strtok(NULL," ");
    uart_send(ps);	
    return atof(ps);
}

int16_t get_all_par(){
	char *ps = strtok(msg_sys," ");
	while(ps)
	{
        for(int k=0;k<NPAR;k++)
        {
		    if(!strcmp(ps,PARNAME[k]))
            {
                ps = strtok(NULL," ");
                //uart_send(ps);
                PARVAL[k]=atof(ps);
                break; 
            }
        }
        ps = strtok(NULL," ");  
    }

    return 0;
}

///////////////////////////////////////////////////////////  Interrupts //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Timer3 Overflow Interrupt     -------------->   Here the data is send
ISR(TIMER3_OVF_vect)
{
	
	TCNT3 = INIT_COUNT(gate-MAX_GATE*OVERFLOW(gate))*(tot_overflow==0);    // Counter value depends on overflow value, same as the initialization  
	if(tot_overflow<0)                                                     // Condition for send the value (gate time is reached)
	{
		uart1_send("··················");
		uart1_flush();
		
		
		freq = (float)((TCNT5)/gate);                                    
		TCNT5=0;												   // Reset the coincidence counter
		dtostrf(freq,8, 0, (char*)freq_char); 
        dtostrf(gate,8, 0, (char*)gate_char); 
        dtostrf(lastAzimuth,8, 0, (char*)azimuth_char);
        dtostrf(lastPolar,8, 0, (char*)polar_char);
        dtostrf(lastDistance,8, 0, (char*)distance_char);
//dtostrf() on stdlib.h, to cast a float to char*
		data_send[53] = freq_char[0];
		data_send[54] = freq_char[1];
		data_send[55] = freq_char[2];
		data_send[56] = freq_char[3];
		data_send[57] = '\0';
		
        //uart_send(data_send);
    strcat(serialData,"freq:");
    strcat(serialData,freq_char);
    strcat(serialData,"gate:");
    strcat(serialData,gate_char);
    strcat(serialData,"aa:");
    strcat(serialData,azimuth_char);
    strcat(serialData,"pa:");
    strcat(serialData,polar_char);
    strcat(serialData,"dx:");
    strcat(serialData,distance_char);
        uart_send(serialData);

        	
		uart1_send("AT+CIPSEND=4,59");                             
		//uart_send(uart1_read_cmd());
        uart1_read_cmd();
		uart1_send(data_send);
        uart1_read_cmd();		
        //uart_send(uart1_read_cmd());                                    // Read the response
		
		gate=new_gate;
		
		uart1_send("AT+CIPCLOSE=4");
		//uart_send(uart1_read_cmd());
		uart1_read_cmd();
		uart1_send("AT+CIPSTART=4,\"TCP\",\"200.1.16.248\",80");       // Gate time works like the delay
		//uart_send(uart1_read_cmd());
		uart1_read_cmd();
		//uart1_flush();
		
		tot_overflow=OVERFLOW(gate);                                    // Same as the initialization
	}
	tot_overflow--;
}

void prog_init()      //  Initialization function ...
{
	WDT_off();
	timer5_init();
	timer3_init();                      // Initialization 
	uart_init();
	uart1_init();
	
	uart1_send("AT+RST"); _delay_ms(2000);            // Reset the module
	uart_send("RESET OK");
	
	uart1_send("AT+CIPMUX=1"); _delay_ms(300);        // Configured for multiple connections
	uart_send("CIPMUX OK");
	
	uart1_send("AT+CWMODE=1"); _delay_ms(300);        // Configured as STA
	uart_send("CWMODE OK");
	
	uart1_send("AT+CWJAP=\"atlasgw\",\"SiLabaLiS\""); _delay_ms(5000);       // Connect to Local Network
    uart_send(uart1_read_cmd());
	
	uart1_send("AT+CIPSTART=4,\"TCP\",\"200.1.16.248\",80");       // Port to Send Data
	_delay_ms(500);
	uart1_send("AT+CIPSTART=3,\"TCP\",\"200.1.16.248\",80");       // Port to Read Dara
	_delay_ms(500);
	uart_send("ready");
}

void uart_pin_init(){
    DDRC &= ~(0x01); // Arduino PIN 37 INPUT
    PORTC |= 0x01; //Pull up resistor ON pin37
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////// MAIN ////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

int main()
{

    uart_pin_init();    
    uart_enable = PINC0; 

/******Object declarations******/
    Fdcr finButton;     // End race sensor
    imudof imu;         // Accelerometer, Gyroscope and Magnetometer
    Servo servo;        // Servo motor
    ultraSonic us;      // Ultrasonic distance sensor

/******Config initializations******/

    us.init(); 
    finButton.init(); 
    imu.init();
    servo.init();
    Stepper stepx(&PORTE, &PORTE, X_DIR, X_STP, X_FdC);
    Stepper stepy(&PORTH, &PORTE, Y_DIR, Y_STP, Y_FdC);
    Stepper stepz(&PORTH, &PORTG, Z_DIR, Z_STP, Z_FdC);
    Stepper stepa(&PORTB, &PORTB, A_DIR, A_STP, A_FdC);
    
/****** Cheking if all is working****/
	if(uart_enable == 0) {
        prog_init();
        stepx.stop();
        serialData = (char*)malloc(32);

        if (imu.testConnection()) uart_send("IMU connection Ok\n");
        else {uart_send("IMU connection error\n"); return -1;}
    }
    else
        if (imu.testConnection()==0) return -1;
        
    
    us.sendPulse();
    us.getDistance();
   // uart_send("Distancia inicial MPPC: ");    
   // uart_send(us.buff);
   // uart_send("\n");

    imu.readData();
   // uart_send("Ángulo inicial: ");    
   // uart_send(imu.kAccelC[2]);
   // uart_send("\n");
    angleInit = 180 - imu.kAccelF[2];    
    servo.setAngle(angleInit);
	
    stepx.stop();
    _delay_ms(1000); 
	while(1)     
	{
  
        if(uart_enable){
            uart_recieve_data();
            get_all_par();
            new_polar   = PARVAL[0];		
            new_azimuth   = PARVAL[1];
            new_distance = PARVAL[2];        
            new_gate    = PARVAL[3];
                    
        }
        else{
		    uart1_send("AT+CIPSEND=3,25");
		    uart1_flush();              
		    _delay_ms(100);
	
		    uart1_send("GET /orsosa/getdata.php");                 				// Send the query
		    uart_send(uart1_read_query());
	
		    uart_send("-------*|*----------");
		    uart_send(msg_sys);
		    uart_send("-------*|*----------");
            
		    get_all_par();
            new_polar   = PARVAL[0];		
            new_azimuth   = PARVAL[1];
            new_distance = PARVAL[2];        
            new_gate    = PARVAL[3];
        }

        if(new_distance < 20) new_distance = 20;       
    /*    
        dtostrf(new_distance,7, 2, (char*)distanceRead);
        uart_send("Distance read from database: ");        
        uart_send(distanceRead);
        uart_send("\n");
    */    
    /*    
        dtostrf(new_azimuth,7, 2, (char*)angleRead);
        uart_send("Azimuth angle read: ");        
        uart_send(angleRead);
        uart_send("\n");
    */   
             
        imu.readData();                 //Leer sensores en IMU (Acelerometro, gyroscopio y magnetometro)
    
       //  Rutina para medir angulo y setear posicion del servo:
       if(old_azimuth != new_azimuth){
            old_azimuth = new_azimuth;
            lastAzimuth = 180 - imu.kAccelF[2];        
            if(new_azimuth > lastAzimuth){
                while(new_azimuth > lastAzimuth){
                    servo.setAngle(angleInit++);
                    imu.readData();
                   // uart_send(imu.kAccelC[2]);
                   // uart_send("\n");
                    lastAzimuth = 180 - imu.kAccelF[2];
                }
            }
            if(new_azimuth < lastAzimuth){
                while(new_azimuth < lastAzimuth){
                    servo.setAngle(angleInit--);
                    imu.readData();
                   // uart_send(imu.kAccelC[2]);
                   // uart_send("\n");
                    lastAzimuth = 180 - imu.kAccelF[2];
                 }                    
            }
            lastAzimuth = imu.kAccelF[2];
        }
    
        // Rutina para mover los stepper segun distancia en el sensor ultrasonico
        us.sendPulse();
        us.getDistance();
        lastDistance = atoi(us.buff);
       // uart_send("Distancia ");                
       // uart_send(us.buff);
        if((old_distance != new_distance) || (new_distance != lastDistance)){  
            old_distance=new_distance; 
            if(new_distance < lastDistance){
                stepx.enable();
                while((new_distance < lastDistance)){

                    finButton.readState();
                    if((finButton.val1&&finButton.val2&&finButton.val4)!=1) break;

                    for(int i = 0; i<200; i++){

                        stepx.stepUp(true);
                        stepa.stepUp(true);
                        stepy.stepUp(true);
                        stepz.stepUp(true);
                        finButton.readState();
                        if((finButton.val1&&finButton.val2&&finButton.val4)!=1)break;
                        stepa.stepDown(true);
                        stepx.stepDown(true);
                        stepz.stepDown(true);
                        stepy.stepDown(true);
                        
                    }
                us.sendPulse();
                us.getDistance();
                lastDistance = atoi(us.buff);
               // uart_send("Distancia ");                
               // uart_send(us.buff);
                }
            }
                
            if(new_distance > lastDistance){
                stepx.enable();
                while(new_distance > lastDistance){

                    finButton.readState();
                    if((finButton.val1&&finButton.val2&&finButton.val4)!=1)break;

                    for(int i = 0; i<200; i++){
                        stepx.stepUp(false);
                        stepa.stepUp(false);
                        stepy.stepUp(false);
                        stepz.stepUp(false);

                        finButton.readState();
                        if((finButton.val1&&finButton.val2&&finButton.val4)!=1)break;

                        stepa.stepDown(false);
                        stepx.stepDown(false);
                        stepz.stepDown(false);
                        stepy.stepDown(false);
                    }
                
                us.sendPulse();
                us.getDistance();
                lastDistance = atoi(us.buff);
               // uart_send("Distancia ");                
               // uart_send(us.buff);
                }
            }       
        }
        stepx.stop();

        if(uart_enable==0){
		    uart1_send("AT+CIPCLOSE=3");      

		    _delay_ms(1000);
		    uart1_send("AT+CIPSTART=3,\"TCP\",\"200.1.16.248\",80");       // Gate time works like the delay
		    //uart_send(uart1_read_cmd());
        }
		
	}
	
	return 0;
}
