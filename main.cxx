/*
Libreria para el uso del prototiopo de hodoscopio SiLab 
Versión enero 2018
*/

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include "i2c_master.h"
#include "imudof.h"
#include "communication.h"
#include "Servo.h"
#include "us.h"
#include "stepper.h"
#include "fdcr.h"

bool a=0;
int i=0;
float angle, lastAngle;
char *setAngC;
char *setDistC;
int angleInit, dist, lastDist;


int main(void){
/******Object declarations******/
  
    Fdcr finButton;     // End race sensor
    SerialCom uart;     // Principal UART comms 
    imudof imu;         // Accelerometer, Gyroscope and Magnetometer
    Servo servo;        // Servo motor
    ultraSonic us;      // Ultrasonic distance sensor

/******Config initializations******/

    us.init();
	uart.init(57600);   
    finButton.init(); 
    imu.init();
    
    Stepper stepx(&PORTE, &PORTE, X_DIR, X_STP, X_FdC);
    Stepper stepy(&PORTH, &PORTE, Y_DIR, Y_STP, Y_FdC);
    Stepper stepz(&PORTH, &PORTG, Z_DIR, Z_STP, Z_FdC);
    Stepper stepa(&PORTB, &PORTB, A_DIR, A_STP, A_FdC);

    servo.init();

/******Verification rutines******/
    stepx.stop();                   //Se deshabilita el shield para controlar motores stepper
    
    if (imu.testConnection()) uart.sendData("IMU connection Ok\n");
    else {uart.sendData("IMU connection error\n"); return -1;}
    
    us.sendPulse();
    us.getDistance();
    uart.sendData("Distancia inicial MPPC: ");    
    uart.sendData(us.buff);
    uart.sendData("\n");

    imu.readData();
    uart.sendData("Ángulo inicial: ");    
    uart.sendData(imu.kAccelC[2]);
    uart.sendData("\n");
    angleInit = 180 - imu.kAccelF[2];    
    servo.setAngle(angleInit);
      
    while(1){
        stepx.stop();                   //Se deshabilita el shield para controlar motores stepper
      /*       
        setAngC = uart.readData();      //Lectura de seteo de angulo
        
       
        angle = atof(setAngC);          //Conversion de datos                
        uart.sendData(setAngC);         //Verificacion de recepcion de datos
        uart.sendData("°");
        uart.sendData("\n");
        uart.sendData(setDistC);
        uart.sendData("[cm]");
        uart.sendData("\n");
       */ 
        
       //imu.readData();                 //Leer sensores en IMU (Acelerometro, gyroscopio y magnetometro)
    
       /*  Rutina para medir angulo y setear posicion del servo:*/
     /*
        lastAngle = 180 - imu.kAccelF[2];        
        if(angle > lastAngle){
            while(angle > lastAngle){
            servo.setAngle(angleInit++);
            imu.readData();
            lastAngle = 180 - imu.kAccelF[2];
            uart.sendData("Aumentando angulo\n");
            uart.sendData(imu.kAccelC[2]);
            uart.sendData("\n");  
            }
        }
       if(angle < lastAngle){
                while(angle < lastAngle){
                servo.setAngle(angleInit--);
                imu.readData();
                lastAngle = 180 - imu.kAccelF[2];
                uart.sendData("Disminuyendo angulo: \n");
                uart.sendData(imu.kAccelC[2]);
                uart.sendData("\n");  
                }                    
        }
    
        lastAngle = imu.kAccelF[2];
        uart.sendData("Angulo actual: ");  
        uart.sendData(imu.kAccelC[2]);
        uart.sendData("\n"); 
    */
    /* Rutina para llegar a la separación correspondiente entre MPPC*/
       
        setDistC = uart.readData();     //Lectura de seteo de distancia entre MPPC
        dist = atoi(setDistC);
        uart.sendData(setDistC);
        uart.sendData("[cm]");
        uart.sendData("\n");        
        us.sendPulse();
        us.getDistance();
        lastDist = atoi(us.buff);
      
   
        if(dist < lastDist){
            stepx.enable();
            while(dist < lastDist){
                for(int i = 0; i<200; i++){
                    stepx.stepUp(true);
                    stepa.stepUp(true);
                    stepy.stepUp(true);
                    stepz.stepUp(true);
                    stepa.stepDown(true);
                    stepx.stepDown(true);
                    stepz.stepDown(true);
                    stepy.stepDown(true);
                }
            us.sendPulse();
            us.getDistance();
            lastDist = atoi(us.buff);
            uart.sendData(us.buff);
            uart.sendData("\n");
            }

        }
        
        stepx.stop();    
        if(dist > lastDist){
            stepx.enable();
            while(dist > lastDist){
                for(int i = 0; i<200; i++){
                    stepx.stepUp(false);
                    stepa.stepUp(false);
                    stepy.stepUp(false);
                    stepz.stepUp(false);
                    stepa.stepDown(false);
                    stepx.stepDown(false);
                    stepz.stepDown(false);
                    stepy.stepDown(false);
                }
            
            us.sendPulse();
            us.getDistance();
            lastDist = atoi(us.buff);
            uart.sendData(us.buff);
            uart.sendData("\n");
            }
        }       
        
        stepx.stop();
   
    }
    return 0;
}
