/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Modificaciones realizadas por: Alberto Díaz Rodríguez
// Porgrama para la recepción de las velocidades de los motores, cálculo de la odometría y envío de la pose
// Fecha 07/05/2019
// Universidad de La Laguna

// Velocidad de la comunicación en el puerto serial
#define BAUDRATE     57600

#include <LiquidCrystal_I2C.h> // Libreria LCD_I2C
LiquidCrystal_I2C lcd(0x27,16,2); // Dirección de la pantalla


// Se incluye el archivo con la definición de los comandos
#include "commands.h"

// Iniciación de las variables
// Variables para la recepción del mensaje de ROS
int arg = 0;
int index = 0;

// Almacenará el caracter que se esté identificando del mensaje recibido
char chr;

// Se almacena el comando identificado
char cmd;

// Arrays para almacenar los 3 argumentos recibidos
char argv1[16];
char argv2[16];
char argv3[16];

// Se convierten los argumentos a variables de tipo long
long arg1;
long arg2;
long arg3;

// String para la respuesta
String respuesta = "";

// Variables globales para la odometría del robot

// Vector que almacenará la pose del robot (x e y en centímetros y Theta en grados).
float Pose[3] = {0, 0, 90};
int avanceMotores[6] = {0,0,0,0,0,0}; // Vector en el que se anotarán los pasos que den los motores
int ciclos = 0;
float avance = (2 * PI * 31)/(512 * 4);
float rotacion = (avance * 360)/(6 * PI * 68.9);

// Variables globales para la función que activa el movimiento de los motores

// Motor 1
int Interrupciones_A = 255; 
int numero_cambio_secuencia_A = 0;
int Secuencia_Actual_A = 0;

// Motor 2
int Interrupciones_B = 255;
int numero_cambio_secuencia_B = 0;
int Secuencia_Actual_B = 0;

// Motor 3
int Interrupciones_C = 255;
int numero_cambio_secuencia_C = 0;
int Secuencia_Actual_C = 0;

// Función para vaciar las variables en las que se almacenan los datos recibidos
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}

// Función para la activación de la orden recibida
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);

  // Estructura switch-case en la que se determina que acciones realizar según la orden recibida
  switch(cmd) {
    case MOTORS_SPEED: //Función para la asignación de las velocidades de los motores
      numero_cambio_secuencia_A = arg1;
      numero_cambio_secuencia_B = arg2;
      numero_cambio_secuencia_C = arg3;
    break;
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
    break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      Serial.println("El pin que se esta leyendo es el: ");
      Serial.print(arg1);
    break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
    break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK"); 
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0){
        digitalWrite(arg1, LOW);
      }else if (arg2 == 1){ 
        digitalWrite(arg1, HIGH);
      };
      Serial.println("OK");  
    break;
    case PIN_MODE:
      if (arg2 == 0){
        pinMode(arg1, INPUT);
      }else if (arg2 == 1){
        pinMode(arg1, OUTPUT);
      }
      Serial.println("OK");
    break;
  default:
    Serial.println("Comando recibido inválido");
    break;
  }
}

String Stringchr = "";

// Una vez se ha configurado todo lo necesario del Ros_Arduino_Bridge se inician las interrupciones

// Interrupciones para el movimiento de los motores
#include "Interrupciones.h"


// Configuración inicial
void setup() {
   // Se establecen los pines de control de los motores como salidas.
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M13, OUTPUT);
  pinMode(M14, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  pinMode(M23, OUTPUT);
  pinMode(M24, OUTPUT);
  pinMode(M31, OUTPUT);
  pinMode(M32, OUTPUT);
  pinMode(M33, OUTPUT);
  pinMode(M34, OUTPUT);
  Serial.begin(BAUDRATE);

  Serial3.begin(115200);
  sendData("AT+CWMODE=3\r\n",1000); // configuración punto de acceso
  sendData("AT+CWJAP=\"WifiMovil\",\"12345678\"\r\n",1000);//red wifi, usuario y clave
  
  delay(5000);
  sendData("AT+CIPMUX=1\r\n",1000); // multiples conexiones
  
  sendData("AT+CIFSR\r\n",1000); // se obtienen direcciones IP 

  sendData("AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",4445,4445,2\r\n",1000); // Para recibir la información procedente de ROS en el puerto 4445
  sendData("AT+CIPSTART=1,\"UDP\",\"192.168.43.189\",4446,4446,2\r\n",1000); // IP del ordenador destino. Se envía al puerto 4446
  sendData("AT+CIPDINFO=0\r\n",1000);
 
  delay(500);

  setupTimer3_25mS();
  delay(1000);

  // Se muestra por la pantalla LCD un mensaje para que el usuario sepa que la configuración ha terminado
  lcd.begin(); // En windows es lcd.init();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Robot"); 
  lcd.setCursor (0,1);
  lcd.print("Omnidireccional");
  delay(1000);
  lcd.noBacklight(); // Se apaga la pantalla tras el aviso para reducir el consumo de batería
}

// Bucle principal
void loop() {
  // Se actualiza la odometría.
  
  ciclos += 1;
  
  if(avanceMotores[0] > 0){ // Sentido positivo motor 1
    Pose[0] += (avance/10)*cos(((Pose[2]*PI)/180)-(PI/6));
    Pose[1] += (avance/10)*sin(((Pose[2]*PI)/180)-(PI/6));
    Pose[2] -= rotacion;
    avanceMotores[0] -=1;
  }
  if(avanceMotores[1] > 0){ // Sentido negativo motor 1
    Pose[0] -= (avance/10)*cos(((Pose[2]*PI)/180)-(PI/6));
    Pose[1] -= (avance/10)*sin(((Pose[2]*PI)/180)-(PI/6));
    Pose[2] += rotacion;
    avanceMotores[1] -=1;
  }
  if(avanceMotores[2] > 0){ // Sentido positivo motor 2
    Pose[0] += (avance/10)*sin(((Pose[2]*PI)/180)-(PI/3));
    Pose[1] -= (avance/10)*cos(((Pose[2]*PI)/180)-(PI/3));
    Pose[2] -= rotacion;
    avanceMotores[2] -=1;
  }
  if(avanceMotores[3] > 0){ // Sentido negativo motor 2
    Pose[0] -= (avance/10)*sin(((Pose[2]*PI)/180)-(PI/3));
    Pose[1] += (avance/10)*cos(((Pose[2]*PI)/180)-(PI/3));
    Pose[2] += rotacion;
    avanceMotores[3] -=1;
  }
  if(avanceMotores[4] > 0){ // Sentido positivo motor 3
    Pose[0] -= (avance/10)*cos(((Pose[2]*PI/180))-(PI/2));
    Pose[1] -= (avance/10)*sin(((Pose[2]*PI/180))-(PI/2));
    Pose[2] -= rotacion;
    avanceMotores[4] -=1;
  }
  if(avanceMotores[5] > 0){ // Sentido negativo motor 3
    Pose[0] += (avance/10)*cos(((Pose[2]*PI/180))-(PI/2));
    Pose[1] += (avance/10)*sin(((Pose[2]*PI/180))-(PI/2));
    Pose[2] += rotacion;
    avanceMotores[5] -=1;
  }

  // Se corrige el ángulo de la pose para que este entre 0 y 360 grados
  if(Pose[2] < 0){
    Pose[2] += 360;
  }
  if(Pose[2] > 360){
    Pose[2] -= 360;
  }

  if (Serial3.available()) { // Si ha legado un mensaje se almacena el contenido del mismo
    Serial3.print("AT+CIPRECVMODE=1\r\n"); // Modo de recepción de datos
    getData(10);

    for(int j = 0; j < Stringchr.length(); j++){ // Se analiza el mensaje letra por letra hasta el caracter CR
    chr = Stringchr[j];
   
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
        else if (arg == 3) argv3[index] = NULL;
        Serial.print("Command ");
        Serial.println(cmd);
        runCommand();
        resetCommand();
    }
    // Se utilizan espacios para delimitar las partes del mensaje
    else if (chr == ' ') {
      // Se cambia al segundo argumento
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      } Se cambia al tercer argumento
      else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // El primer argumento es un comando de una letra
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
    }
  }

  // Se envía la nueva pose
  
  if(ciclos > 6000){ // Solo se realiza el envío si han pasado un número de ciclos para reducir la sobrecarga
    respuesta = "(" + String(Pose[0]) + "," + String(Pose[1]) + "," + String(Pose[2]) + ")";
    sendData("AT+CIPSEND=" + String(1) + "," + respuesta.length() + "\r\n", 10); // Modo de envío de datos
    sendData(respuesta, 10);
    ciclos = 0;
  }
    
  }
}

// Función para enviar datos
String sendData(String command, const int timeout){
  
    String response = "";
    Serial3.print(command);
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial3.available()){ 
        char c = Serial3.read();
        response+=c;
      }  
    }
    
    return response;
}

// Función para recibir datos
void getData( const int timeout){
    
    Stringchr = "";
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial3.available()){ 
        char c = Serial3.read();
        Stringchr +=c;
      }  
    }
}
