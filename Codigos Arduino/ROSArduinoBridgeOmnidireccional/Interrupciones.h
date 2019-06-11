// Código en el que se implementa el movimiento de los motores utilizando las interrupciones software

// Nombres de los pines
const int M11 = 2;
const int M12 = 3;
const int M13 = 4;
const int M14 = 5;

const int M21 = 6;
const int M22 = 7;
const int M23 = 8;
const int M24 = 9;

const int M31 = 10;
const int M32 = 11;
const int M33 = 12;
const int M34 = 13;

// Función para deshabilitar el Timer3
void disableTimer3(){
  TCCR3A = 0;
  TCCR3B = 0;
}

// Configuración del Timer3
void setupTimer3_25mS(){
  // Se deshabilitan las interrupciones
  cli();

  disableTimer3();
 
  // Se ajusta el parámetro para el tiempo de cuenta deseado
  OCR3A = 575;

  // Se activa el modo CTC. WGM32 establece los valores de los bits CS10 y CS12 para un preescalado del Timer3 de 1024
  TCCR3B = bit (WGM32) | bit (WGM32) | bit (CS30);

  // Se activa el temporizador
  TIMSK3 |= (1 << OCIE3A);
 
  // Se habilitan las interrupciones
  sei();
}

ISR(TIMER3_COMPA_vect){ // Cada vez que pasa el tiempo establecido se ejecuta esta interrupción.


// Motor A
Interrupciones_A = Interrupciones_A -1;

if (Interrupciones_A <= abs(numero_cambio_secuencia_A)) {
  Interrupciones_A = 255;
  // Toca la siguiente secuencia

  if(numero_cambio_secuencia_A > 0){
    Secuencia_Actual_A +=1;
    avanceMotores[0] +=1;
  }
  if(numero_cambio_secuencia_A < 0){
    Secuencia_Actual_A -=1;
    avanceMotores[1] +=1;
  }

  if (Secuencia_Actual_A == 4){
     Secuencia_Actual_A = 0;
  }
  if (Secuencia_Actual_A == -1){
     Secuencia_Actual_A = 3;
  }

  
  switch(Secuencia_Actual_A) {
  case 3:
      digitalWrite(M11, LOW); 
      digitalWrite(M12, LOW);  
      digitalWrite(M13, HIGH);  
      digitalWrite(M14, HIGH);
      break;
  case 2:
      digitalWrite(M11, LOW); 
      digitalWrite(M12, HIGH);  
      digitalWrite(M13, HIGH);  
      digitalWrite(M14, LOW);
      break;
  case 1:
      digitalWrite(M11, HIGH); 
      digitalWrite(M12, HIGH);  
      digitalWrite(M13, LOW);  
      digitalWrite(M14, LOW);
      break;
  case 0:
      digitalWrite(M11, HIGH); 
      digitalWrite(M12, LOW);  
      digitalWrite(M13, LOW);  
      digitalWrite(M14, HIGH);
      break;
  }
}
  

// Motor B
Interrupciones_B = Interrupciones_B -1;

if (Interrupciones_B <= abs(numero_cambio_secuencia_B)) {
  Interrupciones_B = 255;
  // Toca la siguiente secuencia

  if(numero_cambio_secuencia_B > 0){
    Secuencia_Actual_B +=1;
    avanceMotores[2] +=1;
  }
  if(numero_cambio_secuencia_B < 0){
    Secuencia_Actual_B -=1;
    avanceMotores[3] +=1;
  }

  if (Secuencia_Actual_B == 4){
     Secuencia_Actual_B = 0;
  }
  if (Secuencia_Actual_B == -1){
     Secuencia_Actual_B = 3;
  }

  
  switch(Secuencia_Actual_B) {
  case 3:
      digitalWrite(M21, LOW); 
      digitalWrite(M22, LOW);  
      digitalWrite(M23, HIGH);  
      digitalWrite(M24, HIGH);
      break;
  case 2:
      digitalWrite(M21, LOW); 
      digitalWrite(M22, HIGH);  
      digitalWrite(M23, HIGH);  
      digitalWrite(M24, LOW);
      break;
  case 1:
      digitalWrite(M21, HIGH); 
      digitalWrite(M22, HIGH);  
      digitalWrite(M23, LOW);  
      digitalWrite(M24, LOW);
      break;
  case 0:
      digitalWrite(M21, HIGH); 
      digitalWrite(M22, LOW);  
      digitalWrite(M23, LOW);  
      digitalWrite(M24, HIGH);
      break;
  }
}

// Motor C
Interrupciones_C = Interrupciones_C -1;

if (Interrupciones_C <= abs(numero_cambio_secuencia_C)) {
  Interrupciones_C = 255;
  // Toca la siguiente secuencia

  if(numero_cambio_secuencia_C > 0){
    Secuencia_Actual_C +=1;
    avanceMotores[4] +=1;
  }
  if(numero_cambio_secuencia_C < 0){
    Secuencia_Actual_C -=1;
    avanceMotores[5] +=1;
  }

  if (Secuencia_Actual_C == 4){
     Secuencia_Actual_C = 0;
  }
  if (Secuencia_Actual_C == -1){
     Secuencia_Actual_C = 3;
  }

  
  switch(Secuencia_Actual_C) {
  case 3:
      digitalWrite(M31, LOW); 
      digitalWrite(M32, LOW);  
      digitalWrite(M33, HIGH);  
      digitalWrite(M34, HIGH);
      break;
  case 2:
      digitalWrite(M31, LOW); 
      digitalWrite(M32, HIGH);  
      digitalWrite(M33, HIGH);  
      digitalWrite(M34, LOW);
      break;
  case 1:
      digitalWrite(M31, HIGH); 
      digitalWrite(M32, HIGH);  
      digitalWrite(M33, LOW);  
      digitalWrite(M34, LOW);
      break;
  case 0:
      digitalWrite(M31, HIGH); 
      digitalWrite(M32, LOW);  
      digitalWrite(M33, LOW);  
      digitalWrite(M34, HIGH);
      break;
  }
}


}
