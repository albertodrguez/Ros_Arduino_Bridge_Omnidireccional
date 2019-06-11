// Control del movimiento del robot mediante servidor web implementado en Arduino MEGA con módulo ESP-01
// Autor: Alberto Díaz Rodríguez
// Fecha: 11/02/19
// Universidad de La Laguna

// El código de números será el siguiente:
// -IDENTIFICADOR --------MOTORES ACTIVADOS -------- SENTIDO DE GIRO-
//                       M1      M2      M3         M1     M2      M3
//        0              NO      NO      NO         -      -       -       
//        1              SI      SI      SI         D      D       D
//        2              SI      SI      SI         I      I       I
//        3              SI      SI      NO         D      I       -
//        4              SI      NO      SI         D      -       I
//        5              NO      SI      SI         -      D       I
//        6              SI      SI      NO         I      D       -
//        7              SI      NO      SI         I      -       D
//        8              NO      SI      SI         -      I       D
//--------------------------------------------------------------------

// Librería
#include <LiquidCrystal_I2C.h> // Libreria LCD_I2C

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


// Otras variables
bool primeraEjecucion = true; // Sirve para que en la primera ejecución del programa se envie la información al cliente.
String URL = "";

int identificador = 0; // Variable que almacenará el número de movimiento que se debe realizar.
int valorDelay = 2; // Valor que determina la velocidad de giro de los motores.

int Secuencia_Actual_A = 0;
int Secuencia_Actual_B = 0;
int Secuencia_Actual_C = 0;

int mostrar = 0; // Para que solo muestre 1 vez el valor de los grados girados

// Configuración inicial
void setup(){
  // Se establece la dirección para la pantalla LCD con I2C, y se enciende y limpia la pantalla.
  LiquidCrystal_I2C lcd(0x27,16,2);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Se establecen los pines como salidas.
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

  delay(1000);
  Serial3.begin(115200); 

 // Se envian todos los comandos AT necesarios para configurar el módulo ESP-01.
 sendData("AT+CWMODE=3\r\n",1000); // ESP-01 en modo (3): enviar y recibir datos.
 sendData("AT+CIPMUX=1\r\n",1000); // Se configura para múltiples conexiones, aunque en principio solo se producirá una.
 sendData("AT+CIPSERVER=1,80\r\n",1000); // Se activa el servidor en el puerto 80.
 delay(1000);
 sendData("AT+CWJAP=\"WifiMovil\",\"12345678\"\r\n",1000);//Se indica el SSID y la contraseña de la red WIFI para establecer la conexión.
 delay(8000);
 sendDataURL("AT+CIFSR\r\n",1000); // Se obtiene la dirección IP a la que conectarse.
 delay(1000);
 // Se muestra la dirección IP por la pantalla LCD para que el usuario se conecte.
 lcd.setCursor(0,0);
 lcd.print("URL:"); 
 lcd.setCursor (0,1);
 lcd.print(URL);
 lcd.display();
 delay(500);
}


// Bucle principal
void loop(){
  
  if(Serial3.available()){

    if(primeraEjecucion == true){ // La primera vez que se reciben datos del ESP-01 se muestra la web

      LiquidCrystal_I2C lcd(0x27,16,2);
      lcd.init();
      lcd.noDisplay(); // Se apaga la pantalla una vez se ha conectado un cliente.
      
      if(Serial3.find("+IPD,")){
        delay(1000); // Este delay parece que se puede quitar.   
   
        int connectionId = Serial3.read() - 48; // Se resta 48 porque la función read() devuelve el código ASCII

        // Se prepara el servidor HTTP
        String webpage = "<head></head>"; // Cabecera de la página.
        webpage+="<h1 align=\"center\">Control del robot omnidireccional</h1>";
        webpage+="<br />";
        webpage+="<h2 align=\"center\"><button onclick=location=\"/ORDEN=2\">Enviar un 2</button>&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp";
        webpage+="<button onclick=location=\"/ORDEN=1\">Enviar un 1</button></h2>";
        webpage+="<h2 align=\"center\"><button onclick=location=\"/ORDEN=6\">Enviar un 6</button>&nbsp&nbsp&nbsp&nbsp";
        webpage+="<button onclick=location=\"/ORDEN=3\">Enviar un 3</button></h2>";
        webpage+="<h2 align=\"center\"><button onclick=location=\"/ORDEN=0\">Enviar un 0</button></h2>";
        webpage+="<h2 align=\"center\"><button onclick=location=\"/ORDEN=8\">Enviar un 8</button>&nbsp&nbsp&nbsp&nbsp";
        webpage+="<button onclick=location=\"/ORDEN=5\">Enviar un 5</button>&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp";
        webpage+="<button onclick=location=\"/ORDEN=7\">Enviar un 7</button>&nbsp&nbsp&nbsp&nbsp";
        webpage+="<button onclick=location=\"/ORDEN=4\">Enviar un 4</button></h2>";
        webpage+="<br/>";

        sendData("AT+CIPSEND=" + String(connectionId) + "," + webpage.length() + "\r\n", 500);
        sendData(webpage, 1000); 
        sendData("AT+CIPCLOSE=" + String(connectionId) + "\r\n", 1000); 
     
        primeraEjecucion = false; // Cuando ya se ha mostrado la web se pasa a comprobar las órdenes que se mandan desde la misma.
     } 

    }else{
      // Tras la primera ejecución se intenta recibir información procedente del ESP8266
      getData("AT+CIPRECVMODE=1\r\n", 50);
    }
  }

  // A partir de la orden recogida se activan los motores según proceda
  
  Secuencia_Actual_A = 0;
  Secuencia_Actual_B = 0;
  Secuencia_Actual_C = 0;
  
  for(int i = 0; i < 4; i++){
    moverMotores(identificador);
    delay(valorDelay);
  }
}

// Función para enviar datos a la URL
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

// Función para obtener la URL
String sendDataURL(String command, const int timeout){
    String response = "";
    Serial3.print(command); 
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial3.available()){ 
        char c = Serial3.read(); 
        response+=c;
      }  
    }
            
    for(int j = 121; j < 135; j++){
      URL += response[j];
    }

    if (response[135] != 34){
      URL += response[135];
    }
      
    return response;
}

// Función para recibir datos de la URL
void getData(String command, const int timeout){
    String response = "";
    Serial3.print(command); 
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial3.available()){ 
        char c = Serial3.read();
        response+=c;
      }  
    }
    
  // Se identifica la orden recibida
  if (response.indexOf("/ORDEN=0") != -1) {
    identificador = 0;
  } 
  if (response.indexOf("/ORDEN=1") != -1) {
   identificador = 1;
  } 
  if (response.indexOf("/ORDEN=2") != -1) {
    identificador = 2;
  }
  if (response.indexOf("/ORDEN=3") != -1) {
    identificador = 3;
  }
  if (response.indexOf("/ORDEN=4") != -1) {
    identificador =  4;
  }
  if (response.indexOf("/ORDEN=5") != -1) {
    identificador = 5;
  }
  if (response.indexOf("/ORDEN=6") != -1) {
    identificador = 6;
  }
  if (response.indexOf("/ORDEN=7") != -1) {
    identificador = 7;
  }
  if (response.indexOf("/ORDEN=8") != -1) {
    identificador = 8;
  }

}


// Función para el movimiento de los motores
void moverMotores(int identificador){
  
  // Motor 1 (M1)

  // Los identificadores que indican movimiento positivo de M1
  if(identificador == 1 || identificador == 3|| identificador == 4){
    Secuencia_Actual_A +=1;
  }
  // Los identificadores que indican movimiento negativo de M1
  if(identificador == 2 || identificador == 6 || identificador == 7){
    Secuencia_Actual_A -=1;
  }

  if (Secuencia_Actual_A == 4){
     Secuencia_Actual_A = 0;
  }
  if (Secuencia_Actual_A == -1){
     Secuencia_Actual_A = 3;
  }

  switch(Secuencia_Actual_A) {
  case 0:
      digitalWrite(M11, LOW); 
      digitalWrite(M12, LOW);  
      digitalWrite(M13, HIGH);  
      digitalWrite(M14, HIGH);
      break;
  case 1:
      digitalWrite(M11, LOW); 
      digitalWrite(M12, HIGH);  
      digitalWrite(M13, HIGH);  
      digitalWrite(M14, LOW);
      break;
  case 2:
      digitalWrite(M11, HIGH); 
      digitalWrite(M12, HIGH);  
      digitalWrite(M13, LOW);  
      digitalWrite(M14, LOW);
      break;
  case 3:
      digitalWrite(M11, HIGH); 
      digitalWrite(M12, LOW);  
      digitalWrite(M13, LOW);  
      digitalWrite(M14, HIGH);
      break;
  default:
      digitalWrite(M11, LOW); 
      digitalWrite(M12, LOW);  
      digitalWrite(M13, LOW);  
      digitalWrite(M14, LOW);
      break;
   
  }

  
  // Motor 2 (M2)

  // Los identificadores que indican movimiento positivo de M2
  if(identificador == 1 || identificador == 5|| identificador == 6){
    Secuencia_Actual_B +=1;
  }
  // Los identificadores que indican movimiento negativo de M2
  if(identificador == 2 || identificador == 3 || identificador == 8){
    Secuencia_Actual_B -=1;
  }

  if (Secuencia_Actual_B == 4){
     Secuencia_Actual_B = 0;
  }
  if (Secuencia_Actual_B == -1){
     Secuencia_Actual_B = 3;
  }

  switch(Secuencia_Actual_B) {
  case 0:
      digitalWrite(M21, LOW); 
      digitalWrite(M22, LOW);  
      digitalWrite(M23, HIGH);  
      digitalWrite(M24, HIGH);
      break;
  case 1:
      digitalWrite(M21, LOW); 
      digitalWrite(M22, HIGH);  
      digitalWrite(M23, HIGH);  
      digitalWrite(M24, LOW);
      break;
  case 2:
      digitalWrite(M21, HIGH); 
      digitalWrite(M22, HIGH);  
      digitalWrite(M23, LOW);  
      digitalWrite(M24, LOW);
      break;
  case 3:
      digitalWrite(M21, HIGH); 
      digitalWrite(M22, LOW);  
      digitalWrite(M23, LOW);  
      digitalWrite(M24, HIGH);
      break;
  default:
      digitalWrite(M21, LOW); 
      digitalWrite(M22, LOW);  
      digitalWrite(M23, LOW);  
      digitalWrite(M24, LOW);
      break;
   
  }

  // Motor 3 (M3)

  // Los identificadores que indican movimiento positivo de M3
  if(identificador == 1 || identificador == 7|| identificador == 8){
    Secuencia_Actual_C +=1;
  }
  // Los identificadores que indican movimiento negativo de M3
  if(identificador == 2 || identificador == 4 || identificador == 5){
    Secuencia_Actual_C -=1;
  }

  if (Secuencia_Actual_C == 4){
     Secuencia_Actual_C = 0;
  }
  if (Secuencia_Actual_C == -1){
     Secuencia_Actual_C = 3;
  }

  switch(Secuencia_Actual_C) {
  case 0:
      digitalWrite(M31, LOW); 
      digitalWrite(M32, LOW);  
      digitalWrite(M33, HIGH);  
      digitalWrite(M34, HIGH);
      break;
  case 1:
      digitalWrite(M31, LOW); 
      digitalWrite(M32, HIGH);  
      digitalWrite(M33, HIGH);  
      digitalWrite(M34, LOW);
      break;
  case 2:
      digitalWrite(M31, HIGH); 
      digitalWrite(M32, HIGH);  
      digitalWrite(M33, LOW);  
      digitalWrite(M34, LOW);
      break;
  case 3:
      digitalWrite(M31, HIGH); 
      digitalWrite(M32, LOW);  
      digitalWrite(M33, LOW);  
      digitalWrite(M34, HIGH);
      break;
  default:
      digitalWrite(M31, LOW); 
      digitalWrite(M32, LOW);  
      digitalWrite(M33, LOW);  
      digitalWrite(M34, LOW);
      break;
   
  }
  
}


