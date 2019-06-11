// Implementación de un servidor web utilizando la placa Arduino WeMos D1
// Autor: Alberto Díaz Rodríguez
// Fecha: 9/11/18
// Universidad de La Laguna

// Librerías
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

// Variables

int identificador = 0; // Variable que almacenará el número de movimiento que se debe realizar.
int valorDelay = 2; // Valor que determina la velocidad de giro de los motores, en este caso es una simulación.

// Variables que almacenan los pasos que da cada motor (el signo indica el sentido en el que ha girado).
float pasosM1 = 0; 
float pasosM2 = 0;
float pasosM3 = 0;

// Variables que almacenan los grados que ha girado cada motor (el signo indica el sentido en el que ha girado). Se trabaja en grados.
float gradosM1 = 0;
float gradosM2 = 0;
float gradosM3 = 0;

// Se introducen los datos del punto de acceso
const char* ssid = "WifiMovil";
const char* password = "12345678";

String orden =  "";

WiFiServer server(80); //Se crea el servidor en el puerto 80


// Configuración inicial
void setup() {

  // Se inicia la cominicación serial
  Serial.begin(74880);

  // Se realiza la conexión a la red WIFI
  Serial.print("\nConectando a la red: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);  // ESP8266 en modo estación
  WiFi.begin(ssid, password); // ssid y contraseña de la red

  while (WiFi.status() != WL_CONNECTED) { // Se espera hasta que se haya establecido la conexión
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectada"); // Se ha realizado la conexión correctamente

  // Se inicia el servidor
  server.begin();
  Serial.println("Servidor creado.");

  // Se muestra la dirección IP a la que hay que conectarse para interactuar con el software
  Serial.print("URL : http://");
  Serial.println(WiFi.localIP()); 

  delay(6000); // Tiempo que se ofrece para introducir la IP en un navegador en milisegundos
}


// Bucle principal
void loop() {
   
  // Se busca un cliente activo
  WiFiClient client = server.available();

  // Se llama a la función que simula el movimiento de los motores
  for(int i = 0; i < 4; i++){
    moverMotores(identificador);
    delay(valorDelay);
  }
  
  if (!client) {
    return;
  }
  

  // Se espera la respuesta del cliente
  while(!client.available()){
    delay(1);
  }
    
  // Se lee la orden enviada por el cliente
  orden = client.readStringUntil('\r');
  client.flush();
  
  // Se averigua el identificador enviado

  if (orden.indexOf("/ORDEN=0") != -1) {
    identificador = 0;
  } 
  if (orden.indexOf("/ORDEN=1") != -1) {
    identificador = 1;
  } 
  if (orden.indexOf("/ORDEN=2") != -1) {
    identificador = 2;
  }
  if (orden.indexOf("/ORDEN=3") != -1) {
    identificador = 3;
  }
  if (orden.indexOf("/ORDEN=4") != -1) {
    identificador = 4;
  }
  if (orden.indexOf("/ORDEN=5") != -1) {
    identificador = 5;
  }
  if (orden.indexOf("/ORDEN=6") != -1) {
    identificador = 6;
  }
  if (orden.indexOf("/ORDEN=7") != -1) {
    identificador = 7;
  }
  if (orden.indexOf("/ORDEN=8") != -1) {
    identificador = 8;
  }

  // Configuración servidor HTTP
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("");
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");


  // Se muestra al cliente el identificador actual

  switch(identificador) {
    case 0: client.print("Identificador actual: 0");break;
    case 1: client.print("Identificador actual: 1");break;
    case 2: client.print("Identificador actual: 2");break;
    case 3: client.print("Identificador actual: 3");break;
    case 4: client.print("Identificador actual: 4");break;
    case 5: client.print("Identificador actual: 5");break;
    case 6: client.print("Identificador actual: 6");break;
    case 7: client.print("Identificador actual: 7");break;
    case 8: client.print("Identificador actual: 8");break;
  }

  // Se muestran las opciones que puede escoger el cliente para enviar órdenes
  client.println("<br><br>");
  client.println("Haz click <a href=\"/ORDEN=0\">aqui</a> para enviar un 0<br>");
  client.println("Haz click <a href=\"/ORDEN=1\">aqui</a> para enviar un 1<br>");
  client.println("Haz click <a href=\"/ORDEN=2\">aqui</a> para enviar un 2<br>");
  client.println("Haz click <a href=\"/ORDEN=3\">aqui</a> para enviar un 3<br>");
  client.println("Haz click <a href=\"/ORDEN=4\">aqui</a> para enviar un 4<br>");
  client.println("Haz click <a href=\"/ORDEN=5\">aqui</a> para enviar un 5<br>");
  client.println("Haz click <a href=\"/ORDEN=6\">aqui</a> para enviar un 6<br>");
  client.println("Haz click <a href=\"/ORDEN=7\">aqui</a> para enviar un 7<br>");
  client.println("Haz click <a href=\"/ORDEN=8\">aqui</a> para enviar un 8<br>");
  
  // Se muestran también los grados girados
  // La relación entre los pasos y los grados girados es: 512 pasos son 360 grados. Por tanto:

  gradosM1 = float(pasosM1 * 360 / 4 * 512);
  gradosM2 = float(pasosM2 * 360 / 4 * 512);
  gradosM3 = float(pasosM3 * 360 / 4 * 512);

  client.println("<br>");
  client.println("<br>");
  client.println("Los grados girados por cada motor son:<br>");
  client.print("GradosM1: ");
  client.println(gradosM1);
  client.println("<br>");
  client.print("GradosM2: ");
  client.println(gradosM2);
  client.println("<br>");
  client.print("GradosM3: ");
  client.println(gradosM3);
  client.println("</html>");
  Serial.println("");
  
}

// Función para la simulación del movimiento de los motores
void moverMotores(int identificador){
  
  // Motor 1 (M1)

  // Los identificadores que indican movimiento positivo de M1
  if(identificador == 1 || identificador == 3|| identificador == 4){ 
    pasosM1 ++;
  }
  // Los identificadores que indican movimiento negativo de M1
  if(identificador == 2 || identificador == 6 || identificador == 7){
    pasosM1 --;
  }
   
  // Motor 2 (M2)

  // Los identificadores que indican movimiento positivo de M2
  if(identificador == 1 || identificador == 5|| identificador == 6){
    pasosM2 ++;
  }
  // Los identificadores que indican movimiento negativo de M2
  if(identificador == 2 || identificador == 3 || identificador == 8){
    pasosM2 --;
  }

  // Motor 3 (M3)

  // Los identificadores que indican movimiento positivo de M3
  if(identificador == 1 || identificador == 7|| identificador == 8){ 
    pasosM3 ++;
  }
  // Los identificadores que indican movimiento negativo de M3
  if(identificador == 2 || identificador == 4 || identificador == 5){
    pasosM3 --;
  }

}


