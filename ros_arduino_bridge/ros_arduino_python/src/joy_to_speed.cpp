// Programa para la generacion de un topico en el que se publiquen las velocidades de los motores
// a partir de los datos recibidos del nodo joy_node

// Autor: Alberto Diaz Rodriguez
// Universidad de La Laguna


// Se incluyen las librerias necesarias
#include <ros/ros.h> // Incluye algunas cabeceras de uso habitual en ROS.
#include <geometry_msgs/Twist.h> // Para poder trabajar con este tipo de mensaje.
#include <sensor_msgs/Joy.h> // Para poder leer los mensajes enviados por el joystick.
#include "std_msgs/String.h" // Variables tipo string
#include <sstream> // Para la salida de datos
#include <cmath> // Para utilizar las funciones con las que calcular raices y arcotangentes
#include <geometry_msgs/Vector3.h> // Variable de tipo Vector3 

#define MODMAX 100 // El valor maximo que se puede recibir del eje de un joystick
#define MAXVEL 206 // Valor maximo que se permite que sean las velocidades generadas

// Variables globales
geometry_msgs::Twist twistJoystickIzq;
geometry_msgs::Twist twistJoystickDer;

// Variables para la velocidad de los motores
float m1_vel = 0;
float m2_vel = 0;
float m3_vel = 0;

// Variable para corregir valores mayores al maximo
float corregirVel = 0;

class ControlRemoto
{
public:
  ControlRemoto(); // Constructor por defecto de la clase

private:
  // Metodo de la clase
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); // Lee el mensaje enviado por el joystick
  
  
  // Atributos de la clase
  ros::NodeHandle n_; // Punto de acceso para comunicaciones ROS.
  
  int linear_, angular_;
  

  ros::Subscriber joy_sub_; // Nodo
};

// Se inicializan variables con las que se podra acceder a los datos del mando deseados
ControlRemoto::ControlRemoto():
  linear_(1),
  angular_(2)
{

  n_.param("axis_linear", linear_, linear_);
  
  // Se subscribe el nodo al topico joy para recibir los datos del mando
  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 10, &ControlRemoto::joyCallback, this);

}

// Funcion en la que se recogen los valores de los ejes de los josticks
void ControlRemoto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Ejes X e Y del joystick izquierdo
  twistJoystickIzq.linear.x = -100 * joy->axes[linear_ - 1];
  twistJoystickIzq.linear.y = 100 * joy->axes[linear_];

  // Eje X del joystick derecho
  twistJoystickDer.linear.x = -100 * joy->axes[linear_ + 2]; 
    
}

// Funcion para la determinacion de los valores de las velocidades de los motores
void getVel(){
  // Se inicializan las velocidades a 0
  m1_vel = 0;
  m2_vel = 0;
  m3_vel = 0;

  // La velocidad de cada motor se determinara en base a la contribucion de cada eje.
  
  // Contribucion del eje X del joystick derecho (Rotacion pura).
  
  m1_vel = 255*twistJoystickDer.linear.x/MODMAX;
  m2_vel = 255*twistJoystickDer.linear.x/MODMAX;
  m3_vel = 255*twistJoystickDer.linear.x/MODMAX;
  
  // Contribucion del eje Y del joystick izquierdo
  
  m1_vel += 255*twistJoystickIzq.linear.y/MODMAX;
  m2_vel -= 255*twistJoystickIzq.linear.y/MODMAX;
  m3_vel += 0;
  
  // Contribucion del eje X del joystick izquierdo
  
  m1_vel += 147.23*twistJoystickIzq.linear.x/MODMAX;
  m2_vel += 147.23*twistJoystickIzq.linear.x/MODMAX;
  m3_vel -= 294.46*twistJoystickIzq.linear.x/MODMAX;
  
  // Se reescalan los resultados para hacer que el robot no se mueva excesivamente lento
  // en caso de rotacion pura o traslacion pura
  
  // Rotacion pura
    if(twistJoystickIzq.linear.x == 0 && twistJoystickIzq.linear.y == 0){
      m1_vel *= 2.4;
      m2_vel *= 2.4;
      m3_vel *= 2.4;
    }
  
  // Traslacion pura
  if(twistJoystickDer.linear.x == 0){
      m1_vel *= 1.7;
      m2_vel *= 1.7;
      m3_vel *= 1.7;
    }
  
  // Se hace la media de las 3 contribuciones
  m1_vel = m1_vel/3;
  m2_vel = m2_vel/3;
  m3_vel = m3_vel/3;
  
  /*
  // Este fragmento es si se quiere utilizar el metodo de aproximar al maximo posible los valores
  // Es posible que tras el reescalado el resultado supere el maximo valor de velocidad. Se corrige
  
  if(abs(m1_vel) > MAXVEL || abs(m2_vel) > MAXVEL || abs(m3_vel) > MAXVEL){
    // Se averigua cual es el mayor
    if(abs(m1_vel) >= abs(m2_vel)){
      if(abs(m1_vel) >= abs(m3_vel)){
        corregirVel = abs(m1_vel)/MAXVEL;
      }
      else{
        corregirVel = abs(m3_vel)/MAXVEL;
      }
    }else{
      if(fabs(m2_vel) >= fabs(m3_vel)){
        corregirVel = abs(m2_vel)/MAXVEL;
      }else{
        corregirVel = abs(m3_vel)/MAXVEL;
      }
    }
    m1_vel = m1_vel/corregirVel;
    m2_vel = m2_vel/corregirVel;
    m3_vel = m3_vel/corregirVel;
  }
  */
  
  // Se convierte el resultado en un entero antes de enviarlo
  m1_vel = int(m1_vel);
  m2_vel = int(m2_vel);
  m3_vel = int(m3_vel);
  
}

// Se inicia el main
int main(int argc, char** argv)
{

  ros::init(argc, argv, "joy_to_speed"); // Se inicializa el nodo ROS.
  ros::NodeHandle n2;
  
  // Se publicara en el topico motors_speed la velocidad de los motores
  ros::Publisher chatter_pub = n2.advertise<geometry_msgs::Vector3>("motors_speed", 1000);
  ros::Rate loop_rate(5); // Frecuencia de publicacion de los mensajes
  ControlRemoto mando; //Se crea un objeto de la clase ControlRemoto

  
  while (ros::ok())
  {
  
  // Se determinan las velocidades de los motores
  getVel();
  
  
  // Se introducen las velocidades de los tres motores en un vector de 3 posiciones
  
  geometry_msgs::Vector3 msg;

  msg.x = m1_vel;
  msg.y = m2_vel;
  msg.z = m3_vel;

  // Se publican las velocidades
  chatter_pub.publish(msg);

  ros::spinOnce();

  loop_rate.sleep();
  }
}
