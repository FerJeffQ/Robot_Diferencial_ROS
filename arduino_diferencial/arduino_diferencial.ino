#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <PID_v1.h>
#include <ros/time.h>



ros::NodeHandle  nh;

//Asignacion de pines para motores
Motor right(8,9,20,21); //(IN1, IN2, verde-20, naraja-21)
Motor left(10,11,2,3);  //(IN3, IN4,  verde-2, naranja-3)  

//Pines de Enables de cada motor
int enableA = 6;
int enableB = 7;

//--------------------------Variables para valor de los encoders
volatile long encoder0Pos = 0;    
volatile long encoder1Pos = 0;    


volatile byte ant_left = 0;
volatile byte act_left = 0;

volatile byte ant_right = 0;
volatile byte act_right = 0;

//---------------------------Valores de PID en este caso solo proporcional
double left_kp = 10, left_ki = 0  , left_kd = 0;             
double right_kp = 10, right_ki = 0 , right_kd = 0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT); 

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

unsigned long currentMillis;
unsigned long prevMillis;

float demandx=0;
float demandz=0;

double demand_speed_left=0;
double demand_speed_right=0;

float encoder0Diff;
float encoder1Diff;


float encoder0Prev;
float encoder1Prev;

double pos_act_left = 0;                    
double pos_act_right = 0;   

//--------------------------------ROS---------------------------------
//---------Para lectura de velocidad lineal y angular
void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}
// Suscripcion a topico cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );   


// Variables de mensaje encoders 
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel" , &left_wheel_msg);
std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel" , &right_wheel_msg);                        
//--------------------------------ROS---------------------------------
                 

void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(left_wheel_pub); 
  nh.advertise(right_wheel_pub);  


  Serial.begin(57600);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
  //Configuracion de pines control de los motores ENA, ENB
  digitalWrite (enableA, HIGH);
  digitalWrite (enableB, HIGH);
 
  
  //Pines de interrupcion de enconders con funciones de incremento y decremento
  attachInterrupt(digitalPinToInterrupt(left.en_a), encoder_left, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(left.en_b), encoder_left, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(right.en_a), encoder_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b), encoder_right, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= 10){
    prevMillis = currentMillis;

    //velocidad motor - distancia entre llantas
    demand_speed_left = demandx - (demandz*0.24); //es longitud entre llantas 
    demand_speed_right = demandx + (demandz*0.24);

    pos_act_left = encoder0Pos;  
    pos_act_right = encoder1Pos;
  
    //PID controller for speed control
    //Obtenga la diferencia entre los ticks para calcular la velocidad
    encoder0Diff = encoder0Pos - encoder0Prev; 
    encoder1Diff = encoder1Pos - encoder1Prev;


    encoder0Prev = encoder0Pos; 
    encoder1Prev = encoder1Pos;

    // para obtener el valor:  1800 ticks 1n 1 metro * 10 ms (numero de ticks en una vuelta)

    left_setpoint = demand_speed_left*18;  
    right_setpoint = demand_speed_right*18;
  
    // La entrada al controlador PID es la diferencia actual
    left_input = encoder0Diff; 
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);


    publishPos();
    nh.spinOnce();
  }

 }
    

//Publish function for odometry
void publishPos() {
 left_wheel_msg.data = pos_act_left;      
 right_wheel_msg.data = pos_act_right;    
 left_wheel_pub.publish(&left_wheel_msg);   
 right_wheel_pub.publish(&right_wheel_msg);         

}


// ************** encoders interrupts **************


// Encoder precision cuadruple
void encoder_left(void) {
  ant_left = act_left;

  if (digitalRead(left.en_a)) {
    bitSet(act_left, 0);
  } else {
    bitClear(act_left, 0);
  }
  if (digitalRead(left.en_b)) {
    bitSet(act_left, 1);
  } else {
    bitClear(act_left, 1);
  }

  if (ant_left == 2 && act_left == 0) encoder0Pos++;
  if (ant_left == 0 && act_left == 1) encoder0Pos++;
  if (ant_left == 3 && act_left == 2) encoder0Pos++;
  if (ant_left == 1 && act_left == 3) encoder0Pos++;

  if (ant_left == 1 && act_left == 0) encoder0Pos--;
  if (ant_left == 3 && act_left == 1) encoder0Pos--;
  if (ant_left == 0 && act_left == 2) encoder0Pos--;
  if (ant_left == 2 && act_left == 3) encoder0Pos--;
}


// Encoder precision cuadruple
void encoder_right(void) {
  ant_right = act_right;

  if (digitalRead(right.en_b)) {
    bitSet(act_right, 0);
  } else {
    bitClear(act_right, 0);
  }
  if (digitalRead(right.en_a)) {
    bitSet(act_right, 1);
  } else {
    bitClear(act_right, 1);
  }

  if (ant_right == 2 && act_right == 0) encoder1Pos++;
  if (ant_right == 0 && act_right == 1) encoder1Pos++;
  if (ant_right == 3 && act_right == 2) encoder1Pos++;
  if (ant_right == 1 && act_right == 3) encoder1Pos++;

  if (ant_right == 1 && act_right == 0) encoder1Pos--;
  if (ant_right == 3 && act_right == 1) encoder1Pos--;
  if (ant_right == 0 && act_right == 2) encoder1Pos--;
  if (ant_right == 2 && act_right == 3) encoder1Pos--;
}
