#include <PID_v1.h>

//controle PID
  double Setpoint, pidIN, pidOUT; //Entradas PID

  //Specify the links and initial tuning parameters
//  double Kp=11.94, Ki=0, Kd=0.025;
    double Kp=9.94, Ki=0, Kd=0.6;
  PID myPID(&pidIN, &pidOUT, &Setpoint, Kp, Ki, Kd, DIRECT);

//Entradas
const int button1 = 2;
const int sysIN = A0; // pino de entrada do potenciômetro
const int setPointPIN= A2;

//Valores
int ERRO;
double lowerLimit=80;
double upperLimit=255;
volatile bool button=0;
unsigned int set;

//Interface ponte H
const int EN1 = 11;
const int IN1 = 9;
const int IN2 = 10;
int IN1e = 0;
int IN2e = 0; 
//___________________________________________________
void setup() {
Serial.begin(230400);
pinMode(IN1,OUTPUT);  //ponte H - Motor direto
pinMode(IN2,OUTPUT);  //ponte H - Motor reverso
pinMode(EN1,OUTPUT);  //ponte H - Motor habilitação controle PWM
pinMode(button1, INPUT);

pinMode(sysIN, INPUT);      //entrada do sistema potenciometro A0
pinMode(setPointPIN, INPUT);   //potenciometro setpoint A2

  attachInterrupt(digitalPinToInterrupt(button1), buttonISR, HIGH);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(lowerLimit,upperLimit);
  myPID.SetSampleTime(1);

//Setpoint=165;
}

void loop() {

pidIN = map(analogRead(sysIN),0,1023,0,255);
Setpoint = map(analogRead(setPointPIN),0,1023,96,229);
//Setpoint=165;

  ERRO = Setpoint - pidIN;

if(ERRO > 1){
  IN1e= 0;
  IN2e= 1;
    myPID.SetOutputLimits(lowerLimit,upperLimit);
  myPID.SetControllerDirection(DIRECT);
}
if(ERRO < -1){
  IN1e= 1;
  IN2e= 0;
    myPID.SetOutputLimits(lowerLimit,upperLimit);
  myPID.SetControllerDirection(REVERSE);
}
if(ERRO == 0 || ERRO == -1 || ERRO == 1){
  IN1e= 0;
  IN2e= 0;
//  myPID.SetOutputLimits(0,255);

}


 digitalWrite(IN1, IN1e);
 digitalWrite(IN2, IN2e);

 
myPID.Compute();
analogWrite(EN1,pidOUT);  //ponte H - Motor habilitação controle PWM

    if(button==1){
    analogWrite(11,0);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    delay(8000);
    button=0;
    }
//Serial.println(pidIN);
//Serial.println(Setpoint);

//  dummy = int(PID);
////
  Serial.print("Ref: ");
  Serial.print(Setpoint);
  Serial.print(","); 
  Serial.print("Erro: ");
  Serial.print(ERRO);
  Serial.print(",");
  Serial.print("Sensor: ");
  Serial.println(pidIN);
//  Serial.print(",");

//  Serial.print("  ");
//  Serial.print("Controle: ");
//  Serial.println(pidOUT);
//  Serial.print("  ");
////  Serial.write("PID: ");
////  Serial.write(dummy);
//  Serial.write("\n");

}

void buttonISR(){
  button=1;
}
