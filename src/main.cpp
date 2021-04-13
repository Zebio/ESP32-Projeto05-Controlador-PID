#include <Arduino.h>
/*---------------- Mapeamento de Hardware ----------------*/
#define pin_sensor_luz  13
#define led1            32
#define led2            33


/*---------------- Constantes de Projeto   ---------------*/
#define dt 100


/*---------------- variáveis GLobais  --------------------*/
float     kp =-4,
          ki =2,
          kd =0.12;



/*---------------- Declaração das Funções  ---------------*/
float controle_PID(float,float);


/*--------------- Inicialização do Sistema----------------*/
void setup() {
  Serial.begin(115200);
  ledcAttachPin(led1,0);
  ledcAttachPin(led2,0);
  ledcSetup(0,10000, 12);
}


/*--------------- Loop infinito -------------------------*/
void loop() {
  float sensor_luz=analogRead(pin_sensor_luz);
  Serial.println(sensor_luz);
  float pid=(controle_PID(sensor_luz,1500))+2048;
  ledcWrite(0,pid);
  delay(100);
}


/*------------ Desenvolvimento das Funções ---------------*/
float controle_PID(float medida,float setpoint)
{
  float  erro;
  float proporcional;
  float derivativa;
  static float ultima_medida;
  static float integral;

  erro = setpoint - medida;

  proporcional = kp*erro;

  integral -= (erro*ki)*(dt/1000.0);

  derivativa = ((ultima_medida - medida)*kd)/(dt/1000.0);

  ultima_medida = medida;

  return (proporcional + integral + derivativa);
}