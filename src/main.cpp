#include <Arduino.h>
#include <string>
#include <stdlib.h>

/*---------------- Mapeamento de Hardware ----------------*/
#define pin_sensor_luz  13
#define led1            32
#define led2            33


/*---------------- Constantes de Projeto   ---------------*/
#define dt 100



/*---------------- constantes globais      ---------------*/
const char espaco = ' ';


/*---------------- variáveis GLobais  --------------------*/
float     kp =-4,
          ki =2,
          kd =0.12;



/*---------------- Declaração das Funções  ---------------*/
float controle_PID(float,float);
void atuliza_PID();


/*--------------- Inicialização do Sistema----------------*/
void setup() {
  Serial.begin(115200);
  ledcAttachPin(led1,0);
  ledcAttachPin(led2,0);
  ledcSetup(0,10000, 12);
}


/*--------------- Loop infinito -------------------------*/
void loop() {
  atuliza_PID();
  float sensor_luz=analogRead(pin_sensor_luz);
  Serial.print(kp);
  Serial.print("  ");
  Serial.println(sensor_luz);
  float pid=(controle_PID(sensor_luz,1500))+2048;
  if (pid>4095)
    pid =4095;
  if (pid<0)
    pid =0;
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


void atuliza_PID()
{
  if (Serial.available())
  {
    String serial = Serial.readString();
    String  serialp,
            seriali,
            seriald; 
    int i=0;
    while (serial[i]!=',')
    {
      serialp+=serial[i];
      i++;
    }
    i++;
    while (serial[i]!=',')
    {
      seriali+=serial[i];
      i++;
    }
    i++;
    while (serial[i]!=',')
    {
      seriald+=serial[i];
      i++;
    }
    Serial.println(serialp);
    Serial.println(seriali);
    Serial.println(seriald);
    

    /*
    String valoresPID=Serial.readString();
    char *cstr[20];
    strcpy(*cstr, valoresPID.c_str());
    char virgula = ',';
    kp=atof(strtok(*cstr,&virgula));
    ki=atof(strtok(*cstr,&virgula));
    kd=atof(strtok(*cstr,&virgula));
  

  kp=atof(Serial.readString().c_str());
  */}
}