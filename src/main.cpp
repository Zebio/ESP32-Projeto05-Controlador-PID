#include <Arduino.h>
#include <string>
#include <stdlib.h>

/*---------------- Mapeamento de Hardware ----------------*/
#define pin_sensor_luz  13
#define led1            32
#define led2            33


/*---------------- Constantes de Projeto   ---------------*/
#define dt 0.1



/*---------------- constantes globais      ---------------*/
const int numChars  = 32;
const int Setpoint  = 2500;
const float zero=0;


/*---------------- variáveis GLobais  --------------------*/
float     kp =1,
          ki =0,
          kd =0;
char      charsRecebidos[numChars];
char      charTemporario[numChars];
bool      novosDados =false;



/*---------------- Declaração das Funções  ---------------*/
void recebeDadosSeriais(); //recebe dados das constantes para tunagem do PID
void atualiza_PID();  //atualiza os valores das constantes PID
float controle_PID(float,float);  //realiza o cálculo baseado nas constantes, valor lido e setpoint


/*--------------- Inicialização do Sistema----------------*/
void setup() {
  Serial.begin(115200);   //inicia a comunicação serial 
  ledcSetup(0,10000, 12); //inicia o canal PWM 0, com 10000 de frequencia e resolução de 12 bits

  ledcAttachPin(led1,0);  //liga o led1 ao canal PWM 0
  ledcAttachPin(led2,0);  //liga o led2 ao canal PWM 0

}

/*--------------- Loop infinito -------------------------*/
void loop() {
  recebeDadosSeriais(); //verifica se foi mandado algum dado pela porta serial, se sim seta a flag "novos dados"
                        //e armazena no char[] charsRecebidos

  if (novosDados)       //verifica a flag
  {
    strcpy(charTemporario, charsRecebidos); //se foram enviados dados pelo serial, copia esses dados para
                                            //"charTemporario" 
    atualiza_PID();     //a partir dos dados de "charTemporario" atualiza os valores PID baseado
                        //naquiles que foram enviados pelo terminal
    novosDados =false;  //agora podemos voltar a flag ao estado inicial
  }

  uint16_t sensor_luz=4097-analogRead(pin_sensor_luz);  //usa o adc para receber o valor de luminosidade atual


  float pid=(controle_PID(sensor_luz,Setpoint)); //chamamos a função de controle PID
                                                                  //baseado no valor lido "sensor_luz"
                                                                  //e no valor desejado "Setpoint"
                                                                  //somamos com 2048 pois é a metade do PWM de 12 bits

  if(pid<0)
   pid=0;  
  if(pid>4094)
    pid=4094;
  Serial.print("Leitura:"); 
  Serial.print(sensor_luz);                   //imprimimos a luminosidade atualo no Serial Monitor
  Serial.print("  PID:");
  Serial.print(pid);
  Serial.print(" 0:");
  Serial.print(zero);
  Serial.print(" Setpoint:");
  Serial.println(
Setpoint);


  ledcWrite(0,pid); //aqui escrevemos no canal 0 (led1 e led2) do PWM o valor processado pelo pid
  
  delay(dt*1000);        //delay em ms do mesmo valor da integral e derivada do pid.
}


/*------------ Desenvolvimento das Funções ---------------*/


void recebeDadosSeriais()
//Essa função foi baseada num guia: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
//Onde ele explica muito bem varias maneiras de obter dados das portas seriais, lá esta tudo explicado
//veja o exemplo 5.
//Esta função espera receber os dados via serial no formato: "(kp,ki,kd)" 
{
  static bool recebendoDados=false;
  static int indice = 0;
  char charInicio = '(';
  char charFinal  = ')';
  char charRecebido;

  while (Serial.available()>0 && novosDados == false)
  {
    charRecebido = Serial.read();

    if (recebendoDados ==true)
    {
      if (charRecebido !=charFinal)
      {
        charsRecebidos[indice]=charRecebido;
        indice ++;
        if (indice>=numChars)
        {
          indice=numChars-1;
        }
      }
    else{
      charsRecebidos[indice]= '\0';
      recebendoDados =false;
      indice =0;
      novosDados = true;
      }
    }
    else if (charRecebido==charInicio)
      recebendoDados=true;
  }
}

void atualiza_PID()
//nessa função vamos dividir os dados enviados pelo terminal por Virgulas(,) usando strtok e
// atualizar os valores das constantes PID baseado nesses valores enviados.
// lembrando que os dados enviados pelo usuario no monitor tem que estar no formato (kp,ki,kd)
// senão haverá erro e o sistema vai reiniciar
{
  char * indiceStrtok;

  indiceStrtok=strtok(charTemporario,",");
  kp=atof(indiceStrtok);

  indiceStrtok=strtok(NULL,",");
  ki=atof(indiceStrtok);

  indiceStrtok=strtok(NULL,",");
  kd=atof(indiceStrtok);

}

float controle_PID(float output,float setpoint)
{
  static float output_anterior,integral;

  float        erro,proporcional,derivativa;

  erro = setpoint - output;

  proporcional = kp*erro;

  integral = integral + (erro*ki) * (dt);

  derivativa = ((output_anterior - output)*kd) / (dt);

  output_anterior = output;

  float sinal_de_controle=(proporcional+integral+derivativa);

  return sinal_de_controle;
}
