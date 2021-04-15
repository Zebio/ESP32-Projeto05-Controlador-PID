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
const int numChars  = 32;
const int luminosidadeDesejada  = 1500;


/*---------------- variáveis GLobais  --------------------*/
float     kp =0,
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
  ledcAttachPin(led1,0);  //liga o led1 ao canal PWM 0
  ledcAttachPin(led2,0);  //liga o led2 ao canal PWM 0
  ledcSetup(0,10000, 12); //inicia o canal PWM 0, com 10000 de frequencia e resolução de 12 bits
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

  float sensor_luz=analogRead(pin_sensor_luz);  //usa o adc para receber o valor de luminosidade atual
  Serial.print(sensor_luz);                   //imprimimos a luminosidade atualo no Serial Monitor

  float pid=(controle_PID(sensor_luz,luminosidadeDesejada))+2048; //chamamos a função de controle PID
                                                                  //baseado no valor lido "sensor_luz"
                                                                  //e no valor desejado "luminosidadeDesejada"
                                                                  //somamos com 2048 pois é a metade do PWM de 12 bits
  
  if (pid>4095)   //Nesses 2 IFs vamos garantir que se estourarmos os limites do PWM (0 a 4095) manteremos
    pid =4095;    //os limites na borda.
    
  if (pid<0)
    pid =0;

  ledcWrite(0,pid); //aqui escrevemos no canal 0 (led1 e led2) do PWM o valor processado pelo pid

  Serial.print("  kp: ");
  Serial.print(kp);
  Serial.print("ki: ");
  Serial.print(ki);
  Serial.print("kd: ");
  Serial.println(kd);
  
  delay(dt);        //delay em ms do mesmo valor da integral e derivada do pid.
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
