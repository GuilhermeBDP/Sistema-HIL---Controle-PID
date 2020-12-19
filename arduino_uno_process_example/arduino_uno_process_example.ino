//DECLARAÇÃO DOS PINOS DE SINAL
#define plantIn A3 // Entrada do arduino(planta) que vem do esp32(pid) CASO O SINAL FOR Analogico
#define plantOut 5 // Saida do arduino(planta) que entra no esp32(pid) passando pelo filtro RC
#define botao 4

//LED INTERNO
#define pinLED 13 // Declaração do pino do LED interno
#define nivelLED LOW // Estado inicial do LED interno
bool estado = HIGH;

// Intervalo do timer
#define tempoTimer 1000

//BIBLIOTECAS
#include <TimerOne.h>

//FUNCOES
void planta();

//VARIAVEIS DA PLANTA
/*
 * y: primeiro integrador, y2: segundo integrador; T: constante de tempo
 * kg: ganho da planta, k1 e k2: parâmetros da planta,
 * fatorA: fator de limite da ação de controle, ucorrigida: esforço de controle corrigido.
 * y2corrigida: saída do segundo integrador corrigida
 */
 
double y, y2, T, u, kg, k1, k2, sum, fatorA, ucorrigida, y2corrigida;
int aux;

void setup(){
  bool btcontrol = true; // Liga-desliga função de controle por botao
  Serial.begin(9600);

  if(btcontrol){
    Serial.println("Para iniciar o programa, aperte o botão.");
    pinMode(botao, INPUT);
    while (estado != LOW){
    Serial.println("Aperte o botao.");
    delay(500);
    estado = digitalRead(botao);    
    }
  }
  
  pinMode(plantOut, OUTPUT);

  Serial.println("/--INICIANDO PROGRAMA--/");
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, !nivelLED);

  //INICIO DO TIMER
  Timer1.initialize(tempoTimer); // Inicializa o timer com o tempo estipulado
  Timer1.attachInterrupt(planta); // Define a função "planta" como a função a ser executada
                                  // quando a interrupção for disparada

  //CONFIGURAÇâO INICIAL DA ENTRADA
  u = 0;

  //CONFIGURAÇâO INICIAL DA PLANTA
  y = 0;
  y2 = 0;
  k1 = 10;
  k2 = 3;
  T = 0.001; // resolução (aumenta a precisão)
  kg = 1;
  fatorA = 0.1;
  sum = 0;
  aux = 0;
  
  Serial.println("/--PROGRAMA INICIADO--/");
}

void loop(){
  // Escrita de alguns parâmetros da planta na porta Serial
  Serial.print("y2: "); Serial.print(y2);
  Serial.print(" Entrada: "); Serial.print(u);
  Serial.print(" Saida: "); Serial.println(y2corrigida);
}

void planta() {
  u = analogRead(plantIn);
  ucorrigida = u * 1.515151 * 4;
  ucorrigida = (ucorrigida * 32) - 65536; // Descompressão
  ucorrigida = ucorrigida * (1/fatorA);
  //---------------------------------------------
  sum = ucorrigida - (k1 * y) - (k2 * y2);
  y = y + T * sum * kg;
  y2 = y2 + T * y * kg;
  //---------------------------------------------
  y2corrigida = y2 + 65536;
  if (y2corrigida > 131072){
    y2corrigida = 131072;
  }
  if (y2corrigida < 0){
    y2corrigida = 0;
  }  
  y2corrigida = y2corrigida*0.03125*0.25*0.25*0.66; // Compressão e conversão
  if (y2corrigida > 169){
    y2corrigida = 169;
  }
  if (y2corrigida < 0){
    y2corrigida = 0;
  }
  analogWrite(plantOut, y2corrigida);
}
