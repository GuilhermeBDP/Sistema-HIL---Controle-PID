#include <WiFi.h>
#include <PubSubClient.h>

// Fatores recebidos via mqtt
double kpr, kir, kdr, far, fbr;

// WIFI E MQTT
// Definições do LED
#define PIN_LED     25
// Definições do MQTT
#define TOPICO_SUBSCRIBE_KP         "topico_ler_kp"
#define TOPICO_SUBSCRIBE_KI         "topico_ler_ki"
#define TOPICO_SUBSCRIBE_KD         "topico_ler_kd"
#define TOPICO_SUBSCRIBE_FA         "topico_ler_fa"
#define TOPICO_SUBSCRIBE_FB         "topico_ler_fb"
#define TOPICO_PUBLISH_RESP          "topico_envia_resp" 
#define ID_MQTT  "esp32_mqtt_GUIBDP"

// Definição das conexões 
const char* SSID = "Nome da Rede"; // Nome da rede WI-FI a se conectar
const char* PASSWORD = "Senha da rede"; // Senha da rede WI-FI a se conectar
const char* BROKER_MQTT = "broker.hivemq.com"; //URL do broker MQTT
int BROKER_PORT = 1883; // Porta do broker MQTT

// Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
WiFiServer server(80);

String header;

PubSubClient MQTT(espClient); // Cria o cliente MQTT
 
// Declaração das funções
void initWiFi(void);
void initMQTT(void);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT(void);
void reconnectWiFi(void);
void VerificaConexoesWiFIEMQTT(void);

// Inicia a conexão com o Wifi
void initWiFi(void) 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
      
    reconnectWiFi();
}
 
// Inicializa a conexão com o Broker MQTT
void initMQTT(void) 
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   // Informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            // Atribui função de callback
}


 
// Função que lê a informação dos tópicos subescritos
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    String msg;
    String topico;
  
    /* obtem a string do payload recebido */
    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload[i];
       msg += c;
    }

    topico = String(topic); // Converte em String a informação da variável ponteiro
 
    Serial.println("Chegou a seguinte string via MQTT: ");
    Serial.print("Tópico: ");
    Serial.println(topico);

    // Parâmetro Kp
    if(topico=="topico_ler_kp"){
      Serial.print("Kp trocado para: ");
      int i = 0;
      String valor;
      while(msg[i]!=NULL){
        char c = (char)msg[i];
        valor += c;
        i++;
      }
      kpr = valor.toDouble();
      Serial.println(kpr);
    }

    // Parâmetro Ki
    else if(topico=="topico_ler_ki"){
      Serial.print("Ki trocado para: ");
      int i = 0;
      String valor;
      while(msg[i]!=NULL){
        char c = (char)msg[i];
        valor += c;
        i++;
      }
      kir = valor.toDouble();
      Serial.println(kir);
    }

    // Parâmetro Kd
    else if(topico=="topico_ler_kd"){
      Serial.print("Kd trocado para: ");
      int i = 0;
      String valor;
      while(msg[i]!=NULL){
        char c = (char)msg[i];
        valor += c;
        i++;
      }
      kdr = valor.toDouble();
      Serial.println(kdr);
    }

    // Parâmetro Fator A
    else if(topico=="topico_ler_fa"){
      Serial.print("Fator A trocado para: ");
      int i = 0;
      String valor;
      while(msg[i]!=NULL){
        char c = (char)msg[i];
        valor += c;
        i++;
      }
      far = valor.toDouble();
      Serial.println(far);
    }

    // Parâmetro Fator B
    else if(topico=="topico_ler_fb"){
      Serial.print("Fator B trocado para: ");
      int i = 0;
      String valor;
      while(msg[i]!=NULL){
        char c = (char)msg[i];
        valor += c;
        i++;
      }
      fbr = valor.toDouble();
      Serial.println(fbr);
    }
    
    // Caso qualquer mensagem chegue que não for de nenhum dos tópicos acima
    else{
      Serial.println("Mensagem recebida mas não utilizada");
    }
    
}
 
// Reconecta ao Broker em caso de queda ou falha na conexão
void reconnectMQTT(void) 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE_KP); 
            MQTT.subscribe(TOPICO_SUBSCRIBE_KI); 
            MQTT.subscribe(TOPICO_SUBSCRIBE_KD);
            MQTT.subscribe(TOPICO_SUBSCRIBE_FA);
            MQTT.subscribe(TOPICO_SUBSCRIBE_FB);
            
        } 
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
        }
    }
}
 
// Verifica situação da conexão do Wifi e do Broker
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected()) 
        reconnectMQTT(); // Se não houver conexão com o Broker, a conexão é refeita
      
     reconnectWiFi(); // Se não houver conexão com o WiFI, a conexão é refeita
}
 
// Reconecta ao Wifi
void reconnectWiFi(void) 
{
    // Se o ESP32 já estiver conectado a rede WI-FI, nada é feito. 
    // Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
          
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
      
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}

// LED INTERNO
#define pinLED 2 // Definição do pino em que o LED interno está conectado
#define nivelLED LOW // Estado inicial do LED

// DECLARAÇÃO DOS PINOS
#define ardPin 35 // Sinal que chega do arduino(planta)
#define setPoint 34 // Sinal de referencia (potenciometro)
#define pidOut 19 // Sinal que sai do esp(pid) e entra no arduino(planta)

// CONFIGURAÇÃO DO pidOut (sinal de saida do pid)
#define pidOutFrequency 1000 // Frequencia do sinal pwm de saída
#define canal 0 // Canal que foi utilizado (0 - 15)
#define resolution 12 // Resolução do sinal (1 - 16bits)

#define botao 33
bool estado = HIGH;

// DECLARAÇÃO DE VARIAVEIS EXTRAS
double potValue = 0;
double dutyCycle = 0;
double ardValue = 0;

// VARIAVEIS PID
/* saida: saida do PID, err: erro, pValue: sinal proporcional,
 dValue: sinal derivativo, iValue: sinal integrador, errAnt: erro anterior
 outCorrigida: saída a ser comprimida, fatorA: fator de limitação da saída,
 fatorB: fator de limite da ação integral, limite: limite da parcela integral*/
double saida, err, pValue, dValue, iValue, errAnt, outCorrigida, fatorA, fatorB, limite;
double kp, ki, kd;
double T, kg;
float tempo = 0;



// Função que o temporizador irá chamar, para reiniciar o ESP32
void pid() {

  // Verificação de mudança dos parâmetros
  if(kp != kpr){
    kp = kpr;
  }
  if(kd != kdr){
    kd = kdr;
  }
  if(ki != kir){
    ki = kir;
  }
  if(fatorA != far){
    fatorA = far;
  }
  if(fatorB != fbr){
    fatorB = fbr;
  }

  // Início do controle
  
  potValue = analogRead(setPoint)*0.5;
  ardValue = analogRead(ardPin);
  potValue = potValue * 32;
  ardValue = (ardValue * 32)-65536; // Descompressão
  err = potValue - ardValue;
  pValue = err * kp;    // Parcela proporcional
  iValue = (iValue + T * (ki * err) * kg); // Parcela integral
  limite = pow(2,fatorB);
  if (iValue > limite){ // Limite da parcela integral
    iValue = limite;
  }
  else if (iValue < -limite){
    iValue = -limite; 
  }  
  dValue = kd * (err - errAnt) / T; // Parcela derivada
  errAnt = err; // Salva o erro atual
  saida = pValue + iValue + dValue; // Esforço de controle bruto
  outCorrigida = saida*fatorA + 65536;
  if (outCorrigida > 131072){
    outCorrigida = 131072;
  }
  if (outCorrigida < 0){
    outCorrigida = 0;
  }
  outCorrigida = outCorrigida / 32; // Compressão
  errAnt = err; // Atualiza o erro anterior
  ledcWrite(canal, outCorrigida); // Escrita do sinal PWM
}

// TIMER
hw_timer_t * timer = NULL; // Instância do timer para fazer o controle do temporizador (interrupção por tempo)

void startTimer(){ // Função para iniciar o timer
  /*
    hw_timer_t * timerBegin(num, divider, countUp)  
    num: é a ordem do temporizador. Podemos ter quatro temporizadores, então a ordem pode ser [0,1,2,3].
    divider: É um prescaler (reduz a frequencia por fator). Para fazer um agendador de um segundo,
    usaremos o divider como 80 (clock principal do ESP32 é 80MHz). Cada instante será T = 1/(80) = 1us
    countUp: True o contador será progressivo
  */
  
  timer = timerBegin(0, 80, true); // timerID 0, div 80
  // timer, callback, interrupção de borda
  timerAttachInterrupt(timer, &pid, true);
  // timer, tempo (us), repetição
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer); // Habilita a interrupção
  timerWrite(timer, 0);
}


void setup() {
  Serial.begin(9600);
  // Inicia conexão WiFi
  initWiFi();
 
  // Inicia conexão MQTT
  initMQTT();
  
  Serial.println("\nPara iniciar o programa, aperte o botão.");
  pinMode(botao, INPUT);
  
  while (estado != LOW){ // Botão para iniciar o programa
    Serial.println("Aperte o botao.");
    delay(500);
    estado = digitalRead(botao);
  }
  
  Serial.println("/--INICIANDO PROGRAMA--/");
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, !nivelLED);
  pinMode(pidOut, OUTPUT);

  // CONFIGURAÇÃO PWM
  ledcSetup(canal, pidOutFrequency, resolution);
  ledcAttachPin(pidOut, canal);

  startTimer(); // Inicialização do timer
  
  // CONFIGURACAO INICIAL DO PID
  kp = 1;
  ki = 0.3;
  kd = 0.1;
  T = 0.001;
  kg = 1;
  fatorA = 0.1;
  fatorB = 19;
  potValue = analogRead(setPoint);
  ardValue = analogRead(ardPin); 
  kpr = kp;
  kir = ki;
  kdr = kd;
  far = fatorA;
  fbr = fatorB;
    
  Serial.println("/--PROGRAMA INICIADO--/");
}

void loop(){
  /* Garante funcionamento das conexões WiFi e ao broker MQTT */
  VerificaConexoesWiFIEMQTT();

  // Escrita na porta Serial de alguns valores do controlador
  Serial.print(" Saida (V): ");
  Serial.println((outCorrigida)*3.3/4096);
  Serial.print(" PID (esforço): "); Serial.print(outCorrigida); 
  Serial.print(" Erro: "); Serial.print(err); 
  Serial.print(" Ação Proporcional: "); Serial.print(pValue);
  Serial.print(" Ação Integral: "); Serial.print(iValue); 
  Serial.print(" Ação Derivativa: "); Serial.print(dValue);
  Serial.println("");

  // Envio da resposta da planta para o Broker
  float resp2 = (ardValue/32)*3.3/4096;
  char resposta[10] = {0};
  sprintf(resposta,"%.2f",resp2);
  MQTT.publish(TOPICO_PUBLISH_RESP, resposta);
  /* keep-alive da comunicação com broker MQTT */
  MQTT.loop();
  delay(1000);
}
