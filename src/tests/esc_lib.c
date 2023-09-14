//código de apresentação da primeira etapa:
#include <Servo.h>  // Inclui a biblioteca Servo.h

Servo Esc30A;                   // Cria um objeto Servo para controlar o motor
const int pino_pot = 0;         // Define o pino analógico utilizado para o potenciômetro
const int pino_motor = 9;       // Define o pino PWM utilizado para o motor
const int pino_pulse = 2;       // Define o pino digital ligado em uma das fases do motor
int Speed;                      // Variável para armazenar a velocidade do motor
int pulseCount = 0;             // Variável para armazenar o número de pulsos contados
float RPM;                      // Variável para armazenar a velocidade em rotações por minuto
unsigned long lastTime = 0;     // Variável para armazenar a última vez que um pulso foi detectado
float pulses_per_rotation = 7;  // Número de pulsos por rotação do motor
int PWM = 0;
int aux = 0;
int i;
int PWM1 = 1180;
int pwm1 = 1180;
double vel_medida;
float time1, pulse;
double Ts = 22;  //tempo de amostragem em ms
double tempo;
int contloop =0;

float SetPoint = 5000;
float erro, int_erro = 0;
float Kp = 0.08567;
float Ki = 0.012656;

// Função para contar os pulsos
void countPulses() {
  pulseCount++;  // Incrementa o contador de pulsos
}

void rpm() {
  detachInterrupt(0);
  if (i == 0) {
    time1 = micros();
    i++;
  } else {
    i++;
    if (i == 9) {
      pulse = micros() - time1;
      i = 0;
      vel_medida = 60000000 / pulse;
    }
  }

  attachInterrupt(0, rpm, RISING);
}

void setup() {
  Serial.begin(2000000);                  // Inicializa a comunicação serial a uma taxa de 9600 bps
  Serial.setTimeout(5);                   //limita o tempo de espera de caracteres de entrada da serial
  Esc30A.attach(pino_motor, 1000, 2000);  // Inicializa o objeto Servo com o pino do motor
  //Serial.println("Iniciando..."); // Imprime "Iniciando..." na porta serial
  delay(3000);  // Aguarda 3 segundos
  //attachInterrupt(digitalPinToInterrupt(pino_pulse), rpm, RISING); // Configura a detecção de pulso de subida no pino digital
  attachInterrupt(0, rpm, RISING);
  Esc30A.writeMicroseconds(1050); //initialize the signal to 1000
  delay(7000);
  Esc30A.writeMicroseconds(1109);
  delay(7000);
  Serial.println("Saiu do Setup");
  delay(500);

}

int retornaPWM() {
  PWM = Serial.parseInt();           //lê caracteres ASCII da serial e converte em um número inteiro
  PWM = constrain(PWM, 1000, 2000);  //limita os valores da variável PWM entre 0 a 180
  //PWM = 10;
  // Serial.print("VALOR DE PWM DENTRO DO IF ");
  // Serial.println(PWM);
  //analogWrite(pino_motor, PWM);  //atribui o valor do PWM à porta de saída 3
  //Esc30A.write(PWM);


  return PWM;
}

void loop() {
  tempo = millis();  //armazena o valor do tempo na variável "tempo" (em ms)

  //delay(5000);
 
    //Speed = analogRead(pino_pot); // Lê o valor do potenciômetro
  //Speed = map(Speed, 0, 1023, 0, 179); // Mapeia o valor do potenciômetro para um valor entre 0 e 179
  //Esc30A.write(Speed); // Define a velocidade do motor com o valor do potenciômetro
  //Serial.println(Speed);
  /*

    unsigned long currentTime = millis(); // Armazena o tempo atual em milissegundos
    if (currentTime - lastTime >= 10  00) { // Verifica se passou 1000ms desde a última detecção de pulso
      float timeElapsed = (float)(currentTime - lastTime) / 1000.0; // Calcula o tempo decorrido em segundos
      if(pulseCount < 1){
        RPM = (60.0 / timeElapsed) * (float)pulseCount / (pulses_per_rotation+1);
        //Serial.println("Primeiro pulso");
      }else{
        RPM = (60.0 / timeElapsed) * (float)pulseCount / pulses_per_rotation; // Calcula a velocidade do motor em rotações por minuto
      }
      
      //Serial.print("RPM: "); // Imprime "RPM: " na porta serial
      Serial.print(PWM1);
      Serial.print(" ");
      Serial.print(RPM); // Imprime a velocidade do motor em rotações por minuto na porta serial
      Serial.println(" ");
      //Serial.print("Contagem de pulsos: "); // Imprime "Contagem de pulsos: " na porta serial
      //Serial.println(pulseCount); // Imprime o número de pulsos contados na porta serial
      pulseCount = 0; // Reseta o contador de pulsos
      lastTime = currentTime; // Armazena o tempo da última detecção de pulso 

      //tirar textos nao necessários do serial monitor
    }
*/
 // Serial.print(vel_medida);
 // Serial.print(" ");
 // Serial.println(pwm1);


  if (Serial.available() > 0) {  //verifica se há dados na serial
    //PWM = Serial.parseInt(); //lê caracteres ASCII da serial e converte em um número inteiro
    // PWM = constrain(PWM, 0, 255);  //limita os valores da variável PWM entre 0 a 255
    //Serial.print("VALOR DE PWM DENTRO DO IF ");
    //Serial.println(PWM);
    //analogWrite(pino_motor, PWM);  //atribui o valor do PWM à porta de saída 3
    //Esc30A.write(PWM);

    //PWM = Serial.parseInt(); //lê caracteres ASCII da serial e converte em um número inteiro
    //PWM = constrain(PWM, 0, 180);  //limita os valores da variável PWM entre 0 a 180
    //PWM1= PWM;
    //pwm1 = retornaPWM();
    SetPoint = Serial.parseInt(); //lê caracteres ASCII da serial e converte em um número inteiro
    SetPoint = constrain(SetPoint, 3000, 7000);  //limita os valores da variável PWM entre 0 a 255
  }

  //Serial.print("VALOR DE PWM FORA DO IF ");
  //Serial.println(PWM1);

  //analogWrite(pino_motor, PWM1);  //atribui o valor do PWM à porta de saída 3
  //Esc30A.writeMicroseconds(PWM1);
  //Esc30A.writeMicroseconds(PWM1); //using val as the signal to esc

  erro = SetPoint - vel_medida;  //calcula o erro entre SP e PV

  if (erro > (SetPoint * 0.1) || erro < (-SetPoint * 0.1))  //reset do integrador
  {
    int_erro = 0;  //zera o erro integral se o erro for maior que 10% de SP (em modulo). Isto reduz o overshoot
  } else {
    int_erro += erro;  //habilita o erro integral se o erro for menor que 10% de SP (em modulo)
  }


  pwm1 = int(erro * Kp + int_erro * Ki) + 1000;  //calculo do controlador PI, atribuida à variável PWM

  pwm1 = constrain(pwm1, 1000, 2000);  //limita os valores da variável PWM entre 0 a 255

  //analogWrite(PINO_VELOCIDADE_MOTOR, pwm);  //escreve o valor do PWM no pino 3
  Esc30A.writeMicroseconds(pwm1);

  Serial.print(SetPoint, 1);
  Serial.print(" ");  //envia o valor do SetPoint (valor desejado)
  
  //Serial.print(pwm1);
  //Serial.print(" ");  //envia o valor do PWM

  //Serial.print(erro);
  //Serial.print(" ");

  Serial.print(vel_medida, 2);  //envia o valor da Temperatura com 1 casa decimal
  Serial.println(" ");

  //temp de amostragem de 5ms -> 200hz
  //delay(100); // Aguarda 100ms antes de executar novamente o loop
  //reduzir delay
 
  while ((tempo + Ts) > millis()) {}  //espera o tempo necessário para completar o tempo de amostragem Ts

  }