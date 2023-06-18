#include <stdint.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>

#include "ThingSpeak.h"

uint8_t temprature_sens_read();
uint64_t chipid;

// Netwrok credentials
char ssid[] = "Galaxy A52AM"; // your network SSID (name)
char pass[] = "12345678"; // your network password
int keyIndex = 0;
WiFiClient client;

// ThingSpeak channel details
unsigned long myChannelNumber = 2147857;
const char * myWriteAPIKey = "E8KPXS1QIVOS6FKM";

// Timer variables
unsigned long lastTime = 0;
unsigned long timeBeforeUpdate = 60000;  //1min 

String myStatus = "";

float voltageArray[100]; 
float currentArray[100];

float powerArray[100];


/*--------------------------*/
int f_p1_volt, f_p2_volt, f_p1_curr, f_p2_curr;
int half_p_volt, half_p_curr;
float period_volt, period_curr;
float freq_voltage, freq_current;
int period_discrepancy;
float anglePowerFactor;
float arrayPowerFactor[9];
unsigned long t_aux;
float f_volt[9];
float f_curr[9];
float voltRMSaux[9];
float currRMSaux[9];
float angle;
float voltageFreq;
float currentFreq;
float voltageEf;
float currentEf;
float pFactor;

float voltageRMS = 0;
float currentRMS = 0;
float truePower = 0;
float apparentPower = 0;
float powerFactor = 0;
/*--------------------------*/
float arrayPower[100];    //*********2 CASO***********//
float potenciaUmCiclo;    //*********2 CASO***********//
float vetorPotencia[9];   //*********2 CASO***********//
float potenciaATIVA;      //*********2 CASO***********//
float fatorPotencia;      //*********2 CASO***********//    



void setup() {
  // put your setup code here, to run once:
  pinMode(24, INPUT);
  pinMode(25, INPUT);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  ThingSpeak.begin(client); // Initialize ThingSpeak

  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

}
  

void loop() {
  // put your main code here, to run repeatedly:
  chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length:6bytes)
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  lastTime = millis();
  
  while((millis() - lastTime) < timeBeforeUpdate){

    
    int cont;
    
    float voltage;
    float current;


    for(cont=0; cont<9; cont++){     
      int i;
      int p_volt, p_curr;

      //100 samples --> sample rate = 5000Hz
      for(i=0; i<100; i++){
        t_aux = micros();

        // Get the new values reading
        voltage = 3.3 * analogRead(34)/4096;
        current = 3.3 * analogRead(35)/4096;

        voltageArray[i] = voltage;
        currentArray[i] = current;

        while(micros()-t_aux < 200){/* do nothing*/}
      }

      realVoltage(voltageArray);  //enviar vetor de 100 amostras de tensao para obtermos o valor real de cada amostra
      realCurrent(currentArray);  //enviar vetor de 100 amostras de corrente para obtermos o valor real de cada amostra  

      getPower(voltageArray, currentArray, arrayPower);  //*********2 CASO***********//
      potenciaUmCiclo = average(arrayPower, 100);        //*********2 CASO***********//
      vetorPotencia[cont] = potenciaUmCiclo;             //*********2 CASO***********//

      voltageRMS = trueRMSMeasure(voltageArray);  //valor eficaz correspondente a um ciclo de tensao
      currentRMS = trueRMSMeasure(currentArray);  //valor eficaz correspondente a um ciclo de corrente

      voltRMSaux[cont] = voltageRMS;
      currRMSaux[cont] = currentRMS;

      freq_voltage = getFreq(voltageArray, &p_volt);
      freq_current = getFreq(currentArray, &p_curr);

      f_volt[cont] = freq_voltage;
      f_curr[cont] = freq_current; 


      /* Power Factor */
      period_discrepancy =  abs(p_volt-p_curr);
      anglePowerFactor = period_discrepancy * 3.6;
      powerFactor = cos(anglePowerFactor);

      arrayPowerFactor[cont] = powerFactor;

    }

    pFactor = average(arrayPowerFactor, 9);

    voltageFreq = average(f_volt, 9);
    currentFreq = average(f_curr, 9);
    
    voltageEf = average(voltRMSaux, 9);
    currentEf = average(currRMSaux, 9);

    apparentPower = measurementApparentPower(voltageEf, currentEf); //voltageEf * currentEf;
    
    truePower = measurementTruePower(voltageEf, currentEf, pFactor); //apparentPower * angle;

    potenciaATIVA = average(vetorPotencia, 9);    //*********2 CASO***********//
    fatorPotencia = potenciaATIVA/apparentPower;  //*********2 CASO***********//
    
  }

  /*Send updated info to ThingSpeak*/
    ThingSpeak.setField(1, voltageEf);
    ThingSpeak.setField(2, voltageFreq);
    ThingSpeak.setField(3, currentEf);
    ThingSpeak.setField(4, currentFreq);
    ThingSpeak.setField(5, potenciaATIVA);   //truePower ou potenciaATIVA
    ThingSpeak.setField(6, apparentPower);
    ThingSpeak.setField(7, fatorPotencia);   //pFactor ou fatorPotencia


    myStatus = String("Update new info"); 
    ThingSpeak.setStatus(myStatus);


    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200){
      Serial.println("Channel update successful." );
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x) );
    }
  
}

float measurementTruePower(float vol, float curr, float pfactor){
  float truePower;
  
  truePower = vol * curr * pfactor;

  return truePower;
}

float measurementApparentPower(float vol, float curr){
  float apparentPower;
  
  apparentPower = vol * curr;

  return apparentPower;
}

float average(float arrayFloat[], int upperLimit){
  float avg = 0.0;
  int i;

  for(i=0; i<upperLimit; i++){ 
    avg += arrayFloat[i];
  }

  avg /= upperLimit;
  
  return avg;
}

void realVoltage(float arrayFloat[]){
  float volt = average(arrayFloat, 100);  //Equals to offset of signal

  for(int i=0; i<100; i++){
    arrayFloat[i] = (-1) * (arrayFloat[i] - volt) * 264; 
  }

}

void realCurrent(float arrayFloat[]){
  float curr = average(arrayFloat, 100);  //Equals to offset of signal

  for(int i=0; i<100; i++){
    arrayFloat[i] = (-1) * (arrayFloat[i] - curr) * 20;
  }

}

float trueRMSMeasure(float arrayFloat[]){
  float soma = 0.0;
  float aux, aux_vector;
  float soma_ret; 

  for(int i=0; i<100; i++){
    aux_vector = arrayFloat[i];
    aux = (float) pow(aux_vector,2);
    soma += aux;
  }

  soma /= 100;  //"periodo" -> num. amostras
  soma_ret = sqrt(soma);

  return soma_ret;
}

void getPower(float voltageArray[], float currentArray[], float power[]){
  int i;

  for(i=0; i<100; i++){
    power[i] = voltageArray[i] * currentArray[i];
  }

}

float getFreq(float arrayFloat[], int* point){
  int i, j;
  int p = -1, p1 = -1, p2 = -1;
  int h_p; 
  float period, freq;

  if(arrayFloat[j]==0 && j==0){
    for(i=1; i<100; i++){
      if((voltageArray[i-1]<0 && voltageArray[i]>0) || (voltageArray[i-1]>0 && voltageArray[i]<0)) p = i; 
    }
    period = (p*2.0*200)/1000000;
  }
  else if(arrayFloat[j]==0 && j==99){
    for(i=1; i<100; i++){
      if((voltageArray[i-1]<0 && voltageArray[i]>0) || (voltageArray[i-1]>0 && voltageArray[i]<0)) p = i;
    }
    h_p = 99 - p;
    period = (h_p*2.0*200)/1000000;
  }  
  else{
    for(i=1; i<100; i++){
      if(voltageArray[i] > 0 && voltageArray[i-1] < 0) p1 = i;
      else if(voltageArray[i] < 0 && voltageArray[i-1] > 0) p2 = i;  
    }
    h_p = abs(p1-p2);
    period = (h_p*2.0*200)/1000000;
  }

  freq = 1/period;

  if(p != -1) *point = p;
  else *point = p1;

  return freq;  
}

