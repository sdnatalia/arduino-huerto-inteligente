//Natalia DHT11 y ventilador
//Santiago Servo y Ultrasonico
//Israel Led y fotoresistencia
//Carlos Sensor de llamas / buzzer
//Cris Mlx sensor tempe/ lcd

#include "DHT.h" //Libreria DHT11
#include <Servo.h>//Libreria Servomotor
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#define DHTPIN 8     // Pin donde está conectado el sensor DHT
#define DHTTYPE DHT11   //Se usa el DHT 11

//Variable Natalia
    int cooler = A0; //Ventilador
    DHT dht(DHTPIN, DHTTYPE);
    
//Variable Santiago
    Servo ioe;
    int trig = 2;
    int echo = 3;
    int tiempo;
    int distancia;
    
//Variable Isra
    int led = 13;
    int lecturasensor;
    
//Variable Carlos
    int buzzer = 11;      // selecciona el pin para el zumbador
    int valorSensor = 0;  // variable para almacenar el valor proveniente del sensor
    
//Variable Chris
    Adafruit_MLX90614 termometroIR = Adafruit_MLX90614(); //Instanciar objeto

//Setup
void setup() {
  
  Serial.begin(9600);
  Serial.println("Iniciando..."); //Mensaje de inicio
  
  dht.begin();
  //Santiago
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    ioe.attach(4);
  //Isra
    pinMode(led, OUTPUT);
  //Carlos
    pinMode(buzzer, OUTPUT);
  //Chris
    termometroIR.begin();  // Iniciar termómetro infrarrojo con Arduino
}

void loop() {
//Metodo Natalia
  float h = dht.readHumidity(); //Leemos la Humedad
  float t = dht.readTemperature(); //Leemos la temperatura en grados Celsius

  //Lecturas de DHT11 y ventilador
  if (t >= 20)
  {
    if (h >= 70)
    {
      Serial.print("*Temperatura Excedida*");
      Serial.print(t);
      Serial.println(" °C");
      Serial.print("Humedad Excedida= ");
      Serial.print(h);
      Serial.println(" %");
      analogWrite(cooler, 500);
    }
    else {
      Serial.print("*Temperatura Excesiva*");
      Serial.print(t);
      Serial.println(" °C");
      Serial.print("Humedad Normal= ");
      Serial.print(h);
      Serial.println(" %");
      analogWrite(cooler, 250);
    }
  }
  else
  {
    Serial.print("*Temperatura Normal*");
    Serial.print(t);
    Serial.println(" °C");
    Serial.print("Humedad Normal= ");
    Serial.print(h);
    Serial.println(" %");
    analogWrite(cooler, 0);
  }
  //Fin Lecturas de DHT11 y ventilador
  delay(1000);

//Metodo Santiago
    digitalWrite(trig, HIGH);
    delay(1);
    digitalWrite(trig, LOW);
    tiempo=pulseIn(echo,HIGH);
    distancia=tiempo/58.2;
    
    Serial.print("Distancia (cm) = ");
    Serial.println(distancia);
    
      if(distancia <=20){
        ioe.write(180);
      }
      if(distancia > 20){
        ioe.write(0);
      }

 //Metodo Isra
      
      lecturasensor = analogRead(A1);
      Serial.print("Lectura de fotoresistencia =  ");
      Serial.println(lecturasensor);
      if (lecturasensor > 700) {
        digitalWrite(led, HIGH);
      } else {
        digitalWrite(led, LOW);
      }

  //Metodo Carlos
    // leer el valor del sensor:
      valorSensor = analogRead(A2);
       Serial.println("--------------------------------------------------------------------");
       Serial.print("Valor Flama= ");
       Serial.println(valorSensor);
     Serial.println("--------------------------------------------------------------------");
  // activa el buzzer
  if (valorSensor < 20){ //Cambie a 500
   digitalWrite(buzzer, HIGH);
   delay(100);
  // Desactiva el buzzer
  digitalWrite(buzzer, LOW);
  delay(50);  
   }
  //Método Chris
    // Obtener temperaturas grados Celsius
  float temperaturaAmbiente = termometroIR.readAmbientTempC();
  float temperaturaObjeto = termometroIR.readObjectTempC();
 
  // Mostrar información
  Serial.print("Temp. ambiente => ");
  Serial.print(temperaturaAmbiente);
  Serial.println("ºC");
 
  Serial.print("Temp. objeto => ");
  Serial.print(temperaturaObjeto);
  Serial.println("ºC");
  
  //delay(2000);  
}
