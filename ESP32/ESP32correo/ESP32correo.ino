#include "ESP32_MailClient.h"
const char* ssid = "tu_red_wifi";
const char* password = "constrasena";
SMTPData datosSMTP;
int boton=0;
int cuenta=0;
void(* Resetea) (void) = 0;//Función de reseteo por software del esp32
void setup() {
  Serial.begin(115200);//Velocidad del puerto serial
   pinMode(2, OUTPUT);//Led onboard
   pinMode(23, OUTPUT);//Led en pin 23
   pinMode(12, INPUT);//Pulsador
  Serial.println();
  Serial.print("Conentando");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
//30 intentos antes de conexión antes de reiniciar el esp32
    cuenta=cuenta+1;
    Serial.print("Intento: ");
    Serial.println(cuenta);
    if(cuenta==30){
    Serial.println("Reinciando x software al esp32!!!");  
     Resetea();//Llama a la función de reinicio  
      }
    delay(200);
  }
  Serial.println();
  Serial.println("Red Wifi conectada!!!");
  Serial.println();
  digitalWrite(2, HIGH);//Enciende un led cuando el módulo esp32 se conecta a la red wifi
}
void loop() {
boton=digitalRead(12);
if(boton==0){
Serial.print("Iniciando correo!!!");
delay(200);
correo();
}
//Serial.print("Estado: ");
//Serial.println(boton);
//delay(200);
}
void correo(){
//digitalWrite(2, HIGH);
//Configuración del servidor de correo electrónico SMTP, host, puerto, cuenta y contraseña
datosSMTP.setLogin("smtp.gmail.com", 465, "diegolimon1234ar@gmail.com", "tu_clave_de_correo");
// Establecer el nombre del remitente y el correo electrónico
datosSMTP.setSender("ESP32S", "diegolimon1234ar@gmail.com");
// Establezca la prioridad o importancia del correo electrónico High, Normal, Low o 1 a 5 (1 es el más alto)
datosSMTP.setPriority("High");
// Establecer el asunto
datosSMTP.setSubject("Probando envio de correo con ESP32");
// Establece el mensaje de correo electrónico en formato de texto (sin formato)
datosSMTP.setMessage("Hola soy el esp32s! y me estoy comunicando contigo", false);
// Agregar destinatarios, se puede agregar más de un destinatario
datosSMTP.addRecipient("direccion_de_destino@correo_cualquiera.com");
 //Comience a enviar correo electrónico.
if (!MailClient.sendMail(datosSMTP))
Serial.println("Error enviando el correo, " + MailClient.smtpErrorReason());
//Borrar todos los datos del objeto datosSMTP para liberar memoria
datosSMTP.empty();
delay(10000);
//digitalWrite(2, LOW);
}