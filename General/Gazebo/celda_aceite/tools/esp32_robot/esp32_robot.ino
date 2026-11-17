// ESP32 de un robot de la celda. Mensajes con el PC (UDP 5005):
// R2 HOLA / R2 BIENVENIDO, R2 HAZ ir 17 -> R2 OK ir 17 -> R2 LISTO ir 17, R3 POS ..., R3 MUEVE ... 18
#include <WiFi.h>
#include <WiFiUdp.h>

const int ROBOT = 2;
const char *WIFI_RED = "MI_RED";
const char *WIFI_CLAVE = "MI_CLAVE";
IPAddress IP_PC(192, 168, 1, 100);
const uint16_t PUERTO_PC = 5005;
const uint16_t PUERTO_LOCAL = 6000 + ROBOT;

WiFiUDP udp;
bool conectado = false;
unsigned long ultimoHola = 0, ultimoPos = 0;
String ultimaOrden = "";

void enviar(const String &txt) {
  udp.beginPacket(IP_PC, PUERTO_PC);
  udp.print(txt);
  udp.endPacket();
}

// tramos: ir, llevar, volver (R1: salir, cargar, verter, volver). Volver cuando el robot termino.
void mover_tramo(const String &tramo) {
  if (tramo == "ir") {
  } else if (tramo == "llevar") {
  } else if (tramo == "volver") {
  } else if (tramo == "salir" || tramo == "cargar" || tramo == "verter") {
  }
  delay(3000);  // borrar
}

int leer_angulos(float *v) {
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
  v[3] = 0.0;
  return 4;
}

void mover_articulaciones(float *v, int n) {
  delay(2000);  // borrar
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_RED, WIFI_CLAVE);
  while (WiFi.status() != WL_CONNECTED) delay(300);
  Serial.println(WiFi.localIP());
  udp.begin(PUERTO_LOCAL);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    conectado = false;
    return;
  }
  if (!conectado && millis() - ultimoHola > 1000) {
    enviar("R" + String(ROBOT) + " HOLA");
    ultimoHola = millis();
  }
  if (conectado && millis() - ultimoPos > 100) {
    float v[8];
    int k = leer_angulos(v);
    String txt = "R" + String(ROBOT) + " POS";
    for (int i = 0; i < k; i++) txt += " " + String(v[i], 4);
    enviar(txt);
    ultimoPos = millis();
  }

  if (udp.parsePacket() <= 0) return;
  char buf[128];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = 0;
  String msg = String(buf);
  msg.trim();
  String yo = "R" + String(ROBOT) + " ";
  if (!msg.startsWith(yo)) return;
  String resto = msg.substring(yo.length());

  if (resto == "BIENVENIDO") {
    conectado = true;
  } else if (resto.startsWith("MUEVE ")) {
    String orden = resto.substring(6);
    int fin = orden.lastIndexOf(' ');
    String num = orden.substring(fin + 1);
    enviar(yo + "OK MUEVE " + num);
    if (("MUEVE " + num) == ultimaOrden) return;
    ultimaOrden = "MUEVE " + num;
    float v[8];
    int k = 0, desde = 0;
    while (k < 8 && desde < fin) {
      int hasta = orden.indexOf(' ', desde);
      v[k++] = orden.substring(desde, hasta).toFloat();
      desde = hasta + 1;
    }
    mover_articulaciones(v, k);
    enviar(yo + "LISTO MUEVE " + num);
  } else if (resto.startsWith("HAZ ")) {
    String orden = resto.substring(4);
    enviar(yo + "OK " + orden);
    if (orden == ultimaOrden) return;
    ultimaOrden = orden;
    mover_tramo(orden.substring(0, orden.indexOf(' ')));
    enviar(yo + "LISTO " + orden);
  }
}
