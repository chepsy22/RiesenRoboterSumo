/*
 * Proyecto: Riesen Roboter - Sumo
 * Autor: Jose Carlos Flores Rosales (241216)
 * Fecha: Nov 2026
 * 
 * Notas: 
 * - Se usa Bluepad32 para control de Xbox.
 * - Puente H TB6612FNG. PWMA cambiado al GPIO23 por falla en el 35.
 * - Boton A arranca cuenta de 5s, RT = turbo (embestida), B = paro de emergencia.
 */

#include <Arduino.h>
#include <Bluepad32.h>

// --- Pines ---
// Los numeros son GPIOs reales, no el orden de la placa
const uint8_t PWMA = 23, AIN1 = 33, AIN2 = 32;
const uint8_t PWMB = 14, BIN1 = 26, BIN2 = 27, STBY = 25;

// --- Configuracion PWM y Motores ---
const int pwmNormal = 204; // Aprox 80%
const int pwmEmbestida = 255; // 100% full power
const uint32_t PWM_HZ = 20000;

// Flags de sentido (ajustados en las pruebas de pista)
const bool canalAIzquierdo = true;
const bool invertirA = true;
const bool invertirB = true;

// Tiempos y rampas
const uint32_t esperaInicio = 5000;
const uint32_t intervaloRampa = 10;
const uint32_t pausaReversa = 50;
const int pasoRampa = 4;

// --- Estados ---
enum Estado { DETENIDO, CUENTA_REGRESIVA, ACTIVO };
Estado estado = DETENIDO;

ControllerPtr mando = nullptr;

bool pwmOk = false;
bool falloPWM = false;
bool aAnterior = true;

uint32_t inicioCuenta = 0;
uint32_t ultimoPaso = 0;
uint32_t ultimoLog = 0;

int segundosMostrados = -1;
int pwmA = 0;
int pwmB = 0;
int sentidoA = 0;
int sentidoB = 0;
uint32_t ceroA = 0;
uint32_t ceroB = 0;

// Devuelve 1 o -1 dependiendo del signo
int signo(int valor) {
    return (valor > 0) - (valor < 0);
}

bool iniciarPwm() {
    // Usando la API de ESP32 v2
    // ledcSetup(canal, frecuencia, resolucion_bits)
    ledcSetup(0, PWM_HZ, 8); // Canal 0 para el motor A
    ledcSetup(1, PWM_HZ, 8); // Canal 1 para el motor B

    // Asignamos los pines a sus respectivos canales
    ledcAttachPin(PWMA, 0);
    ledcAttachPin(PWMB, 1);

    return true; 
}

void escribirPwm(int a, int b) {
    a = constrain(a, 0, pwmEmbestida);
    b = constrain(b, 0, pwmEmbestida);

    // En v2, ledcWrite usa el número de canal (0 y 1), no el pin
    ledcWrite(0, a);
    ledcWrite(1, b);
}

void pararMotores() {
    digitalWrite(STBY, LOW); // Corta ambos canales

    if (pwmOk) escribirPwm(0, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    if (pwmA != 0) ceroA = millis();
    if (pwmB != 0) ceroB = millis();

    pwmA = 0;
    pwmB = 0;
}

void deshabilitar() {
    if (estado != DETENIDO && mando && mando->isConnected()) {
        mando->playDualRumble(0, 0, 0, 0); // apaga vibracion
    }
    estado = DETENIDO;
    pararMotores();
}

// Callbacks de Bluepad32
void onConnectedController(ControllerPtr ctl) {
    Serial.printf("Mando conectado: %s\n", ctl->getModelName().c_str());
    if (mando != nullptr) return;

    mando = ctl;
    aAnterior = true;
    deshabilitar();
    Serial.println("Listo. Presiona A para los 5 segundos.");
}

void onDisconnectedController(ControllerPtr ctl) {
    if (mando != ctl) return;
    mando = nullptr;
    aAnterior = true;
    deshabilitar();
    Serial.println("Mando desconectado!");
}

// Logica de suavizado de motores (rampa)
int actualizarRampa(int objetivo, int actual, int &sentido, uint32_t &momentoCero, uint32_t ahora) {
    objetivo = constrain(objetivo, -pwmEmbestida, pwmEmbestida);

    // Bajar a cero antes de invertir giro
    if (actual != 0 && objetivo != 0 && signo(actual) != signo(objetivo)) {
        objetivo = 0;
    }

    // Delay de seguridad al cambiar de direccion
    if (actual == 0 && objetivo != 0 && sentido != 0 && signo(objetivo) != sentido && (ahora - momentoCero) < pausaReversa) {
        return 0;
    }

    int nuevo = actual + constrain(objetivo - actual, -pasoRampa, pasoRampa);

    if (nuevo != 0) {
        sentido = signo(nuevo);
    } else if (actual != 0) {
        momentoCero = ahora;
    }

    return nuevo;
}

void aplicarMotores(int a, int b) {
    if (a == 0 && b == 0) {
        pararMotores();
        return;
    }

    escribirPwm(0, 0);

    digitalWrite(AIN1, a > 0 ? HIGH : LOW);
    digitalWrite(AIN2, a < 0 ? HIGH : LOW);
    digitalWrite(BIN1, b > 0 ? HIGH : LOW);
    digitalWrite(BIN2, b < 0 ? HIGH : LOW);

    escribirPwm(abs(a), abs(b));

    if (falloPWM) {
        deshabilitar();
        return;
    }

    digitalWrite(STBY, HIGH);
}

void procesarMovimiento(uint8_t cruceta, int potencia, uint32_t ahora) {
    bool arriba = cruceta & 0x01;
    bool abajo = cruceta & 0x02;
    bool derecha = cruceta & 0x04;
    bool izquierda = cruceta & 0x08;

    // Frenado por direcciones opuestas o sin pulsar
    if (cruceta == 0 || (arriba && abajo) || (derecha && izquierda)) {
        pararMotores();
        return;
    }

    int avance = int(arriba) - int(abajo);
    int giro = int(derecha) - int(izquierda);

    int izq = avance + giro;
    int der = avance - giro;

    // Para que no sume doble velocidad en diagonales
    int divisor = (abs(izq) > 1 || abs(der) > 1) ? 2 : 1;

    izq = izq * potencia / divisor;
    der = der * potencia / divisor;

    int objetivoA = canalAIzquierdo ? izq : der;
    int objetivoB = canalAIzquierdo ? der : izq;

    if (invertirA) objetivoA = -objetivoA;
    if (invertirB) objetivoB = -objetivoB;

    if ((ahora - ultimoPaso) < intervaloRampa) return;
    ultimoPaso = ahora;

    pwmA = actualizarRampa(objetivoA, pwmA, sentidoA, ceroA, ahora);
    pwmB = actualizarRampa(objetivoB, pwmB, sentidoB, ceroB, ahora);

    aplicarMotores(pwmA, pwmB);
}

void procesarMando(uint32_t ahora) {
    uint8_t cruceta = mando->dpad() & 0x0f;
    int rt = mando->throttle();

    bool btnA = mando->a();
    bool pulsacionA = btnA && !aAnterior;
    aAnterior = btnA;

    // Freno de emergencia con B
    if (mando->b()) {
        if (estado != DETENIDO) Serial.println("Freno de emergencia (B)");
        deshabilitar();
        return;
    }

    if (estado == DETENIDO) {
        pararMotores();

        if (pulsacionA) {
            // Validar que no esten tocando nada mas al iniciar
            if (cruceta != 0 || rt > 100) {
                Serial.println("Error: Suelta los controles y vuelve a presionar A.");
                return;
            }

            estado = CUENTA_REGRESIVA;
            inicioCuenta = ahora;
            segundosMostrados = -1;
            
            // Vibracion de aviso
            mando->playDualRumble(esperaInicio, 1000, 0x80, 0x80);
        }
    }

    if (estado == CUENTA_REGRESIVA) {
        pararMotores();

        // Si se mueven antes de los 5s, se cancela
        if (cruceta != 0 || rt > 100) {
            deshabilitar();
            Serial.println("Falta! Inicio cancelado.");
            return;
        }

        uint32_t transcurrido = ahora - inicioCuenta;

        if (transcurrido >= esperaInicio) {
            Serial.println("GO!");
            estado = ACTIVO;
        } else {
            int segundos = (esperaInicio - transcurrido + 999) / 1000;
            if (segundos != segundosMostrados) {
                segundosMostrados = segundos;
                Serial.printf("Faltan %d...\n", segundos);
            }
        }
        return;
    }

    if (estado == ACTIVO) {
        int potencia = rt > 512 ? pwmEmbestida : pwmNormal;
        procesarMovimiento(cruceta, potencia, ahora);
    }
}

void setup() {
    digitalWrite(STBY, LOW);
    pinMode(STBY, OUTPUT);

    const uint8_t pines[] = { PWMA, AIN1, AIN2, PWMB, BIN1, BIN2 };
    for (uint8_t pin : pines) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }

    Serial.begin(115200);

    pwmOk = iniciarPwm();
    falloPWM = !pwmOk;
    deshabilitar();

    Serial.println("\n--- Riesen Roboter Iniciado ---");
    if (falloPWM) {
        Serial.println("ERROR PWM: Revisa los pines o la config de ledc");
    }

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
}

void loop() {
    BP32.update();
    uint32_t ahora = millis();
    bool conectado = mando && mando->isConnected();

    if (!conectado || !mando->isGamepad() || falloPWM) {
        deshabilitar();
        aAnterior = true;
    } else {
        procesarMando(ahora);
    }

    // Print de debug cada medio segundo (util para ver que lee el mando)
    if (ahora - ultimoLog >= 500) {
        ultimoLog = ahora;
        Serial.printf("status: con=%d st=%d pwmA=%d pwmB=%d err=%d\n", conectado, estado, pwmA, pwmB, falloPWM);
    }

    delay(1);
}