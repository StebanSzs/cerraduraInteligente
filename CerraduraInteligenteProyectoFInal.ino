/**
 * @file    CerraduraInteligenteProyectoFInal.ino
 * @brief   Cerradura Inteligente Empresarial con RFID y Monitoreo Ambiental.
 *
 * @details
 *   Sketch para Arduino Mega 2560 que controla una cerradura con autenticacion
 *   RFID, claves por usuario, perfiles, horarios de acceso y monitoreo
 *   ambiental. La logica principal esta organizada como una maquina de estados
 *   no bloqueante apoyada por `AsyncTask`.
 *
 *   Maquina de estados segun diagrama del proyecto:
 *
 * @verbatim
 * INICIO --(#)--> CONFIG --(boton)--> INICIO
 * INICIO --(A + clave o RFID dentro del horario)--> SERVO_ABIERTO
 * SERVO_ABIERTO --(5 s)--> MONITOR_AMBIENTAL
 * MONITOR_AMBIENTAL --(5 s)--> MONITOR_PUERTAS
 * MONITOR_PUERTAS --(2 s)--> MONITOR_AMBIENTAL
 * MONITOR_AMBIENTAL --(temp>umbral y luz>umbral)--> ALARMA
 * MONITOR_PUERTAS --(Hall>umbral Y sonido>umbral)--> ALARMA
 * ALARMA --(3 alarmas consecutivas)--> GESTION
 * GESTION --(*)--> INICIO
 * BLOQUEO --(7 s)--> INICIO
 * @endverbatim
 *
 * @par Plataforma
 *   Arduino Mega 2560.
 *
 * @author   Esteban, Jhonatam, Cristian
 * @version  1.0
 * @date     2026
 *
 * @par Librerias requeridas
 *   * `AsyncTaskLib`
 *   * `StateMachineLib`
 *   * `MFRC522`
 *   * `EEPROM`
 *   * `LiquidCrystal`
 *   * `Keypad`
 *   * `Servo`
 *
 * @warning
 *   El reloj interno se basa en `millis()`. Sin un modulo RTC externo, la hora
 *   debe configurarse despues de cada reinicio para habilitar accesos por
 *   horario.
 *
 * @defgroup cerradura_inteligente Cerradura inteligente
 * @brief Componentes de autenticacion, control de acceso y monitoreo.
 */

/**
 * @mainpage Cerradura Inteligente Empresarial
 *
 * @section overview Descripcion general
 *
 * Este firmware implementa una cerradura inteligente para Arduino Mega 2560.
 * El sistema permite autenticacion por tarjeta RFID o clave numerica por
 * usuario, aplica horarios por perfil y supervisa sensores ambientales y de
 * seguridad.
 *
 * @section credentials Credenciales y perfiles
 *
 * Cada credencial registrada queda asociada a un perfil de acceso. Las claves
 * numericas y tarjetas RFID usan el mismo flujo de autorizacion:
 * primero se valida la credencial, despues se consulta el horario activo del
 * perfil y finalmente se abre la cerradura o se bloquea el acceso.
 * Las claves numericas se vencen despues de `USOS_CLAVE_MAX` accesos exitosos
 * y deben cambiarse desde `CONFIG > C`.
 *
 * @section failed_attempts Intentos fallidos
 *
 * El sistema conserva el contador de intentos de clave incorrecta entre
 * bloqueos temporales. Al alcanzar `MAX_INTENTOS`, se activa `ST_ALARMA`,
 * suena el buzzer y luego se retorna a `ST_INICIO`.
 *
 * @section monitoring Monitoreo no bloqueante
 *
 * Las tareas periodicas se ejecutan con `AsyncTask` y la maquina de estados
 * recibe eventos diferidos. El sketch evita esperas bloqueantes para mantener
 * activos teclado, RFID, LCD, sensores, buzzer y servo.
 *
 * @section sensors Sensores
 *
 * - `KY-013` en `A0`: temperatura en grados Celsius.
 * - `KY-035` en `A1`: sensor Hall, normalizado para que un iman cercano baje
 *   la lectura.
 * - `KY-037` en `A2`: sonido, normalizado para que ruido fuerte suba la
 *   lectura.
 * - `KY-018` en `A3`: luz, normalizada como `0` oscuro y `1023` claro.
 */

//Librerias


#include "AsyncTaskLib.h"
#include "StateMachineLib.h"

#include <MFRC522.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <Servo.h>
#include <SPI.h>
#include <stdint.h>
#include <avr/pgmspace.h>

//Pines 


/** @name RFID RC522 */
/**@{*/
#define PIN_RFID_SS   53
#define PIN_RFID_RST  49
/**@}*/

/** @name LCD 16x2 */
/**@{*/
#define PIN_LCD_RS  23
#define PIN_LCD_EN  25
#define PIN_LCD_D4  27
#define PIN_LCD_D5  29
#define PIN_LCD_D6  31
#define PIN_LCD_D7  33
/**@}*/

/** @name Actuadores y avisos */
/**@{*/
#define PIN_SERVO       9
#define PIN_LED_ROJO   35
#define PIN_LED_VERDE  37
#define PIN_LED_AZUL   39
#define PIN_BUZZER     10
#define PIN_BOTON      41    /**< Boton fisico: salir de CONFIG.        */
/**@}*/

/** @name Sensores ambientales */
/**@{*/
#define PIN_KY013   A0       /**< Temperatura.                          */
#define PIN_KY035   A1       /**< Hall (puerta).                        */
#define PIN_KY037   A2       /**< Sonido.                               */
#define PIN_KY018   A3       /**< Luz.                                  */
/**@}*/

// teclado

#define ROWS 4
#define COLS 4
#define LCD_COLUMNAS 16
#define LCD_BUFFER_TAM (LCD_COLUMNAS + 1)
#define HORA_DIGITOS 4
#define HORA_BUFFER_TAM (HORA_DIGITOS + 2)
#define TEMP_BUFFER_TAM 8
#define BUFFER_UMBRAL_TAM 9
#define RFID_UID_BUFFER_TAM 10

/**
 * @brief Mapa del teclado.
 *
 * @details
 *   Teclas especiales usadas por la interfaz:
 *   * `#`: entrar a `CONFIG` desde `INICIO` o confirmar capturas.
 *   * `*`: borrar capturas o salir de menus secundarios.
 *   * `A`: ingresar o confirmar clave desde `INICIO`.
 */
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {22, 24, 26, 28};
byte colPins[COLS] = {30, 32, 34, 36};

Keypad teclado = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//4. Constantes
/** @name General */
/**@{*/
#define LONGITUD_PASSWORD  6  
#define MAX_INTENTOS       3  
#define MAX_USUARIOS      15  
#define TAM_USUARIO       16   
#define USUARIO_OFF_ESTADO    0   
#define USUARIO_OFF_PERFIL    1   
#define USUARIO_OFF_UID       2   
#define USUARIO_UID_TAM       4   
#define USUARIO_OFF_CLAVE     6  
#define USUARIO_OFF_FLAGS    12   
#define USUARIO_OFF_USOS_CLAVE 13 
#define USUARIO_ACTIVO        1  
#define USUARIO_FLAG_RFID  0x01   
#define USUARIO_FLAG_CLAVE 0x02   
#define USUARIO_FLAGS_VALIDOS (USUARIO_FLAG_RFID | USUARIO_FLAG_CLAVE)
#define USOS_CLAVE_MAX        4   
/**@}*/

/** @name Direcciones EEPROM */
/**@{*/
#define DIR_FLAG_INIT         0   
#define DIR_USUARIOS         16   
#define DIR_LOG_INDEX       512   
#define DIR_LOG_BASE        513   
#define LOG_TAMANO           16  
#define DIR_FLAG_UMBRALES   529   
#define DIR_UMBRAL_TEMP     530  
#define DIR_UMBRAL_LUZ      534   
#define DIR_UMBRAL_HALL     536   
#define DIR_UMBRAL_RUIDO    538   
#define DIR_FLAG_RELOJ      540   
#define DIR_RELOJ_MINUTOS   541   
#define DIR_FLAG_HORARIOS   543   
#define DIR_HORARIOS_BASE   544   
#define TAM_HORARIO_ROL       4   
#define VALOR_FLAG_OK      0xA5   
#define VALOR_UMBRALES_OK  0x5B  
#define VALOR_RELOJ_OK     0xC7  
#define VALOR_HORARIOS_OK  0x3C  
/**@}*/

/**
 * @brief Identifica los perfiles de usuario autorizables.
 *
 * @details
 *   Los valores se guardan en EEPROM junto al UID de cada tarjeta y se usan
 *   como indice para la tabla de horarios de acceso.
 */
enum Perfil {
  PERFIL_NINGUNO     = 0,  
  PERFIL_SEGURIDAD   = 1,  
  PERFIL_OPERARIO    = 2,  
  PERFIL_COORDINADOR = 3,  
  PERFIL_GERENTE     = 4   
};

#define PERFILES_CONFIGURABLES (PERFIL_GERENTE - PERFIL_SEGURIDAD + 1)
#define PERFIL_ARRAY_TAM (PERFIL_GERENTE + 1)

/** @name Umbrales ambientales (del diagrama) */
/**@{*/
#define UMBRAL_TEMP_DEFAULT          24   
#define UMBRAL_LUZ_DEFAULT          400   
#define UMBRAL_HALL_DEFAULT         600   
#define UMBRAL_RUIDO_DEFAULT        700   
#define UMBRAL_ANALOGICO_MAX       1023   
#define MUESTRAS_ANALOGICAS          8    
#define KY013_SERIE_ESTANDAR   10000.0   
#define KY013_SERIE_MONTAJE     1700.0   
#define KY013_NTC_NOMINAL      10000.0   
#define KY013_NTC_BETA          3950.0   
#define KY013_TEMP_NOMINAL_K     298.15 
#define KY013_CANDIDATOS           4     
#define TEMP_SENSOR_MIN_C        -40.0  
#define TEMP_SENSOR_MAX_C        125.0   
/**@}*/

/** @name Tiempos (ms, del diagrama) */
/**@{*/
#define TIEMPO_SERVO_ABIERTO     5000UL 
#define TIEMPO_AMBIENTAL_PUERTAS 5000UL  
#define TIEMPO_PUERTAS_AMBIENTAL 5000UL  
#define TIEMPO_ALARMA_AMBIENTAL  3000UL  
#define TIEMPO_ALARMA_PUERTAS    4000UL 
#define TIEMPO_BLOQUEO           7000UL  
/**@}*/

/** @name Parpadeos de LED (del diagrama) */
/**@{*/
#define BLOQUEO_LED_ON   300  
#define BLOQUEO_LED_OFF  700  
#define ALARMA_LED_ON    100  
#define ALARMA_LED_OFF   200 
#define LED_TICK_MS      100  
#define ALARMA_TICKS_ON  (ALARMA_LED_ON / LED_TICK_MS)
#define ALARMA_TICKS_OFF (ALARMA_LED_OFF / LED_TICK_MS)
#define BLOQUEO_TICKS_ON (BLOQUEO_LED_ON / LED_TICK_MS)
#define BLOQUEO_TICKS_OFF (BLOQUEO_LED_OFF / LED_TICK_MS)
/**@}*/

#define MAX_ALARMAS_CONSECUTIVAS 3   
#define MINUTOS_DIA           1440   

/** @name Melodia de alarma: At Doom's Gate */
/**@{*/
#define NOTE_E2    82
#define NOTE_AS2  117
#define NOTE_B2   123
#define NOTE_C3   131
#define NOTE_D3   147
#define NOTE_E3   165
#define REST        0
#define TEMPO_DOOM_ALARMA 225U
#define DURACION_REDONDA_DOOM_MS ((60000UL * 4UL) / TEMPO_DOOM_ALARMA)
#define TICK_MELODIA_MS 25U

const byte notasDoomAlarma[] PROGMEM = {
  NOTE_E2, NOTE_E2, NOTE_E3, NOTE_E2, NOTE_E2, NOTE_D3, NOTE_E2, NOTE_E2,
  NOTE_C3, NOTE_E2, NOTE_E2, NOTE_AS2, NOTE_E2, NOTE_E2, NOTE_B2, NOTE_C3,
  NOTE_E2, NOTE_E2, NOTE_E3, NOTE_E2, NOTE_E2, NOTE_D3, NOTE_E2, NOTE_E2,
  NOTE_C3, NOTE_E2, NOTE_E2, NOTE_AS2
};

const int8_t duracionesDoomAlarma[] PROGMEM = {
  8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, -2
};

#define ELEMENTOS_MELODIA_ALARMA (sizeof(notasDoomAlarma) / sizeof(notasDoomAlarma[0]))
/**@}*/

//5. Objetos globales

/** @brief Pantalla LCD 16x2 usada como interfaz de usuario. */
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4,
                  PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

/** @brief Lector RFID RC522 conectado por SPI. */
MFRC522 lectorRFID(PIN_RFID_SS, PIN_RFID_RST);

/** @brief Servo que acciona fisicamente la cerradura. */
Servo lockServo;

//6. Variables globales

char    passwordIngresada[LONGITUD_PASSWORD + 1]; 
byte    posPassword       = 0;                    
bool    modoIngresoClave  = false;                
byte    intentosFallidos  = 0;                   
byte    perfilActual      = PERFIL_NINGUNO;      
bool    accesoFueConClave = false;                
byte    usosClaveActual   = 0;                    

float   temperaturaActual = 22.0; 
unsigned int lecturaTempRaw = 0;    
unsigned int lecturaHall    = 1023; 
unsigned int lecturaSonido  = 0;    
unsigned int lecturaLuz     = 0;    

byte    umbralTemp        = UMBRAL_TEMP_DEFAULT;  
unsigned int umbralLuz         = UMBRAL_LUZ_DEFAULT;   
unsigned int umbralHallAbierto = UMBRAL_HALL_DEFAULT;  
unsigned int umbralRuido       = UMBRAL_RUIDO_DEFAULT; 

byte    alarmasConsecutivas = 0;    
bool    estadoLedRojo       = false; 
bool    alarmaVieneDePuertas = false; 
bool    alarmaVieneDeAcceso = false;   
bool    melodiaAlarmaActiva  = false; 
byte    indiceMelodiaAlarma  = 0;     
byte    ticksParpadeoRojo    = 0;      
byte    ticksNotaAlarma      = 0;  

volatile bool eventoAmbientalAlarma = false; 
volatile bool eventoPuertaAlarma    = false; 

char  subOpcionConfig    = 0;              
byte  perfilParaRegistro = PERFIL_OPERARIO;
byte  perfilHorarioConfig = PERFIL_NINGUNO;
char  opcionGestionUmbral = 0;             
char  bufferUmbral[BUFFER_UMBRAL_TAM];    
byte  posBufferUmbral = 0;                 
bool  botonDebouncePendiente = false;      
bool  ultimoEstadoBoton = HIGH;           

bool relojConfigurado = false;             
unsigned int minutoBaseReloj = 0;         
unsigned long millisBaseReloj = 0;         
unsigned int horarioInicioPorPerfil[PERFIL_ARRAY_TAM] = {0, 0, 480, 420, 0}; 
unsigned int horarioFinPorPerfil[PERFIL_ARRAY_TAM]    = {0, 1439, 1020, 1140, 1439}; 
char lineaBloqueo1[LCD_BUFFER_TAM] = "BLOQUEADO";      
char lineaBloqueo2[LCD_BUFFER_TAM] = "Espere 7 seg"; 

//maquina de estados


/**
 * @brief Enumera los estados principales de la maquina de control.
 *
 * @details
 *   Cada valor representa una pantalla/actividad exclusiva del sistema. Las
 *   transiciones se configuran en configurarEstados().
 */
enum Estado {
  ST_INICIO = 0,          
  ST_CONFIG,               
  ST_SERVO_ABIERTO,      
  ST_MONITOR_AMBIENTAL,   
  ST_MONITOR_PUERTAS,      
  ST_ALARMA,              
  ST_GESTION,            
  ST_BLOQUEO,             
  ST_NUM_ESTADOS           
};

/**
 * @brief Enumera los eventos que disparan transiciones de estado.
 *
 * @details
 *   Los eventos se escriben en eventoActual o eventoProgramado para desacoplar
 *   callbacks de temporizadores y evitar reentradas en la maquina de estados.
 */
enum Entrada {
  EV_NINGUNO = 0,        
  EV_TECLA_CONFIG,       
  EV_BOTON,              
  EV_CLAVE_OK,           
  EV_SERVO_TIMEOUT,      
  EV_A_PUERTAS,          
  EV_A_AMBIENTAL,        
  EV_AMBIENTAL_ALARMA,   
  EV_PUERTA_ALARMA,      
  EV_INTENTOS_ALARMA,    
  EV_A_BLOQUEO,         
  EV_A_INICIO,           
  EV_A_GESTION,          
  EV_ESTRELLA,           
  EV_BLOQUEO_TIMEOUT     
};

byte eventoActual     = EV_NINGUNO;
byte eventoProgramado = EV_NINGUNO;


StateMachine miMaquina(ST_NUM_ESTADOS, 30);

//Declaracion adelantada

void configurarEstados();
void onEnterInicio();
void onEnterConfig();
void onEnterServoAbierto();
void onEnterMonitorAmbiental();
void onEnterMonitorPuertas();
void onEnterAlarma();
void onEnterGestion();
void onEnterBloqueo();

void leerTemperatura();
void leerHall();
void leerSonido();
void leerLuz();
void parpadeoLedAzul();
void parpadeoLedAlarma();
void parpadeoLedBloqueo();
void cerrarServo();
void avanzarPuertas();
void volverAmbientalDesdePuertas();
void volverAmbientalDesdeAlarma();
void volverPuertasDesdeAlarma();
void volverInicioDesdeAlarma();
void volverGestionDesdeAlarma();
void salirBloqueo();
void disparoProgramado();
void confirmarBotonDebounce();

void inicializarEEPROM();
bool buscarUIDenEEPROM(byte *uid, byte longitud, byte *perfilEncontrado);
bool buscarClaveUsuarioEEPROM(const char *intento, byte *perfilEncontrado, unsigned int *baseEncontrada);
bool buscarClavePerfilEEPROM(byte perfil, unsigned int *baseEncontrada);
bool registrarNuevaTarjeta(byte *uid, byte longitud, byte perfil);
bool registrarClaveUsuario(const char *nuevoPass, byte perfil);
bool eliminarTarjeta(byte *uid, byte longitud);
byte leerFlagsUsuario(unsigned int base);
void desactivarClavesPerfil(byte perfil, unsigned int baseConservada);
byte leerUsosClaveUsuario(unsigned int base);
bool claveUsuarioVencida(unsigned int base);
void registrarUsoClaveUsuario(unsigned int base);
void reiniciarUsosClaveUsuario(unsigned int base);
bool claveUsuarioCoincide(unsigned int base, const char *intento);
void registrarIntentoClaveFallido();
void mostrarUsosClaveLCD(byte usos);
void registrarEventoLog(byte codigoEvento);
void cargarRelojEEPROM();
void guardarRelojEEPROM();
void cargarHorariosEEPROM();
void guardarHorariosEEPROM();
void cargarHorariosDefault();
bool horariosValidos();

void apagarLEDs();
void activarBuzzer(unsigned int duracion, unsigned int frecuencia);
void detenerBuzzer();
void iniciarMelodiaAlarma();
void reproducirSiguienteNotaAlarma();
void mostrarMensajeLCD(const char *l1, const char *l2);
void mostrarPantallaInicio();
void resetearBufferPassword();
void prepararMensajeBloqueo(const char *l1, const char *l2);
void resetearMensajeBloqueo();

void procesarTecla(char tecla);
void procesarBoton();
void procesarTarjetaRFID();

unsigned int leerAnalogicoPromediado(byte pin);
unsigned int leerAnalogicoPico(byte pin);
unsigned int leerAnalogicoValle(byte pin);
unsigned int suavizarAnalogico(unsigned int anterior, unsigned int nuevo);
unsigned int invertirLecturaAnalogica(unsigned int lectura);
unsigned int leerHallKY035();
unsigned int leerSonidoKY037();
unsigned int leerLuzKY018();
float calcularTemperaturaNTC(float resistencia);
float convertirTemperaturaC(unsigned int lectura);
void formatearTemperatura(char *destino, byte tamano, float temperatura);
void actualizarLecturasSensores(bool filtrar);
void limpiarAlarmasPendientes();
bool condicionAmbientalAlarma();
bool condicionPuertasAlarma();
void cargarUmbralesEEPROM();
void guardarUmbralesEEPROM();
bool umbralesValidos();
unsigned int minutoActualDelDia();
bool configurarHoraActual(const char *hhmm);
bool configurarHorarioPerfil(byte perfil, const char *hhmmInicioFin);
bool horarioPermiteMinuto(unsigned int minuto, unsigned int inicio, unsigned int fin);
bool accesoPermitidoPorHorario(byte perfil);
int convertirHHMMaMinutos(const char *texto);
void formatearHora(unsigned int minuto, char *destino, byte tamano);
const char *nombrePerfilLCD(byte perfil);
void mostrarMenuConfig();
void mostrarMenuRelojHorario();
void mostrarHorarioPerfil(byte perfil);
void mostrarMenuGestion();
void iniciarEdicionUmbral(char opcion);
void procesarTeclaGestion(char tecla);
void resetearBufferUmbral();
void aplicarUmbralActual();
void actualizarLCDMonitoreo();
void imprimirLineaLCD(byte fila, const char *texto);

//9. Tareas asincronicas

/** @brief Tarea periodica para muestrear temperatura. */
AsyncTask taskTemperatura(1000, true, leerTemperatura);
/** @brief Tarea periodica para muestrear el sensor Hall. */
AsyncTask taskHall(300,  true, leerHall);
/** @brief Tarea periodica para muestrear el microfono. */
AsyncTask taskSonido(200, true, leerSonido);
/** @brief Tarea periodica para muestrear luz ambiente. */
AsyncTask taskLuz(500,    true, leerLuz);

/** @brief Tarea de latido visual del LED azul. */
AsyncTask taskLedAzul(800, true, parpadeoLedAzul);

/** @brief Tarea de parpadeo rapido durante alarma. */
AsyncTask taskLedAlarma(LED_TICK_MS, true, parpadeoLedAlarma);
/** @brief Tarea de parpadeo lento durante bloqueo. */
AsyncTask taskLedBloqueo(LED_TICK_MS, true, parpadeoLedBloqueo);

/** @brief Temporizador de cierre automatico del servo. */
AsyncTask taskServo(TIEMPO_SERVO_ABIERTO, false, cerrarServo);
/** @brief Temporizador para pasar de ambiente a puertas. */
AsyncTask taskAmbPuertas(TIEMPO_AMBIENTAL_PUERTAS, false, avanzarPuertas);
/** @brief Temporizador para volver de puertas a ambiente. */
AsyncTask taskPuertasAmb(TIEMPO_PUERTAS_AMBIENTAL, false, volverAmbientalDesdePuertas);
/** @brief Temporizador para salir de alarma hacia ambiente. */
AsyncTask taskAlarmaAmb(TIEMPO_ALARMA_AMBIENTAL, false, volverAmbientalDesdeAlarma);
/** @brief Temporizador para salir de alarma hacia puertas. */
AsyncTask taskAlarmaPuertas(TIEMPO_ALARMA_PUERTAS, false, volverPuertasDesdeAlarma);
/** @brief Temporizador para salir de alarma hacia inicio. */
AsyncTask taskAlarmaInicio(TIEMPO_ALARMA_AMBIENTAL, false, volverInicioDesdeAlarma);
/** @brief Temporizador para entrar a gestion despues de alarmas repetidas. */
AsyncTask taskAlarmaGestion(2000, false, volverGestionDesdeAlarma);
/** @brief Temporizador para terminar el bloqueo temporal. */
AsyncTask taskBloqueo(TIEMPO_BLOQUEO, false, salirBloqueo);

/** @brief Disparador diferido de transiciones programadas. */
AsyncTask taskAvanceAuto(50, false, disparoProgramado);
/** @brief Antirrebote del boton fisico. */
AsyncTask taskBotonDebounce(50, false, confirmarBotonDebounce);
/** @brief Refresco periodico del LCD en monitoreo. */
AsyncTask taskLCDMonitoreo(750, true, actualizarLCDMonitoreo);
/** @brief Avance no bloqueante de la melodia de alarma. */
AsyncTask taskMelodiaAlarma(TICK_MELODIA_MS, true, reproducirSiguienteNotaAlarma);

// Setup
/**
 * @brief Inicializa periferia, EEPROM, FSM y tareas.
 *
 * @details
 *   Configura pines, LCD, servo, SPI, lector RFID, valores persistidos y
 *   tareas asincronas. Finaliza dejando la maquina de estados en `ST_INICIO`.
 *
 * @par Parameters
 *    None.
 *
 * @par Returns
 *    Nothing.
 */
void setup() {
  Serial.begin(9600);
  Serial.println(F("== Cerradura Inteligente =="));

  lcd.begin(16, 2);
  mostrarMensajeLCD("Iniciando...", "Por favor espere");

  pinMode(PIN_LED_ROJO,  OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AZUL,  OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_BOTON,     INPUT_PULLUP);  
  apagarLEDs();
  detenerBuzzer();

  lockServo.attach(PIN_SERVO);
  lockServo.write(0);                   

  SPI.begin();
  lectorRFID.PCD_Init();
  byte v = lectorRFID.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print(F("[RFID] Version: 0x")); Serial.println(v, HEX);

  inicializarEEPROM();
  cargarUmbralesEEPROM();
  cargarRelojEEPROM();
  cargarHorariosEEPROM();
  actualizarLecturasSensores(false);

  configurarEstados();
  miMaquina.SetState(ST_INICIO, false, true);

  taskTemperatura.Start();
  taskHall.Start();
  taskSonido.Start();
  taskLuz.Start();
  taskLedAzul.Start();

  Serial.println(F("Sistema listo. Pass por defecto: 123456"));
}

/**
 * @brief Ejecuta el ciclo principal no bloqueante.
 *
 * @details
 *   Actualiza todas las tareas, procesa teclado, boton y RFID, evalua alarmas
 *   pendientes y refresca el LCD de monitoreo sin bloqueos.
 */
void loop() {
  taskTemperatura.Update();
  taskHall.Update();
  taskSonido.Update();
  taskLuz.Update();
  taskLedAzul.Update();
  taskLedAlarma.Update();
  taskLedBloqueo.Update();
  taskServo.Update();
  taskAmbPuertas.Update();
  taskPuertasAmb.Update();
  taskAlarmaAmb.Update();
  taskAlarmaPuertas.Update();
  taskAlarmaInicio.Update();
  taskAlarmaGestion.Update();
  taskBloqueo.Update();
  taskAvanceAuto.Update();
  taskBotonDebounce.Update();
  taskLCDMonitoreo.Update();
  taskMelodiaAlarma.Update();

  byte est = (byte)miMaquina.GetState();

  char tecla = teclado.getKey();
  if (tecla != NO_KEY) procesarTecla(tecla);

  procesarBoton();

  if (est == ST_INICIO || est == ST_CONFIG) procesarTarjetaRFID();

  // Eventos ambientales -> ALARMA
  if (eventoAmbientalAlarma && est == ST_MONITOR_AMBIENTAL) {
    if (condicionAmbientalAlarma()) {
      eventoActual = EV_AMBIENTAL_ALARMA;
    } else {
      eventoAmbientalAlarma = false;
    }
  } else if (eventoPuertaAlarma && est == ST_MONITOR_PUERTAS) {
    if (condicionPuertasAlarma()) {
      eventoActual = EV_PUERTA_ALARMA;
    } else {
      eventoPuertaAlarma = false;
    }
  }

  if (eventoActual != EV_NINGUNO) {
    miMaquina.Update();
    eventoActual = EV_NINGUNO;
  }
}

// Configuracion maquina de estados

/**
 * @brief Configurar transiciones y callbacks de entrada.
 *
 * @details
 *   Registra todas las transiciones declarativas de `miMaquina`. Las
 *   condiciones leen eventoActual, por lo que el resto del sketch solo necesita
 *   publicar eventos y llamar a `miMaquina.Update()`.
 *
 * @par Parameters
 *    None.
 *
 * @par Returns
 *    Nothing.
 */
void configurarEstados() {

  // INICIO
  miMaquina.AddTransition(ST_INICIO, ST_CONFIG,
                          [](){ return eventoActual == EV_TECLA_CONFIG; });
  miMaquina.AddTransition(ST_INICIO, ST_SERVO_ABIERTO,
                          [](){ return eventoActual == EV_CLAVE_OK; });
  miMaquina.AddTransition(ST_INICIO, ST_ALARMA,
                          [](){ return eventoActual == EV_INTENTOS_ALARMA; });
  miMaquina.AddTransition(ST_INICIO, ST_BLOQUEO,
                          [](){ return eventoActual == EV_A_BLOQUEO; });

  // CONFIG
  miMaquina.AddTransition(ST_CONFIG, ST_INICIO,
                          [](){ return eventoActual == EV_BOTON; });

  // SERVO_ABIERTO -> MONITOR_AMBIENTAL
  miMaquina.AddTransition(ST_SERVO_ABIERTO, ST_MONITOR_AMBIENTAL,
                          [](){ return eventoActual == EV_SERVO_TIMEOUT; });

  // MONITOR_AMBIENTAL
  miMaquina.AddTransition(ST_MONITOR_AMBIENTAL, ST_MONITOR_PUERTAS,
                          [](){ return eventoActual == EV_A_PUERTAS; });
  miMaquina.AddTransition(ST_MONITOR_AMBIENTAL, ST_ALARMA,
                          [](){ return eventoActual == EV_AMBIENTAL_ALARMA; });
  miMaquina.AddTransition(ST_MONITOR_AMBIENTAL, ST_GESTION,
                          [](){ return eventoActual == EV_A_GESTION; });

  // MONITOR_PUERTAS
  miMaquina.AddTransition(ST_MONITOR_PUERTAS, ST_MONITOR_AMBIENTAL,
                          [](){ return eventoActual == EV_A_AMBIENTAL; });
  miMaquina.AddTransition(ST_MONITOR_PUERTAS, ST_ALARMA,
                          [](){ return eventoActual == EV_PUERTA_ALARMA; });
  miMaquina.AddTransition(ST_MONITOR_PUERTAS, ST_GESTION,
                          [](){ return eventoActual == EV_A_GESTION; });

  // ALARMA
  miMaquina.AddTransition(ST_ALARMA, ST_MONITOR_AMBIENTAL,
                          [](){ return eventoActual == EV_A_AMBIENTAL; });
  miMaquina.AddTransition(ST_ALARMA, ST_MONITOR_PUERTAS,
                          [](){ return eventoActual == EV_A_PUERTAS; });
  miMaquina.AddTransition(ST_ALARMA, ST_INICIO,
                          [](){ return eventoActual == EV_A_INICIO; });
  miMaquina.AddTransition(ST_ALARMA, ST_GESTION,
                          [](){ return eventoActual == EV_A_GESTION; });

  // GESTION
  miMaquina.AddTransition(ST_GESTION, ST_INICIO,
                          [](){ return eventoActual == EV_ESTRELLA; });

  // BLOQUEO
  miMaquina.AddTransition(ST_BLOQUEO, ST_INICIO,
                          [](){ return eventoActual == EV_BLOQUEO_TIMEOUT; });

  // Callbacks de entrada
  miMaquina.SetOnEntering(ST_INICIO,            onEnterInicio);
  miMaquina.SetOnEntering(ST_CONFIG,            onEnterConfig);
  miMaquina.SetOnEntering(ST_SERVO_ABIERTO,     onEnterServoAbierto);
  miMaquina.SetOnEntering(ST_MONITOR_AMBIENTAL, onEnterMonitorAmbiental);
  miMaquina.SetOnEntering(ST_MONITOR_PUERTAS,   onEnterMonitorPuertas);
  miMaquina.SetOnEntering(ST_ALARMA,            onEnterAlarma);
  miMaquina.SetOnEntering(ST_GESTION,           onEnterGestion);
  miMaquina.SetOnEntering(ST_BLOQUEO,           onEnterBloqueo);
}

/* ============================================================================
 *                  |     13. ACCIONES POR ESTADO
 * ========================================================================= */

/**
 * @brief Entrar al estado inicial de acceso.
 *
 * @details
 *   Limpia autenticacion, alarmas, temporizadores y buffers antes de mostrar
 *   la pantalla inicial de acceso por clave o RFID.
 */
void onEnterInicio() {
  Serial.println(F(">>> INICIO"));
  apagarLEDs();
  detenerBuzzer();
  resetearBufferPassword();
  modoIngresoClave = false;
  perfilActual = PERFIL_NINGUNO;
  accesoFueConClave = false;
  usosClaveActual = 0;
  subOpcionConfig = 0;
  perfilHorarioConfig = PERFIL_NINGUNO;
  alarmaVieneDeAcceso = false;
  resetearMensajeBloqueo();
  taskLedAlarma.Stop();
  taskLedBloqueo.Stop();
  taskAmbPuertas.Stop();
  taskPuertasAmb.Stop();
  taskLCDMonitoreo.Stop();
  taskAlarmaAmb.Stop();
  taskAlarmaPuertas.Stop();
  taskAlarmaInicio.Stop();
  taskAlarmaGestion.Stop();
  taskAvanceAuto.Stop();
  alarmasConsecutivas = 0;
  limpiarAlarmasPendientes();
  mostrarPantallaInicio();
}

/**
 * @brief Entrar al menu de configuracion.
 *
 * @details
 *   Prepara la interfaz administrativa para editar reloj, horarios por rol,
 *   tarjetas RFID y claves por usuario. El boton fisico vuelve a `ST_INICIO`.
 */
void onEnterConfig() {
  Serial.println(F(">>> CONFIG"));
  apagarLEDs();
  digitalWrite(PIN_LED_AZUL, HIGH);
  detenerBuzzer();
  taskLCDMonitoreo.Stop();
  subOpcionConfig = 0;
  perfilHorarioConfig = PERFIL_NINGUNO;
  modoIngresoClave = false;
  resetearBufferPassword();
  resetearBufferUmbral();
  mostrarMenuConfig();
}

/**
 * @brief Abrir la cerradura despues de un acceso valido.
 *
 * @details
 *   Enciende el LED verde, activa un tono corto, mueve el servo a posicion de
 *   apertura y programa el cierre automatico.
 */
void onEnterServoAbierto() {
  Serial.println(F(">>> SERVO_ABIERTO"));
  taskLCDMonitoreo.Stop();
  digitalWrite(PIN_LED_ROJO,  LOW);
  digitalWrite(PIN_LED_AZUL,  LOW);
  digitalWrite(PIN_LED_VERDE, HIGH);     // LED verde acceso
  Serial.println(F("[LED] Verde ENCENDIDO"));
  activarBuzzer(150, 2000);
  lockServo.write(90);                    // abrir cerradura
  registrarEventoLog(0x01);
  intentosFallidos = 0;
  if (accesoFueConClave) {
    char l2[LCD_BUFFER_TAM];
    byte restantes = (usosClaveActual >= USOS_CLAVE_MAX) ? 0 : (USOS_CLAVE_MAX - usosClaveActual);
    snprintf(l2, sizeof(l2), "Clave %d/%d usos", restantes, USOS_CLAVE_MAX);
    mostrarMensajeLCD("Acceso permitido", l2);
  } else {
    mostrarMensajeLCD("Acceso permitido", "Puerta abierta 5s");
  }
  taskServo.Start();
}

/**
 * @brief Iniciar el monitoreo ambiental.
 *
 * @details
 *   Refresca lecturas de temperatura y luz, muestra valores actuales y arma el
 *   temporizador que alterna hacia monitoreo de puertas.
 */
void onEnterMonitorAmbiental() {
  Serial.println(F(">>> MONITOR_AMBIENTAL"));
  apagarLEDs();
  detenerBuzzer();
  taskLedAlarma.Stop();
  taskLedBloqueo.Stop();
  taskPuertasAmb.Stop();
  taskLCDMonitoreo.Stop();
  eventoAmbientalAlarma = false;
  actualizarLecturasSensores(false);
  digitalWrite(PIN_LED_AZUL, HIGH);
  actualizarLCDMonitoreo();
  taskLCDMonitoreo.Start();
  taskAmbPuertas.Start();

  // Evaluacion inmediata: si la condicion sigue activa, marca alarma
  // (el loop la procesara en el siguiente ciclo y contara la alarma)
  if (condicionAmbientalAlarma()) {
    eventoAmbientalAlarma = true;
    alarmaVieneDePuertas = false;
    alarmaVieneDeAcceso = false;
  }
}

/**
 * @brief Iniciar el monitoreo de puertas.
 *
 * @details
 *   Limpia alarmas pendientes antes de supervisar la condicion compuesta de
 *   puerta: Hall y sonido deben superar sus umbrales simultaneamente.
 */
void onEnterMonitorPuertas() {
  Serial.println(F(">>> MONITOR_PUERTAS"));
  apagarLEDs();
  detenerBuzzer();
  taskLedAlarma.Stop();
  taskLedBloqueo.Stop();
  taskAmbPuertas.Stop();
  taskLCDMonitoreo.Stop();
  eventoPuertaAlarma = false;
  actualizarLecturasSensores(false);
  digitalWrite(PIN_LED_AZUL, HIGH);
  actualizarLCDMonitoreo();
  taskLCDMonitoreo.Start();
  taskPuertasAmb.Start();
}

/**
 * @brief Entrar al estado de alarma.
 *
 * @details
 *   Detiene monitoreos, arranca melodia y parpadeo rojo, registra el evento y
 *   decide si volver a monitoreo o pasar a `ST_GESTION` por alarmas repetidas.
 */
void onEnterAlarma() {
  Serial.println(F(">>> ALARMA"));
  detenerBuzzer();
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AZUL,  LOW);
  taskAmbPuertas.Stop();
  taskPuertasAmb.Stop();
  taskLedAlarma.Stop();
  taskLedBloqueo.Stop();
  taskLCDMonitoreo.Stop();
  taskAlarmaAmb.Stop();
  taskAlarmaPuertas.Stop();
  taskAlarmaInicio.Stop();
  taskAlarmaGestion.Stop();
  taskAvanceAuto.Stop();
  // Limpiar los flags para que esta alarma sea un evento discreto.
  // Asi, al volver al monitoreo, la condicion debe re-cumplirse para
  // contar la siguiente alarma. Esto hace fiable el conteo consecutivo.
  eventoAmbientalAlarma = false;
  eventoPuertaAlarma    = false;
  iniciarMelodiaAlarma();

  // Parpadeo rapido del LED rojo: 100 ms ON / 200 ms OFF
  estadoLedRojo = false;
  ticksParpadeoRojo = 0;
  taskLedAlarma.Start();

  if (alarmaVieneDeAcceso) {
    Serial.println(F("[ALARMA] 3 intentos de clave"));
    registrarEventoLog(0x06);
    mostrarMensajeLCD("!! ALARMA !!", "3 intentos clave");
    intentosFallidos = 0;
    taskAlarmaInicio.Start();
    return;
  }

  alarmasConsecutivas++;
  Serial.print(F("[ALARMA] consecutiva "));
  Serial.print(alarmasConsecutivas);
  Serial.print(F(" de "));
  Serial.println(MAX_ALARMAS_CONSECUTIVAS);
  registrarEventoLog(0x04);
  char l2[LCD_BUFFER_TAM];
  snprintf(l2, sizeof(l2), "Alarma %d de %d", alarmasConsecutivas, MAX_ALARMAS_CONSECUTIVAS);
  mostrarMensajeLCD("!! ALARMA !!", l2);

  // 3 alarmas consecutivas -> GESTION (cambiar umbrales)
  if (alarmasConsecutivas >= MAX_ALARMAS_CONSECUTIVAS) {
    Serial.println(F("[ALARMA] 3 consecutivas -> GESTION"));
    taskAlarmaGestion.Start();
  }
  // Si la alarma vino de PUERTAS, vuelve a PUERTAS tras 4s
  else if (alarmaVieneDePuertas) {
    taskAlarmaPuertas.Start();
  }
  // Si vino de AMBIENTAL, vuelve a AMBIENTAL tras 3s
  else {
    taskAlarmaAmb.Start();
  }
}

/**
 * @brief Entrar al menu de gestion de umbrales.
 *
 * @details
 *   Prepara el LCD y los buffers para editar umbrales ambientales tras una
 *   secuencia de alarmas o por solicitud desde monitoreo.
 */
void onEnterGestion() {
  Serial.println(F(">>> GESTION"));
  apagarLEDs();
  detenerBuzzer();
  taskLedAlarma.Stop();
  taskLedBloqueo.Stop();
  taskAmbPuertas.Stop();
  taskPuertasAmb.Stop();
  taskLCDMonitoreo.Stop();
  taskAlarmaAmb.Stop();
  taskAlarmaPuertas.Stop();
  taskAlarmaInicio.Stop();
  taskAlarmaGestion.Stop();
  taskAvanceAuto.Stop();
  limpiarAlarmasPendientes();
  actualizarLecturasSensores(false);
  digitalWrite(PIN_LED_AZUL, HIGH);
  alarmasConsecutivas = 0;            // reinicia el conteo de alarmas
  opcionGestionUmbral = 0;
  resetearBufferUmbral();
  mostrarMenuGestion();
}

/**
 * @brief Entrar al bloqueo temporal.
 *
 * @details
 *   Muestra el motivo preparado en lineaBloqueo1/lineaBloqueo2, registra el
 *   evento y programa la vuelta automatica a `ST_INICIO`.
 */
void onEnterBloqueo() {
  Serial.println(F(">>> BLOQUEO"));
  detenerBuzzer();
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AZUL,  LOW);
  taskAmbPuertas.Stop();
  taskPuertasAmb.Stop();
  taskLCDMonitoreo.Stop();
  taskLedAlarma.Stop();
  taskAlarmaAmb.Stop();
  taskAlarmaPuertas.Stop();
  taskAlarmaInicio.Stop();
  taskAlarmaGestion.Stop();
  registrarEventoLog(0x03);
  mostrarMensajeLCD(lineaBloqueo1, lineaBloqueo2);

  estadoLedRojo = false;
  ticksParpadeoRojo = 0;
  taskLedBloqueo.Start();

  taskBloqueo.Start();
}

//Call backs de tareas

/**
 * @brief Leer temperatura y evaluar alarma ambiental.
 *
 * @details
 *   Actualiza temperaturaActual con filtrado exponencial y marca
 *   eventoAmbientalAlarma cuando temperatura y luz superan sus umbrales.
 */
void leerTemperatura() {
  lecturaTempRaw = leerAnalogicoPromediado(PIN_KY013);
  float tempC = convertirTemperaturaC(lecturaTempRaw);
  if (tempC > TEMP_SENSOR_MIN_C && tempC < TEMP_SENSOR_MAX_C) {
    temperaturaActual = 0.7 * temperaturaActual + 0.3 * tempC;
  }

  // Alarma ambiental: temperatura y luz deben superar sus umbrales.
  if ((Estado) miMaquina.GetState() == ST_MONITOR_AMBIENTAL) {
    if (condicionAmbientalAlarma()) {
      eventoAmbientalAlarma = true;
      alarmaVieneDePuertas = false;   // origen: ambiental
      alarmaVieneDeAcceso = false;
    }
  }
}

/**
 * @brief Leer el sensor Hall y evaluar alarma de puerta.
 *
 * @details
 *   En `ST_MONITOR_PUERTAS`, marca alarma solo cuando el Hall y el microfono
 *   superan simultaneamente sus umbrales configurados.
 */
void leerHall() {
  lecturaHall = suavizarAnalogico(lecturaHall, leerHallKY035());

  if ((Estado) miMaquina.GetState() != ST_MONITOR_PUERTAS) return;

  if (condicionPuertasAlarma()) {
    eventoPuertaAlarma = true;
    alarmaVieneDePuertas = true;
    alarmaVieneDeAcceso = false;
    Serial.println(F("[PUERTA] Hall y sonido superan umbrales"));
  }
}

/**
 * @brief Leer el microfono y evaluar alarma de puerta.
 *
 * @details
 *   En `ST_MONITOR_PUERTAS`, marca alarma solo cuando el sonido y el Hall
 *   superan simultaneamente sus umbrales configurados.
 */
void leerSonido() {
  lecturaSonido = leerSonidoKY037();

  if ((Estado) miMaquina.GetState() != ST_MONITOR_PUERTAS) return;

  if (condicionPuertasAlarma()) {
    eventoPuertaAlarma = true;
    alarmaVieneDePuertas = true;
    alarmaVieneDeAcceso = false;
    Serial.println(F("[PUERTA] Hall y sonido superan umbrales"));
  }
}

/**
 * @brief Leer luz ambiente.
 *
 * @details
 *   Actualiza lecturaLuz con filtrado suave. La condicion de alarma ambiental
 *   se evalua junto con la temperatura en leerTemperatura().
 */
void leerLuz() {
  lecturaLuz = suavizarAnalogico(lecturaLuz, leerLuzKY018());
}

/**
 * @brief Alternar el LED azul mientras el sistema esta en `ST_INICIO`.
 *
 * @details
 *   Callback periodico de indicacion visual. Solo modifica el LED si la FSM
 *   permanece en el estado inicial.
 */
void parpadeoLedAzul() {
  if ((Estado) miMaquina.GetState() == ST_INICIO) {
    digitalWrite(PIN_LED_AZUL, !digitalRead(PIN_LED_AZUL));
  }
}

/**
 * @brief Alternar el LED rojo con cadencia rapida durante `ST_ALARMA`.
 *
 * @details
 *   Usa ticks fijos para producir la cadencia de alarma sin bloquear el
 *   `loop()`.
 */
void parpadeoLedAlarma() {
  if ((Estado) miMaquina.GetState() != ST_ALARMA) return;
  if (ticksParpadeoRojo > 0) {
    ticksParpadeoRojo--;
    return;
  }
  estadoLedRojo = !estadoLedRojo;
  digitalWrite(PIN_LED_ROJO, estadoLedRojo ? HIGH : LOW);
  ticksParpadeoRojo = estadoLedRojo ? (ALARMA_TICKS_ON - 1) : (ALARMA_TICKS_OFF - 1);
}

/**
 * @brief Alternar el LED rojo con cadencia lenta durante `ST_BLOQUEO`.
 *
 * @details
 *   Mantiene la senal visual de bloqueo mientras la FSM no haya regresado a
 *   `ST_INICIO`.
 */
void parpadeoLedBloqueo() {
  if ((Estado) miMaquina.GetState() != ST_BLOQUEO) return;
  if (ticksParpadeoRojo > 0) {
    ticksParpadeoRojo--;
    return;
  }
  estadoLedRojo = !estadoLedRojo;
  digitalWrite(PIN_LED_ROJO, estadoLedRojo ? HIGH : LOW);
  ticksParpadeoRojo = estadoLedRojo ? (BLOQUEO_TICKS_ON - 1) : (BLOQUEO_TICKS_OFF - 1);
}

/**
 * @brief Cerrar el servo y programar monitoreo ambiental.
 *
 * @details
 *   Callback de taskServo. Publica EV_SERVO_TIMEOUT por medio de
 *   eventoProgramado para que la transicion se ejecute fuera del callback.
 */
void cerrarServo() {
  Serial.println(F("[Tarea] Servo cerrado -> monitoreo"));
  lockServo.write(0);
  digitalWrite(PIN_LED_VERDE, LOW);
  eventoProgramado = EV_SERVO_TIMEOUT;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la transicion de ambiente a puertas.
 *
 * @details
 *   Publica `EV_A_PUERTAS` de forma diferida para que el callback de tiempo no
 *   modifique la FSM directamente.
 */
void avanzarPuertas() {
  eventoProgramado = EV_A_PUERTAS;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la transicion de puertas a ambiente.
 *
 * @details
 *   Publica `EV_A_AMBIENTAL` despues del intervalo de monitoreo de puertas.
 */
void volverAmbientalDesdePuertas() {
  eventoProgramado = EV_A_AMBIENTAL;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la salida de alarma hacia ambiente.
 *
 * @details
 *   Retorna al ciclo ambiental cuando la alarma no provino del monitoreo de
 *   puertas.
 */
void volverAmbientalDesdeAlarma() {
  eventoProgramado = EV_A_AMBIENTAL;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la salida de alarma hacia puertas.
 *
 * @details
 *   Retorna al monitoreo de puertas cuando la alarma fue originada por Hall o
 *   sonido.
 */
void volverPuertasDesdeAlarma() {
  eventoProgramado = EV_A_PUERTAS;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la salida de alarma hacia inicio.
 */
void volverInicioDesdeAlarma() {
  eventoProgramado = EV_A_INICIO;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la salida de alarma hacia gestion.
 */
void volverGestionDesdeAlarma() {
  eventoProgramado = EV_A_GESTION;
  taskAvanceAuto.Start();
}

/**
 * @brief Programar la salida de bloqueo hacia inicio.
 *
 * @details
 *   Publica `EV_BLOQUEO_TIMEOUT` cuando termina el periodo de bloqueo.
 */
void salirBloqueo() {
  Serial.println(F("[Tarea] Fin del bloqueo"));
  eventoProgramado = EV_BLOQUEO_TIMEOUT;
  taskAvanceAuto.Start();
}

/**
 * @brief Publicar una transicion diferida.
 *
 * @details
 *   Copia eventoProgramado a eventoActual y limpia el valor pendiente. Evita
 *   llamar a la maquina de estados desde callbacks de tareas.
 */
void disparoProgramado() {
  if (eventoProgramado != EV_NINGUNO) {
    eventoActual = eventoProgramado;
    eventoProgramado = EV_NINGUNO;
  }
}

//Procesamiento del teclado
/**
 * @brief Despachar una tecla segun el estado actual.
 *
 * @details
 *   Maneja captura de clave, menus de configuracion, seleccion de perfiles,
 *   configuracion de hora/horarios y edicion de umbrales.
 *
 * @param [in] tecla  Caracter entregado por `Keypad::getKey()`.
 */
void procesarTecla(char tecla) {
  byte est = (byte)miMaquina.GetState();

  // GESTION: mini-menu de umbrales. '*' sale a INICIO.
  if (est == ST_GESTION) {
    procesarTeclaGestion(tecla);
    return;
  }

  // ---- INICIO ----
  // Diagrama: tecla A = ingresar clave, tecla # = ir a CONFIG, RFID tambien valida.
  if (est == ST_INICIO) {

    // Tecla # -> entrar a CONFIG
    if (tecla == '#') {
      eventoActual = EV_TECLA_CONFIG;
      return;
    }

    // Tecla A -> habilitar captura o confirmar la clave ingresada.
    if (tecla == 'A') {
      if (!modoIngresoClave) {
        modoIngresoClave = true;
        resetearBufferPassword();
        mostrarMensajeLCD("Ingrese clave:", "6 dig luego A");
        return;
      }

      if (posPassword == 0) {
        mostrarMensajeLCD("Ingrese clave:", "6 dig luego A");
        return;
      }
      // Validar la clave de usuario igual que una credencial RFID.
      byte perfilClave = PERFIL_NINGUNO;
      unsigned int baseClave = 0;
      if (posPassword == LONGITUD_PASSWORD
          && buscarClaveUsuarioEEPROM(passwordIngresada, &perfilClave, &baseClave)) {
        if (claveUsuarioVencida(baseClave)) {
          Serial.println(F("[CLAVE] vencida"));
          prepararMensajeBloqueo("Clave vencida", "Cambie CONFIG C");
          resetearBufferPassword();
          modoIngresoClave = false;
          eventoActual = EV_A_BLOQUEO;
          return;
        }
        perfilActual = perfilClave;
        Serial.print(F("[CLAVE] valida, perfil ")); Serial.println(perfilActual);
        if (accesoPermitidoPorHorario(perfilActual)) {
          registrarUsoClaveUsuario(baseClave);
          accesoFueConClave = true;
          usosClaveActual = leerUsosClaveUsuario(baseClave);
          mostrarUsosClaveLCD(usosClaveActual);
          intentosFallidos = 0;
          resetearBufferPassword();
          modoIngresoClave = false;
          eventoActual = EV_CLAVE_OK;        // estado del servo
        } else {
          registrarEventoLog(0x05);
          resetearBufferPassword();
          modoIngresoClave = false;
          eventoActual = EV_A_BLOQUEO;       // cuando esta fuera de horario
        }
      } else {
        modoIngresoClave = false;
        registrarIntentoClaveFallido();
      }
      return;
    }

    // Digitos 0-9: solo se aceptan despues de pulsar A.
    if (tecla >= '0' && tecla <= '9') {
      if (!modoIngresoClave) {
        mostrarMensajeLCD("Pulse A clave", "RFID o # config");
        return;
      }
      if (posPassword < LONGITUD_PASSWORD) {
        passwordIngresada[posPassword++] = tecla;
        passwordIngresada[posPassword] = '\0';
        lcd.setCursor(posPassword - 1, 1);
        lcd.print('*');
      }
      return;
    }

    // Tecla * -> borrar lo escrito
    if (tecla == '*') {
      resetearBufferPassword();
      modoIngresoClave = false;
      mostrarPantallaInicio();
      return;
    }
    return;
  }

  // ---- CONFIG ----
  if (est == ST_CONFIG) {
    if (tecla == '*' && subOpcionConfig != 0) {
      subOpcionConfig = 0;
      perfilHorarioConfig = PERFIL_NINGUNO;
      resetearBufferPassword();
      resetearBufferUmbral();
      mostrarMenuConfig();
      return;
    }

    if (subOpcionConfig == 0 && (tecla == 'A' || tecla == 'B' || tecla == 'C' || tecla == 'D')) {
      subOpcionConfig = tecla;
      if (tecla == 'A') mostrarMenuRelojHorario();
      else if (tecla == 'B') mostrarMensajeLCD("Perfil 1-4:", "1S 2O 3C 4G");
      else if (tecla == 'C') { perfilHorarioConfig = PERFIL_NINGUNO; resetearBufferPassword(); mostrarMensajeLCD("Clave perfil:", "1S 2O 3C 4G"); }
      else if (tecla == 'D') mostrarMensajeLCD("Eliminar RFID", "Acerque tarjeta");
    }
    else if (subOpcionConfig == 'A') {
      if (tecla == '1') {
        subOpcionConfig = 'H';
        resetearBufferUmbral();
        char hora[HORA_BUFFER_TAM];
        formatearHora(minutoActualDelDia(), hora, sizeof(hora));
        char l1[LCD_BUFFER_TAM];
        snprintf(l1, sizeof(l1), "Hora %s", hora);
        mostrarMensajeLCD(l1, "HHMM + #");
      } else if (tecla == '2') {
        subOpcionConfig = 'P';
        perfilHorarioConfig = PERFIL_NINGUNO;
        mostrarMensajeLCD("Rol 1S 2O 3C", "4G * atras");
      } else {
        mostrarMenuRelojHorario();
      }
    }
    else if (subOpcionConfig == 'H') {
      if (tecla >= '0' && tecla <= '9' && posBufferUmbral < 4) {
        bufferUmbral[posBufferUmbral++] = tecla;
        bufferUmbral[posBufferUmbral] = '\0';
        imprimirLineaLCD(1, bufferUmbral);
      } else if (tecla == '#' && posBufferUmbral == 4) {
        if (configurarHoraActual(bufferUmbral)) {
          char hora[HORA_BUFFER_TAM];
          char l2[LCD_BUFFER_TAM];
          formatearHora(minutoActualDelDia(), hora, sizeof(hora));
          snprintf(l2, sizeof(l2), "%s guardada", hora);
          mostrarMensajeLCD("Hora actual", l2);
          subOpcionConfig = 0;
        } else {
          mostrarMensajeLCD("Hora invalida", "Use HHMM");
        }
        resetearBufferUmbral();
      }
    }
    else if (subOpcionConfig == 'P') {
      if (tecla >= '1' && tecla <= '4') {
        perfilHorarioConfig = tecla - '0';
        subOpcionConfig = 'R';
        resetearBufferUmbral();
        mostrarHorarioPerfil(perfilHorarioConfig);
      } else {
        mostrarMensajeLCD("Rol 1S 2O 3C", "4G * atras");
      }
    }
    else if (subOpcionConfig == 'R') {
      if (tecla >= '0' && tecla <= '9' && posBufferUmbral < 8) {
        bufferUmbral[posBufferUmbral++] = tecla;
        bufferUmbral[posBufferUmbral] = '\0';
        imprimirLineaLCD(1, bufferUmbral);
      } else if (tecla == '#' && posBufferUmbral == 8) {
        if (configurarHorarioPerfil(perfilHorarioConfig, bufferUmbral)) {
          char ini[HORA_BUFFER_TAM], fin[HORA_BUFFER_TAM];
          char l2[LCD_BUFFER_TAM];
          formatearHora(horarioInicioPorPerfil[perfilHorarioConfig], ini, sizeof(ini));
          formatearHora(horarioFinPorPerfil[perfilHorarioConfig], fin, sizeof(fin));
          snprintf(l2, sizeof(l2), "%s-%s", ini, fin);
          mostrarMensajeLCD("Horario guardado", l2);
          subOpcionConfig = 0;
          perfilHorarioConfig = PERFIL_NINGUNO;
        } else {
          mostrarMensajeLCD("Horario invalido", "HHMMHHMM + #");
        }
        resetearBufferUmbral();
      }
    }
    else if (subOpcionConfig == 'B') {
      if (tecla == '1') { perfilParaRegistro = PERFIL_SEGURIDAD;   mostrarMensajeLCD("Perfil Seguridad", "Acerque tarjeta"); }
      else if (tecla == '2') { perfilParaRegistro = PERFIL_OPERARIO;    mostrarMensajeLCD("Perfil Operario", "Acerque tarjeta"); }
      else if (tecla == '3') { perfilParaRegistro = PERFIL_COORDINADOR; mostrarMensajeLCD("Perfil Coordin.", "Acerque tarjeta"); }
      else if (tecla == '4') { perfilParaRegistro = PERFIL_GERENTE;     mostrarMensajeLCD("Perfil Gerente", "Acerque tarjeta"); }
    }
    else if (subOpcionConfig == 'C') {
      if (perfilHorarioConfig == PERFIL_NINGUNO) {
        if (tecla >= '1' && tecla <= '4') {
          perfilHorarioConfig = tecla - '0';
          resetearBufferPassword();
          char l1[LCD_BUFFER_TAM];
          snprintf(l1, sizeof(l1), "Clave %s", nombrePerfilLCD(perfilHorarioConfig));
          mostrarMensajeLCD(l1, "6 dig + #");
        } else {
          mostrarMensajeLCD("Clave perfil:", "1S 2O 3C 4G");
        }
      } else if (tecla >= '0' && tecla <= '9' && posPassword < LONGITUD_PASSWORD) {
        passwordIngresada[posPassword++] = tecla;
        passwordIngresada[posPassword] = '\0';
        lcd.setCursor(posPassword - 1, 1);
        lcd.print('*');
      } else if (tecla == '#' && posPassword == LONGITUD_PASSWORD) {
        if (registrarClaveUsuario(passwordIngresada, perfilHorarioConfig)) {
          mostrarMensajeLCD("Clave usuario", "Guardada OK");
        } else {
          mostrarMensajeLCD("Error clave", "Duplicada/llena");
        }
        subOpcionConfig = 0;
        perfilHorarioConfig = PERFIL_NINGUNO;
        resetearBufferPassword();
      }
    }
    return;
  }
}

/**
 * @brief Mostrar el menu principal de `CONFIG`.
 *
 * @details
 *   Presenta las cuatro ramas administrativas: reloj/horarios, RFID, claves de
 *   usuario y eliminacion de tarjeta.
 */
void mostrarMenuConfig() {
  mostrarMensajeLCD("A:Reloj B:RFID", "C:Clave D:Borrar");
}

/**
 * @brief Mostrar el submenu de reloj y horarios.
 *
 * @details
 *   Permite elegir entre configurar la hora actual o editar el horario de un
 *   perfil.
 */
void mostrarMenuRelojHorario() {
  mostrarMensajeLCD("1:Hora actual", "2:Horario rol");
}

/**
 * @brief Mostrar el horario actual de un perfil antes de editarlo.
 *
 * @param [in] perfil  Perfil entre `PERFIL_SEGURIDAD` y `PERFIL_GERENTE`.
 */
void mostrarHorarioPerfil(byte perfil) {
  char l1[LCD_BUFFER_TAM];
  char l2[LCD_BUFFER_TAM];
  char ini[HORA_BUFFER_TAM], fin[HORA_BUFFER_TAM];

  formatearHora(horarioInicioPorPerfil[perfil], ini, sizeof(ini));
  formatearHora(horarioFinPorPerfil[perfil], fin, sizeof(fin));
  snprintf(l1, sizeof(l1), "%s %s", nombrePerfilLCD(perfil), ini);
  snprintf(l2, sizeof(l2), "%s HHMMHHMM", fin);
  mostrarMensajeLCD(l1, l2);
}

/**
 * @brief Mostrar el menu principal de gestion de umbrales.
 *
 * @details
 *   Lista los sensores editables durante `ST_GESTION` y recuerda las teclas de
 *   salida y guardado.
 */
void mostrarMenuGestion() {
  mostrarMensajeLCD("A:T B:L C:H D:M", "* sale  # guarda");
}

/**
 * @brief Preparar la captura de un nuevo valor de umbral.
 *
 * @param [in] opcion  Tecla de umbral: `A` temperatura, `B` luz, `C` Hall o
 *                     `D` ruido.
 */
void iniciarEdicionUmbral(char opcion) {
  opcionGestionUmbral = opcion;
  resetearBufferUmbral();
  actualizarLecturasSensores(false);

  char l1[LCD_BUFFER_TAM];
  char tempTxt[TEMP_BUFFER_TAM];
  if (opcion == 'A') {
    formatearTemperatura(tempTxt, sizeof(tempTxt), temperaturaActual);
    snprintf(l1, sizeof(l1), "T:%s U:%d", tempTxt, (int)umbralTemp);
  } else if (opcion == 'B') {
    snprintf(l1, sizeof(l1), "L:%u U:%u", lecturaLuz, umbralLuz);
  } else if (opcion == 'C') {
    snprintf(l1, sizeof(l1), "H:%u U:%u", lecturaHall, umbralHallAbierto);
  } else {
    snprintf(l1, sizeof(l1), "M:%u U:%u", lecturaSonido, umbralRuido);
  }

  mostrarMensajeLCD(l1, "Nuevo valor #");
}

/**
 * @brief Procesar teclado dentro de `ST_GESTION`.
 *
 * @param [in] tecla  Caracter entregado por `Keypad::getKey()`.
 */
void procesarTeclaGestion(char tecla) {
  if (tecla == '*') {
    eventoActual = EV_ESTRELLA;
    return;
  }

  if (opcionGestionUmbral == 0) {
    if (tecla == 'A' || tecla == 'B' || tecla == 'C' || tecla == 'D') {
      iniciarEdicionUmbral(tecla);
    } else {
      mostrarMenuGestion();
    }
    return;
  }

  if (tecla >= '0' && tecla <= '9' && posBufferUmbral < 4) {
    bufferUmbral[posBufferUmbral++] = tecla;
    bufferUmbral[posBufferUmbral] = '\0';
    imprimirLineaLCD(1, bufferUmbral);
    return;
  }

  if (tecla == '#') {
    aplicarUmbralActual();
    return;
  }

  if (tecla == 'A' || tecla == 'B' || tecla == 'C' || tecla == 'D') {
    iniciarEdicionUmbral(tecla);
  }
}

/**
 * @brief Limpiar el buffer usado para capturar umbrales, horas y horarios.
 *
 * @details
 *   Borra todo el arreglo y reinicia el indice de escritura para una nueva
 *   captura numerica.
 */
void resetearBufferUmbral() {
  for (byte restantes = sizeof(bufferUmbral); restantes > 0; restantes--) {
    bufferUmbral[restantes - 1] = 0;
  }
  posBufferUmbral = 0;
}

/**
 * @brief Validar, aplicar y guardar el umbral seleccionado.
 *
 * @details
 *   Interpreta bufferUmbral segun opcionGestionUmbral y persiste el valor si
 *   cae dentro del rango permitido del sensor correspondiente.
 */
void aplicarUmbralActual() {
  if (posBufferUmbral == 0) {
    mostrarMensajeLCD("Sin valor", "Digite y #");
    return;
  }

  unsigned int valor = (unsigned int)atoi(bufferUmbral);
  bool valido = true;

  if (opcionGestionUmbral == 'A') {
    valido = (valor <= 99);
    if (valido) umbralTemp = (byte)valor;
  } else if (opcionGestionUmbral == 'B') {
    valido = (valor <= UMBRAL_ANALOGICO_MAX);
    if (valido) umbralLuz = valor;
  } else if (opcionGestionUmbral == 'C') {
    valido = (valor <= UMBRAL_ANALOGICO_MAX);
    if (valido) umbralHallAbierto = valor;
  } else if (opcionGestionUmbral == 'D') {
    valido = (valor <= UMBRAL_ANALOGICO_MAX);
    if (valido) umbralRuido = valor;
  } else {
    valido = false;
  }

  if (!valido) {
    if (opcionGestionUmbral == 'A') {
      mostrarMensajeLCD("Fuera de rango", "Temp 0-99");
    } else {
      mostrarMensajeLCD("Fuera de rango", "Rango 0-1023");
    }
    resetearBufferUmbral();
    return;
  }

  guardarUmbralesEEPROM();
  limpiarAlarmasPendientes();
  actualizarLecturasSensores(false);
  Serial.print(F("[UMBRALES] T=")); Serial.print(umbralTemp);
  Serial.print(F(" L=")); Serial.print(umbralLuz);
  Serial.print(F(" H=")); Serial.print(umbralHallAbierto);
  Serial.print(F(" M=")); Serial.println(umbralRuido);

  opcionGestionUmbral = 0;
  resetearBufferUmbral();
  mostrarMensajeLCD("Umbral guardado", "* sale A-D edit");
}

// 16. Boton fisico

/**
 * @brief Leer el boton fisico con antirrebote.
 *
 * @details
 *   El boton conectado a PIN_BOTON usa `INPUT_PULLUP`. En `ST_CONFIG` solicita
 *   volver a inicio; en estados de monitoreo solicita entrar a `ST_GESTION`.
 */
void procesarBoton() {
  bool lectura = digitalRead(PIN_BOTON);

  if (lectura != ultimoEstadoBoton && !botonDebouncePendiente) {
    botonDebouncePendiente = true;
    taskBotonDebounce.Start();
  }
}

/**
 * @brief Confirmar el cambio del boton despues del antirrebote.
 */
void confirmarBotonDebounce() {
  botonDebouncePendiente = false;
  bool lectura = digitalRead(PIN_BOTON);
  if (lectura == ultimoEstadoBoton) return;

  ultimoEstadoBoton = lectura;
  if (lectura == LOW) {                       // presionado
    byte estadoActual = (byte)miMaquina.GetState();
    if (estadoActual == ST_CONFIG) {
      Serial.println(F("[BOTON] salir de CONFIG"));
      eventoActual = EV_BOTON;
    } else if (estadoActual == ST_MONITOR_AMBIENTAL ||
               estadoActual == ST_MONITOR_PUERTAS) {
      Serial.println(F("[BOTON] monitoreo -> GESTION"));
      eventoActual = EV_A_GESTION;
    }
  }
}

//17. Procesar RFID
/*
 * @brief Procesar una tarjeta RFID si esta presente.
 *
 * @details
 *   En `ST_CONFIG` registra o elimina tarjetas segun subOpcionConfig. En
 *   `ST_INICIO` valida UID, perfil y horario antes de permitir apertura.
 */
void procesarTarjetaRFID() {
  if (!lectorRFID.PICC_IsNewCardPresent()) return;
  if (!lectorRFID.PICC_ReadCardSerial())   return;

  byte uid[RFID_UID_BUFFER_TAM];
  byte largo = lectorRFID.uid.size;
  for (byte restantes = largo; restantes > 0; restantes--) {
    byte i = largo - restantes;
    uid[i] = lectorRFID.uid.uidByte[i];
  }

  Serial.print(F("[RFID] UID:"));
  for (byte restantes = largo; restantes > 0; restantes--) {
    byte i = largo - restantes;
    Serial.print(' ');
    if (uid[i] < 0x10) Serial.print('0');
    Serial.print(uid[i], HEX);
  }
  Serial.println();

  byte est = (byte)miMaquina.GetState();
  byte perfilEnc = PERFIL_NINGUNO;

  if (est == ST_CONFIG) {
    if (subOpcionConfig == 'B') {
      if (registrarNuevaTarjeta(uid, largo, perfilParaRegistro))
        mostrarMensajeLCD("Tarjeta", "Registrada OK");
      else
        mostrarMensajeLCD("Error: llena", "o duplicada");
      subOpcionConfig = 0;
    }
    else if (subOpcionConfig == 'D') {
      if (eliminarTarjeta(uid, largo))
        mostrarMensajeLCD("Tarjeta", "Eliminada OK");
      else
        mostrarMensajeLCD("Tarjeta", "No encontrada");
      subOpcionConfig = 0;
    }
  }
  else if (est == ST_INICIO) {
    if (buscarUIDenEEPROM(uid, largo, &perfilEnc)) {
      perfilActual = perfilEnc;
      Serial.print(F("[RFID] valida, perfil ")); Serial.println(perfilEnc);
      if (accesoPermitidoPorHorario(perfilActual)) {
        accesoFueConClave = false;
        usosClaveActual = 0;
        intentosFallidos = 0;
        eventoActual = EV_CLAVE_OK;             // -> SERVO_ABIERTO
      } else {
        registrarEventoLog(0x05);
        eventoActual = EV_A_BLOQUEO;            // registrada, pero fuera de horario
      }
    } else {
      // El diagrama indica: tarjeta incorrecta -> BLOQUEO directo
      Serial.println(F("[RFID] no registrada -> BLOQUEO"));
      prepararMensajeBloqueo("Tarjeta", "No autorizada");
      eventoActual = EV_A_BLOQUEO;
    }
  }

  lectorRFID.PICC_HaltA();
  lectorRFID.PCD_StopCrypto1();
}

/* ============================================================================
 *                       18. GESTION DE EEPROM
 * ========================================================================= */
/**
 * @brief Inicializar EEPROM en el primer arranque.
 *
 * @details
 *   Limpia usuarios y log circular cuando no existe la marca VALOR_FLAG_OK.
 */
void inicializarEEPROM() {
  byte flag = EEPROM.read(DIR_FLAG_INIT);
  if (flag != VALOR_FLAG_OK) {
    Serial.println(F("EEPROM: inicializando..."));
    for (byte i = MAX_USUARIOS * TAM_USUARIO; i > 0; i--)
      EEPROM.update(DIR_USUARIOS + i - 1, 0);
    EEPROM.update(DIR_LOG_INDEX, 0);
    for (byte i = LOG_TAMANO; i > 0; i--) EEPROM.update(DIR_LOG_BASE + i - 1, 0);
    EEPROM.update(DIR_FLAG_INIT, VALOR_FLAG_OK);
    Serial.println(F("EEPROM lista. Configure claves en CONFIG > C."));
  }
}

/**
 * @brief Cargar umbrales desde EEPROM.
 *
 * @details
 *   Si la marca o los rangos no son validos, restaura umbrales por defecto y
 *   los persiste inmediatamente.
 */
void cargarUmbralesEEPROM() {
  if (EEPROM.read(DIR_FLAG_UMBRALES) == VALOR_UMBRALES_OK) {
    float umbralTempGuardado = UMBRAL_TEMP_DEFAULT;
    EEPROM.get(DIR_UMBRAL_TEMP, umbralTempGuardado);
    umbralTemp = (umbralTempGuardado >= 0.0 && umbralTempGuardado <= 99.0)
                   ? (byte)(umbralTempGuardado + 0.5)
                   : 100;
    EEPROM.get(DIR_UMBRAL_LUZ, umbralLuz);
    EEPROM.get(DIR_UMBRAL_HALL, umbralHallAbierto);
    EEPROM.get(DIR_UMBRAL_RUIDO, umbralRuido);
  }

  if (EEPROM.read(DIR_FLAG_UMBRALES) != VALOR_UMBRALES_OK || !umbralesValidos()) {
    umbralTemp = UMBRAL_TEMP_DEFAULT;
    umbralLuz = UMBRAL_LUZ_DEFAULT;
    umbralHallAbierto = UMBRAL_HALL_DEFAULT;
    umbralRuido = UMBRAL_RUIDO_DEFAULT;
    guardarUmbralesEEPROM();
  }

  Serial.print(F("[UMBRALES] Cargados T=")); Serial.print(umbralTemp);
  Serial.print(F(" L=")); Serial.print(umbralLuz);
  Serial.print(F(" H=")); Serial.print(umbralHallAbierto);
  Serial.print(F(" M=")); Serial.println(umbralRuido);
}

/**
 * @brief Guardar umbrales configurables en EEPROM.
 *
 * @details
 *   Persiste los limites ambientales y de intrusiones junto con una marca de
 *   validez para la siguiente inicializacion.
 */
void guardarUmbralesEEPROM() {
  float umbralTempGuardado = (float)umbralTemp;
  EEPROM.put(DIR_UMBRAL_TEMP, umbralTempGuardado);
  EEPROM.put(DIR_UMBRAL_LUZ, umbralLuz);
  EEPROM.put(DIR_UMBRAL_HALL, umbralHallAbierto);
  EEPROM.put(DIR_UMBRAL_RUIDO, umbralRuido);
  EEPROM.update(DIR_FLAG_UMBRALES, VALOR_UMBRALES_OK);
}

/**
 * @brief Validar los umbrales cargados.
 *
 * @retval true   Todos los umbrales estan dentro de rangos permitidos.
 * @retval false  Al menos un umbral esta fuera de rango.
 */
bool umbralesValidos() {
  if (umbralTemp > 99) return false;
  if (umbralLuz > UMBRAL_ANALOGICO_MAX) return false;
  if (umbralHallAbierto > UMBRAL_ANALOGICO_MAX) return false;
  if (umbralRuido > UMBRAL_ANALOGICO_MAX) return false;
  return true;
}

/**
 * @brief Cargar la ultima hora guardada como referencia.
 *
 * @details
 *   No habilita relojConfigurado porque `millis()` se reinicia al encender el
 *   Arduino. La hora debe confirmarse nuevamente desde `CONFIG > A > 1`.
 */
void cargarRelojEEPROM() {
  unsigned int minutoGuardado = 0;
  bool horaGuardadaValida = false;
  relojConfigurado = false;

  if (EEPROM.read(DIR_FLAG_RELOJ) == VALOR_RELOJ_OK) {
    EEPROM.get(DIR_RELOJ_MINUTOS, minutoGuardado);
    if (minutoGuardado < MINUTOS_DIA) {
      minutoBaseReloj = minutoGuardado;
      horaGuardadaValida = true;
    }
  }

  millisBaseReloj = millis();

  if (horaGuardadaValida) {
    char hora[HORA_BUFFER_TAM];
    formatearHora(minutoBaseReloj, hora, sizeof(hora));
    Serial.print(F("[RELOJ] Ultima hora guardada ")); Serial.println(hora);
    Serial.println(F("[RELOJ] Configure la hora tras arrancar: CONFIG > A > 1."));
  } else {
    minutoBaseReloj = 0;
    Serial.println(F("[RELOJ] No configurado. Use CONFIG > A > 1."));
  }
}

/**
 * @brief Guardar la hora base como referencia para la siguiente configuracion.
 *
 * @details
 *   Conserva el minuto configurado, aunque la hora debe confirmarse de nuevo
 *   tras reiniciar porque el reloj depende de `millis()`.
 */
void guardarRelojEEPROM() {
  EEPROM.put(DIR_RELOJ_MINUTOS, minutoBaseReloj);
  EEPROM.update(DIR_FLAG_RELOJ, VALOR_RELOJ_OK);
}

/**
 * @brief Cargar valores iniciales de acceso por rol.
 *
 * @details
 *   Seguridad y gerencia quedan en acceso completo; operario y coordinador se
 *   limitan a horarios laborales predefinidos.
 */
void cargarHorariosDefault() {
  horarioInicioPorPerfil[PERFIL_SEGURIDAD] = 0;       // 00:00
  horarioFinPorPerfil[PERFIL_SEGURIDAD]    = 1439;    // 23:59
  horarioInicioPorPerfil[PERFIL_OPERARIO]  = 480;     // 08:00
  horarioFinPorPerfil[PERFIL_OPERARIO]     = 1020;    // 17:00
  horarioInicioPorPerfil[PERFIL_COORDINADOR] = 420;   // 07:00
  horarioFinPorPerfil[PERFIL_COORDINADOR]    = 1140;  // 19:00
  horarioInicioPorPerfil[PERFIL_GERENTE]   = 0;       // 00:00
  horarioFinPorPerfil[PERFIL_GERENTE]      = 1439;    // 23:59
}

/**
 * @brief Cargar horarios por perfil desde EEPROM.
 *
 * @details
 *   Lee inicio/fin de cada perfil. Si la tabla no tiene marca valida o algun
 *   minuto esta fuera de rango, restaura los horarios por defecto.
 */
void cargarHorariosEEPROM() {
  bool ok = (EEPROM.read(DIR_FLAG_HORARIOS) == VALOR_HORARIOS_OK);

  if (ok) {
    for (byte perfil = PERFIL_GERENTE + 1; perfil > PERFIL_SEGURIDAD; perfil--) {
      byte perfilActual = perfil - 1;
      unsigned int base = DIR_HORARIOS_BASE + (perfilActual - 1) * TAM_HORARIO_ROL;
      EEPROM.get(base, horarioInicioPorPerfil[perfilActual]);
      EEPROM.get(base + 2, horarioFinPorPerfil[perfilActual]);
    }
    ok = horariosValidos();
  }

  if (!ok) {
    cargarHorariosDefault();
    guardarHorariosEEPROM();
  }

  Serial.println(F("[HORARIOS] Cargados por perfil"));
  for (byte perfil = PERFIL_GERENTE + 1; perfil > PERFIL_SEGURIDAD; perfil--) {
    byte perfilActual = perfil - 1;
    char ini[HORA_BUFFER_TAM], fin[HORA_BUFFER_TAM];
    formatearHora(horarioInicioPorPerfil[perfilActual], ini, sizeof(ini));
    formatearHora(horarioFinPorPerfil[perfilActual], fin, sizeof(fin));
    Serial.print(F("  ")); Serial.print(nombrePerfilLCD(perfilActual));
    Serial.print(F(": ")); Serial.print(ini);
    Serial.print(F("-")); Serial.println(fin);
  }
}

/**
 * @brief Guardar horarios por perfil en EEPROM.
 *
 * @details
 *   Escribe el inicio y fin permitido de cada perfil configurable y marca la
 *   tabla como valida.
 */
void guardarHorariosEEPROM() {
  for (byte perfil = PERFIL_GERENTE + 1; perfil > PERFIL_SEGURIDAD; perfil--) {
    byte perfilActual = perfil - 1;
    unsigned int base = DIR_HORARIOS_BASE + (perfilActual - 1) * TAM_HORARIO_ROL;
    EEPROM.put(base, horarioInicioPorPerfil[perfilActual]);
    EEPROM.put(base + 2, horarioFinPorPerfil[perfilActual]);
  }
  EEPROM.update(DIR_FLAG_HORARIOS, VALOR_HORARIOS_OK);
}

/**
 * @brief Validar rangos de horario cargados.
 *
 * @retval true   Todos los horarios estan entre `0` y `MINUTOS_DIA - 1`.
 * @retval false  Al menos un horario esta fuera del dia.
 */
bool horariosValidos() {
  for (byte perfil = PERFIL_GERENTE + 1; perfil > PERFIL_SEGURIDAD; perfil--) {
    byte perfilActual = perfil - 1;
    if (horarioInicioPorPerfil[perfilActual] >= MINUTOS_DIA) return false;
    if (horarioFinPorPerfil[perfilActual] >= MINUTOS_DIA) return false;
  }
  return true;
}

/**
 * @brief Leer flags de credenciales de un usuario activo.
 *
 * @details
 *   Registros creados por versiones anteriores no tienen flags; se interpretan
 *   como tarjetas RFID para no perder usuarios ya registrados.
 *
 * @param [in] base  Direccion inicial del registro de usuario.
 *
 * @return Mascara `USUARIO_FLAG_*` reconocida.
 */
byte leerFlagsUsuario(unsigned int base) {
  byte flags = EEPROM.read(base + USUARIO_OFF_FLAGS);
  if ((flags & USUARIO_FLAGS_VALIDOS) == flags && flags != 0) return flags;
  return USUARIO_FLAG_RFID;
}

/**
 * @brief Desactivar claves antiguas de un perfil.
 *
 * @details
 *   Mantiene `baseConservada` como clave vigente y elimina la bandera de clave
 *   en los demas registros del mismo perfil. Si un registro no conserva RFID,
 *   se marca como inactivo.
 *
 * @param [in] perfil          Perfil cuyas claves se depuran.
 * @param [in] baseConservada  Registro que conserva la clave vigente.
 */
void desactivarClavesPerfil(byte perfil, unsigned int baseConservada) {
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (base == baseConservada) continue;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) continue;
    if (EEPROM.read(base + USUARIO_OFF_PERFIL) != perfil) continue;

    byte flags = leerFlagsUsuario(base);
    if (!(flags & USUARIO_FLAG_CLAVE)) continue;

    flags = (byte)(flags & ~USUARIO_FLAG_CLAVE);
    if (flags & USUARIO_FLAG_RFID) {
      EEPROM.update(base + USUARIO_OFF_FLAGS, flags);
      reiniciarUsosClaveUsuario(base);
    } else {
      EEPROM.update(base + USUARIO_OFF_ESTADO, 0);
    }
  }
}

/**
 * @brief Leer usos acumulados de una clave de usuario.
 *
 * @param [in] base  Direccion inicial del registro de usuario.
 *
 * @return Usos de clave acumulados, limitados a `USOS_CLAVE_MAX`.
 */
byte leerUsosClaveUsuario(unsigned int base) {
  byte usos = EEPROM.read(base + USUARIO_OFF_USOS_CLAVE);
  return (usos > USOS_CLAVE_MAX) ? USOS_CLAVE_MAX : usos;
}

/**
 * @brief Determinar si una clave llego al limite de usos.
 *
 * @param [in] base  Direccion inicial del registro de usuario.
 *
 * @retval true   La clave ya no debe abrir la cerradura.
 * @retval false  La clave aun tiene usos disponibles.
 */
bool claveUsuarioVencida(unsigned int base) {
  return leerUsosClaveUsuario(base) >= USOS_CLAVE_MAX;
}

/**
 * @brief Registrar un uso exitoso de una clave.
 *
 * @param [in] base  Direccion inicial del registro de usuario.
 */
void registrarUsoClaveUsuario(unsigned int base) {
  byte usos = leerUsosClaveUsuario(base);
  if (usos < USOS_CLAVE_MAX) {
    EEPROM.update(base + USUARIO_OFF_USOS_CLAVE, usos + 1);
  }
}

/**
 * @brief Reiniciar el contador de usos de una clave.
 *
 * @param [in] base  Direccion inicial del registro de usuario.
 */
void reiniciarUsosClaveUsuario(unsigned int base) {
  EEPROM.update(base + USUARIO_OFF_USOS_CLAVE, 0);
}

/**
 * @brief Comparar una clave digitada con la clave de un registro.
 *
 * @param [in] base     Direccion inicial del registro de usuario.
 * @param [in] intento  Clave de `LONGITUD_PASSWORD` digitos.
 *
 * @retval true   La clave coincide byte a byte.
 * @retval false  Al menos un digito es diferente.
 */
bool claveUsuarioCoincide(unsigned int base, const char *intento) {
  for (byte restantes = LONGITUD_PASSWORD; restantes > 0; restantes--) {
    byte i = LONGITUD_PASSWORD - restantes;
    if (EEPROM.read(base + USUARIO_OFF_CLAVE + i) != intento[i]) return false;
  }
  return true;
}

/**
 * @brief Buscar un UID activo en EEPROM.
 *
 * @param [in]  uid                Bytes del UID leido desde el RC522.
 * @param [in]  longitud           Cantidad de bytes disponibles en `uid`.
 * @param [out] perfilEncontrado   Perfil asociado cuando el UID existe.
 *
 * @retval true   El UID esta registrado y activo.
 * @retval false  El UID no aparece en la tabla de usuarios activos.
 */
bool buscarUIDenEEPROM(byte *uid, byte longitud, byte *perfilEncontrado) {
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) continue;
    if (!(leerFlagsUsuario(base) & USUARIO_FLAG_RFID)) continue;
    bool igual = true;
    byte bytesUID = (longitud < USUARIO_UID_TAM) ? longitud : USUARIO_UID_TAM;
    for (byte restantesUID = bytesUID; restantesUID > 0; restantesUID--) {
      byte k = bytesUID - restantesUID;
      if (EEPROM.read(base + USUARIO_OFF_UID + k) != uid[k]) { igual = false; break; }
    }
    if (igual) { *perfilEncontrado = EEPROM.read(base + USUARIO_OFF_PERFIL); return true; }
  }
  return false;
}

/**
 * @brief Buscar una clave activa en EEPROM.
 *
 * @param [in]  intento           Cadena de `LONGITUD_PASSWORD` digitos.
 * @param [out] perfilEncontrado  Perfil asociado a la clave encontrada.
 * @param [out] baseEncontrada    Direccion EEPROM del registro encontrado.
 *
 * @retval true   La clave pertenece a un usuario activo.
 * @retval false  No existe un usuario activo con esa clave.
 */
bool buscarClaveUsuarioEEPROM(const char *intento, byte *perfilEncontrado, unsigned int *baseEncontrada) {
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) continue;
    if (!(leerFlagsUsuario(base) & USUARIO_FLAG_CLAVE)) continue;
    if (claveUsuarioCoincide(base, intento)) {
      *perfilEncontrado = EEPROM.read(base + USUARIO_OFF_PERFIL);
      *baseEncontrada = base;
      return true;
    }
  }
  return false;
}

/**
 * @brief Buscar la clave activa de un perfil.
 *
 * @param [in]  perfil          Perfil asociado a la clave.
 * @param [out] baseEncontrada  Direccion EEPROM del registro encontrado.
 *
 * @retval true   El perfil tiene una clave numerica registrada.
 * @retval false  No hay clave numerica activa para el perfil.
 */
bool buscarClavePerfilEEPROM(byte perfil, unsigned int *baseEncontrada) {
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) continue;
    if (EEPROM.read(base + USUARIO_OFF_PERFIL) != perfil) continue;
    if (!(leerFlagsUsuario(base) & USUARIO_FLAG_CLAVE)) continue;
    *baseEncontrada = base;
    return true;
  }
  return false;
}

/**
 * @brief Registrar una tarjeta en el primer slot libre.
 *
 * @param [in] uid       Bytes del UID leido desde el RC522.
 * @param [in] longitud  Cantidad de bytes disponibles en `uid`.
 * @param [in] perfil    Perfil que se asignara a la tarjeta.
 *
 * @retval true   La tarjeta se guardo correctamente.
 * @retval false  La tarjeta ya existia o no hay slots disponibles.
 */
bool registrarNuevaTarjeta(byte *uid, byte longitud, byte perfil) {
  byte tmp;
  if (buscarUIDenEEPROM(uid, longitud, &tmp)) return false;
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) {
      EEPROM.update(base + USUARIO_OFF_ESTADO, USUARIO_ACTIVO);
      EEPROM.update(base + USUARIO_OFF_PERFIL, perfil);
      for (byte restantesUID = USUARIO_UID_TAM; restantesUID > 0; restantesUID--) {
        byte k = USUARIO_UID_TAM - restantesUID;
        byte valor = (k < longitud) ? uid[k] : 0;
        EEPROM.update(base + USUARIO_OFF_UID + k, valor);
      }
      for (byte restantesClave = LONGITUD_PASSWORD; restantesClave > 0; restantesClave--) {
        byte k = LONGITUD_PASSWORD - restantesClave;
        EEPROM.update(base + USUARIO_OFF_CLAVE + k, 0);
      }
      reiniciarUsosClaveUsuario(base);
      EEPROM.update(base + USUARIO_OFF_FLAGS, USUARIO_FLAG_RFID);
      return true;
    }
  }
  return false;
}

/**
 * @brief Registrar una clave numerica como usuario de acceso.
 *
 * @param [in] nuevoPass  Clave de `LONGITUD_PASSWORD` digitos.
 * @param [in] perfil     Perfil cuyo horario se aplicara a esta clave.
 *
 * @retval true   La clave fue guardada en un slot libre.
 * @retval false  El perfil es invalido, la clave ya existe o no hay espacio.
 */
bool registrarClaveUsuario(const char *nuevoPass, byte perfil) {
  if (perfil < PERFIL_SEGURIDAD || perfil > PERFIL_GERENTE) return false;

  byte perfilExistente = PERFIL_NINGUNO;
  unsigned int baseExistente = 0;
  if (buscarClaveUsuarioEEPROM(nuevoPass, &perfilExistente, &baseExistente)) return false;

  unsigned int basePerfil = 0;
  if (buscarClavePerfilEEPROM(perfil, &basePerfil)) {
    for (byte restantesClave = LONGITUD_PASSWORD; restantesClave > 0; restantesClave--) {
      byte k = LONGITUD_PASSWORD - restantesClave;
      EEPROM.update(basePerfil + USUARIO_OFF_CLAVE + k, nuevoPass[k]);
    }
    byte flags = (byte)(leerFlagsUsuario(basePerfil) | USUARIO_FLAG_CLAVE);
    EEPROM.update(basePerfil + USUARIO_OFF_FLAGS, flags);
    reiniciarUsosClaveUsuario(basePerfil);
    desactivarClavesPerfil(perfil, basePerfil);
    return true;
  }

  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) {
      EEPROM.update(base + USUARIO_OFF_ESTADO, USUARIO_ACTIVO);
      EEPROM.update(base + USUARIO_OFF_PERFIL, perfil);
      for (byte restantesUID = USUARIO_UID_TAM; restantesUID > 0; restantesUID--) {
        byte k = USUARIO_UID_TAM - restantesUID;
        EEPROM.update(base + USUARIO_OFF_UID + k, 0);
      }
      for (byte restantesClave = LONGITUD_PASSWORD; restantesClave > 0; restantesClave--) {
        byte k = LONGITUD_PASSWORD - restantesClave;
        EEPROM.update(base + USUARIO_OFF_CLAVE + k, nuevoPass[k]);
      }
      EEPROM.update(base + USUARIO_OFF_FLAGS, USUARIO_FLAG_CLAVE);
      reiniciarUsosClaveUsuario(base);
      desactivarClavesPerfil(perfil, base);
      return true;
    }
  }
  return false;
}

/**
 * @brief Registrar un intento fallido de clave.
 *
 * @details
 *   Los primeros intentos incorrectos producen bloqueo temporal. Al alcanzar
 *   `MAX_INTENTOS`, el contador se reinicia y se dispara `ST_ALARMA`.
 */
void registrarIntentoClaveFallido() {
  intentosFallidos++;
  Serial.print(F("[CLAVE] incorrecta "));
  Serial.print(intentosFallidos);
  Serial.print(F(" de "));
  Serial.println(MAX_INTENTOS);
  resetearBufferPassword();

  if (intentosFallidos >= MAX_INTENTOS) {
    alarmaVieneDeAcceso = true;
    alarmaVieneDePuertas = false;
    intentosFallidos = 0;
    eventoActual = EV_INTENTOS_ALARMA;
    return;
  }

  char l2[LCD_BUFFER_TAM];
  snprintf(l2, sizeof(l2), "Intento %d/%d", intentosFallidos, MAX_INTENTOS);
  prepararMensajeBloqueo("Clave incorrecta", l2);
  eventoActual = EV_A_BLOQUEO;
}

/**
 * @brief Mostrar el contador de usos de la clave en el LCD.
 *
 * @param [in] usos  Cantidad de usos acumulados despues del acceso exitoso.
 */
void mostrarUsosClaveLCD(byte usos) {
  char l2[LCD_BUFFER_TAM];
  byte restantes = (usos >= USOS_CLAVE_MAX) ? 0 : (USOS_CLAVE_MAX - usos);
  snprintf(l2, sizeof(l2), "Quedan %d/%d", restantes, USOS_CLAVE_MAX);
  mostrarMensajeLCD("Usos de clave", l2);
}

/**
 * @brief Marcar una tarjeta como inactiva.
 *
 * @param [in] uid       Bytes del UID leido desde el RC522.
 * @param [in] longitud  Cantidad de bytes disponibles en `uid`.
 *
 * @retval true   La tarjeta existia y fue desactivada.
 * @retval false  No se encontro una tarjeta activa con ese UID.
 */

bool eliminarTarjeta(byte *uid, byte longitud) {
  for (byte restantes = MAX_USUARIOS; restantes > 0; restantes--) {
    byte i = MAX_USUARIOS - restantes;
    unsigned int base = DIR_USUARIOS + i * TAM_USUARIO;
    if (EEPROM.read(base + USUARIO_OFF_ESTADO) != USUARIO_ACTIVO) continue;
    byte flags = leerFlagsUsuario(base);
    if (!(flags & USUARIO_FLAG_RFID)) continue;
    bool igual = true;
    byte bytesUID = (longitud < USUARIO_UID_TAM) ? longitud : USUARIO_UID_TAM;
    for (byte restantesUID = bytesUID; restantesUID > 0; restantesUID--) {
      byte k = bytesUID - restantesUID;
      if (EEPROM.read(base + USUARIO_OFF_UID + k) != uid[k]) { igual = false; break; }
    }
    if (igual) {
      flags = (byte)(flags & ~USUARIO_FLAG_RFID);
      if (flags & USUARIO_FLAG_CLAVE) {
        for (byte restantesUID = USUARIO_UID_TAM; restantesUID > 0; restantesUID--) {
          byte k = USUARIO_UID_TAM - restantesUID;
          EEPROM.update(base + USUARIO_OFF_UID + k, 0);
        }
        EEPROM.update(base + USUARIO_OFF_FLAGS, flags);
      } else {
        EEPROM.update(base + USUARIO_OFF_ESTADO, 0);
      }
      return true;
    }
  }
  return false;
}

/**
 * @brief Guardar un evento en el log circular de EEPROM.
 *
 * @param [in] codigoEvento  Codigo compacto del evento a almacenar.
 */
void registrarEventoLog(byte codigoEvento) {
  byte idx = EEPROM.read(DIR_LOG_INDEX);
  EEPROM.update(DIR_LOG_BASE + idx, codigoEvento);
  idx = (idx + 1) % LOG_TAMANO;
  EEPROM.update(DIR_LOG_INDEX, idx);
}


// 19. Funciones

/* 
 * @brief Promediar varias muestras analogicas.
 *
 * @param [in] pin  Pin analogico a leer.
 *
 * @return Valor promedio en el rango de `0` a `UMBRAL_ANALOGICO_MAX`.
 */
unsigned int leerAnalogicoPromediado(byte pin) {
  unsigned int suma = 0;
  for (byte restantes = MUESTRAS_ANALOGICAS; restantes > 0; restantes--) {
    suma += analogRead(pin);
  }
  return suma / MUESTRAS_ANALOGICAS;
}

/**
 * @brief Leer el pico analogico mas alto.
 *
 * @param [in] pin  Pin analogico a muestrear.
 *
 * @return Mayor lectura encontrada en `MUESTRAS_ANALOGICAS` muestras.
 */
unsigned int leerAnalogicoPico(byte pin) {
  unsigned int pico = 0;
  for (byte restantes = MUESTRAS_ANALOGICAS; restantes > 0; restantes--) {
    unsigned int lectura = analogRead(pin);
    if (lectura > pico) pico = lectura;
  }
  return pico;
}

/**
 * @brief Leer el valle analogico mas bajo.
 *
 * @param [in] pin  Pin analogico a muestrear.
 *
 * @return Menor lectura encontrada en `MUESTRAS_ANALOGICAS` muestras.
 */
unsigned int leerAnalogicoValle(byte pin) {
  unsigned int valle = UMBRAL_ANALOGICO_MAX;
  for (byte restantes = MUESTRAS_ANALOGICAS; restantes > 0; restantes--) {
    unsigned int lectura = analogRead(pin);
    if (lectura < valle) valle = lectura;
  }
  return valle;
}

/**
 * @brief Suavizar una lectura analogica.
 *
 * @param [in] anterior  Valor filtrado anterior.
 * @param [in] nuevo     Lectura nueva sin filtrar.
 *
 * @return Valor filtrado con peso mayor para la lectura anterior.
 */
unsigned int suavizarAnalogico(unsigned int anterior, unsigned int nuevo) {
  return (anterior * 3 + nuevo) / 4;
}

/**
 * @brief Corregir sensores analogicos cableados con respuesta invertida.
 *
 * @param [in] lectura  Valor ADC original.
 *
 * @return Valor normalizado en el rango `0` a `UMBRAL_ANALOGICO_MAX`.
 */
unsigned int invertirLecturaAnalogica(unsigned int lectura) {
  if (lectura > UMBRAL_ANALOGICO_MAX) lectura = UMBRAL_ANALOGICO_MAX;
  return UMBRAL_ANALOGICO_MAX - lectura;
}

/**
 * @brief Leer el sensor Hall KY-035 con escala corregida.
 *
 * @return Valor donde un iman cercano baja la lectura y sin iman sube.
 */
unsigned int leerHallKY035() {
  return leerAnalogicoPromediado(PIN_KY035);
}

/**
 * @brief Leer el sensor de sonido KY-037 con escala corregida.
 *
 * @return Valor donde el silencio queda bajo y el sonido fuerte alto.
 */
unsigned int leerSonidoKY037() {
  return invertirLecturaAnalogica(leerAnalogicoValle(PIN_KY037));
}

/**
 * @brief Leer luz ambiente del KY-018 con escala corregida.
 *
 * @return Valor donde 0 es oscuro y 1023 es maxima luz.
 */
unsigned int leerLuzKY018() {
  return invertirLecturaAnalogica(leerAnalogicoPromediado(PIN_KY018));
}

/**
 * @brief Convertir resistencia NTC del KY-013 a grados Celsius.
 *
 * @param [in] resistencia  Resistencia estimada del termistor.
 *
 * @return Temperatura calculada con la ecuacion Beta.
 */
float calcularTemperaturaNTC(float resistencia) {
  if (resistencia <= 0.0) return 999.0;

  float tempK = 1.0 / ((log(resistencia / KY013_NTC_NOMINAL) / KY013_NTC_BETA)
                       + (1.0 / KY013_TEMP_NOMINAL_K));
  return tempK - 273.15;
}

/**
 * @brief Convertir una lectura real del KY-013 a grados Celsius.
 *
 * @details
 *   El KY-013 es un NTC. Algunos montajes entregan el divisor en sentido
 *   inverso o con una resistencia serie distinta; se prueban ambas curvas y se
 *   conserva la temperatura fisicamente valida mas coherente con la lectura
 *   filtrada previa.
 *
 * @param [in] lectura  Valor ADC del sensor.
 *
 * @return Temperatura estimada en grados Celsius.
 */
float convertirTemperaturaC(unsigned int lectura) {
  if (lectura == 0) lectura = 1;
  if (lectura >= UMBRAL_ANALOGICO_MAX) lectura = UMBRAL_ANALOGICO_MAX - 1;

  float divisor = (float)UMBRAL_ANALOGICO_MAX / (float)lectura - 1.0;
  float candidatos[KY013_CANDIDATOS] = {
    calcularTemperaturaNTC(KY013_SERIE_ESTANDAR * divisor),
    calcularTemperaturaNTC(KY013_SERIE_ESTANDAR / divisor),
    calcularTemperaturaNTC(KY013_SERIE_MONTAJE * divisor),
    calcularTemperaturaNTC(KY013_SERIE_MONTAJE / divisor)
  };

  float mejor = temperaturaActual;
  float mejorDistancia = 9999.0;
  bool hayCandidato = false;
  for (byte restantes = KY013_CANDIDATOS; restantes > 0; restantes--) {
    byte i = KY013_CANDIDATOS - restantes;
    if (candidatos[i] <= TEMP_SENSOR_MIN_C || candidatos[i] >= TEMP_SENSOR_MAX_C) {
      continue;
    }
    float distancia = candidatos[i] - temperaturaActual;
    if (distancia < 0.0) distancia = -distancia;
    if (!hayCandidato || distancia < mejorDistancia) {
      mejor = candidatos[i];
      mejorDistancia = distancia;
      hayCandidato = true;
    }
  }

  return mejor;
}

/**
 * @brief Formatear temperatura con una decimal.
 *
 * @param [out] destino      Buffer que recibira el texto.
 * @param [in]  tamano       Capacidad de `destino`, en bytes.
 * @param [in]  temperatura  Temperatura en grados Celsius.
 */
void formatearTemperatura(char *destino, byte tamano, float temperatura) {
  int decimas = (temperatura >= 0.0)
                  ? (int)(temperatura * 10.0 + 0.5)
                  : (int)(temperatura * 10.0 - 0.5);
  int entero = decimas / 10;
  byte decimal = abs(decimas % 10);
  snprintf(destino, tamano, "%d.%dC", entero, decimal);
}

/**
 * @brief Actualizar todas las lecturas reales sin disparar eventos.
 *
 * @param [in] filtrar  Si es `true`, aplica suavizado a lecturas lentas.
 */
void actualizarLecturasSensores(bool filtrar) {
  lecturaTempRaw = leerAnalogicoPromediado(PIN_KY013);
  float tempC = convertirTemperaturaC(lecturaTempRaw);
  if (tempC > TEMP_SENSOR_MIN_C && tempC < TEMP_SENSOR_MAX_C) {
    temperaturaActual = filtrar ? (0.7 * temperaturaActual + 0.3 * tempC) : tempC;
  }

  unsigned int hall = leerHallKY035();
  unsigned int luz = leerLuzKY018();
  unsigned int sonido = leerSonidoKY037();

  if (filtrar) {
    lecturaHall = suavizarAnalogico(lecturaHall, hall);
    lecturaLuz = suavizarAnalogico(lecturaLuz, luz);
  } else {
    lecturaHall = hall;
    lecturaLuz = luz;
  }
  lecturaSonido = sonido;
}

/**
 * @brief Borrar flags para evitar heredar alarmas viejas.
 *
 * @details
 *   Restablece eventos ambientales, eventos de puerta y origen de alarma antes
 *   de cambiar de fase de monitoreo.
 */
void limpiarAlarmasPendientes() {
  eventoAmbientalAlarma = false;
  eventoPuertaAlarma = false;
  alarmaVieneDePuertas = false;
  alarmaVieneDeAcceso = false;
}

/**
 * @brief Evaluar la alarma ambiental con umbrales configurables.
 *
 * @retval true   Temperatura y luz superan sus umbrales.
 * @retval false  Al menos una condicion esta bajo su umbral.
 */
bool condicionAmbientalAlarma() {
  // Usa los umbrales configurados; 24C y 400 solo son valores iniciales.
  bool tempAlta = temperaturaActual > umbralTemp;
  bool luzAlta = lecturaLuz > umbralLuz;
  return (tempAlta && luzAlta);
}

/**
 * @brief Evaluar la alarma de puerta con condicion AND.
 *
 * @details
 *   El Hall queda activo cuando su lectura supera el umbral configurado. Como
 *   el sensor sube cuando no tiene iman cerca, esta condicion representa puerta
 *   abierta o iman ausente. La alarma solo se dispara si tambien hay ruido por
 *   encima de su umbral.
 *
 * @retval true   Hall y sonido superan sus umbrales configurados.
 * @retval false  Al menos una de las dos condiciones no se cumple.
 */
bool condicionPuertasAlarma() {
  bool hallSuperaUmbral = lecturaHall > umbralHallAbierto;
  bool sonidoSuperaUmbral = lecturaSonido > umbralRuido;
  return (hallSuperaUmbral && sonidoSuperaUmbral);
}

/**
 * @brief Obtener el minuto actual del dia segun el reloj interno.
 *
 * @return Minuto desde medianoche, en el rango `0` a `1439`.
 */
unsigned int minutoActualDelDia() {
  unsigned int minutosTranscurridos =
    (unsigned int)(((millis() - millisBaseReloj) / 60000UL) % MINUTOS_DIA);
  return (minutoBaseReloj + minutosTranscurridos) % MINUTOS_DIA;
}

/**
 * @brief Convertir texto `HHMM` a minutos desde medianoche.
 *
 * @param [in] texto  Cadena de 4 digitos con hora en formato de 24 horas.
 *
 * @return Minutos desde medianoche, o `-1` si el texto no es valido.
 */
int convertirHHMMaMinutos(const char *texto) {
  for (byte restantes = HORA_DIGITOS; restantes > 0; restantes--) {
    byte i = HORA_DIGITOS - restantes;
    if (texto[i] < '0' || texto[i] > '9') return -1;
  }

  byte hora = (texto[0] - '0') * 10 + (texto[1] - '0');
  byte minuto = (texto[2] - '0') * 10 + (texto[3] - '0');
  if (hora > 23 || minuto > 59) return -1;
  return hora * 60 + minuto;
}

/**
 * @brief Configurar la hora actual desde texto `HHMM`.
 *
 * @param [in] hhmm  Cadena de 4 digitos en formato de 24 horas.
 *
 * @retval true   La hora fue aceptada y guardada como referencia.
 * @retval false  El texto no representa una hora valida.
 */
bool configurarHoraActual(const char *hhmm) {
  int minuto = convertirHHMMaMinutos(hhmm);
  if (minuto < 0) return false;

  minutoBaseReloj = (unsigned int)minuto;
  millisBaseReloj = millis();
  relojConfigurado = true;
  guardarRelojEEPROM();

  char hora[HORA_BUFFER_TAM];
  formatearHora(minutoBaseReloj, hora, sizeof(hora));
  Serial.print(F("[RELOJ] Hora configurada: "));
  Serial.println(hora);
  return true;
}

/**
 * @brief Configurar el horario de un perfil desde `HHMMHHMM`.
 *
 * @param [in] perfil         Perfil entre `PERFIL_SEGURIDAD` y
 *                            `PERFIL_GERENTE`.
 * @param [in] hhmmInicioFin  Cadena con inicio `HHMM` seguido de fin `HHMM`.
 *
 * @retval true   El horario fue aceptado y guardado.
 * @retval false  El perfil o el formato de hora no son validos.
 */
bool configurarHorarioPerfil(byte perfil, const char *hhmmInicioFin) {
  if (perfil < PERFIL_SEGURIDAD || perfil > PERFIL_GERENTE) return false;
  int inicio = convertirHHMMaMinutos(hhmmInicioFin);
  int fin = convertirHHMMaMinutos(hhmmInicioFin + 4);
  if (inicio < 0 || fin < 0) return false;

  horarioInicioPorPerfil[perfil] = (unsigned int)inicio;
  horarioFinPorPerfil[perfil] = (unsigned int)fin;
  guardarHorariosEEPROM();

  char ini[HORA_BUFFER_TAM], finTxt[HORA_BUFFER_TAM];
  formatearHora(horarioInicioPorPerfil[perfil], ini, sizeof(ini));
  formatearHora(horarioFinPorPerfil[perfil], finTxt, sizeof(finTxt));
  Serial.print(F("[HORARIO] ")); Serial.print(nombrePerfilLCD(perfil));
  Serial.print(F(" = ")); Serial.print(ini);
  Serial.print(F("-")); Serial.println(finTxt);
  return true;
}

/**
 * @brief Comprobar si un minuto cae dentro de una ventana horaria.
 *
 * @param [in] minuto  Minuto a evaluar, contado desde medianoche.
 * @param [in] inicio  Minuto inicial permitido, incluido.
 * @param [in] fin     Minuto final permitido, incluido.
 *
 * @retval true   El minuto esta dentro del horario, incluso si cruza medianoche.
 * @retval false  El minuto queda fuera del horario.
 */
bool horarioPermiteMinuto(unsigned int minuto, unsigned int inicio, unsigned int fin) {
  if (inicio == fin) return true;
  if (inicio < fin) return (minuto >= inicio && minuto <= fin);
  return (minuto >= inicio || minuto <= fin);
}

/**
 * @brief Validar si un perfil puede abrir en este momento.
 *
 * @param [in] perfil  Perfil de usuario autenticado.
 *
 * @retval true   El reloj esta configurado y el perfil esta dentro de horario.
 * @retval false  El perfil es invalido, falta hora o esta fuera de horario.
 */
bool accesoPermitidoPorHorario(byte perfil) {
  if (perfil < PERFIL_SEGURIDAD || perfil > PERFIL_GERENTE) {
    prepararMensajeBloqueo("Perfil invalido", "Espere 7 seg");
    return false;
  }

  if (!relojConfigurado) {
    Serial.println(F("[HORARIO] Acceso negado: reloj no configurado"));
    prepararMensajeBloqueo("Config hora", "CONFIG A1");
    return false;
  }

  unsigned int ahora = minutoActualDelDia();
  bool permitido = horarioPermiteMinuto(ahora,
                                        horarioInicioPorPerfil[perfil],
                                        horarioFinPorPerfil[perfil]);

  char hora[HORA_BUFFER_TAM], ini[HORA_BUFFER_TAM], fin[HORA_BUFFER_TAM];
  formatearHora(ahora, hora, sizeof(hora));
  formatearHora(horarioInicioPorPerfil[perfil], ini, sizeof(ini));
  formatearHora(horarioFinPorPerfil[perfil], fin, sizeof(fin));

  Serial.print(F("[HORARIO] ")); Serial.print(nombrePerfilLCD(perfil));
  Serial.print(F(" ahora ")); Serial.print(hora);
  Serial.print(F(" permitido ")); Serial.print(ini);
  Serial.print(F("-")); Serial.print(fin);
  Serial.print(F(" -> "));
  Serial.println(permitido ? F("OK") : F("DENEGADO"));

  if (!permitido) {
    char l2[LCD_BUFFER_TAM];
    snprintf(l2, sizeof(l2), "%s no acceso", hora);
    prepararMensajeBloqueo("Fuera horario", l2);
  }

  return permitido;
}

/**
 * @brief Formatear minutos desde medianoche como `HH:MM`.
 *
 * @param [in]  minuto   Minuto desde medianoche. Se normaliza con modulo dia.
 * @param [out] destino  Buffer que recibira el texto.
 * @param [in]  tamano   Capacidad de `destino`, en bytes.
 */
void formatearHora(unsigned int minuto, char *destino, byte tamano) {
  minuto %= MINUTOS_DIA;
  snprintf(destino, tamano, "%02u:%02u", minuto / 60, minuto % 60);
}

/**
 * @brief Obtener el nombre corto de un perfil.
 *
 * @param [in] perfil  Valor de Perfil.
 *
 * @return Puntero a una cadena constante apta para LCD/Serial.
 */
const char *nombrePerfilLCD(byte perfil) {
  switch (perfil) {
    case PERFIL_SEGURIDAD:   return "Seguridad";
    case PERFIL_OPERARIO:    return "Operario";
    case PERFIL_COORDINADOR: return "Coordin.";
    case PERFIL_GERENTE:     return "Gerente";
    default:                 return "Ninguno";
  }
}

/**
 * @brief Refrescar el LCD durante estados de monitoreo.
 *
 * @details
 *   Limita la actualizacion a intervalos de 750 ms para evitar parpadeo y
 *   muestra el par de sensores correspondiente al estado activo.
 */
void actualizarLCDMonitoreo() {
  byte est = (byte)miMaquina.GetState();
  if (est != ST_MONITOR_AMBIENTAL && est != ST_MONITOR_PUERTAS) return;

  char l2[LCD_BUFFER_TAM];
  if (est == ST_MONITOR_AMBIENTAL) {
    char tempTxt[TEMP_BUFFER_TAM];
    formatearTemperatura(tempTxt, sizeof(tempTxt), temperaturaActual);
    snprintf(l2, sizeof(l2), "T:%s L:%u", tempTxt, lecturaLuz);
    imprimirLineaLCD(0, "MONITOR AMBIENTE");
    imprimirLineaLCD(1, l2);
  } else {
    snprintf(l2, sizeof(l2), "H:%u M:%u", lecturaHall, lecturaSonido);
    imprimirLineaLCD(0, "MONITOR PUERTA");
    imprimirLineaLCD(1, l2);
  }
}

/**
 * @brief Apagar los tres LEDs indicadores.
 *
 * @details
 *   Deja en bajo las salidas roja, verde y azul para reiniciar cualquier
 *   indicacion visual activa.
 */
void apagarLEDs() {
  digitalWrite(PIN_LED_ROJO,  LOW);
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_AZUL,  LOW);
}

/**
 * @brief Activar un tono no bloqueante.
 *
 * @param [in] duracion    Duracion del tono, en milisegundos.
 * @param [in] frecuencia  Frecuencia del tono, en hertz.
 */
void activarBuzzer(unsigned int duracion, unsigned int frecuencia) {
  melodiaAlarmaActiva = false;
  taskMelodiaAlarma.Stop();
  tone(PIN_BUZZER, frecuencia, duracion);
}

/**
 * @brief Silenciar el buzzer y detener cualquier melodia activa.
 *
 * @details
 *   Cancela la bandera de melodia y llama a `noTone()` para liberar el pin del
 *   buzzer.
 */
void detenerBuzzer() {
  melodiaAlarmaActiva = false;
  taskMelodiaAlarma.Stop();
  noTone(PIN_BUZZER);
}

/**
 * @brief Iniciar la melodia de alarma sin bloquear la maquina de estados.
 *
 * @details
 *   Reinicia el indice de melodia y deja que la tarea no bloqueante reproduzca
 *   las notas cuando la FSM ya esta en `ST_ALARMA`.
 */
void iniciarMelodiaAlarma() {
  indiceMelodiaAlarma = 0;
  ticksNotaAlarma = 0;
  melodiaAlarmaActiva = true;
  taskMelodiaAlarma.Start();
}

/**
 * @brief Actualizar la melodia de alarma.
 *
 * @details
 *   Avanza nota por nota usando una tarea y datos almacenados en PROGMEM. Si la
 *   FSM sale de `ST_ALARMA`, detiene el buzzer.
 */
void reproducirSiguienteNotaAlarma() {
  if (!melodiaAlarmaActiva) return;

  if ((Estado) miMaquina.GetState() != ST_ALARMA) {
    detenerBuzzer();
    return;
  }

  if (ticksNotaAlarma > 0) {
    ticksNotaAlarma--;
    return;
  }

  if (indiceMelodiaAlarma >= ELEMENTOS_MELODIA_ALARMA) {
    indiceMelodiaAlarma = 0;
  }

  noTone(PIN_BUZZER);

  byte nota = pgm_read_byte_near(notasDoomAlarma + indiceMelodiaAlarma);
  int8_t divisor = (int8_t) pgm_read_byte_near(duracionesDoomAlarma + indiceMelodiaAlarma);
  byte divisorAbsoluto = (divisor < 0) ? (byte) -divisor : (byte) divisor;
  unsigned int duracionNota = DURACION_REDONDA_DOOM_MS / divisorAbsoluto;
  if (divisor < 0) {
    duracionNota = (unsigned int)((duracionNota * 3UL) / 2UL);
  }
  unsigned int duracionTono = (unsigned int)((duracionNota * 9UL) / 10UL);

  if (nota == REST) {
    noTone(PIN_BUZZER);
  } else {
    tone(PIN_BUZZER, nota, duracionTono);
  }

  indiceMelodiaAlarma++;
  ticksNotaAlarma = (duracionNota + TICK_MELODIA_MS - 1) / TICK_MELODIA_MS;
  if (ticksNotaAlarma > 0) ticksNotaAlarma--;
}

/**
 * @brief Imprimir una linea de 16 caracteres en el LCD.
 *
 * @param [in] fila   Fila del LCD, `0` o `1`.
 * @param [in] texto  Cadena a imprimir. Se rellena con espacios hasta limpiar
 *                    restos de textos anteriores.
 */
void imprimirLineaLCD(byte fila, const char *texto) {
  lcd.setCursor(0, fila);
  bool finTexto = false;
  for (byte restantes = LCD_COLUMNAS; restantes > 0; restantes--) {
    byte i = LCD_COLUMNAS - restantes;
    if (!finTexto && texto[i] != '\0') {
      lcd.print(texto[i]);
    } else {
      finTexto = true;
      lcd.print(' ');
    }
  }
}

/**
 * @brief Mostrar dos lineas en el LCD.
 *
 * @param [in] l1  Texto para la primera fila.
 * @param [in] l2  Texto para la segunda fila.
 */
void mostrarMensajeLCD(const char *l1, const char *l2) {
  imprimirLineaLCD(0, l1);
  imprimirLineaLCD(1, l2);
}

/**
 * @brief Mostrar la pantalla de espera con hora actual cuando esta disponible.
 *
 * @details
 *   En la segunda linea muestra la hora configurada o un aviso cuando el reloj
 *   aun no fue ajustado en la sesion.
 */
void mostrarPantallaInicio() {
  if (relojConfigurado) {
    char hora[HORA_BUFFER_TAM];
    char l2[LCD_BUFFER_TAM];
    formatearHora(minutoActualDelDia(), hora, sizeof(hora));
    snprintf(l2, sizeof(l2), "%s o tarjeta", hora);
    mostrarMensajeLCD("A=clave #=cfg", l2);
  } else {
    mostrarMensajeLCD("A=clave #=cfg", "Sin hora config");
  }
}

/**
 * @brief Copiar una linea segura de hasta 16 caracteres para el LCD.
 *
 * @param [out] destino  Buffer de al menos 17 bytes.
 * @param [in]  origen   Texto de origen terminado en `\0`.
 */
void copiarLineaBloqueo(char *destino, const char *origen) {
  byte i = 0;
  for (byte restantes = LCD_COLUMNAS; restantes > 0; restantes--) {
    i = LCD_COLUMNAS - restantes;
    if (origen[i] == '\0') {
      destino[i] = '\0';
      return;
    }
    destino[i] = origen[i];
  }
  destino[LCD_COLUMNAS] = '\0';
}

/**
 * @brief Definir el texto que se mostrara al entrar a `ST_BLOQUEO`.
 *
 * @param [in] l1  Primera linea del mensaje de bloqueo.
 * @param [in] l2  Segunda linea del mensaje de bloqueo.
 */
void prepararMensajeBloqueo(const char *l1, const char *l2) {
  copiarLineaBloqueo(lineaBloqueo1, l1);
  copiarLineaBloqueo(lineaBloqueo2, l2);
}

/**
 * @brief Restaurar el texto generico de bloqueo.
 *
 * @details
 *   Repone el mensaje por defecto usado cuando no hay un motivo especifico de
 *   bloqueo pendiente.
 */
void resetearMensajeBloqueo() {
  prepararMensajeBloqueo("BLOQUEADO", "Espere 7 seg");
}

/**
 * @brief Limpiar el buffer de clave ingresada.
 *
 * @details
 *   Borra los digitos capturados y deja `posPassword` listo para una nueva
 *   entrada de usuario.
 */
void resetearBufferPassword() {
  for (byte restantes = LONGITUD_PASSWORD + 1; restantes > 0; restantes--) {
    passwordIngresada[restantes - 1] = 0;
  }
  posPassword = 0;
}
