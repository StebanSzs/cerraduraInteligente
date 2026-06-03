# Cerradura Inteligente Empresarial

Firmware para Arduino Mega 2560 que implementa una cerradura inteligente con
autenticacion por RFID, clave numerica, perfiles de usuario, horarios de acceso,
monitoreo ambiental, monitoreo de puerta, alarma visual y alarma sonora.

El codigo esta organizado como una maquina de estados no bloqueante. Las tareas
periodicas se ejecutan con `AsyncTaskLib` y las transiciones principales se
manejan con `StateMachineLib`.

## 1. Informacion general

| Campo | Valor |
| --- | --- |
| Archivo principal | `CerraduraInteligenteProyectoFInal.ino` |
| Plataforma | Arduino Mega 2560 |
| Interfaz | LCD 16x2, teclado 4x4 y boton fisico |
| Autenticacion | RFID RC522 y clave numerica de 6 digitos |
| Persistencia | EEPROM interna |
| Control fisico | Servo en pin 9 |
| Alarma sonora | Buzzer en pin 10 con melodia no bloqueante |
| Documentacion | Comentarios Doxygen dentro del codigo |

El sistema no usa `delay()`. Ademas, por solicitud del profesor, `millis()` no
se usa para controlar alarmas, temporizadores, parpadeos, antirrebote ni melodia.
El unico uso de `millis()` queda reservado al reloj interno de horarios, ya que
sin un modulo RTC externo el Arduino necesita una referencia para calcular la
hora despues de configurarla.

## 2. Librerias requeridas

| Libreria | Uso |
| --- | --- |
| `AsyncTaskLib` | Tareas periodicas y temporizadores no bloqueantes |
| `StateMachineLib` | Maquina de estados principal |
| `MFRC522` | Lector RFID RC522 |
| `EEPROM` | Persistencia de usuarios, horarios, umbrales y logs |
| `LiquidCrystal` | Pantalla LCD 16x2 |
| `Keypad` | Teclado matricial 4x4 |
| `Servo` | Control del servo de la cerradura |
| `SPI` | Comunicacion con el modulo RFID |
| `avr/pgmspace` | Almacenamiento de melodia en memoria de programa |

## 3. Hardware y mapa de pines

### 3.1 RFID RC522

| Senal | Pin Arduino Mega |
| --- | --- |
| SS/SDA | 53 |
| RST | 49 |
| SPI | Pines SPI del Arduino Mega |

### 3.2 LCD 16x2

| Senal LCD | Pin Arduino Mega |
| --- | --- |
| RS | 23 |
| EN | 25 |
| D4 | 27 |
| D5 | 29 |
| D6 | 31 |
| D7 | 33 |

### 3.3 Teclado 4x4

| Linea | Pin Arduino Mega |
| --- | --- |
| Fila 1 | 22 |
| Fila 2 | 24 |
| Fila 3 | 26 |
| Fila 4 | 28 |
| Columna 1 | 30 |
| Columna 2 | 32 |
| Columna 3 | 34 |
| Columna 4 | 36 |

Mapa de teclas:

```text
1 2 3 A
4 5 6 B
7 8 9 C
* 0 # D
```

### 3.4 Actuadores e indicadores

| Componente | Pin | Funcion |
| --- | ---: | --- |
| Servo | 9 | Abre y cierra la cerradura |
| Buzzer | 10 | Tonos cortos y alarma |
| LED rojo | 35 | Bloqueo y alarma |
| LED verde | 37 | Acceso permitido |
| LED azul | 39 | Espera, configuracion y monitoreo |
| Boton fisico | 41 | Salir de configuracion o entrar a gestion |

El boton fisico trabaja con `INPUT_PULLUP`, por lo que debe conectarse entre el
pin 41 y GND.

### 3.5 Sensores analogicos

| Sensor | Pin | Interpretacion usada |
| --- | --- | --- |
| KY-013 Temperatura | A0 | Temperatura real en grados Celsius |
| KY-035 Hall | A1 | Iman cerca baja el valor; sin iman sube |
| KY-037 Sonido | A2 | Silencio bajo; sonido fuerte alto |
| KY-018 Luz | A3 | Oscuro 0; maxima luz 1023 |

## 4. Perfiles de usuario

El sistema maneja cuatro perfiles configurables:

| Perfil | Valor interno | Nombre en LCD | Horario por defecto |
| --- | ---: | --- | --- |
| Seguridad | 1 | `Seguridad` | 00:00 a 23:59 |
| Operario | 2 | `Operario` | 08:00 a 17:00 |
| Coordinador | 3 | `Coordin.` | 07:00 a 19:00 |
| Gerente | 4 | `Gerente` | 00:00 a 23:59 |

Cada credencial RFID o clave numerica queda asociada a un perfil. El acceso solo
se permite cuando la credencial existe, esta activa y el perfil esta dentro de
su horario configurado.

## 5. Credenciales y seguridad

### 5.1 Claves por perfil

Cada perfil puede tener una clave numerica de 6 digitos. La clave se configura
desde `CONFIG > C` y se guarda en EEPROM como una credencial de usuario.

Flujo para configurar una clave:

1. Desde `INICIO`, presionar `#` para entrar a `CONFIG`.
2. Presionar `C`.
3. Elegir perfil con `1`, `2`, `3` o `4`.
4. Digitar una clave de 6 digitos.
5. Confirmar con `#`.

Cuando se registra una nueva clave para un perfil, las claves anteriores de ese
mismo perfil quedan desactivadas. Si la clave pertenece a un registro que tambien
tiene RFID, solo se desactiva la bandera de clave y se conserva la tarjeta.

### 5.2 Limite de usos de clave

Cada clave numerica puede abrir la cerradura un maximo de 4 veces. Despues de
cuatro usos exitosos, la clave queda vencida y debe cambiarse desde `CONFIG > C`.

Al usar una clave correcta, el LCD muestra el contador de usos restantes:

```text
Acceso permitido
Clave X/4 usos
```

Si la clave ya llego al limite de usos, el sistema entra en bloqueo temporal y
muestra:

```text
Clave vencida
Cambie CONFIG C
```

### 5.3 Intentos fallidos

Las claves incorrectas incrementan el contador `intentosFallidos`.

| Intentos fallidos | Comportamiento |
| ---: | --- |
| 1 | Bloqueo temporal y aviso en LCD |
| 2 | Bloqueo temporal y aviso en LCD |
| 3 | Se activa la alarma |

Al tercer intento fallido, el sistema entra a `ST_ALARMA`, reproduce la alarma
sonora, muestra el mensaje de alarma y luego regresa a `ST_INICIO`.

### 5.4 RFID

Las tarjetas se registran desde `CONFIG > B` y se eliminan desde `CONFIG > D`.
Al registrar una tarjeta, el usuario selecciona primero el perfil al que quedara
asociada.

Una tarjeta no registrada provoca bloqueo temporal directo.

## 6. Reloj y horarios

El reloj interno se calcula con `millis()` solamente dentro de las funciones de
horario. Este uso se mantiene porque el Arduino Mega no tiene reloj de tiempo
real incorporado.

Despues de reiniciar el Arduino se debe configurar la hora actual para que el
control por horario funcione correctamente.

### 6.1 Configurar hora actual

1. Desde `INICIO`, presionar `#`.
2. Presionar `A`.
3. Presionar `1`.
4. Digitar la hora en formato `HHMM`.
5. Confirmar con `#`.

Ejemplo para las 08:30:

```text
0830#
```

### 6.2 Configurar horario de un perfil

1. Desde `INICIO`, presionar `#`.
2. Presionar `A`.
3. Presionar `2`.
4. Elegir perfil con `1`, `2`, `3` o `4`.
5. Digitar inicio y fin en formato `HHMMHHMM`.
6. Confirmar con `#`.

Ejemplo para permitir acceso de 08:00 a 17:00:

```text
08001700#
```

El sistema tambien acepta horarios que cruzan medianoche. Si inicio y fin son
iguales, el firmware interpreta que el perfil tiene acceso durante todo el dia.

## 7. Sensores y umbrales

| Umbral | Constante por defecto | Valor | Condicion |
| --- | --- | ---: | --- |
| Temperatura | `UMBRAL_TEMP_DEFAULT` | 24 | Alarma si temperatura supera el umbral |
| Luz | `UMBRAL_LUZ_DEFAULT` | 400 | Alarma si luz supera el umbral |
| Hall | `UMBRAL_HALL_DEFAULT` | 600 | Condicion si Hall supera el umbral |
| Ruido | `UMBRAL_RUIDO_DEFAULT` | 700 | Evento si sonido supera el umbral |

La alarma ambiental se activa cuando se cumplen al mismo tiempo:

```cpp
temperaturaActual > umbralTemp && lecturaLuz > umbralLuz
```

La alarma por puerta se activa durante `ST_MONITOR_PUERTAS` solo cuando Hall y
sonido superan sus umbrales al mismo tiempo:

```cpp
lecturaHall > umbralHallAbierto && lecturaSonido > umbralRuido
```

La condicion es una AND logica: si solo se activa Hall o solo se activa sonido,
el sistema no entra a alarma por puerta.

## 8. Maquina de estados

| Estado | Funcion |
| --- | --- |
| `ST_INICIO` | Espera clave, RFID o entrada a configuracion |
| `ST_CONFIG` | Registra RFID, claves, hora y horarios |
| `ST_SERVO_ABIERTO` | Abre la cerradura durante 5 segundos |
| `ST_MONITOR_AMBIENTAL` | Supervisa temperatura y luz |
| `ST_MONITOR_PUERTAS` | Supervisa Hall y sonido |
| `ST_ALARMA` | Activa alarma visual y sonora |
| `ST_GESTION` | Permite modificar umbrales |
| `ST_BLOQUEO` | Bloqueo temporal de 7 segundos |

Flujo principal:

```text
INICIO --(#)--> CONFIG --(boton)--> INICIO
INICIO --(A + clave valida en horario)--> SERVO_ABIERTO
INICIO --(RFID valido en horario)--> SERVO_ABIERTO
INICIO --(3 claves incorrectas)--> ALARMA
SERVO_ABIERTO --(5 s)--> MONITOR_AMBIENTAL
MONITOR_AMBIENTAL --(5 s)--> MONITOR_PUERTAS
MONITOR_PUERTAS --(5 s)--> MONITOR_AMBIENTAL
MONITOR_AMBIENTAL --(temperatura y luz altas)--> ALARMA
MONITOR_PUERTAS --(Hall>umbral Y sonido>umbral)--> ALARMA
ALARMA --(3 alarmas consecutivas)--> GESTION
GESTION --(*)--> INICIO
BLOQUEO --(7 s)--> INICIO
```

## 9. Tareas asincronas

| Tarea | Intervalo | Funcion |
| --- | ---: | --- |
| `taskTemperatura` | 1000 ms | Leer temperatura |
| `taskHall` | 300 ms | Leer sensor Hall |
| `taskSonido` | 200 ms | Leer microfono |
| `taskLuz` | 500 ms | Leer luz |
| `taskLedAzul` | 800 ms | Parpadeo/latido en espera |
| `taskLedAlarma` | 100 ms inicial | Parpadeo rojo en alarma |
| `taskLedBloqueo` | 300 ms inicial | Parpadeo durante bloqueo |
| `taskServo` | 5000 ms | Cerrar servo |
| `taskAmbPuertas` | 5000 ms | Pasar de ambiente a puertas |
| `taskPuertasAmb` | 5000 ms | Volver de puertas a ambiente |
| `taskAlarmaAmb` | 3000 ms | Salir de alarma hacia ambiente |
| `taskAlarmaPuertas` | 4000 ms | Salir de alarma hacia puertas |
| `taskAlarmaInicio` | 3000 ms | Salir de alarma hacia inicio |
| `taskAlarmaGestion` | 2000 ms | Entrar a gestion tras alarmas repetidas |
| `taskBloqueo` | 7000 ms | Terminar bloqueo |
| `taskAvanceAuto` | 50 ms | Disparar transiciones diferidas |
| `taskBotonDebounce` | 50 ms | Confirmar boton fisico sin rebotes |
| `taskLCDMonitoreo` | 750 ms | Refrescar LCD en monitoreo |
| `taskMelodiaAlarma` | 25 ms | Avanzar melodia de alarma |

Estas tareas reemplazan esperas bloqueantes. Por eso el sistema puede seguir
leyendo teclado, RFID, sensores y eventos mientras el servo, los LEDs o el buzzer
estan activos.

## 10. Alarma sonora

La alarma usa el riff principal de `At Doom's Gate`. La melodia se guarda en dos
arreglos en `PROGMEM`:

| Arreglo | Tipo | Motivo |
| --- | --- | --- |
| `notasDoomAlarma[]` | `byte` | Las frecuencias usadas van de 82 a 165 Hz y caben en 8 bits |
| `duracionesDoomAlarma[]` | `int8_t` | Las duraciones incluyen valores negativos para notas con puntillo |

La reproduccion no usa `delay()`. La funcion `reproducirSiguienteNotaAlarma()`
calcula la duracion de cada nota con `DURACION_REDONDA_DOOM_MS`, toca el buzzer
con `tone()` durante el 90% del tiempo y espera la siguiente nota usando ticks de
`taskMelodiaAlarma`.

La primera nota se reproduce desde la tarea cuando la maquina ya esta en
`ST_ALARMA`. Esto evita que el tono se pierda durante la entrada al estado.

## 11. EEPROM

### 11.1 Direcciones principales

| Direccion | Constante | Contenido |
| ---: | --- | --- |
| 0 | `DIR_FLAG_INIT` | Marca de EEPROM inicializada |
| 16 | `DIR_USUARIOS` | Inicio de tabla de credenciales |
| 512 | `DIR_LOG_INDEX` | Indice circular del log |
| 513 | `DIR_LOG_BASE` | Inicio del log circular |
| 529 | `DIR_FLAG_UMBRALES` | Marca de umbrales validos |
| 530 | `DIR_UMBRAL_TEMP` | Umbral de temperatura |
| 534 | `DIR_UMBRAL_LUZ` | Umbral de luz |
| 536 | `DIR_UMBRAL_HALL` | Umbral Hall |
| 538 | `DIR_UMBRAL_RUIDO` | Umbral de ruido |
| 540 | `DIR_FLAG_RELOJ` | Marca de hora guardada |
| 541 | `DIR_RELOJ_MINUTOS` | Minutos desde medianoche |
| 543 | `DIR_FLAG_HORARIOS` | Marca de horarios validos |
| 544 | `DIR_HORARIOS_BASE` | Inicio de tabla de horarios |

### 11.2 Registro de usuario

Cada usuario ocupa 16 bytes.

| Offset | Constante | Contenido |
| ---: | --- | --- |
| 0 | `USUARIO_OFF_ESTADO` | Activo o inactivo |
| 1 | `USUARIO_OFF_PERFIL` | Perfil asociado |
| 2 | `USUARIO_OFF_UID` | Inicio del UID RFID |
| 6 | `USUARIO_OFF_CLAVE` | Inicio de clave numerica |
| 12 | `USUARIO_OFF_FLAGS` | Flags de credenciales |
| 13 | `USUARIO_OFF_USOS_CLAVE` | Usos acumulados de clave |

Flags de credenciales:

| Flag | Valor | Significado |
| --- | ---: | --- |
| `USUARIO_FLAG_RFID` | `0x01` | El registro contiene tarjeta RFID |
| `USUARIO_FLAG_CLAVE` | `0x02` | El registro contiene clave numerica |

La cantidad maxima de credenciales registrables es `MAX_USUARIOS = 15`.

## 12. Optimizaciones aplicadas

### 12.1 Cambio de tipos de variables

Se cambiaron variables que antes podian declararse como `int` por tipos mas
pequenos cuando el rango real del dato lo permitia. Esto reduce el consumo de
SRAM y hace mas claro que valores puede almacenar cada variable.

| Tipo usado | Donde se aplica | Por que se cambio |
| --- | --- | --- |
| `byte` | Pines, perfiles, contadores pequenos, indices de arreglos, flags y estados | Sus valores estan entre 0 y 255; ocupa 1 byte |
| `bool` | Banderas como `alarmaVieneDePuertas`, `melodiaAlarmaActiva` y eventos pendientes | Solo guardan verdadero o falso; ocupa menos y expresa mejor la intencion |
| `unsigned int` | Lecturas analogicas, direcciones EEPROM, minutos del dia y umbrales de 0 a 1023 | No requieren numeros negativos y necesitan mas rango que `byte` |
| `unsigned long` | Tiempos base del reloj interno | Es necesario para operar con valores grandes de `millis()` |
| `int8_t` | Duraciones de la melodia Doom | Permite valores positivos y negativos en solo 1 byte |
| `float` | Temperatura y calculos del termistor | Se mantiene porque la temperatura requiere decimales |

Ejemplos concretos del codigo:

```cpp
byte intentosFallidos = 0;
byte perfilActual = PERFIL_NINGUNO;
unsigned int lecturaLuz = 0;
unsigned int minutoBaseReloj = 0;
unsigned long millisBaseReloj = 0;
const int8_t duracionesDoomAlarma[] PROGMEM = { 8, 8, 8, -2 };
```

La optimizacion no consiste en cambiar todo a `byte`, sino en elegir el tipo mas
pequeno que no pierda informacion. Por eso las lecturas analogicas siguen como
`unsigned int`, porque `analogRead()` puede entregar valores hasta 1023.

### 12.2 Uso de `PROGMEM`

La melodia de alarma y otros datos constantes se almacenan en memoria de programa
cuando corresponde. Esto evita gastar SRAM en datos que no cambian durante la
ejecucion.

En la alarma:

```cpp
const byte notasDoomAlarma[] PROGMEM = { ... };
const int8_t duracionesDoomAlarma[] PROGMEM = { ... };
```

Las notas se leen con `pgm_read_byte_near()`, porque estan guardadas en flash y
no directamente en RAM.

### 12.3 Eliminacion de esperas bloqueantes

El codigo evita `delay()` para que el sistema no quede congelado mientras espera.
En su lugar se usan tareas y ticks:

| Antes | Ahora |
| --- | --- |
| Esperar con `delay()` | Tareas con `AsyncTask` |
| Pausar la melodia con `delay(pausa)` | `taskMelodiaAlarma` cada 25 ms |
| Controlar parpadeos con esperas | Contadores `ticksParpadeoRojo` |
| Antirrebote por espera fija | `taskBotonDebounce` |
| Refresco manual bloqueante del LCD | `taskLCDMonitoreo` |

Esto permite que teclado, RFID, sensores, LCD, LEDs, servo y buzzer sigan
funcionando de forma simultanea.

### 12.4 Uso limitado de `millis()`

El profesor solicito no usar `millis()` en el codigo excepto en la parte de
horario. Por eso se reviso el sketch para dejar `millis()` solamente en:

| Uso | Funcion |
| --- | --- |
| Guardar referencia de hora | `configurarHoraActual()` |
| Restaurar base de reloj | carga inicial del reloj guardado |
| Calcular minuto actual | `minutoActualDelDia()` |

La alarma, la melodia, los parpadeos, los cambios de estado, el bloqueo, el
servo, el LCD y el boton no dependen de llamadas directas a `millis()`.

### 12.5 Uso de `#define`

Se usa `#define` para valores constantes del proyecto: pines, tamanos de buffer,
limites, tiempos, direcciones EEPROM, notas musicales y parametros de la melodia.

Ejemplos:

```cpp
#define PIN_BUZZER 10
#define MAX_INTENTOS 3
#define UMBRAL_LUZ_DEFAULT 400
#define TICK_MELODIA_MS 25U
#define DURACION_REDONDA_DOOM_MS ((60000UL * 4UL) / TEMPO_DOOM_ALARMA)
```

La razon principal es que estos valores no cambian durante la ejecucion. Al ser
macros, el preprocesador reemplaza el nombre por el valor antes de compilar, por
lo que no se reserva una variable en SRAM para guardarlos.

Tambien mejora la lectura del codigo: es mas claro escribir `PIN_BUZZER` que
escribir directamente `10`, y si se cambia un pin o un limite solo se modifica
una linea.

No se usa `#define` para datos que cambian mientras el programa corre. Para esos
casos se mantienen variables normales, como `intentosFallidos`, `lecturaLuz`,
`perfilActual` o `melodiaAlarmaActiva`.

## 13. Interfaz por teclado

### 13.1 Desde INICIO

| Tecla | Accion |
| --- | --- |
| `A` | Iniciar captura de clave o confirmar clave ya digitada |
| Digitos `0-9` | Capturar clave solo despues de presionar `A` |
| `*` | Borrar/cancelar clave ingresada |
| `#` | Entrar a configuracion |
| RFID | Validar tarjeta |

### 13.2 Desde CONFIG

| Tecla | Accion |
| --- | --- |
| `A` | Reloj y horarios |
| `B` | Registrar RFID |
| `C` | Configurar clave por perfil |
| `D` | Eliminar RFID |
| `*` | Volver al menu anterior |
| Boton fisico | Salir a `INICIO` |

### 13.3 Desde GESTION

| Tecla | Accion |
| --- | --- |
| `A` | Editar umbral de temperatura |
| `B` | Editar umbral de luz |
| `C` | Editar umbral Hall |
| `D` | Editar umbral de ruido |
| Digitos `0-9` | Ingresar nuevo valor |
| `#` | Guardar |
| `*` | Salir a `INICIO` |

## 14. Log de eventos

El firmware guarda codigos compactos en un log circular de EEPROM.

| Codigo | Evento |
| ---: | --- |
| `0x01` | Acceso permitido |
| `0x03` | Bloqueo temporal |
| `0x04` | Alarma de monitoreo |
| `0x05` | Acceso fuera de horario |
| `0x06` | Alarma por tres intentos de clave |

El log circular tiene `LOG_TAMANO = 16` posiciones.

## 15. Generacion de documentacion Doxygen

El proyecto conserva comentarios Doxygen en el archivo principal y el archivo
`Doxyfile` para generar la documentacion HTML.

Para regenerarla:

```powershell
doxygen Doxyfile
```

La documentacion se puede volver a generar desde el codigo fuente si `doxygen`
esta instalado y disponible en el `PATH`.

## 16. Puesta en marcha recomendada

1. Cargar el sketch en Arduino Mega 2560.
2. Verificar cableado de LCD, teclado, RFID, servo, LEDs, buzzer y sensores.
3. Abrir el monitor serial a 9600 baudios.
4. Entrar a `CONFIG` con `#`.
5. Configurar la hora actual en `CONFIG > A > 1`.
6. Configurar horarios de perfiles si se necesitan cambios.
7. Registrar tarjetas RFID en `CONFIG > B`.
8. Registrar claves por perfil en `CONFIG > C`.
9. Probar acceso con RFID y clave dentro del horario permitido.
10. Revisar en LCD que la clave muestre usos restantes.
11. Probar sensores en monitoreo y ajustar umbrales desde `GESTION` si hace falta.
12. Provocar una alarma para verificar el LED rojo y el tono Doom del buzzer.

## 17. Resumen de requisitos cumplidos

| Requisito | Estado |
| --- | --- |
| Temperatura real en grados centigrados | Cumplido |
| KY-013 en A0 | Cumplido |
| KY-035 Hall en A1 | Cumplido |
| KY-037 Sonido en A2 | Cumplido |
| KY-018 Luz en A3 | Cumplido |
| Hall baja con iman cerca y sube sin iman | Cumplido |
| Sonido sube cuando detecta ruido | Cumplido |
| Luz sube cuando hay mas luz | Cumplido |
| Clave por perfil de usuario | Cumplido |
| Clave valida solo dentro del horario | Cumplido |
| Cada clave dura 4 usos | Cumplido |
| Contador de usos visible en LCD | Cumplido |
| Alarma despues de 3 intentos fallidos | Cumplido |
| Melodia Doom en alarma | Cumplido |
| Sin `delay()` | Cumplido |
| `millis()` solo para reloj/horario | Cumplido |
| Variables optimizadas por rango | Cumplido |
| Bucles `for` decrementales | Cumplido |
| Constantes principales con `#define` | Cumplido |
| Documentacion Doxygen en codigo | Cumplido |
