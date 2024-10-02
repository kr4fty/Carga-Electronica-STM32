# CARGA ELECTRÓNICA FANTASMA

La carga electrónica fantasma es un dispositivo diseñado para simular una carga eléctrica, utilizado principalmente para pruebas y mediciones. 

Este proyecto está basado en un microcontrolador STM32F103, el cual controla la Vgs del MOSFET para regular la corriente.

**Estado del proyecto:** Funcional, pero aún en proceso de creación.

----------------- **BAJO SU RESPONSABILIDAD** ----------------------

## Preparación Previa

Dado que cada transistor MOSFET presenta un voltaje umbral de puerta a fuente (Vgs Threshold) único, independientemente del fabricante o del código de identificación, es imperativo llevar a cabo mediciones específicas para determinar este voltaje umbral, así como la relación entre el ciclo de trabajo (Duty Cycle) y la corriente.

Para ello, se empleará el proyecto [Data Logger](https://github.com/kr4fty/DataLogger_STM32) con el fin de registrar estos parámetros. Posteriormente, los valores obtenidos se integrarán en este proyecto para facilitar el control adecuado de nuestra carga electrónica.

## Materiales

- **Placa:** BLUE PILL (STM32F103)
- **Encoder con botón**
- **Pantalla LCD:** Nokia 5110
- **Buzzer**
- **Sensor de temperatura:** LM35
- **Cooler Fan**
- **Transistor/s MOSFET:** IRFZ44
- **Sensor de corriente:** ACS712-05B

## Principio de Funcionamiento

El MOSFET opera en la zona de saturación (o zona lineal), lo que permite mantener una corriente constante independientemente de la tensión Vds de entrada. Esta característica es ventajosa para nuestra aplicación, ya que al variar la tensión Vgs podemos ajustar la corriente de salida de manera precisa.

La tensión Vds será controlada mediante el STM32F103 utilizando modulación por ancho de pulso (PWM) y un filtro pasa-bajos para obtener una señal de conversión digital a analógica (DAC) de 12 bits.

Debido a la inestabilidad de la potencia con respecto a la temperatura de los mosfset, el lazo de control sera realizado mediante un PID, el cual debera tener en cuenta el rango de temperaturas no solo para mantener la corriente constante, si no tambien para mantener a salvo al dispositivo.

#### Consideraciones importantes de los transistores Mosfet

- **Tensión de umbral (Vgs threshold):** Es el voltaje mínimo que debe aplicarse entre la puerta y la fuente para que el MOSFET comience a conducir corriente entre el drenador y la fuente.
- **Disipación de potencia:** Operar en la zona de saturación genera una significativa disipación de potencia en el MOSFET. Por lo tanto, es necesario implementar un sistema de disipación forzada para evitar el sobrecalentamiento.

## Interfaz

- **Encendido:** 

Al encender el dispositivo, se mostrará la tensión de entrada y la corriente deseada para la prueba, con la carga inicialmente apagada.

- **Ajuste de Corriente:** 

Gire la perilla para seleccionar una corriente mayor o menor a la configurada por defecto.

- **Inicio del Proceso:** 

Presione el botón para que la carga electrónica se inicie. Presione nuevamente para interrumpir o pausar el proceso.

- **Visualización de Datos:**

Durante el funcionamiento, se mostrarán:
- Tensión actual de la bateria/fuente
- Corriente actual que se esta consumiendo
- Amperes-hora consumidos
- Watts-hora consumidos
- Tiempo que la carga esta en funcionamiento
- Temperatura de los mosfet

### Indicaciones y Alarmas

- **Comprobación de la conexión de voltaje en la entrada:** 

Al iniciar, se verifica la conexión de la batería/fuente. Si no hay conexión, el dispositivo permanecerá en reposo hasta que se detecte una fuente.
- **Detención por Ausencia de Tensión:**

Si luego de iniciado la carga electronica, se detectara la ausencia de tensión, el proceso se detiene inmediatamente. Se activa una alarma sonora y todos los procesos se pausarán, quedando en espera de reconexión.
- **Alarma de Temperatura:**

Se monitorea la temperatura de los MOSFET. Si se supera la temperatura crítica, todos los procesos se detendrán de inmediato.

## Hardware

El diseño del hardware se realizó utilizando KiCad, y se ha optimizado para facilitar la conexión de todos los componentes. A continuación, se detallan los aspectos clave del diseño:

### Esquema de Conexiones

- **Microcontrolador (STM32F103):** 
  - Conectado al encoder para la selección de corriente.
  - Interfaz con la pantalla LCD Nokia 5110 para la visualización de datos.
  - Controla el MOSFET (IRFZ44) a través de un pin PWM.
  - Monitorea la temperatura mediante el LM35 y la corriente a través del ACS712-05B.

- **Encoder con botón:**
  - Conectado a un pin digital del STM32 para la entrada de usuario.

- **Pantalla LCD Nokia 5110:**
  - Conectada a los pines de datos y control del STM32 para la visualización de información.

- **Buzzer:**
  - Conectado a un pin digital para emitir alarmas sonoras.

- **Sensor de temperatura (LM35):**
  - Conectado a un pin analógico del STM32 para la lectura de temperatura.

- **Ventilador Cooler Fan:**
  - Controlado por el STM32 para activar la disipación forzada en caso de sobrecalentamiento.

- **Transistor MOSFET (IRFZ44):**
  - Conectado a la carga y controlado por el STM32 para regular la corriente.

- **Sensor de corriente (ACS712-05B):**
  - Conectado a un pin analógico del STM32 para medir la corriente de salida.

### Consideraciones de Diseño

- **Disipación de Calor:** Asegúrate de que el MOSFET esté montado en un disipador de calor adecuado para evitar el sobrecalentamiento durante su operación.
- **Conexiones Seguras:** Verifica que todas las conexiones sean seguras y que no haya cortocircuitos, especialmente en la zona de alimentación.
- **Alimentación:** Utiliza una fuente de alimentación adecuada que pueda proporcionar la corriente necesaria para la carga que se va a simular.

### Imagen del PCB

<img align="center" src="https://i.ibb.co/pQYX3jh/Sin-nombre-resized.png" alt="Esquema del Hardware" width="720">
---

Este documento proporciona una visión general del proyecto y su funcionamiento. Asegúrate de seguir las indicaciones de seguridad al trabajar con componentes electrónicos y realizar pruebas.