# CARGA ELECTRÓNICA FANTASMA

La carga electrónica fantasma es un dispositivo diseñado para simular una carga eléctrica, utilizado principalmente para pruebas y mediciones. 

Este proyecto está basado en un microcontrolador STM32F103, el cual controla la Vgs del MOSFET para regular la corriente.

**Estado del proyecto:** Funcional, pero aún en proceso de creación.

----------------- **BAJO SU RESPONSABILIDAD** ----------------------

### Video

[Demo](https://github.com/user-attachments/assets/59047016-5cc4-44e5-a77e-8e39651e7eba)


## Preparación Previa

Dado que cada transistor MOSFET presenta un voltaje umbral de puerta a fuente (Vgs Threshold) único, independientemente del fabricante o del código de identificación, es imperativo llevar a cabo mediciones específicas para determinar este voltaje umbral, así como la relación entre el ciclo de trabajo (Duty Cycle) y la corriente.

Para ello, se empleará el proyecto [Data Logger](https://github.com/kr4fty/DataLogger_STM32) con el fin de registrar estos parámetros. Posteriormente, los valores obtenidos se integrarán en este proyecto para facilitar el control adecuado de nuestra carga electrónica.

## Que incluye este repositorio?

Actualmente este repositorio cuenta con:
 - Diseño del hardware necesario para el proyecto
 - Codigo fuentes, tanto el de Control como la de adquicision de Datos
 - Archivo necesario para simular la placa BluePill, mediante PICSIMLAB

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

### Modos de Funcionamiento
* Modo de **Corriente Constante**: Este es el modo predeterminado de operación, en el cual el dispositivo mantiene un nivel de corriente constante, determinado por el operador. Este modo es ideal para evaluar el comportamiento de una fuente o batería al operar bajo una corriente estable.
* Modo de **Potencia Constante**: En este modo, el dispositivo ajusta automáticamente la corriente para mantener la potencia consumida constante en la carga. Este enfoque es útil en situaciones donde es crítico mantener un nivel de potencia específico.
* Modo de **Tiempo**: Este modo permite al operador establecer un intervalo de tiempo durante el cual la carga permanecerá activa. Se pueden utilizar cualquiera de los modos anteriores en conjunto con este, según las necesidades de la aplicación.


## Hardware

El diseño del hardware se realizó utilizando KiCad, y se ha optimizado para facilitar la conexión de todos los componentes. A continuación, se detallan los aspectos clave del diseño:


### Consideraciones de Diseño

- **Disipación de Calor:** Asegúrate de que el MOSFET esté montado en un disipador de calor adecuado para evitar el sobrecalentamiento durante su operación.
- **Conexiones Seguras:** Verifica que todas las conexiones sean seguras y que no haya cortocircuitos, especialmente en la zona de alimentación.
- **Alimentación:** Utiliza una fuente de alimentación adecuada que pueda proporcionar la corriente necesaria para la carga que se va a simular.

### Imagen del PCB

<img align="center" src="https://i.ibb.co/pQYX3jh/Sin-nombre-resized.png" alt="Esquema del Hardware" width="720">
---

Este documento proporciona una visión general del proyecto y su funcionamiento. Asegúrate de seguir las indicaciones de seguridad al trabajar con componentes electrónicos y realizar pruebas.
