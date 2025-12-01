# Equipo 02 - Fundamentos de Diseño 2025-2  
### Carrera de Ingeniería Informática / Industrial  
**Universidad Peruana Cayetano Heredia**


<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/c4207bbe-a760-44e4-b0a5-8a2c6042af49" />


## 📑 Índice

1. [📘 Presentación del Equipo](#presentación-del-equipo)
2. [⭐ ODS Prioritario](#ods-prioritario)
3. [📸 Fotografía del Equipo](#fotografía-del-equipo)
4. [👥 Integrantes del Equipo](#integrantes-del-equipo)
5. [📝 Problemática en el Perú](#problemática-en-el-perú)
6. [💡 Propuesta de Solución – SmartVent](#propuesta-de-solución--smartvent)
7. [⚙️ Funciones del Sistema](#funciones-del-sistema)
8. [📦 Módulos del Proyecto](#módulos-del-proyecto)
    - [🔌 Módulo Electrónico](#módulo-electrónico)
    - [🛠️ Módulo Mecánico](#módulo-mecánico)
    - [💻 Módulo de Software](#módulo-de-software)
9. [📋 Componentes Utilizados](#componentes-utilizados)
10. [✨ Conclusiones y Trabajo Futuro](#conclusiones-y-trabajo-futuro)

    
## 📘 Presentación del Equipo

Somos el Equipo 02 del curso Fundamentos de Diseño 2025-2, conformado por estudiantes comprometidos con el desarrollo de soluciones innovadoras con impacto social, tecnológico y ambiental.

Nuestro trabajo se alinea con los Objetivos de Desarrollo Sostenible (ODS), especialmente los relacionados con ciudades sostenibles y resilientes.

## ⭐ ODS Prioritario

**ODS 11 – Ciudades y comunidades sostenibles**

Buscamos aportar a la construcción de ciudades más seguras, saludables y resilientes mediante tecnologías accesibles y adaptables a la realidad urbana.


## 📸 Fotografía del Equipo  
<p align="center">

<img width="1000" height="800" alt="image" src="https://github.com/user-attachments/assets/6e37ebff-9d30-4d1d-a354-e620167c4d93" /> <br>
  <em>Figura 1. Fotografía del equipo 2</em>
</p>


## 👥 Integrantes del Equipo  

| Foto | Nombre | Rol | Intereses |
|------|--------|-----|-----------|
| <img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/b80a23aa-58e7-47dd-81fd-aa661d1a2c28" /> | **Idania Channara Parhuay Meza** | Líder del equipo | Innovación social, sostenibilidad |
| <img width="300" height="800" alt="image" src="https://github.com/user-attachments/assets/4dcac393-f906-4e7a-ae3c-b5c699ce6ed2" /> | **Eber Pauccara Huancara** | Diseñador/a | Diseño de prototipos, creatividad aplicada |
| <img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/114daca6-934d-404e-8a1e-c031229eef30" /> | **Cristofer Andherson Quina Pumahuillca** | Encargado/a de documentación | Comunicación científica, redacción técnica |
|<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/7b9d1414-1e5d-4bdc-a8f5-bdd2beb63d2c" />| **Sebastian Calderon Junes** | Programador/a - Modelador/a | Programación, análisis de datos, simulación |


## 📝Problemática en el Perú

En el Perú, la ventilación en viviendas, oficinas y colegios es ineficiente. Muchos espacios permanecen cerrados durante horas, lo que provoca acumulación de gases, humedad y temperaturas elevadas. Esto afecta la salud, la seguridad y el confort de los habitantes.

Además, los incendios urbanos representan un riesgo creciente. La inhalación de humo y gases tóxicos es una de las principales causas de muertes en siniestros domésticos. Muchas ventanas no cuentan con mecanismos de apertura rápida ni sistemas inteligentes que respondan automáticamente ante fugas de gas o humo.

Esta realidad evidencia la necesidad de un sistema retrofit, compacto, adaptable y seguro, capaz de mejorar la ventilación, detectar peligros y actuar sin intervención humana.

<p align="center">
<img width="400" height="300" alt="image" src="https://github.com/user-attachments/assets/be751f8b-8dd3-478c-b8e9-73a5fbc2b129" />

## Propuesta de Solución – SmartVent

SmartVent es un módulo retrofit inteligente que integra sensores ambientales, un mecanismo motorizado de apertura y un sistema de comunicación para emergencias.
Permite ventilar de forma automática ante niveles peligrosos de gas o temperatura.

El sistema analiza continuamente el ambiente interior y responde mediante apertura, activación de alarmas y envío de alertas remotas. Su diseño es compacto, adaptable a marcos existentes y pensado para funcionar sin interrumpir la vida cotidiana.


## Funciones del Sistema

🔹Función principal

- Integrar sensores de gases, temperatura y humedad en una ventana retrofit.
- Detectar fugas, cambios bruscos de temperatura y niveles críticos que comprometan la salud.
- Accionar automáticamente el mecanismo de apertura para ventilar.

🔹 Funciones secundarias

- Enviar alertas locales y remotas.
- Reconocer la calidad del aire y abrirse automáticamente si está contaminado.
- Activar apertura en situaciones de emergencia.

🔹 Funciones de control

- Procesar información en tiempo real con retroalimentación inmediata al usuario.

🔹 Funciones mecánicas

- Lograr movimiento estable mediante engranaje móvil.
- Permitir aperturas graduales.

🔹 Funciones de seguridad

- Detectar humo, gases y partículas tóxicas.
- Permitir apertura manual en caso necesario.

<img width="640" height="360" alt="image" src="https://github.com/user-attachments/assets/be751f8b-8dd3-478c-b8e9-73a5fbc2b129" />


## Módulos del Proyecto

### 🔌Módulo Electrónico

- Incluye sensores de gases, temperatura, humedad y partículas.
- Cuenta con señales de entrada como: encendido, inicio, parada y lecturas de sensores.
- Cuenta con señales de salida como: estado, emergencia, fin de proceso y control de actuadores.
- Funciona conectado a la red eléctrica.
- El hardware incluye mecanismos que abren la ventana.

<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/fe8d0138-987b-4a36-a371-daf679c65595" />

### 🛠️Módulo Mecánico

- El módulo es compacto y adaptable a marcos existentes.
- Su cinemática garantiza un movimiento estable del engranaje y del sistema de apertura.
- El motor debe superar peso, fricción y presión del viento, con margen de seguridad.
- La estructura está diseñada según normas de seguridad contra incendios y calidad del aire.
- Incluye mecanismo de desbloqueo manual para emergencias.
- Está construido para cumplir criterios de ergonomía: suave, silencioso y fácil de usar para cualquier persona.

### 💻Módulo de Software

- Controla sensores, alarmas, extractor y actuadores según el algoritmo.
- Ofrece una aplicación móvil con alertas e historial de calidad del aire.
- Cuenta con dashboard en tiempo real.
- El firmware administra adquisición de datos y control del actuador.
- Gestiona los modos de operación del sistema.


## 📋Componentes Utilizados

A continuación se presenta un cuadro organizado para completar los componentes empleados en el proyecto, clasificados según su función dentro del sistema.

### 🔌Componentes Electrónicos

| Componente | Cantidad | Función | Exigencia Asociada |
|-----------|----------|---------|---------------------|
| Sensor de gas (MQ-x) |1 | Detecta fugas de GLP y gases inflamables | Detectar fugas de gases dentro del ambiente |
| Sensor de temperatura y humedad (DHT/HTU21D) | 1 | Mide condiciones internas del ambiente | Propiedades físicas y químicas: temperatura y humedad |
| Microcontrolador (ESP32)| 1 | Procesamiento, control y comunicación | Centro del sistema: programa autónomo |
| Motor DC 12V 300 RPM | 1 | Acciona la apertura/cierre del sistema mecánico | Superar peso, fricción y presión del viento |
| Driver de motor | 1 | Control del motor mediante el microcontrolador | Control de salidas y actuadores |
| Fuente 12V / regulador | 1 | Alimentación del sistema con protecciones | Alimentación segura y estable |
| Alarma sonora (buzzer) | 1 | Aviso local de emergencias | Señales de emergencia |
| Indicador LED | 1 | Estado del sistema y alertas visuales | Señales luminosas del proceso |

---

### 🛠️Componentes Mecánicos

| Componente | Cantidad | Función | Exigencia Asociada |
|-----------|----------|---------|---------------------|
| Engranaje principal | 1 | Transmitir movimiento al sistema de apertura | Cinemática estable del engranaje |
| Barra móvil / brazo mecánico | 1 | Efectuar el movimiento de apertura | Transmisión directa del motor |
| Estructura de soporte | 1 | Montaje seguro sobre la ventana | Instalación rápida y segura |
| Soportes y anclajes | 3 | Estabilidad del módulo | Robustez y resistencia |

---

### 💻Componentes de Software

| Elemento | Función | Exigencia Asociada |
|----------|---------|---------------------|
| Firmware del microcontrolador | Control de sensores, motor y lógica | Gestión del proceso en tres modos |
| Aplicación móvil / Dashboard | Alertas, datos en tiempo real e historial | Comunicación con el usuario |
| Protocolo IoT (WiFi/Bluetooth) | Envío de notificaciones y monitoreo | Comunicación remota del sistema |

---

## ✨Conclusiones y Trabajo Futuro

SmartVent cumple todas las exigencias técnicas: control, seguridad, ergonomía, energía, señales, mecánica, software e instalación.

El sistema es seguro, silencioso, resistente y accesible.

La modularidad permite mantenimiento sencillo y reemplazo de piezas.

Futuras mejoras pueden incluir:

- Panel solar integrado,

- Más sensores,

- Integración con domótica,

- Versión industrial.



