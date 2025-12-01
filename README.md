# Equipo 02 - Fundamentos de Diseño 2025-2  
### Carrera de Ingeniería Informática / Industrial  
**Universidad Peruana Cayetano Heredia**


<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/c4207bbe-a760-44e4-b0a5-8a2c6042af49" />


## 📑 Índice

1. [📘 Presentación del Equipo](#-presentación-del-equipo)
2. [⭐ ODS Prioritario](#-ods-prioritario)
3. [📸 Fotografía del Equipo](#-fotografía-del-equipo)
4. [👥 Integrantes del Equipo](#-integrantes-del-equipo)
5. [📝 Problemática en el Perú](#-problemática-en-el-perú)
6. [💡 Propuesta de Solución – SmartVent](#-propuesta-de-solución--smartvent)
7. [⚙️ Funciones del Sistema](#️-funciones-del-sistema)
8. [📦 Módulos del Proyecto](#-módulos-del-proyecto)
    - [🔌 Módulo Electrónico](#-módulo-electrónico)
    - [🛠️ Módulo Mecánico](#️-módulo-mecánico)
    - [💻 Módulo de Software](#-módulo-de-software)
9. [📋 Componentes Utilizados](#-componentes-utilizados)
10. [📁 Entregables del Proyecto](#-entregables-del-proyecto)
11. [🧪 Proceso de Diseño](#-proceso-de-diseño)
12. [📂 Estructura del Repositorio](#-estructura-del-repositorio)
13. [✨ Conclusiones y Trabajo Futuro](#-conclusiones-y-trabajo-futuro)

## 📘 Presentación del Equipo

Somos el Equipo 02 del curso Fundamentos de Diseño 2025-2, conformado por estudiantes comprometidos con el desarrollo de soluciones innovadoras con impacto social, tecnológico y ambiental.

Nuestro trabajo se alinea con los Objetivos de Desarrollo Sostenible (ODS), especialmente los relacionados con ciudades sostenibles y resilientes.

## ⭐ ODS Prioritario

**ODS 11 – Ciudades y comunidades sostenibles**

Buscamos aportar a la construcción de ciudades más seguras, saludables y resilientes mediante tecnologías accesibles y adaptables a la realidad urbana.


## 📸 Fotografía del Equipo  
<p align="center">

<img width="1050" height="800" alt="image" src="https://github.com/user-attachments/assets/f30bdf4c-205f-44b1-9b0d-06e9b3b7deda" /> <br>
  <em>Figura 1. Fotografía del equipo 2</em>
</p>


## 👥 Integrantes del Equipo  

| Foto | Nombre | Rol | Intereses |
|------|--------|-----|-----------|
| <img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/b80a23aa-58e7-47dd-81fd-aa661d1a2c28" /> | **Idania Channara Parhuay Meza** | Líder del equipo | Innovación social, sostenibilidad |
| <img width="300" height="800" alt="image" src="https://github.com/user-attachments/assets/4dcac393-f906-4e7a-ae3c-b5c699ce6ed2" /> | **Eber Pauccara Huancara** | Diseñador/a | Diseño de prototipos, creatividad aplicada |
| <img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/114daca6-934d-404e-8a1e-c031229eef30" /> | **Cristofer Andherson Quina Pumahuillca** | Encargado/a de documentación | Comunicación científica, redacción técnica |
|<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/7b9d1414-1e5d-4bdc-a8f5-bdd2beb63d2c" />| **Sebastian Calderon Junes** | Programador/a - Modelador/a | Programación, análisis de datos, simulación |

---

## 📌 Resumen Final  
# ODS 11: Ciudades y comunidades sostenibles
Este objetivo busca que las ciudades sean inclusivas, seguras, resilientes y sostenibles, ya que concentran gran parte de la población mundial. En 2022 se alcanzaron 8000 millones de habitantes, más de la mitad en zonas urbanas, y se estima que en 2050 llegue al 70 %. Actualmente, 1100 millones viven en barrios marginales y podrían sumarse 2000 millones más en 30 años, lo que genera déficit de vivienda, infraestructura y servicios, además de contaminación y falta de espacios públicos. Aunque aumentaron las estrategias contra desastres, solo la mitad de la población urbana accedió al transporte público en 2022, evidenciando la necesidad de transformar la planificación y gestión urbana.

## Metas:
-Meta 11.5: “De aquí a 2030, reducir significativamente el número de muertes y personas afectadas por los desastres, incluidos los relacionados con el agua, y disminuir considerablemente las pérdidas económicas directas ocasionadas por ellos, haciendo especial hincapié en la protección de los pobres y las personas en situaciones de vulnerabilidad.”
-Meta 11.6: “De aquí a 2030, reducir el impacto ambiental negativo per cápita de las ciudades, prestando especial atención a la calidad del aire y a la gestión de l-los desechos municipales y de otro tipo.”

<img width="1500" height="1500" alt="image" src="https://github.com/user-attachments/assets/5873048b-feb3-492f-a972-7676ab07f3e1" />

---

## PROBLEMÁTICA EN EL PERÚ

En Lima se viven dos problemáticas que afectan directamente la salud y seguridad de las personas: la calidad del aire en interiores y la alta vulnerabilidad frente a incendios. Pasamos gran parte del día en espacios cerrados como colegios, oficinas o viviendas, donde el aire no siempre es saludable. La acumulación de dióxido de carbono (CO₂), humedad y otros compuestos deteriora la concentración y el bienestar. A esto se suma la contaminación exterior, en especial las partículas finas (PM2.5), que en Lima superan hasta tres veces los valores recomendados por la OMS (IQAir, 2024). Muchas veces abrir las ventanas para ventilar no resuelve el problema, ya que puede significar dejar entrar más contaminación, lo que genera un dilema cotidiano.

Por otro lado, los incendios representan una amenaza constante en la capital. Según reportes del Cuerpo General de Bomberos y del CENEPRED, Lima concentra la mayor cantidad de siniestros urbanos en el país (INDECI, 2022). En estos casos, la mayoría de muertes no se deben a las llamas, sino a la inhalación de humo y gases tóxicos, que llenan rápidamente los ambientes, reducen la visibilidad y provocan asfixia en pocos minutos (NFPA, 2019).

Esta situación es todavía más crítica en distritos con alta densidad poblacional y bajos recursos económicos, donde muchas familias carecen de sistemas de ventilación adecuados o dispositivos de seguridad accesibles. Los grupos más vulnerables, adultos mayores, niños y personas con movilidad reducida, son quienes enfrentan mayores riesgos en este contexto.

En conclusión, Lima enfrenta un doble desafío: mejorar la calidad del aire en interiores, considerando tanto el CO₂ como la exposición a contaminantes exteriores como el PM2.5, y al mismo tiempo fortalecer la seguridad frente a incendios. Esto evidencia la necesidad de soluciones tecnológicas accesibles, modulares y eficientes que combinen ventilación inteligente con protocolos de emergencia, adaptadas a la realidad urbana de la ciudad.


<img width="640" height="360" alt="image" src="https://github.com/user-attachments/assets/be751f8b-8dd3-478c-b8e9-73a5fbc2b129" />

---

# SmartVent – Respira tranquilo, vive protegido

<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/fe8d0138-987b-4a36-a371-daf679c65595" />

---

## Propuesta de solución

## Ventana Inteligente Retrofit
Es un módulo adaptable a cualquier ventana existente, que integra sensores, actuadores y un sistema de desbloqueo eléctrico. Permite mejorar la calidad del aire interior y actúa como medida de seguridad en emergencias como incendios y salidas en caso de emergencias.

## ¿Cuál será su uso?
En la vida diaria: abrir o cerrar la ventana automáticamente según la calidad del aire interior (CO₂) y la contaminación exterior (PM2.5).
**En emergencias:** desbloquear y abrir la ventana para liberar humo y facilitar la evacuación, además de activar alarmas y notificaciones.

## Objetivo:
Diseñar un sistema económico, modular y escalable que mejore la ventilación en colegios, oficinas y viviendas, y que incremente la seguridad de las personas en caso de incendios o emergencias.

## Funciones que cumplirá:
-Medición ambiental: sensores de CO₂, partículas y humo.
-Ventilación automática: apertura/cierre según parámetros de calidad de aire.
-Respuesta a incendios: desbloqueo eléctrico + apertura total + alarma sonora y visual.
-Notificaciones: envío de alertas vía WiFi/Bluetooth a usuarios o responsables.
-Autonomía: alimentación por corriente y batería de respaldo, con opción solar.




