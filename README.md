# Planning tools: integración de Qwen, CLIPS y ejecución

## Panorama general

`planning_tools` conecta la interpretación de comandos con la planificación
simbólica y la consulta de instrucciones. Proporciona una biblioteca C++
(`Planning`), un launch de integración y un script de prueba del flujo completo.
No es un nodo planificador ni ejecuta por sí mismo las acciones del robot.

El flujo actual utiliza metas JSON, después de retirar los conversores de
primitivas CD y el parser spaCy. La documentación específica está en los
repositorios de [semantic_parser](https://github.com/Biorobotica-UNAM/semantic_parser)
y [CLIPS](https://github.com/Biorobotica-UNAM/clips).

```text
Comando escrito o escuchado
  ↓ semantic_parser / Qwen remoto
JSON {input, goals}
  ↓ goals_to_clips_node
Hechos gpsr-goal
  ↓ clips_node / goals_planning.clp
Plan completo en /planning/clips/results
  ↓ plan_manager_node, con remapeos
Instrucciones una por una
  ↓ nodo coordinador / executor
Acciones del robot
  ↓ durante acciones listen
entity_extractor_node → datos reconocidos → actualización de conocimiento
```

## Responsabilidades de cada componente

| Componente | Responsabilidad |
|---|---|
| `Planning` | Encapsula llamadas ROS desde un nodo C++ existente |
| `qwen_semantic_node` | Envía comandos al servidor Qwen y devuelve metas JSON |
| `goals_to_clips_node` | Normaliza metas, crea hechos, reinicia la memoria de CLIPS e inicia planificación |
| `clips_node` | Serializa el acceso al motor, carga reglas y publica el plan |
| `goals_planning.clp` | Expande cada meta en acciones concretas conservando su orden |
| `plan_manager_node` | Almacena el plan y entrega la siguiente instrucción al solicitarla |
| Nodo coordinador / executor | Solicita instrucciones y controla la ejecución de acciones |
| `entity_extractor_node` | Extrae valores de respuestas habladas mediante catálogos YAML |

Generar un plan y ejecutar sus acciones son etapas separadas. El extractor no
interviene entre Qwen y CLIPS: complementa las acciones `listen` del executor.

## Ciclo de planificación

1. El cliente llama a `parseSemanticCommand()` con texto o un archivo `.txt`,
   o a `listenGpsrCommand()` para escuchar un comando nuevo.
2. Qwen devuelve un string JSON con `input` y una lista `goals`.
3. El cliente valida que la respuesta contenga metas y llama a `goalsToClips()`.
4. El conversor llama a `clips/assert_facts_batch` y `clips/start_planning`.
5. CLIPS publica el plan completo. El gestor lo recibe si está iniciado y
   conectado al topic mediante los remapeos necesarios.
6. El coordinador solicita instrucciones y las envía al executor.

Ejemplo ilustrativo de respuesta semántica:

```json
{
  "input": "Go to the kitchen and find the waving person",
  "goals": ["go(kitchen)", "find(person, gesture=waving)"]
}
```

`goalsToClips()` inicia ya la planificación: no hace falta llamar después a
`startPlanning()`. Su respuesta `OK` indica aceptación de la conversión y del
inicio; el plan se observa por separado en el topic de resultados.
El parser puede devolver `[]` o un JSON `status=aborted`; una respuesta ROS
no vacía no garantiza que se hayan generado metas válidas.

## Biblioteca C++ Planning

Se construye con el nodo del consumidor:

```cpp
#include "planning_tools/Planning.hpp"

auto planning = std::make_shared<Planning>(node);
```

La clase crea los clientes y un `ReentrantCallbackGroup`. Sus métodos son
síncronos: envían una petición asíncrona y esperan el future. El nodo consumidor
debe estar siendo procesado por un executor mientras espera las respuestas.
Si se llaman desde callbacks bloqueantes, usar un `MultiThreadedExecutor`
con hilos disponibles; el callback group por sí solo no procesa respuestas.

| Método | Timeout predeterminado | Resultado y comportamiento |
|---|---:|---|
| `parseSemanticCommand(input, timeout)` | 10 s | String JSON devuelto por Qwen; vacío si falla la llamada o el comando está vacío |
| `listenGpsrCommand(timeout)` | 45 s | JSON de metas o de aborto después de listener + Qwen |
| `goalsToClips(goals_json, timeout)` | 5 s | Respuesta del conversor, normalmente `OK`; puede contener error o estar vacía |
| `startPlanning(timeout)` | 5 s | `bool` del servicio de inicio; útil cuando los hechos ya fueron insertados por otro cliente |
| `requestInstruction(timeout)` | 5 s por intento | Vector de tokens de la siguiente instrucción; reintenta hasta éxito o cierre de ROS |
| `extractEntity(sentence, category, timeout)` | 5 s | Vector con `value` o con `values`; vacío si no hay resultado o falla la llamada |

La espera de disponibilidad del servicio es de un segundo por llamada, aparte
del timeout de respuesta. La API usa nombres absolutos: cambiar el namespace
del nodo consumidor no modifica los destinos de los clientes.

`parseSemanticCommand()` lee el contenido completo cuando `input` apunta a un
archivo existente con extensión `.txt`; en otro caso envía el argumento como
texto. El JSON recibido se devuelve sin validación semántica. El consumidor
debe comprobarlo antes de enviarlo a CLIPS.

`requestInstruction()` reintenta con pausas de 300 ms si el servicio falla,
no hay plan o la respuesta no contiene una instrucción válida. Su timeout no
limita la duración total del método. Divide la instrucción por espacios:
`Justina go_to kitchen` produce `{"Justina", "go_to", "kitchen"}`.

Los comentarios y el log `Semantic CD` que todavía aparecen en la API son
nombres heredados: en la configuración actual el contenido es JSON de metas.

## Servicios y topic de resultados

| Nombre integrado | Tipo | Cliente o uso |
|---|---|---|
| `/planning/semantic_parser/semantic_parser` | `semantic_parser_interfaces/srv/Parser` | `parseSemanticCommand` |
| `/planning/semantic_parser/listen_gpsr` | `semantic_parser_interfaces/srv/Parser` | `listenGpsrCommand` |
| `/planning/goals_to_clips/convert` | `clips_interfaces/srv/String` | `goalsToClips` |
| `/planning/clips/start_planning` | `std_srvs/srv/Trigger` | `startPlanning` |
| `/planning/request_instruction` | `std_srvs/srv/Trigger` | `requestInstruction` |
| `/planning/semantic_parser/extract_entity` | `semantic_parser_interfaces/srv/ExtractEntity` | `extractEntity` |
| `/planning/clips/results` | `std_msgs/msg/String` | Plan completo; lo observa el script de prueba |

`Parser` recibe `command` y devuelve `response`. El conversor CLIPS recibe y
devuelve `data`. Esos campos contienen strings, incluyendo JSON serializado.
El servicio de entidades espera `sentence` y `category` y devuelve un string
`value` o una lista `values`.

## Launch de integración

`launch/planning.launch.xml` añade el namespace `/planning` e incluye los
launches de CLIPS y del parser. Inicia:

- `clips_node`.
- `goals_to_clips_node`, si `use_goals_to_clips=true`.
- `qwen_semantic_node`, único proveedor del servicio de interpretación.
- `entity_extractor_node`, si `use_entity_extractor=true`.

El launch no inicia el servidor HTTP Qwen, el listener de voz, el gestor de
planes ni el executor. Esos componentes requieren despliegue independiente.

| Argumento | Valor predeterminado | Función |
|---|---|---|
| `plan_name` | `goals` | Selección de reglas CLIPS |
| `qwen_url` | vacío | URL base; se recupera de `QWEN_SERVER_URL` o se usa `http://127.0.0.1:8008` |
| `qwen_api_key` | vacío | Clave; se recupera de `QWEN_API_KEY` si no se proporciona |
| `qwen_timeout_sec` | `5.0` | Timeout HTTP transmitido al parser |
| `qwen_response_format` | `goals` | Único formato admitido tras retirar CD |
| `use_entity_extractor` | `true` | Activa el extractor YAML |
| `use_goals_to_clips` | `true` | Activa el conversor de metas |
| `clips_output` | `log` | Destino de logs de los nodos CLIPS |
| `gpsr_listen_service_name` | `listen_gpsr` | Nombre relativo del servicio que escucha un comando |
| `gpsr_listener_service` | `/hri/listener/listen` | Servicio externo de escucha |
| `gpsr_listener_grammar` | vacío | Gramática enviada al listener |
| `gpsr_listen_timeout_sec` | `30.0` | Tiempo de espera de escucha |

Cambiar `gpsr_listen_service_name` también requiere remapear el cliente C++,
que conserva `/planning/semantic_parser/listen_gpsr` como destino.
Los planes fijos siguen disponibles mediante `plan_name`, pero el flujo de
metas documentado utiliza `goals`.

## Conectar el gestor de instrucciones

El gestor incluido en CLIPS utiliza nombres absolutos históricos:
`/clips/results`, `/request_instruction` y `/instruction_msg`.
La biblioteca `Planning` solicita `/planning/request_instruction`, y el launch
integrado publica `/planning/clips/results`. Para conectarlos:

```bash
ros2 run plan_manager_node plan_manager_node --ros-args \
  -r /clips/results:=/planning/clips/results \
  -r /request_instruction:=/planning/request_instruction \
  -r /instruction_msg:=/planning/instruction_msg
```

Asignar solamente un namespace no cambia los nombres absolutos del gestor.
Una llamada directa permite obtener la siguiente instrucción:

```bash
ros2 service call /planning/request_instruction std_srvs/srv/Trigger '{}'
```

El gestor elimina `Paso N:` de cada línea del plan, devuelve una instrucción
y avanza su contador. Al entregar la última, deja de tener un plan activo;
posteriormente responde `success=false`, `message=NoPlanAssigned`.
Entregar la última instrucción no significa que el robot ya la haya ejecutado.

## Extracción durante acciones listen

`executor_tools/Listen.cpp` escucha una respuesta mediante HRI y llama a
`Planning::extractEntity()` con una categoría. El extractor busca términos
conocidos en YAML, devuelve valores normalizados y el executor puede actualizar
el conocimiento de la persona según el modo de la acción.

Por ejemplo, `NAME` reconoce un nombre registrado; otras categorías pueden
devolver varios valores. La biblioteca convierte ambos formatos (`value` o
`values`) a `std::vector<std::string>`. Las categorías disponibles dependen de
los catálogos, no de Qwen ni de las reglas CLIPS.

## Compilación y requisitos pendientes

La biblioteca requiere C++17, `rclcpp`, `std_srvs`, `clips_interfaces` y
`semantic_parser_interfaces`. Los launches requieren además los paquetes
`clips_launch` y `semantic_parser`; el script utiliza Python, `rclpy` y
`std_msgs`. El motor requiere Clipspy y el parser Qwen utiliza `httpx`.

En este checkout falta `ExtractEntity.srv`: `semantic_parser_interfaces` solo
genera `Parser.srv`, mientras `Planning.hpp` incluye `extract_entity.hpp`.
Por eso **la biblioteca planning_tools no compila todavía**, incluso si se
desactiva el extractor en el launch. También faltan los catálogos
`<share de knowledge_bank>/dictionaries/*.yaml` para ejecutar el extractor.

Una vez resuelta la interfaz, compilar desde la raíz del workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to \
  planning_tools semantic_parser clips_launch plan_manager_node
source install/setup.bash
```

Los requisitos de runtime de los launches y del script no están todos declarados
en `package.xml` de `planning_tools`; deben estar disponibles en el workspace.
Los detalles del servidor y de los YAML se describen en el
[README de semantic_parser](https://github.com/Biorobotica-UNAM/semantic_parser).

## Prueba completa de creación de planes con qwen_to_plan.py

`scripts/qwen_to_plan.py` permite probar todo el proceso de creación de un
plan con un único comando en lenguaje natural. Coordina las llamadas al parser
y al conversor de CLIPS y muestra el plan generado en la terminal.
Para esta prueba no se necesita iniciar el gestor de instrucciones ni el
executor: el resultado final es el plan completo publicado por CLIPS.

### 1. Tener activo el servidor Qwen

El servidor HTTP Qwen debe estar iniciado y accesible desde el equipo donde
corre `qwen_semantic_node`. Debe ofrecer `POST /translate` con el contrato
descrito en el [repositorio de semantic_parser](https://github.com/Biorobotica-UNAM/semantic_parser).
El launch ROS conecta con ese servidor; no lo inicia.

Configurar `QWEN_SERVER_URL` y, si el servidor requiere autenticación,
`QWEN_API_KEY`, o pasar `qwen_url` al launch. La URL predeterminada es
`http://127.0.0.1:8008` cuando no se proporciona otra.

### 2. Mantener activo el launch de planificación

En una terminal con los paquetes compilados y sus requisitos resueltos:

```bash
cd /home/sergio/ros2_clips
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch planning_tools planning.launch.xml plan_name:=goals
```

Mantener esa terminal abierta mientras se realizan las pruebas. El launch
inicia Qwen como servicio ROS, el conversor de metas y el motor CLIPS con
`goals_planning.clp`.

Si el servidor está en otra dirección, indicarla explícitamente:

```bash
ros2 launch planning_tools planning.launch.xml \
  plan_name:=goals \
  qwen_url:="http://DIRECCION_DEL_SERVIDOR:8008" \
  clips_output:=screen
```

En el checkout actual, mientras falten la interfaz y los catálogos del
extractor descritos arriba, añadir `use_entity_extractor:=false`. El extractor
no es necesario para crear el plan:

```bash
ros2 launch planning_tools planning.launch.xml \
  plan_name:=goals use_entity_extractor:=false
```

Desactivarlo no resuelve la dependencia de compilación de la biblioteca C++
con `ExtractEntity`; esa interfaz sigue siendo necesaria para compilar
`planning_tools` completo.

### 3. Enviar un comando desde otra terminal

Cargar el mismo entorno ROS y ejecutar el script con el comando entre comillas
para que se reciba como un único argumento:

```bash
cd /home/sergio/ros2_clips
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 planning_tools/scripts/qwen_to_plan.py \
  "Go to the living room, find the waving person, and guide them to the inspection point"
```

Si el paquete está instalado, la invocación equivalente es:

```bash
ros2 run planning_tools qwen_to_plan.py \
  "Go to the living room, find the waving person, and guide them to the inspection point"
```

La ejecución directa del script no importa `ExtractEntity` ni utiliza la
biblioteca C++, pero sí requiere las interfaces `Parser`, `clips_interfaces`
y los servicios activos del flujo de metas.

### 4. Qué hace el script automáticamente

Al arrancar crea el nodo `qwen_to_plan_tester`, dos clientes de servicio y una
suscripción al topic de planes. La suscripción se crea antes de solicitar la
planificación para poder recibir el resultado mientras se procesan las llamadas.

```text
Comando pasado como argumento
  ↓ Parser.Request.command
/planning/semantic_parser/semantic_parser
  ↓ Qwen remoto interpreta el comando
Parser.Response.response = JSON {input, goals}
  ↓ validación de JSON y lista goals
/planning/goals_to_clips/convert, con el JSON en Request.data
  ↓ conversor inserta hechos e inicia CLIPS
Respuesta del conversor, normalmente OK
  ↓ espera del mensaje de resultados
/planning/clips/results
  ↓
Plan completo mostrado en la terminal
```

El script no llama directamente al servidor HTTP: lo hace `qwen_semantic_node`.
Tampoco necesita llamadas manuales a `assert_facts_batch` o `start_planning`:
el servicio `goals_to_clips/convert` coordina ambas operaciones.

La validación del script comprueba que la respuesta sea un objeto JSON con
una lista `goals` en el nivel superior. La interpretación de esas metas, su
normalización y la generación de acciones ocurren después en el conversor
y en las reglas de CLIPS.

Durante las esperas utiliza `rclpy.spin_once()` para procesar tanto las
respuestas de servicios como los mensajes del topic. `_on_plan()` guarda el
último plan recibido y `_wait_for_plan()` lo devuelve cuando está disponible.
No hay un identificador que correlacione el comando con el mensaje: realizar
esta prueba sin otras solicitudes concurrentes de planificación.

### 5. Interpretar la salida

La terminal muestra tres bloques, conforme se completa cada etapa:

```text
=== Qwen response ===
{"input": "...", "goals": ["go(living room)", "find(person, gesture=waving)", "guide(person, to=inspection point)"]}

=== goals_to_clips response ===
OK

=== Generated plan ===
==Justina GPSR_START==
Paso 1: Justina say ...
Paso 1001: Justina go_to living_room
...
==Justina GPSR_DONE==
```

Esta salida es ilustrativa: las metas exactas dependen del servidor Qwen y
los pasos del plan dependen de las reglas y del contexto simbólico.
El primer bloque permite revisar la interpretación del comando; el segundo,
la aceptación del conversor; el tercero, las acciones que generó CLIPS.
`OK` por sí solo no confirma que el script haya recibido un plan.

El script termina con código `0` cuando recibe un plan no vacío y `1` si falla
alguna etapa. No valida cada acción del resultado ni ejecuta el plan en el
robot. Para probar otro comando basta con volver a ejecutar el script mientras
el servidor y el launch siguen activos.

### Opciones y diagnóstico

| Opción del script | Predeterminado | Uso |
|---|---|---|
| `--parser-service` | `/planning/semantic_parser/semantic_parser` | Servicio ROS que consulta Qwen |
| `--goals-service` | `/planning/goals_to_clips/convert` | Servicio de conversión e inicio de planificación |
| `--results-topic` | `/planning/clips/results` | Topic del plan completo |
| `--service-timeout` | `10.0` s | Espera de disponibilidad y de respuesta, en etapas separadas |
| `--plan-timeout` | `5.0` s | Espera del plan después de obtener la respuesta del conversor |

Ejemplo con más tiempo de espera:

```bash
python3 planning_tools/scripts/qwen_to_plan.py \
  "Go to the kitchen and find the waving person" \
  --service-timeout 30 --plan-timeout 10
```

`--service-timeout` limita las esperas del script; el timeout HTTP de Qwen
se configura por separado con `qwen_timeout_sec` en el launch.

| Mensaje o situación | Qué revisar |
|---|---|
| `Parser service unavailable` | Launch activo, namespace y entorno ROS del script |
| `Timed out waiting for Qwen response` | Servidor Qwen, conexión y timeouts del parser y del script |
| Respuesta sin lista `goals` | JSON de error del parser y configuración `qwen_response_format=goals` |
| `Goals service unavailable` | Nodo conversor activo y `use_goals_to_clips=true` |
| Respuesta del conversor que empieza con `ERROR` | Formato de metas y logs del conversor |
| `Timed out waiting for plan` | Reglas `plan_name=goals`, logs de CLIPS y topic configurado |

Para observar la publicación por separado:

```bash
ros2 topic echo /planning/clips/results
```

## Estructura y estado actual

```text
planning_tools/
├── include/planning_tools/Planning.hpp  # API y helper de llamadas ROS
├── src/Planning.cpp                    # Implementación de los clientes
├── launch/planning.launch.xml          # Integración CLIPS + Qwen + extractor
├── scripts/qwen_to_plan.py             # Prueba de generación de planes
├── CMakeLists.txt
├── package.xml
└── README.md
```

Se retiraron `interpretToClips()`, `primitiveToClips()` y sus clientes de los
servicios antiguos. Los consumidores deben usar metas JSON y `goalsToClips()`.
Los tres componentes funcionales del paquete (biblioteca, launch y script)
tienen un propósito en el flujo actual y se conservan.
