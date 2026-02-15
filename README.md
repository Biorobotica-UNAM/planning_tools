# 🧩 planning_tools

## 📌 Descripción

`planning_tools` es un módulo cliente en C++ que actúa como interfaz de alto nivel para el sistema de planificación simbólica basado en ROS 2 + CLIPS.

Este paquete no implementa planificación directamente, sino que:

- Orquesta llamadas a servicios del planificación
- Encapsula la comunicación con:
  - Parser semántico
  - Conversor a primitivas
  - Conversor a CLIPS
  - Motor CLIPS
  - Plan Manager
- Proporciona una API limpia para nodos principales

Permite que un nodo superior controle todo el flujo de planificación mediante simples llamadas a métodos C++.

---

# 🎯 Propósito

Este módulo permite a cualquier nodo:

1. Enviar una instrucción en lenguaje natural
2. Convertirla en representación semántica
3. Transformarla en primitivas
4. Convertirlas en hechos CLIPS
5. Iniciar planificación
6. Obtener instrucciones paso a paso

Todo mediante funciones simples y desacopladas.

---

# 🏗 Arquitectura dentro del sistema

Nodo Principal
→
planning_tools (cliente C++)
→
semantic_parser
→
primitive_to_clips
→
clips_node
→
plan_manager_node

# 🧠 Clase Principal

## `Planning`

Constructor:

```
Planning(const rclcpp::Node::SharedPtr &node);
```

# Recibe un nodo ROS 2 para crear clientes de servicio.

| Servicio                                    | Tipo                                  | Función                        |
| ------------------------------------------- | ------------------------------------- | ------------------------------ |
| `/planning/semantic_parser/semantic_parser` | semantic_parser_interfaces/srv/Parser | Parseo semántico               |
| `/hri/listener/natural_to_clips/interpret`  | clips_interfaces/srv/String           | Interpretación directa a CLIPS |
| `/planning/primitive_to_clips/convert`      | clips_interfaces/srv/String           | Conversión de primitivas       |
| `/planning/clips/start_planning`            | std_srvs/Trigger                      | Inicia CLIPS                   |
| `/planning/request_instruction`             | std_srvs/Trigger                      | Solicita siguiente paso        |
---
## 📁 Estructura

```
planning_tools/
├── include/
│   └── planning_tools/
│       └── Planning.hpp
├── src/
│   └── Planning.cpp
├── CMakeLists.txt
└── package.xml
```
## 1️⃣ primitiveToClips
```
std::string primitiveToClips(const std::string &sentence);
```
Envía primitivas para convertirlas en hechos (goal ...).
---
## 2️⃣ parseSemanticCommand
```
std::string parseSemanticCommand(const std::string &input);
```
Enviar texto directo o enviar un archivo .txt con el comando
---
```
#include "planning_tools/Planning.hpp"

auto planning = std::make_shared<Planning>(node);

std::vector<std::string> planning_result;
std::string primitive;

// Interpretar lenguaje natural
planning->parseSemanticCommand("Go to the kitchen");

// Convertir primitivas a CLIPS
primitive = planning->primitiveToClips(primitive);

// Solicitar siguiente instrucción
planning_result = planning->requestInstruction();
```

## ✅ Requisitos en package.xml
```
<depend>rclcpp</depend>
<depend>std_srvs</depend>
<depend>clips_interfaces</depend>
<depend>semantic_parser_interfaces</depend>
```