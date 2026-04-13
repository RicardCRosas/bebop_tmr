# 🏆 ROS Mission Orchestrator - TMR 2026

Este módulo permite combinar fácilmente las misiones individuales para ejecutarlas en conjunto durante el Torneo Mexicano de Robótica (TMR) 2026, respetando las evaluaciones del reglamento. 

## 📌 Contexto
Según el reglamento de la categoría *Drones Autónomos* (basado en IMAV 2025 sin la prueba de humo en la Misión 4), es posible que las pruebas se hagan por separado o encadenadas. Dado que ROS restringe la inicialización de nodos con el mismo nombre y mantiene los bucles vivos con `rospy.Rate`, el **Orchestador** encapsula este proceso lanzando las misiones una a una como subprocesos independientes, monitoreando su status por el tópico `/mission/status`.

### Misiones Disponibles:
1. **Misión 1**: Entrar al cuarto (Ventana/Túnel) -> `mission_orange_window.py`
2. **Misión 2**: Dibujar en el pizarrón blanco -> `mission_whiteboard_aruco.py`
3. **Misión 3**: Buscar sonido de auxilio y entregar kit médico -> *Stub disponible (`mission_medkit.py`)*
4. **Misión 4**: Aterrizaje en plataforma móvil -> *Stub disponible (`mission_platform_landing.py`)*

## 🚀 ¿Cómo usar las combinaciones?

Puedes correr las misiones usando el script `competition_orchestrator.py` indicando mediante comandos qué misiones quieres encadenar usando el argumento `--combo`.

### Ejecutar todas las misiones (1 a 4 seq):
```bash
python3 competition_orchestrator.py --combo 1,2,3,4
```

### Ejecutar sólo misiones 1 y 2 (Módulo de navegación y Dibujo):
```bash
python3 competition_orchestrator.py --combo 1,2
```

### Ejecutar misiones en otro orden:
```bash
python3 competition_orchestrator.py --combo 2,1
```

## ⚙️ ¿Cómo funciona?

1. El orquestador lee los parámetros y la secuencia.
2. Arranca `mission_orange_window.py`. Éste inicializa su nodo de ROS y su bucle.
3. El orquestador se suscribe al tópico `/mission/status`.
4. Una vez la `mission_orange_window.py` finaliza su lógica y publica `"done"` (o `"failed"`), el orquestador mata su proceso limpiando los recursos de cámara.
5. Inicia e instancia subprocesos para la siguiente misión en secuencia, levantando su respectivo nodo dinámicamente.

(*Reglamento incluido en este mismo subdirectorio*).
