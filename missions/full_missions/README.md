# 🏆 Competition Orchestrator — TMR 2026 (Drones Autónomos)

Orquestador de misiones para el **Torneo Mexicano de Robótica 2026**, categoría **Drones Autónomos**.  
Basado en el rulebook oficial del **IMAV 2025** con la adaptación TMR 2026 (**sin humo en Misión 4**).

> **Referencia:** [Rulebook IMAV 2025](https://femexrobotica.org/imav2025/wp-content/uploads/2025/11/RuleBook_IMAV2025.pdf)

---

## 📋 Misiones

| # | Misión | Script | Estado |
|---|--------|--------|--------|
| 1 | **Enter the Room** — Navegar a través de un túnel/ventana | `mission_orange_window.py` | ✅ Implementada |
| - | **Point to Point** — Navegación libre entre zona 1 y 2 | `mission_point_to_point.py` | ✅ Implementada |
| 2 | **Draw on the Whiteboard** — Detectar ArUco y dibujar línea | `mission_whiteboard_aruco.py` | ✅ Implementada |
| 3 | **Find Cry for Help & Deliver Med-Kit** — Localizar caja con sonido y entregar kit | `mission_medkit.py` | ⚠️ Stub |
| 4 | **Land on Platform** — Aterrizar en plataforma (sin humo TMR) | `mission_4.py` | ✅ Implementada |

### Detalle de las misiones

#### Mission 1: Enter the Room
El MAV tiene 4 opciones de entrada a la arena de navegación (10m × 10m):
- **Free passage** (1m de ancho) → **0 pts**
- **Wide tunnel** (ventana 1.5m × 1.5m, base 0.5m) → **1 pt**
- **Medium tunnel** (ventana 1.0m × 1.0m, base 1.0m) → **2 pts**
- **Small tunnel** (ventana 0.5m × 0.5m, base 1.5m) → **3 pts**

Túneles de 2m de largo, 2m de alto, formados por 5 puertas cuadradas alineadas. Diámetro de tubo: ⌀ 38.1 mm.

#### Mission 2: Draw on the Whiteboard
- Pizarrón de **1.2m × 1.5m** colocado horizontalmente en la pared a **1m del suelo** (al centro).
- **ArUco marker (ID 100)** de 20cm × 20cm al borde izquierdo del pizarrón a 1.6m de altura.
- Dibujar una **línea horizontal continua** tocando el pizarrón.
- **3 pts** por línea mínima de 20cm, **+1 pt** por cada 20cm adicionales.

#### Mission 3: Find Cry for Help & Deliver Med-Kit
- Caja negra (57cm × 78cm × 36cm) en el suelo con bocina de auxilio (≥70 dB).
- Hasta **5 cajas** en posiciones aleatorias (solo una con sonido).
- Med-kit: caja de 20cm × 10cm × 5cm, peso 30g.
- **3 pts** por encontrar la caja, **+2 pts** por entregar el med-kit.

#### Mission 4: Land on a Moving Platform *(sin humo — TMR 2026)*
- Plataforma de **1m × 1m** que se desplaza lateralmente hasta 1m a 0.5 m/s.
- **⚠️ Adaptación TMR 2026:** No se usa humo.
- **2 pts** por aterrizaje en plataforma estática, **+3 pts** si la plataforma está en movimiento.

---

## 📊 Sistema de Puntaje

### Puntos por misión

| Misión | Acción | Puntos |
|--------|--------|--------|
| M1 | Free path | 0 |
| M1 | Wide tunnel (1.5m) | 1 |
| M1 | Medium tunnel (1.0m) | 2 |
| M1 | Small tunnel (0.5m) | 3 |
| M2 | Sin dibujo | 0 |
| M2 | Línea continua ≥ 20cm | 3 |
| M2 | Cada 20cm adicionales | +1 |
| M3 | Caja no encontrada | 0 |
| M3 | Caja encontrada | 3 |
| M3 | Med-kit entregado | 2 |
| M4 | Sin aterrizaje | 0 |
| M4 | Aterrizaje (estática) | 2 |
| M4 | Aterrizaje (en movimiento) | +3 |

### Multiplicadores

| Tipo | Multiplicador |
|------|---------------|
| Single MAV Onboard Processing | × 1.5 |
| Swarm Solution Offboard Processing | × 2.0 |
| Swarm Solution Onboard Processing | × 3.0 |

### Presentación
`Final Score + 3 × (0…10)` — promedio de jueces (eliminando el mayor y menor).

---

## 🚀 Uso

### Ejecutar secuencia completa de vuelo requerida (M1 -> Navegación -> M2 -> M4):
```bash
python3 competition_orchestrator.py --combo 1,p2p,2,4
```

### Ejecutar todas las misiones:
```bash
python3 competition_orchestrator.py --combo 1,2,3,4
```

### Solo misiones 1 y 2 con túnel medio:
```bash
python3 competition_orchestrator.py --combo 1,2 --tunnel medium
```

### Solo misión 4 con plataforma en movimiento:
```bash
python3 competition_orchestrator.py --combo 4 --platform moving
```

### Con multiplicador de procesamiento onboard:
```bash
python3 competition_orchestrator.py --combo 1,2,3,4 --multiplier onboard
```

### Modo pruebas (sin espera de preparación):
```bash
python3 competition_orchestrator.py --combo 1,2 --skip-prep
```

### Ver misiones disponibles:
```bash
python3 competition_orchestrator.py --list-missions
```

---

## ⚙️ Parámetros CLI

| Parámetro | Valores | Default | Descripción |
|-----------|---------|---------|-------------|
| `--combo` | `1,p2p,2,3,4` | `1,p2p,2,4` | Misiones a ejecutar (separadas por coma) |
| `--tunnel` | `free_path`, `wide`, `medium`, `small` | `medium` | Dificultad del túnel en Misión 1 |
| `--platform` | `static`, `moving` | `static` | Modo de plataforma en Misión 4 |
| `--multiplier` | `none`, `onboard`, `swarm_offboard`, `swarm_onboard` | `none` | Multiplicador de scoring |
| `--max-time` | segundos | `1200` (20 min) | Tiempo máximo de competencia |
| `--skip-prep` | flag | `false` | Saltar 2 minutos de preparación inicial |
| `--list-missions` | flag | — | Listar misiones y salir |

---

## 🔧 ¿Cómo funciona internamente?

```
┌───────────────────────────────────────────────────────────┐
│                   COMPETITION ORCHESTRATOR                │
│                                                           │
│  1. Fase de Preparación (2 min)                           │
│     └─ El equipo se instala en la estación de control     │
│                                                           │
│  2. ⏱️ Cronómetro inicia (15-20 min según reglamento)     │
│                                                           │
│  3. Para cada misión en la secuencia:                     │
│     ├─ Verificar tiempo restante                          │
│     ├─ Lanzar script como subprocess                      │
│     ├─ Monitorear /mission/status (done | failed)         │
│     ├─ Registrar puntaje automáticamente                  │
│     ├─ Cleanup del subprocess                             │
│     └─ Pausa inter-misión (para preparar dron)            │
│                                                           │
│  4. Resumen final de puntaje con multiplicadores          │
└───────────────────────────────────────────────────────────┘
```

### Comunicación con las misiones

Cada script de misión publica su estado en el tópico ROS `/mission/status`:
- `"done"` → Misión completada exitosamente
- `"failed"` → Misión falló

El orquestador se suscribe a este tópico y reacciona automáticamente.

---

## ⚠️ Reglas importantes del reglamento

1. **Vuelo 100% autónomo** — bajo ninguna circunstancia se permite pilotar el dron manualmente durante la misión.
2. **Despegue desde el suelo** — zona de despegue designada, igual para todos los equipos.
3. **Intervención solo para despegue/aterrizaje** — el "piloto" designado solo acciona controles para despegue y aterrizaje.
4. **Un dron a la vez** — se pueden tener varios, pero solo uno operando por misión.
5. **Baterías LiPo** — deben ser inspeccionadas por el juez antes de cada misión.
6. **Diámetro máximo del dron** — 60 cm (medido de punta a punta de hélice).
7. **Sin humo** — adaptación TMR 2026, no aplica humo en Misión 4.
8. **Aplazamiento único** — el equipo puede decidir una sola vez posponer el resto de su participación al final de la lista.

---

## 📁 Estructura del proyecto

```
bebop_tmr/
├── missions/
│   ├── mission_orange_window.py       # Mission 1: Enter the room
│   ├── mission_whiteboard_aruco.py    # Mission 2: Draw on whiteboard
│   ├── mission_medkit.py              # Mission 3: Find & deliver (stub)
│   ├── mission_4.py                   # Mission 4: Platform landing
│   ├── mission_helipad.py             # Alternativa de landing (legacy)
│   └── full_missions/
│       ├── competition_orchestrator.py # ← Este orquestador
│       └── README.md                  # ← Este archivo
├── control/
│   └── bebop_teleop_controller.py     # Módulo de movimientos del Bebop
├── perception/
│   ├── window_orange_detector.py      # Detector de ventana naranja (M1)
│   ├── aruco_whiteboard_detector.py   # Detector ArUco + pizarrón (M2)
│   └── helipad_detector.py            # Detector de helipad/plataforma (M4)
└── CategoriaDronesAutonomouos_2026-3.pdf  # Reglamento TMR 2026
```

---

## 🔮 TODO

- [ ] **Mission 3**: Implementar detección de sonido (micrófono + análisis frecuencial) y mecanismo de entrega del med-kit.
- [ ] **Mission 1**: Agregar soporte para distintos tamaños de túnel (actualmente calibrado para ventana naranja).
- [ ] **Mission 4**: Validar tracking de plataforma en movimiento (actual `mission_4.py` con HelipadDetector asume plataforma estática).
- [ ] **Scoring extra M2**: Integrar medición automática de longitud de línea dibujada.
- [ ] **Launch file**: Crear `.launch` con parámetros configurables para ROS.
