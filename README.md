# Drone Assignment Simulator

A comprehensive, type-safe autonomous drone simulation framework built with Python and SimPy for discrete event simulation of multi-drone operations.

## 🚁 Overview

This simulator provides a realistic and extensible platform for modeling autonomous drone fleets with energy-aware flight planning, geographic positioning, and mission coordination. Built on discrete event simulation principles, it supports scalable multi-drone scenarios from single vehicle operations to large fleet coordination.

## ✨ Key Features

### 🔧 Core Simulation Engine

- **SimPy-Based**: Discrete event simulation with precise timing control
- **Scalable Architecture**: Support for hundreds of concurrent vehicles
- **Type-Safe Design**: Comprehensive type system preventing unit mixing errors
- **Performance Optimized**: Memory-efficient design for large-scale simulations

### 🌍 Geographic Accuracy

- **WGS84 Coordinate System**: Real-world geographic positioning
- **Geodesic Calculations**: Accurate Earth-surface distance and bearing computations
- **High Precision**: Suitable for navigation and surveying applications

### ⚡ Energy Management

- **Realistic Battery Models**: State-of-charge tracking with degradation
- **Energy-Aware Planning**: Flight time estimation and range optimization
- **Multiple Battery Types**: Support for various capacity and chemistry models

### 🎯 Mission Framework

- **Abstract Task System**: Extensible mission and task definitions
- **State Machine Control**: Robust task progression with error handling
- **Multi-Waypoint Support**: Complex route planning and execution

## 📦 Package Structure

```
app/
├── vehicles/          # Vehicle simulation framework
│   ├── vehicle.py     # Abstract base class with 3-phase execution model
│   └── drone.py       # Concrete drone implementation
├── unit/              # Type-safe unit system
│   ├── unit_base.py   # Foundation classes and family management
│   ├── unit_float.py  # Float-based units with SI conversion
│   ├── unit_angle.py  # Angular measurements (degrees, radians)
│   ├── unit_distance.py # Distance units (meters, kilometers)
│   ├── unit_time.py   # Time units (seconds, minutes, hours)
│   └── unit_velocity.py # Velocity units (m/s, km/h)
├── geo/               # Geographic coordinate system
│   └── geo_point.py   # WGS84 positioning with geodesic operations
├── energy/            # Battery and energy management
│   ├── battery.py     # Battery status and state management
│   └── unit.py        # Energy units (Wh, kWh, Joules)
├── mission/           # Task and mission framework
│   ├── task.py        # Abstract task base class
│   └── task_delivery.py # Concrete delivery mission implementation
└── main.py            # Simulation entry point
```

## 🚀 Quick Start

### Installation

```bash
git clone https://github.com/sangaje/Drone_Assignment_Simulator.git
cd Drone_Assignment_Simulator
pip install -r requirements.txt
```

### Basic Usage

```python
import simpy
from app.vehicles import Drone
from app.geo import GeoPoint
from app.energy.battery import BatteryStatus
from app.energy.unit import WattHour

# Create simulation environment
env = simpy.Environment()

# Set up drone with battery and position
start_pos = GeoPoint.from_deg(37.5665, 126.9780)  # Seoul
battery = BatteryStatus(WattHour(1000), WattHour(800))  # 1000Wh, 80% charged
drone = Drone(env, start_pos, battery)

# Run simulation
env.run(until=3600)  # Run for 1 hour simulation time
```

## 🔬 Advanced Features

### Three-Phase Execution Model

The simulation uses a sophisticated three-phase update cycle:

1. **Primary Update**: Core vehicle behaviors and operations
2. **External Processing**: Task assignments and inter-vehicle coordination
3. **Post-Processing**: Cleanup, validation, and metrics collection

### Type-Safe Unit System

Comprehensive unit management with automatic SI conversion:

```python
from app.unit import Kilometer, Degree, Hour

distance = Kilometer(5.2)  # Automatically converts to meters internally
heading = Degree(45)       # Converts to radians for calculations
duration = Hour(2.5)       # Converts to seconds for timing
```

### Energy-Aware Operations

Realistic battery modeling with consumption tracking:

```python
# Battery drains based on distance, payload, and environmental factors
if not drone.battery.consume_energy(flight_energy):
    # Handle insufficient battery scenario
    drone.return_to_base()
```

## 📊 Use Cases

- **Fleet Management**: Multi-drone coordination and optimization
- **Mission Planning**: Route optimization with energy constraints
- **Research**: Algorithm development and performance analysis
- **Training**: Educational simulations for autonomous systems
- **Validation**: System testing and scenario evaluation

## 🛠️ Development

### Architecture Patterns

- **Template Method**: Structured vehicle execution phases
- **Abstract Factory**: Extensible vehicle and task types
- **State Machine**: Robust mission state management
- **Observer Pattern**: Event-driven simulation coordination

### Testing

```bash
# Run test suite
python -m pytest tests/

# Run specific test category
python -m pytest tests/test_vehicles.py
```

## 📚 Documentation

Comprehensive docstrings are provided throughout the codebase using Google-style formatting:

- **Module Documentation**: Complete package and module descriptions
- **Class Documentation**: Detailed class behavior and usage patterns
- **Method Documentation**: Full parameter and return value specifications
- **Example Code**: Practical usage examples and integration patterns

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch
3. Add comprehensive tests
4. Update documentation
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📧 Contact

For questions, suggestions, or contributions, please open an issue or contact the maintainers.

---

Built with ❤️ for autonomous systems research and education.
