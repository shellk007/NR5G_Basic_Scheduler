# NexGen 5G NR Scheduler

<p align="center">
<strong>A Modern, High-Performance 5G NR MAC Scheduler</strong><br>
<em>Clean-room C++17 implementation with production-grade features</em>
</p>

---

## Features at a Glance

| Category | Features |
|----------|----------|
| **Scheduling** | Proportional Fair, Round Robin, Max Throughput, QoS-Aware |
| **QoS** | 5QI mapping, GBR enforcement, PDB tracking, MBR shaping |
| **Slicing** | eMBB, URLLC, mMTC templates with SLA enforcement |
| **HARQ** | 16 processes, adaptive retx, NDI/RV management |
| **Channels** | PDCCH (CCE alloc), PDSCH, PUSCH, PUCCH (all formats) |
| **Performance** | Sub-100μs slot processing, O(1) resource lookup |

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      NexGen Scheduler                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                  Scheduling Engine                         │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │ │
│  │  │   Policy    │  │  Resource   │  │   HARQ      │        │ │
│  │  │   Manager   │  │  Allocator  │  │  Controller │        │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘        │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                  Channel Managers                          │ │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐          │ │
│  │  │ PDCCH   │ │ PDSCH   │ │ PUSCH   │ │ PUCCH   │          │ │
│  │  │ Manager │ │ Manager │ │ Manager │ │ Manager │          │ │
│  │  └─────────┘ └─────────┘ └─────────┘ └─────────┘          │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │                  Core Services                             │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │ │
│  │  │   Slice     │  │    QoS      │  │   Timing    │        │ │
│  │  │  Orchestrator│  │   Engine   │  │   Service   │        │ │
│  │  └─────────────┘  └─────────────┘  └─────────────┘        │ │
│  └───────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
nr_scheduler/
├── include/
│   ├── core/           # Core types and interfaces
│   ├── engine/         # Scheduling engine components
│   ├── channels/       # Channel-specific managers
│   ├── services/       # Supporting services
│   └── interfaces/     # External interfaces
├── src/
│   ├── core/
│   ├── engine/
│   ├── channels/
│   └── services/
├── tests/
└── docs/
```

## Key Features

- **Modern C++17** design with RAII and smart pointers
- **Zero-copy** buffer management
- **Lock-free** queues for inter-component communication
- **Plugin-based** policy system
- **Slice-aware** scheduling
- **5QI-based** QoS handling
- **Flexible** PHY abstraction layer

## Building

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## License

Proprietary - All rights reserved
