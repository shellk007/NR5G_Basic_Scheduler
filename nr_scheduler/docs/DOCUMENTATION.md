# NexGen 5G NR Scheduler - Complete Documentation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Core Components](#core-components)
4. [Interfaces](#interfaces)
5. [Channel Managers](#channel-managers)
6. [Services](#services)
7. [Scheduling Algorithms](#scheduling-algorithms)
8. [QoS Framework](#qos-framework)
9. [Network Slicing](#network-slicing)
10. [API Reference](#api-reference)
11. [Integration Guide](#integration-guide)
12. [Performance Tuning](#performance-tuning)
13. [Examples](#examples)

---

## Overview

NexGen 5G NR Scheduler is a modern, high-performance L2 scheduler implementation designed for 5G NR base stations. Built with C++17, it provides:

- **High Performance**: Optimized for sub-100μs slot processing
- **Flexible Policies**: Pluggable scheduling algorithms
- **5QI-based QoS**: Full support for GBR, Non-GBR, and Delay-Critical flows
- **Network Slicing**: Complete slice isolation and resource management
- **Multi-PHY Support**: Abstract interface for various PHY vendors
- **Modern Design**: Clean APIs, RAII, move semantics, no raw pointers

### Key Features

| Feature | Description |
|---------|-------------|
| **Scheduling Strategies** | Round Robin, Proportional Fair, Max Throughput, QoS-Aware |
| **QoS Support** | 5QI mapping, GBR enforcement, PDB tracking, MBR shaping |
| **Slicing** | eMBB, URLLC, mMTC templates with SLA enforcement |
| **HARQ** | Full HARQ management with adaptive retransmission |
| **MIMO** | Multi-layer support, rank adaptation |
| **TDD/FDD** | Full support for both duplex modes |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SCHEDULER ENGINE                              │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │   Policy    │  │  Resource   │  │     QoS     │  │    Slice    │ │
│  │   Manager   │  │  Allocator  │  │   Engine    │  │Orchestrator │ │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │  PDCCH   │  │  PDSCH   │  │  PUSCH   │  │  PUCCH   │             │
│  │ Manager  │  │ Manager  │  │ Manager  │  │ Manager  │             │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘             │
├─────────────────────────────────────────────────────────────────────┤
│                         CORE CONTEXT                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                  │
│  │    Cell     │  │     UE      │  │  Resource   │                  │
│  │   Context   │  │   Context   │  │    Grid     │                  │
│  └─────────────┘  └─────────────┘  └─────────────┘                  │
├─────────────────────────────────────────────────────────────────────┤
│                         INTERFACES                                   │
│  ┌───────────────────────────┐  ┌───────────────────────────┐       │
│  │      PHY Interface        │  │      MAC Interface        │       │
│  │  (FAPI-like abstraction)  │  │  (Upper layer events)     │       │
│  └───────────────────────────┘  └───────────────────────────┘       │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Slot Indication**: PHY triggers slot boundary
2. **Event Processing**: BSR, SR, CSI updates processed
3. **UE Selection**: Eligible UEs identified based on buffer, DRX, slicing
4. **Priority Calculation**: Policy + QoS + Slice factors computed
5. **Resource Allocation**: RBs assigned from resource grid
6. **TTI Building**: PDCCH/PDSCH/PUSCH PDUs constructed
7. **PHY Transmission**: Requests sent to PHY layer

---

## Core Components

### NR Types (`include/core/nr_types.hpp`)

Fundamental type definitions used throughout the scheduler:

```cpp
namespace nexgen::nr {
    using Rnti = uint16_t;
    using CellId = uint16_t;
    using SliceId = uint32_t;
    
    struct FrameTiming {
        uint16_t sfn;   // System Frame Number
        uint8_t slot;   // Slot within frame
    };
    
    struct TimeFrequencyResource {
        ResourceBlockRange rb_range;
        SymbolRange symbol_range;
    };
}
```

### UE Context (`include/core/ue_context.hpp`)

Per-UE state management:

```cpp
class UeContext {
public:
    Rnti rnti;
    SliceId slice_id;
    
    // HARQ entities
    HarqEntity<16> dl_harq;
    HarqEntity<16> ul_harq;
    
    // Channel quality
    ChannelQualityInfo dl_channel_quality;
    
    // Buffer status
    BufferStatus ul_buffer_status;
    std::unordered_map<LcId, uint32_t> dl_buffer_bytes;
    
    // Helper methods
    uint8_t mcs_from_cqi() const;
    bool has_pending_data() const;
};
```

### Cell Context (`include/core/cell_context.hpp`)

Cell configuration and runtime state:

```cpp
class CellContext {
public:
    const CellConfig& config() const;
    SlotContext& current_slot();
    std::shared_ptr<UeRepository> ue_repository();
    
    void advance_to_slot(const FrameTiming& timing);
};
```

### Resource Grid

Tracks RB allocation within a slot:

```cpp
class ResourceGrid {
public:
    bool is_available(const TimeFrequencyResource& res) const;
    bool allocate(const TimeFrequencyResource& res);
    std::optional<ResourceBlockRange> find_contiguous(
        uint16_t num_rbs, uint8_t start_symbol, uint8_t num_symbols);
};
```

---

## Interfaces

### PHY Interface (`include/interfaces/phy_interface.hpp`)

Abstract interface for L1-L2 communication:

```cpp
class IPhyInterface {
public:
    // Slot indication callback
    using SlotCallback = std::function<void(CellId, const FrameTiming&)>;
    void register_slot_callback(SlotCallback callback);
    
    // DL/UL TTI requests
    bool send_dl_tti_request(const DlTtiRequest& request);
    bool send_ul_tti_request(const UlTtiRequest& request);
    
    // Indication callbacks
    void register_crc_callback(CrcCallback callback);
    void register_uci_callback(UciCallback callback);
};
```

### MAC Interface (`include/interfaces/mac_interface.hpp`)

Upper layer event handling:

```cpp
class ISchedulerEventHandler {
public:
    void on_slot_indication(CellId cell_id, const FrameTiming& timing);
    void on_ue_config(const UeConfigRequest& config);
    void on_dl_data(const DlDataNotification& notification);
    void on_bsr(const BufferStatusReport& bsr);
    void on_sr(const SchedulingRequest& sr);
    void on_csi_update(Rnti rnti, const ChannelQualityInfo& cqi);
};
```

---

## Channel Managers

### PDCCH Manager

Control channel allocation with CCE management:

```cpp
class PdcchManager {
public:
    void configure_coreset(const CoresetConfig& config);
    void configure_search_space(const SearchSpaceConfig& config);
    
    std::optional<PdcchAllocation> allocate(
        const UeContext& ue,
        CellContext& cell,
        const DciContent& dci);
    
    AggregationLevel select_aggregation_level(
        const UeContext& ue, uint16_t dci_size);
};
```

### PDSCH Manager

Downlink data channel management:

```cpp
class PdschManager {
public:
    std::optional<PdschAllocation> allocate(
        UeContext& ue,
        CellContext& cell,
        uint32_t bytes_requested);
    
    uint8_t select_mcs(const UeContext& ue, uint16_t num_rbs);
    uint32_t calculate_tbs(uint16_t num_rbs, uint8_t mcs, uint8_t layers);
};
```

### PUSCH Manager

Uplink data channel management:

```cpp
class PuschManager {
public:
    std::optional<PuschGrant> allocate(
        UeContext& ue,
        CellContext& cell,
        uint32_t bytes_requested);
    
    std::optional<PuschGrant> allocate_sr_response(
        UeContext& ue, CellContext& cell);
};
```

### PUCCH Manager

Uplink control channel for HARQ-ACK, SR, CSI:

```cpp
class PucchManager {
public:
    std::optional<PucchAllocation> allocate_harq_ack(
        const UeContext& ue,
        CellContext& cell,
        const FrameTiming& timing,
        uint8_t num_ack_bits);
    
    std::optional<PucchAllocation> allocate_combined_uci(
        const UeContext& ue,
        CellContext& cell,
        uint8_t harq_bits, bool sr, bool csi);
};
```

---

## Services

### QoS Engine (`include/services/qos_engine.hpp`)

5QI-based QoS management:

```cpp
class QosEngine {
public:
    void register_flow(Rnti rnti, const QosFlowParams& params);
    
    QosPriorityResult calculate_priority(
        Rnti rnti, uint8_t qfi, bool is_downlink, uint32_t buffer);
    
    // GBR enforcement
    int64_t get_gbr_deficit(Rnti rnti, uint8_t qfi, bool is_downlink);
    
    // Delay tracking
    std::vector<std::pair<Rnti, uint8_t>> get_delay_critical_flows();
};
```

#### QoS Priority Calculation

```
Final Priority = Base_Priority × GBR_Factor × Delay_Factor

Base_Priority = 128 / 5QI_Priority_Level
GBR_Factor = 1 + (1 - Current_Rate/GBR) × GBR_Weight    [if GBR flow]
Delay_Factor = 1 + (Urgency - Margin) × Delay_Weight    [if critical]
```

### Slice Orchestrator (`include/services/slice_orchestrator.hpp`)

Network slicing with resource isolation:

```cpp
class SliceOrchestrator {
public:
    bool create_slice(const SliceConfig& config);
    bool associate_ue(Rnti rnti, SliceId slice_id);
    
    std::unordered_map<SliceId, SliceResourceBudget> 
    calculate_budgets(const CellContext& cell);
    
    uint16_t request_resources(SliceId id, uint16_t rbs, bool is_dl);
    
    // Templates
    static SliceConfig create_urllc_template(SliceId id, const std::string& name);
};
```

---

## Scheduling Algorithms

### Round Robin

Simple fairness - each UE gets equal opportunity:

```cpp
Priority = Slots_Since_Last_Scheduled
```

### Proportional Fair

Balance throughput and fairness:

```cpp
Priority = Instantaneous_Rate / Average_Rate^α

α = 0: Max throughput
α = 1: Proportional fair
```

### Max Throughput

Prioritize high CQI users:

```cpp
Priority = CQI × log2(1 + Buffer/1000)
```

### QoS-Aware

Extends Proportional Fair with QoS factors:

```cpp
Priority = PF_Metric × QoS_Priority × Slice_Boost
```

---

## QoS Framework

### 5QI Mapping

Standard 5QI values are automatically mapped:

| 5QI | Type | Priority | PDB (ms) | Use Case |
|-----|------|----------|----------|----------|
| 1 | GBR | 20 | 100 | Voice |
| 2 | GBR | 40 | 150 | Video Call |
| 3 | GBR | 30 | 50 | Gaming |
| 82 | Delay-C | 19 | 10 | Discrete Automation |
| 9 | Non-GBR | 90 | 300 | TCP Best Effort |

### GBR Enforcement

Token bucket for rate shaping:

```cpp
struct TokenBucket {
    int64_t tokens;
    int64_t bucket_size;
    int64_t fill_rate_bps;
};
```

### Delay Budget Tracking

Packet age monitoring with urgency calculation:

```cpp
Urgency = Elapsed_Time / Packet_Delay_Budget

if (Urgency > 0.8) → Mark as delay critical
if (Urgency > 1.0) → PDB violation
```

---

## Network Slicing

### Slice Types

- **eMBB**: High throughput, moderate latency
- **URLLC**: Ultra-low latency, high reliability
- **mMTC**: Many devices, low throughput

### Resource Isolation

```
┌─────────────────────────────────────────────────────┐
│              Total Cell Resources (N RBs)           │
├─────────────────────────────────────────────────────┤
│ URLLC │     eMBB      │  mMTC  │   Shared Pool    │
│  20%  │      50%      │  10%   │       20%        │
│(Strict)│ (Preemptible) │(Preempt)│(Dynamic Alloc)  │
└─────────────────────────────────────────────────────┘
```

### SLA Configuration

```cpp
struct SliceSlaConfig {
    uint16_t guaranteed_rbs;
    double resource_share;
    uint32_t max_latency_ms;
    double target_reliability;
    uint8_t slice_priority;
    bool strict_isolation;
    bool preemption_capable;
};
```

---

## API Reference

### Creating the Scheduler

```cpp
#include <nexgen_scheduler/engine/scheduler_engine.hpp>

using namespace nexgen::nr;

// Configuration
SchedulerConfig config;
config.strategy = SchedulerConfig::Strategy::ProportionalFair;
config.enable_qos = true;
config.enable_slicing = true;

// Create scheduler
SchedulerEngine scheduler(config);

// Add cell
CellConfig cell_cfg;
cell_cfg.cell_id = 1;
cell_cfg.num_rbs = 273;
cell_cfg.duplex_mode = DuplexMode::TDD;
scheduler.add_cell(cell_cfg);

// Set PHY interface
auto phy = create_phy_interface(PhyVendor::IntelFlexRAN);
scheduler.set_phy_interface(phy);

// Start
scheduler.start();
```

### Handling Events

```cpp
// UE Configuration
UeConfigRequest ue_cfg;
ue_cfg.rnti = 0x1234;
ue_cfg.cell_id = 1;
ue_cfg.slice_id = 1;
scheduler.on_ue_config(ue_cfg);

// DL Data Arrival
DlDataNotification notif;
notif.rnti = 0x1234;
notif.lc_id = 4;
notif.buffer_bytes = 10000;
scheduler.on_dl_data(notif);

// BSR Processing
BufferStatusReport bsr;
bsr.rnti = 0x1234;
bsr.lcg_id = 1;
bsr.buffer_size_bytes = 5000;
scheduler.on_bsr(bsr);
```

### Custom Policy

```cpp
class MyPolicy : public IPolicyManager {
public:
    double calculate_priority(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) override {
        // Custom priority logic
        return my_custom_metric(ue);
    }
    
    void update_history(const UeContext& ue, uint32_t bytes) override {
        // Track scheduling history
    }
    
    void on_slot_start(const FrameTiming& timing) override {
        // Per-slot initialization
    }
};

scheduler.set_policy_manager(std::make_unique<MyPolicy>());
```

---

## Integration Guide

### With Intel FlexRAN

```cpp
// Create FlexRAN PHY adapter
auto phy = create_phy_interface(PhyVendor::IntelFlexRAN);

// Configure
IPhyInterface::PhyConfig phy_cfg;
phy_cfg.cell_id = 1;
phy_cfg.pci = 100;
phy_cfg.frequency_khz = 3500000;
phy_cfg.bandwidth_mhz = 100;
phy.configure(phy_cfg);

// Connect to scheduler
scheduler.set_phy_interface(phy);
```

### With OAM

```cpp
// Statistics retrieval
auto stats = scheduler.get_statistics();
oam_report("dl_bytes", stats.dl_bytes_scheduled);
oam_report("ul_bytes", stats.ul_bytes_scheduled);
oam_report("avg_latency", stats.avg_slot_processing_us);

// Configuration update
SchedulerConfig new_config = scheduler.config();
new_config.alpha_fairness = 0.5;
scheduler.update_config(new_config);
```

---

## Performance Tuning

### Slot Processing Budget

Target: < 100μs per slot

| Component | Target Time |
|-----------|-------------|
| Event Processing | 10μs |
| UE Selection | 15μs |
| Priority Calculation | 20μs |
| Resource Allocation | 30μs |
| TTI Building | 25μs |

### Memory Optimization

- Pre-allocate UE contexts: `ue_repo.reserve(max_ues)`
- Use object pools for frequent allocations
- Minimize STL container resizing

### Multi-threading

The scheduler is designed for single-threaded slot processing but supports:
- Parallel UE priority calculation
- Async statistics collection
- Background HARQ timeout processing

---

## Examples

### Basic Scheduling Loop

```cpp
void slot_handler(CellId cell_id, const FrameTiming& timing) {
    // Process indications
    process_crc_indications();
    process_uci_indications();
    
    // Run scheduler
    scheduler.on_slot_indication(cell_id, timing);
}

// Register with PHY
phy->register_slot_callback(slot_handler);
```

### Slice-based Operation

```cpp
// Create slices
auto urllc = SliceOrchestrator::create_urllc_template(1, "Factory Automation");
auto embb = SliceOrchestrator::create_embb_template(2, "Consumer Broadband");

orchestrator.create_slice(urllc);
orchestrator.create_slice(embb);

// Associate UEs
orchestrator.associate_ue(0x1001, 1);  // Factory device → URLLC
orchestrator.associate_ue(0x2001, 2);  // Phone → eMBB
```

### QoS Flow Setup

```cpp
QosFlowParams voice_flow;
voice_flow.qfi = 1;
voice_flow.characteristics = get_5qi_characteristics(1);  // Voice
voice_flow.gbr_dl_bps = 64000;
voice_flow.gbr_ul_bps = 64000;

qos_engine.register_flow(ue_rnti, voice_flow);
```

---

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
cmake --build . --parallel

# Run tests
ctest --output-on-failure

# Generate docs
cmake --build . --target docs
```

---

## License

This scheduler implementation is proprietary software.
Copyright © 2024. All rights reserved.

---

## Contact

For questions or support, contact the L2 development team.
