/**
 * @file nr_types.hpp
 * @brief Core type definitions for NexGen 5G NR Scheduler
 * @version 1.0
 */

#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <optional>
#include <variant>
#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace nexgen::nr {

//=============================================================================
// Fundamental Types
//=============================================================================

using Rnti = uint16_t;
using CellId = uint16_t;
using LcId = uint8_t;
using LcgId = uint8_t;
using HarqId = uint8_t;
using BeamId = uint16_t;
using SliceId = uint32_t;

//=============================================================================
// Timing Types
//=============================================================================

struct FrameTiming {
    uint16_t sfn{0};            ///< System Frame Number (0-1023)
    uint8_t  slot{0};           ///< Slot within frame
    uint8_t  symbol{0};         ///< Symbol within slot (0-13)
    uint16_t slots_per_frame{10}; ///< Slots per frame (MUST be set based on SCS)
    
    static constexpr uint16_t MAX_SFN = 1024;
    
    /**
     * @brief Compute absolute slot number
     * @warning Ensure slots_per_frame is correctly set for the numerology!
     */
    [[nodiscard]] constexpr uint32_t absolute_slot() const noexcept {
        return static_cast<uint32_t>(sfn) * slots_per_frame + slot;
    }
    
    /**
     * @brief Advance timing by given number of slots, handling wrap-around
     */
    void advance(uint16_t num_slots) noexcept {
        uint32_t total_slots = slot + num_slots;
        slot = static_cast<uint8_t>(total_slots % slots_per_frame);
        uint16_t frame_advance = static_cast<uint16_t>(total_slots / slots_per_frame);
        sfn = (sfn + frame_advance) % MAX_SFN;
    }
    
    /**
     * @brief Create timing advanced by K slots
     */
    [[nodiscard]] FrameTiming advanced(uint16_t k) const noexcept {
        FrameTiming result = *this;
        result.advance(k);
        return result;
    }
    
    [[nodiscard]] constexpr int64_t diff(const FrameTiming& other) const noexcept {
        int64_t this_abs = static_cast<int64_t>(sfn) * slots_per_frame + slot;
        int64_t other_abs = static_cast<int64_t>(other.sfn) * other.slots_per_frame + other.slot;
        
        int64_t d = this_abs - other_abs;
        int64_t half_cycle = 512 * slots_per_frame;
        if (d > half_cycle) {
            d -= 1024 * slots_per_frame;
        } else if (d < -half_cycle) {
            d += 1024 * slots_per_frame;
        }
        return d;
    }
    
    /**
     * @brief Create FrameTiming with correct slots_per_frame for numerology
     */
    static FrameTiming create(uint16_t sfn_val, uint8_t slot_val, SubcarrierSpacing scs) {
        FrameTiming t;
        t.sfn = sfn_val;
        t.slot = slot_val;
        t.symbol = 0;
        switch (scs) {
            case SubcarrierSpacing::kHz15:  t.slots_per_frame = 10; break;
            case SubcarrierSpacing::kHz30:  t.slots_per_frame = 20; break;
            case SubcarrierSpacing::kHz60:  t.slots_per_frame = 40; break;
            case SubcarrierSpacing::kHz120: t.slots_per_frame = 80; break;
            case SubcarrierSpacing::kHz240: t.slots_per_frame = 160; break;
        }
        return t;
    }
    
    bool operator==(const FrameTiming& other) const noexcept {
        return sfn == other.sfn && slot == other.slot;
    }
};

//=============================================================================
// Numerology Configuration
//=============================================================================

enum class SubcarrierSpacing : uint8_t {
    kHz15  = 0,
    kHz30  = 1,
    kHz60  = 2,
    kHz120 = 3,
    kHz240 = 4
};

struct NumerologyConfig {
    SubcarrierSpacing scs;
    uint8_t  cyclic_prefix;     ///< 0=Normal, 1=Extended
    uint16_t slots_per_frame;
    uint8_t  symbols_per_slot;
    
    static NumerologyConfig from_scs(SubcarrierSpacing scs) {
        NumerologyConfig cfg{};
        cfg.scs = scs;
        cfg.cyclic_prefix = 0;
        cfg.symbols_per_slot = 14;
        
        switch (scs) {
            case SubcarrierSpacing::kHz15:  cfg.slots_per_frame = 10; break;
            case SubcarrierSpacing::kHz30:  cfg.slots_per_frame = 20; break;
            case SubcarrierSpacing::kHz60:  cfg.slots_per_frame = 40; break;
            case SubcarrierSpacing::kHz120: cfg.slots_per_frame = 80; break;
            case SubcarrierSpacing::kHz240: cfg.slots_per_frame = 160; break;
        }
        return cfg;
    }
};

//=============================================================================
// Bandwidth Part (BWP)
//=============================================================================

struct BwpConfig {
    uint8_t  bwp_id;
    uint16_t start_rb;
    uint16_t num_rbs;
    SubcarrierSpacing scs;
    bool     is_active;
    
    [[nodiscard]] uint16_t end_rb() const noexcept { 
        return start_rb + num_rbs - 1; 
    }
};

//=============================================================================
// Resource Allocation
//=============================================================================

enum class ResourceAllocationType : uint8_t {
    Type0,      ///< RBG-based bitmap
    Type1       ///< Contiguous RB allocation
};

struct ResourceBlockRange {
    uint16_t start_rb;
    uint16_t num_rbs;
    
    [[nodiscard]] uint16_t end_rb() const noexcept { 
        return start_rb + num_rbs - 1; 
    }
    
    [[nodiscard]] bool overlaps(const ResourceBlockRange& other) const noexcept {
        return !(end_rb() < other.start_rb || start_rb > other.end_rb());
    }
};

struct SymbolRange {
    uint8_t start_symbol;
    uint8_t num_symbols;
    
    [[nodiscard]] uint8_t end_symbol() const noexcept {
        return start_symbol + num_symbols - 1;
    }
};

struct TimeFrequencyResource {
    ResourceBlockRange rb_range;
    SymbolRange        symbol_range;
    
    [[nodiscard]] uint32_t num_res() const noexcept {
        return rb_range.num_rbs * symbol_range.num_symbols * 12; // 12 subcarriers per RB
    }
};

//=============================================================================
// QoS Parameters
//=============================================================================

enum class QosResourceType : uint8_t {
    GBR,        ///< Guaranteed Bit Rate
    NonGBR,     ///< Non-Guaranteed Bit Rate
    DelayC      ///< Delay Critical GBR
};

struct QosCharacteristics {
    uint8_t         fiveqi;
    QosResourceType resource_type;
    uint8_t         priority_level;     ///< 1 (highest) to 127 (lowest)
    uint32_t        packet_delay_budget_ms;
    double          packet_error_rate;
    uint32_t        averaging_window_ms;
    uint32_t        max_data_burst_volume;
};

struct QosFlowParams {
    uint8_t              qfi;           ///< QoS Flow Identifier
    QosCharacteristics   characteristics;
    uint64_t             gbr_dl_bps;    ///< Guaranteed DL rate
    uint64_t             gbr_ul_bps;    ///< Guaranteed UL rate
    uint64_t             mbr_dl_bps;    ///< Maximum DL rate
    uint64_t             mbr_ul_bps;    ///< Maximum UL rate
};

//=============================================================================
// Channel Quality
//=============================================================================

struct ChannelQualityInfo {
    uint8_t  wideband_cqi;
    uint8_t  rank_indicator;
    std::vector<uint8_t> subband_cqi;
    std::optional<uint16_t> pmi;
    FrameTiming report_time;
    
    [[nodiscard]] bool is_valid(const FrameTiming& now, 
                                 uint16_t validity_slots) const noexcept {
        return now.diff(report_time, now.slots_per_frame) <= validity_slots;
    }
};

//=============================================================================
// HARQ Types
//=============================================================================

enum class HarqState : uint8_t {
    Idle,
    Pending,        ///< Waiting for feedback
    NackReceived,   ///< NACK received, needs retx
    Complete        ///< ACK received
};

struct HarqFeedback {
    HarqId      harq_id;
    bool        ack;        ///< true=ACK, false=NACK
    bool        dtx;        ///< Discontinuous transmission detected
    FrameTiming feedback_time;
};

//=============================================================================
// Buffer Status
//=============================================================================

struct BufferStatus {
    std::array<uint32_t, 8> lcg_buffer_bytes{};  ///< Per-LCG buffer size
    FrameTiming report_time;
    bool        sr_pending;
    
    [[nodiscard]] uint32_t total_buffer() const noexcept {
        uint32_t total = 0;
        for (auto bs : lcg_buffer_bytes) {
            total += bs;
        }
        return total;
    }
};

//=============================================================================
// Transport Block
//=============================================================================

struct TransportBlockConfig {
    uint32_t tb_size_bytes;
    uint8_t  mcs_index;
    uint8_t  mcs_table;     ///< 0=64QAM, 1=256QAM, 2=Low-SE
    uint8_t  rv_index;      ///< Redundancy version (0,2,3,1)
    bool     ndi;           ///< New data indicator
    uint8_t  num_layers;
};

//=============================================================================
// DCI Formats
//=============================================================================

enum class DciFormat : uint8_t {
    Format0_0,      ///< UL scheduling (fallback)
    Format0_1,      ///< UL scheduling (non-fallback)
    Format1_0,      ///< DL scheduling (fallback)
    Format1_1,      ///< DL scheduling (non-fallback)
    Format2_0,      ///< Slot format indicator
    Format2_1,      ///< Preemption indicator
    Format2_2,      ///< Power control
    Format2_3       ///< SRS request
};

//=============================================================================
// Aggregation Level
//=============================================================================

enum class AggregationLevel : uint8_t {
    AL1  = 1,
    AL2  = 2,
    AL4  = 4,
    AL8  = 8,
    AL16 = 16
};

//=============================================================================
// RNTI Types
//=============================================================================

enum class RntiType : uint8_t {
    CRNTI,
    TCRNTI,
    CSRNTI,
    RARNTI,
    SIRNTI,
    PRNTI
};

constexpr Rnti SI_RNTI = 0xFFFF;
constexpr Rnti P_RNTI  = 0xFFFE;

//=============================================================================
// Scheduler Result Types
//=============================================================================

enum class SchedulingResult : uint8_t {
    Success,
    NoResource,
    UeNotFound,
    InvalidConfig,
    QoSViolation,
    HarqBlocked,
    DRXBlocked,
    BeamNotAvailable
};

//=============================================================================
// Utility Functions
//=============================================================================

/**
 * @brief Calculate TBS according to 3GPP 38.214 Section 5.1.3.2
 * @param mcs MCS index (0-28 for Table 5.1.3.1-1)
 * @param num_rbs Number of allocated PRBs
 * @param num_layers Number of MIMO layers
 * @param num_symbols Number of OFDM symbols for PDSCH/PUSCH (typically 12-14)
 * @param dmrs_re_per_rb DMRS overhead in RE per RB (typically 12 for Type 1)
 * @return Transport block size in bytes
 */
[[nodiscard]] inline uint32_t tbs_table_lookup(
    uint8_t mcs, 
    uint16_t num_rbs, 
    uint8_t num_layers,
    uint8_t num_symbols = 12,
    uint8_t dmrs_re_per_rb = 12) noexcept {
    
    // Spectral efficiency * 1024 from 38.214 Table 5.1.3.1-1 (64QAM table)
    static constexpr uint16_t spectral_eff_x1024[] = {
        120, 157, 193, 251, 308, 379, 449, 526,
        602, 679, 772, 873, 948, 1024, 1101, 1176,
        1252, 1327, 1403, 1478, 1554, 1630, 1706, 1783,
        1859, 1935, 2011, 2088, 2165
    };
    
    if (mcs > 28) return 0;
    if (num_rbs == 0 || num_layers == 0) return 0;
    
    // N_RE = min(156, N_RE_prime) * n_PRB where N_RE_prime = 12 * N_symb - dmrs_overhead
    uint32_t re_per_rb = 12 * num_symbols - dmrs_re_per_rb;
    re_per_rb = std::min(re_per_rb, 156u);  // Cap per 38.214
    
    uint32_t n_re = re_per_rb * num_rbs;
    
    // N_info = N_RE * R * Qm * v where R*Qm = spectral_efficiency
    uint32_t n_info = (n_re * spectral_eff_x1024[mcs] * num_layers) / 1024;
    
    // Quantization per 38.214 - simplified version
    if (n_info <= 3824) {
        // Small TBS quantization
        uint32_t n = std::max(3u, static_cast<uint32_t>(std::floor(std::log2(n_info)) - 6));
        uint32_t n_info_prime = std::max(24u, (n_info >> n) << n);
        return (n_info_prime + 7) / 8;
    } else {
        // Large TBS quantization  
        uint32_t n = static_cast<uint32_t>(std::floor(std::log2(n_info - 24)) - 5);
        uint32_t n_info_prime = std::max(3840u, ((n_info - 24) >> n) << n);
        
        // C = ceil((n_info_prime + 24) / 8424) for LDPC base graph selection
        uint32_t tbs;
        if (n_info_prime < 8424) {
            tbs = 8 * ((n_info_prime + 24 + 31) / 32) - 24;
        } else {
            uint32_t c = (n_info_prime + 24 + 8423) / 8424;
            tbs = 8 * c * ((n_info_prime + 24) / (8 * c) + 1) - 24;
        }
        return (tbs + 7) / 8;
    }
}

/**
 * @brief Calculate minimum RBs needed for given TBS
 */
[[nodiscard]] inline uint16_t min_rbs_for_tbs(
    uint32_t tbs_bytes,
    uint8_t mcs,
    uint8_t num_layers,
    uint8_t num_symbols = 12,
    uint16_t max_rbs = 273) noexcept {
    
    // Binary search for minimum RBs
    uint16_t low = 1, high = max_rbs;
    while (low < high) {
        uint16_t mid = (low + high) / 2;
        uint32_t tbs = tbs_table_lookup(mcs, mid, num_layers, num_symbols);
        if (tbs >= tbs_bytes) {
            high = mid;
        } else {
            low = mid + 1;
        }
    }
    return low;
}

} // namespace nexgen::nr
