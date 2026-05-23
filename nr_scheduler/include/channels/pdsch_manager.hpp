/**
 * @file pdsch_manager.hpp
 * @brief PDSCH Channel Manager for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Manages downlink data channel allocation, DMRS configuration,
 * and transport block construction
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include "../core/cell_context.hpp"
#include "pdcch_manager.hpp"
#include <vector>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// PDSCH Configuration
//=============================================================================

struct PdschConfig {
    // Resource allocation type
    ResourceAllocationType ra_type;
    
    // DMRS Configuration
    uint8_t  dmrs_type;             ///< 1 or 2
    uint8_t  dmrs_additional_pos;   ///< 0, 1, 2, or 3
    uint16_t dmrs_symbols;          ///< Bitmap of DMRS symbol positions
    uint8_t  dmrs_ports;            ///< Bitmap of DMRS ports
    
    // PRB Bundling
    uint8_t  prb_bundling_size;     ///< 2 or wideband
    
    // Rate matching
    std::vector<ResourceBlockRange> rate_match_patterns;
    
    // MCS configuration
    uint8_t  mcs_table;             ///< 0=64QAM, 1=256QAM, 2=Low-SE
    uint8_t  max_mcs;
    uint8_t  min_mcs;
    
    // HARQ
    uint8_t  max_harq_retx;
    bool     cbg_transmission;      ///< Code Block Group transmission
    uint8_t  max_cbgs_per_tb;
    
    // Power control
    int8_t   pdsch_power_offset_db;
};

//=============================================================================
// Time Domain Resource Allocation
//=============================================================================

struct PdschTimeDomainAllocation {
    uint8_t     config_index;
    uint8_t     mapping_type;       ///< A or B
    uint8_t     start_symbol;
    uint8_t     num_symbols;
    uint8_t     k0;                 ///< Slot offset from DCI
};

//=============================================================================
// PDSCH Allocation Result
//=============================================================================

struct PdschAllocation {
    Rnti        rnti;
    FrameTiming timing;             ///< Slot where PDSCH is transmitted
    
    // Resource allocation
    ResourceBlockRange rb_allocation;
    SymbolRange        symbol_allocation;
    
    // Transport block
    TransportBlockConfig tb_config;
    HarqId              harq_id;
    bool                is_retransmission;
    
    // DMRS
    uint16_t    dmrs_symbols;
    uint8_t     dmrs_ports;
    uint8_t     dmrs_scrambling_id;
    
    // Layers and antenna ports
    uint8_t     num_layers;
    std::vector<uint8_t> antenna_ports;
    
    // HARQ-ACK timing
    uint8_t     k1;                 ///< Slots until HARQ-ACK
    uint8_t     pucch_resource_id;
    
    // Per-LC data
    struct LcData {
        LcId     lc_id;
        uint32_t bytes;
        uint8_t  priority;
    };
    std::vector<LcData> lc_data;
    
    // Power
    int8_t      power_offset_db;
    
    [[nodiscard]] uint32_t total_bytes() const noexcept {
        uint32_t total = 0;
        for (const auto& lc : lc_data) {
            total += lc.bytes;
        }
        return total;
    }
    
    [[nodiscard]] uint32_t num_res() const noexcept {
        // Approximate RE calculation
        uint32_t total_res = rb_allocation.num_rbs * symbol_allocation.num_symbols * 12;
        uint32_t dmrs_overhead = rb_allocation.num_rbs * __builtin_popcount(dmrs_symbols) * 4;
        return total_res - dmrs_overhead;
    }
};

//=============================================================================
// PDSCH Manager
//=============================================================================

class PdschManager {
public:
    explicit PdschManager(const PdschConfig& config = {});
    ~PdschManager();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    void configure(const PdschConfig& config);
    
    /**
     * @brief Add time domain allocation configuration
     */
    void add_time_domain_allocation(const PdschTimeDomainAllocation& tda);
    
    /**
     * @brief Configure rate matching pattern for SSB/CORESET
     */
    void add_rate_match_pattern(const ResourceBlockRange& pattern);

    //=========================================================================
    // Allocation
    //=========================================================================
    
    /**
     * @brief Allocate PDSCH resources for a UE
     * @param ue UE context
     * @param cell Cell context
     * @param bytes_requested Bytes to transmit
     * @param harq_process HARQ process to use (nullptr for auto-select)
     * @return Allocation result if successful
     */
    std::optional<PdschAllocation> allocate(
        UeContext& ue,
        CellContext& cell,
        uint32_t bytes_requested,
        HarqEntity<16>::Process* harq_process = nullptr);
    
    /**
     * @brief Allocate PDSCH for retransmission
     * @param ue UE context
     * @param cell Cell context
     * @param harq_process HARQ process with pending retx
     * @return Allocation result if successful
     */
    std::optional<PdschAllocation> allocate_retx(
        UeContext& ue,
        CellContext& cell,
        HarqEntity<16>::Process& harq_process);
    
    /**
     * @brief Get all allocations for current slot
     */
    [[nodiscard]] const std::vector<PdschAllocation>& 
    get_slot_allocations() const noexcept;
    
    /**
     * @brief Clear allocations for new slot
     */
    void reset_slot();

    //=========================================================================
    // MCS Selection
    //=========================================================================
    
    /**
     * @brief Select MCS based on channel quality and BLER target
     * @param ue UE context
     * @param num_rbs Number of RBs for allocation
     * @return Selected MCS index
     */
    uint8_t select_mcs(const UeContext& ue, uint16_t num_rbs) const;
    
    /**
     * @brief Adjust MCS based on HARQ feedback history
     */
    void update_mcs_adaptation(Rnti rnti, bool ack);
    
    /**
     * @brief Get BLER target
     */
    double get_bler_target() const noexcept;
    
    /**
     * @brief Set BLER target
     */
    void set_bler_target(double target);

    //=========================================================================
    // Transport Block Size
    //=========================================================================
    
    /**
     * @brief Calculate TBS for given allocation
     */
    uint32_t calculate_tbs(
        uint16_t num_rbs,
        uint8_t num_symbols,
        uint8_t mcs,
        uint8_t num_layers,
        uint16_t dmrs_symbols) const;
    
    /**
     * @brief Get minimum RBs needed for given bytes
     */
    uint16_t min_rbs_for_bytes(
        uint32_t bytes,
        uint8_t mcs,
        uint8_t num_symbols,
        uint8_t num_layers) const;

    //=========================================================================
    // DMRS Configuration
    //=========================================================================
    
    /**
     * @brief Get DMRS symbol positions for allocation
     */
    uint16_t get_dmrs_symbols(
        uint8_t start_symbol,
        uint8_t num_symbols,
        uint8_t mapping_type) const;
    
    /**
     * @brief Get DMRS ports for given number of layers
     */
    uint8_t get_dmrs_ports(uint8_t num_layers) const;

    //=========================================================================
    // LC Multiplexing
    //=========================================================================
    
    /**
     * @brief Multiplex LC data into transport block
     * @param ue UE context
     * @param tbs_bytes Transport block size
     * @return Per-LC byte allocations
     */
    std::vector<std::pair<LcId, uint32_t>> multiplex_lc_data(
        const UeContext& ue,
        uint32_t tbs_bytes) const;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint64_t total_allocations;
        uint64_t new_tx_allocations;
        uint64_t retx_allocations;
        uint64_t allocation_failures;
        uint64_t total_bytes_allocated;
        uint64_t total_rbs_used;
        double   avg_mcs;
        double   avg_spectral_efficiency;
        double   resource_utilization;
    };
    
    [[nodiscard]] Statistics get_statistics() const;
    void reset_statistics();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//=============================================================================
// Helper Functions
//=============================================================================

/**
 * @brief Get spectral efficiency for MCS index
 * @param mcs MCS index (0-28)
 * @param table MCS table (0=64QAM, 1=256QAM, 2=Low-SE)
 * @return Spectral efficiency in bits/RE
 */
double get_spectral_efficiency(uint8_t mcs, uint8_t table);

/**
 * @brief Calculate number of REs available in PDSCH allocation
 */
uint32_t calculate_pdsch_res(
    uint16_t num_rbs,
    uint8_t num_symbols,
    uint16_t dmrs_symbols,
    uint8_t dmrs_type);

} // namespace nexgen::nr
