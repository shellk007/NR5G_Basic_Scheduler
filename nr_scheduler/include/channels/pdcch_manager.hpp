/**
 * @file pdcch_manager.hpp
 * @brief PDCCH Channel Manager for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Manages Control Channel Element (CCE) allocation, aggregation level
 * selection, and DCI format encoding
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include "../core/cell_context.hpp"
#include <vector>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// CORESET Configuration
//=============================================================================

struct CoresetConfig {
    uint8_t     coreset_id;
    uint16_t    start_rb;
    uint8_t     num_rbs;            ///< In units of 6 RBs
    uint8_t     num_symbols;        ///< 1, 2, or 3
    uint8_t     cce_to_reg_mapping; ///< 0=non-interleaved, 1=interleaved
    uint8_t     interleaver_size;   ///< L: 2, 3, or 6
    uint8_t     shift_index;        ///< 0-274
    uint8_t     precoder_granularity;
    
    [[nodiscard]] uint8_t num_cces() const noexcept {
        // Each CCE = 6 REGs, each REG = 1 RB × 1 symbol
        return (num_rbs * num_symbols) / 6;
    }
};

//=============================================================================
// Search Space Configuration
//=============================================================================

enum class SearchSpaceType : uint8_t {
    Common,     ///< Common search space (Type0/1/2/3)
    UeSpecific  ///< UE-specific search space
};

struct SearchSpaceConfig {
    uint8_t         search_space_id;
    uint8_t         coreset_id;
    SearchSpaceType type;
    
    // Monitoring periodicity and offset
    uint16_t        monitoring_slot_periodicity;
    uint16_t        monitoring_slot_offset;
    uint8_t         monitoring_symbols_within_slot;
    
    // Aggregation levels and candidates
    struct AggregationInfo {
        AggregationLevel level;
        uint8_t num_candidates;
    };
    std::vector<AggregationInfo> aggregation_levels;
    
    // DCI formats to monitor
    std::vector<DciFormat> dci_formats;
    
    [[nodiscard]] bool should_monitor(uint16_t sfn, uint8_t slot) const noexcept {
        uint32_t abs_slot = sfn * 10 + slot;  // Simplified
        return (abs_slot % monitoring_slot_periodicity) == monitoring_slot_offset;
    }
};

//=============================================================================
// DCI Content
//=============================================================================

struct DciContent {
    DciFormat format;
    Rnti      rnti;
    
    // Common fields
    ResourceBlockRange rb_allocation;
    uint8_t     time_domain_assignment;
    uint8_t     mcs;
    bool        ndi;
    uint8_t     rv;
    HarqId      harq_id;
    
    // Format 0_1 specific (UL)
    std::optional<int8_t> tpc_command;
    std::optional<uint8_t> srs_request;
    std::optional<uint8_t> csi_request;
    
    // Format 1_1 specific (DL)
    std::optional<uint8_t> antenna_ports;
    std::optional<uint8_t> dmrs_sequence_init;
    std::optional<uint8_t> pucch_resource;
    std::optional<uint8_t> pdsch_to_harq_timing;
    
    // For CBG transmission
    std::optional<uint8_t> cbgti;        ///< CBG Transmission Information
    std::optional<uint8_t> cbgfi;        ///< CBG Flush Indicator
    
    /**
     * @brief Get DCI payload size in bits
     */
    [[nodiscard]] uint16_t get_payload_size() const;
    
    /**
     * @brief Encode DCI to bit stream
     */
    [[nodiscard]] std::vector<uint8_t> encode() const;
};

//=============================================================================
// PDCCH Allocation Result
//=============================================================================

struct PdcchAllocation {
    Rnti                rnti;
    uint8_t             coreset_id;
    uint8_t             search_space_id;
    AggregationLevel    aggregation_level;
    uint8_t             cce_start_index;
    DciContent          dci;
    
    [[nodiscard]] uint8_t num_cces() const noexcept {
        return static_cast<uint8_t>(aggregation_level);
    }
};

//=============================================================================
// PDCCH Manager
//=============================================================================

class PdcchManager {
public:
    PdcchManager();
    ~PdcchManager();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    /**
     * @brief Configure CORESET
     */
    void configure_coreset(const CoresetConfig& config);
    
    /**
     * @brief Configure Search Space
     */
    void configure_search_space(const SearchSpaceConfig& config);
    
    /**
     * @brief Configure UE search spaces
     */
    void configure_ue_search_spaces(Rnti rnti, 
                                    const std::vector<uint8_t>& search_space_ids);

    //=========================================================================
    // Allocation
    //=========================================================================
    
    /**
     * @brief Allocate PDCCH for a DCI
     * @param ue UE context
     * @param cell Cell context (slot context used for CCE pool)
     * @param dci DCI content to send
     * @return Allocation result if successful
     */
    std::optional<PdcchAllocation> allocate(
        const UeContext& ue,
        CellContext& cell,
        const DciContent& dci);
    
    /**
     * @brief Allocate PDCCH with specific aggregation level
     */
    std::optional<PdcchAllocation> allocate_with_al(
        const UeContext& ue,
        CellContext& cell,
        const DciContent& dci,
        AggregationLevel al);
    
    /**
     * @brief Get all allocations for current slot
     */
    [[nodiscard]] const std::vector<PdcchAllocation>& 
    get_slot_allocations() const noexcept;
    
    /**
     * @brief Clear allocations for new slot
     */
    void reset_slot();

    //=========================================================================
    // Aggregation Level Selection
    //=========================================================================
    
    /**
     * @brief Select appropriate aggregation level based on channel conditions
     * @param ue UE context
     * @param dci_size DCI payload size in bits
     * @return Selected aggregation level
     */
    AggregationLevel select_aggregation_level(
        const UeContext& ue,
        uint16_t dci_size) const;
    
    /**
     * @brief Get PDCCH BLER target
     */
    double get_bler_target() const noexcept;
    
    /**
     * @brief Set PDCCH BLER target
     */
    void set_bler_target(double target);

    //=========================================================================
    // CCE Management
    //=========================================================================
    
    /**
     * @brief Get available CCEs for an aggregation level
     */
    uint8_t available_candidates(
        const CellContext& cell,
        uint8_t search_space_id,
        AggregationLevel al) const;
    
    /**
     * @brief Check if CCE resources are available
     */
    bool has_resources(const CellContext& cell, AggregationLevel al) const;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint64_t total_allocations;
        uint64_t allocation_failures;
        std::array<uint64_t, 5> per_al_allocations;  // AL1, AL2, AL4, AL8, AL16
        double   avg_aggregation_level;
        double   cce_utilization;
    };
    
    [[nodiscard]] Statistics get_statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//=============================================================================
// Helper Functions
//=============================================================================

/**
 * @brief Calculate hashing function for CCE candidate location
 */
uint8_t calculate_cce_candidate(
    Rnti rnti,
    uint8_t candidate_index,
    AggregationLevel al,
    uint8_t num_cces_coreset,
    uint16_t sfn,
    uint8_t slot);

/**
 * @brief Get DCI payload size for a format
 */
uint16_t get_dci_size(DciFormat format, uint16_t num_rbs);

} // namespace nexgen::nr
