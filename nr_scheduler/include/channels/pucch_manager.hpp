/**
 * @file pucch_manager.hpp
 * @brief PUCCH Channel Manager for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Manages uplink control channel allocation for HARQ-ACK, SR, and CSI
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include "../core/cell_context.hpp"
#include <vector>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// PUCCH Formats
//=============================================================================

enum class PucchFormat : uint8_t {
    Format0,    ///< 1-2 bits, short PUCCH (1-2 symbols)
    Format1,    ///< 1-2 bits, long PUCCH (4-14 symbols)
    Format2,    ///< >2 bits, short PUCCH (1-2 symbols)
    Format3,    ///< >2 bits, long PUCCH (4-14 symbols)
    Format4     ///< >2 bits, long PUCCH with OCC
};

//=============================================================================
// PUCCH Resource Configuration
//=============================================================================

struct PucchResourceConfig {
    uint8_t     resource_id;
    PucchFormat format;
    
    // Common parameters
    uint16_t    start_prb;
    uint8_t     num_prbs;           ///< 1 for Format 0/1, 1-16 for Format 2/3/4
    uint8_t     start_symbol;
    uint8_t     num_symbols;
    
    // Format-specific
    union {
        struct {
            uint8_t initial_cyclic_shift;
            uint8_t time_domain_occ;
        } format0_1;
        
        struct {
            uint8_t occ_index;
            uint8_t occ_length;
        } format3_4;
    };
    
    // Intra-slot frequency hopping
    bool        intra_slot_hopping;
    uint16_t    second_hop_prb;
    
    // Maximum payload
    [[nodiscard]] uint8_t max_uci_bits() const noexcept {
        switch (format) {
            case PucchFormat::Format0:
            case PucchFormat::Format1:
                return 2;
            case PucchFormat::Format2:
                return num_prbs * num_symbols * 16;  // Approximate
            case PucchFormat::Format3:
            case PucchFormat::Format4:
                return num_prbs * num_symbols * 24;
            default:
                return 0;
        }
    }
};

//=============================================================================
// PUCCH Resource Set
//=============================================================================

struct PucchResourceSet {
    uint8_t     resource_set_id;
    std::vector<uint8_t> resource_ids;  // References to PucchResourceConfig
    uint8_t     max_payload_bits;       // 0 means no limit
    
    // Selection criteria
    enum class Selection : uint8_t {
        FirstAvailable,
        CyclicShiftBased,
        ResourceIndicator
    } selection_method;
};

//=============================================================================
// PUCCH Allocation
//=============================================================================

struct PucchAllocation {
    Rnti        rnti;
    FrameTiming timing;
    
    // Resource
    uint8_t     resource_id;
    PucchFormat format;
    uint16_t    start_prb;
    uint8_t     num_prbs;
    uint8_t     start_symbol;
    uint8_t     num_symbols;
    
    // Format 0/1 specific
    uint8_t     initial_cyclic_shift;
    uint8_t     time_domain_occ;
    
    // UCI content
    bool        has_sr;
    uint8_t     harq_ack_bits;
    uint8_t     csi_part1_bits;
    uint8_t     csi_part2_bits;
    
    // Hopping
    bool        intra_slot_hopping;
    uint16_t    second_hop_prb;
    
    // Power control
    int8_t      tpc_command;
    
    [[nodiscard]] uint8_t total_uci_bits() const noexcept {
        return (has_sr ? 1 : 0) + harq_ack_bits + csi_part1_bits + csi_part2_bits;
    }
};

//=============================================================================
// SR Configuration
//=============================================================================

struct SrConfig {
    uint8_t     sr_resource_id;
    uint8_t     pucch_resource_id;
    uint16_t    periodicity_slots;
    uint16_t    offset_slots;
    
    [[nodiscard]] bool is_sr_occasion(uint16_t sfn, uint8_t slot, 
                                       uint16_t slots_per_frame) const noexcept {
        uint32_t abs_slot = sfn * slots_per_frame + slot;
        return (abs_slot % periodicity_slots) == offset_slots;
    }
};

//=============================================================================
// CSI Report Configuration
//=============================================================================

struct CsiReportConfig {
    uint8_t     report_config_id;
    uint8_t     pucch_resource_id;
    uint16_t    periodicity_slots;
    uint16_t    offset_slots;
    
    // Report content
    bool        report_cqi;
    bool        report_ri;
    bool        report_pmi;
    bool        report_cri;
    bool        report_li;
    
    // Wideband/subband
    bool        wideband_cqi;
    uint8_t     num_subbands;
    
    [[nodiscard]] uint8_t estimate_payload_bits() const noexcept {
        uint8_t bits = 0;
        if (report_cqi) bits += wideband_cqi ? 4 : (4 + num_subbands * 4);
        if (report_ri) bits += 2;
        if (report_pmi) bits += 4;  // Simplified
        if (report_cri) bits += 3;
        if (report_li) bits += 2;
        return bits;
    }
};

//=============================================================================
// PUCCH Manager
//=============================================================================

class PucchManager {
public:
    PucchManager();
    ~PucchManager();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    /**
     * @brief Configure PUCCH resource
     */
    void configure_resource(const PucchResourceConfig& config);
    
    /**
     * @brief Configure PUCCH resource set
     */
    void configure_resource_set(const PucchResourceSet& config);
    
    /**
     * @brief Configure SR for UE
     */
    void configure_sr(Rnti rnti, const SrConfig& config);
    
    /**
     * @brief Configure CSI reporting for UE
     */
    void configure_csi_report(Rnti rnti, const CsiReportConfig& config);
    
    /**
     * @brief Remove UE configuration
     */
    void remove_ue(Rnti rnti);

    //=========================================================================
    // HARQ-ACK Allocation
    //=========================================================================
    
    /**
     * @brief Allocate PUCCH for HARQ-ACK
     * @param ue UE context
     * @param cell Cell context
     * @param timing When PUCCH should be transmitted
     * @param num_ack_bits Number of HARQ-ACK bits
     * @return Allocation if successful
     */
    std::optional<PucchAllocation> allocate_harq_ack(
        const UeContext& ue,
        CellContext& cell,
        const FrameTiming& timing,
        uint8_t num_ack_bits);

    //=========================================================================
    // SR Allocation
    //=========================================================================
    
    /**
     * @brief Check if SR occasion for UE
     */
    bool is_sr_occasion(Rnti rnti, const FrameTiming& timing) const;
    
    /**
     * @brief Allocate PUCCH for SR
     */
    std::optional<PucchAllocation> allocate_sr(
        const UeContext& ue,
        CellContext& cell,
        const FrameTiming& timing);

    //=========================================================================
    // CSI Allocation
    //=========================================================================
    
    /**
     * @brief Check if CSI report occasion for UE
     */
    bool is_csi_occasion(Rnti rnti, const FrameTiming& timing) const;
    
    /**
     * @brief Allocate PUCCH for CSI report
     */
    std::optional<PucchAllocation> allocate_csi(
        const UeContext& ue,
        CellContext& cell,
        const FrameTiming& timing);

    //=========================================================================
    // Combined Allocation
    //=========================================================================
    
    /**
     * @brief Allocate PUCCH for combined UCI
     * Multiplexes HARQ-ACK, SR, and CSI on same PUCCH resource
     */
    std::optional<PucchAllocation> allocate_combined_uci(
        const UeContext& ue,
        CellContext& cell,
        const FrameTiming& timing,
        uint8_t harq_ack_bits,
        bool include_sr,
        bool include_csi);

    //=========================================================================
    // Resource Selection
    //=========================================================================
    
    /**
     * @brief Select PUCCH format for given UCI payload
     */
    PucchFormat select_format(uint8_t total_uci_bits) const;
    
    /**
     * @brief Select resource from resource set
     */
    std::optional<uint8_t> select_resource(
        uint8_t resource_set_id,
        uint8_t uci_bits,
        const CellContext& cell) const;
    
    /**
     * @brief Get PUCCH resource from PDSCH scheduling
     * Implements DCI 1_1 PUCCH resource indicator mapping
     */
    uint8_t get_pucch_resource_from_dci(
        Rnti rnti,
        uint8_t pucch_resource_indicator) const;

    //=========================================================================
    // Slot Management
    //=========================================================================
    
    /**
     * @brief Get all allocations for current slot
     */
    [[nodiscard]] const std::vector<PucchAllocation>& 
    get_slot_allocations() const noexcept;
    
    /**
     * @brief Clear allocations for new slot
     */
    void reset_slot();
    
    /**
     * @brief Check for PUCCH resource collision
     */
    bool has_collision(const PucchAllocation& alloc) const;

    //=========================================================================
    // Power Control
    //=========================================================================
    
    /**
     * @brief Generate TPC command for PUCCH
     */
    int8_t generate_tpc_command(const UeContext& ue) const;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint64_t total_allocations;
        uint64_t harq_ack_allocations;
        uint64_t sr_allocations;
        uint64_t csi_allocations;
        uint64_t combined_allocations;
        uint64_t allocation_failures;
        uint64_t collisions_resolved;
        std::array<uint64_t, 5> per_format_allocations;
        double   resource_utilization;
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
 * @brief Get PUCCH format name
 */
const char* pucch_format_name(PucchFormat format);

/**
 * @brief Calculate cyclic shift for Format 0
 */
uint8_t calculate_cyclic_shift(
    Rnti rnti,
    uint8_t initial_cs,
    uint16_t sfn,
    uint8_t slot,
    uint8_t symbol);

/**
 * @brief Calculate number of PRBs needed for Format 2/3
 */
uint8_t calculate_pucch_prbs(
    uint8_t uci_bits,
    uint8_t num_symbols,
    PucchFormat format);

} // namespace nexgen::nr
