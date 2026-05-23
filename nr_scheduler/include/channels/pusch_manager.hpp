/**
 * @file pusch_manager.hpp
 * @brief PUSCH Channel Manager for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Manages uplink data channel allocation, grant construction,
 * and UCI multiplexing on PUSCH
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include "../core/cell_context.hpp"
#include <vector>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// PUSCH Configuration
//=============================================================================

struct PuschConfig {
    // Resource allocation type
    ResourceAllocationType ra_type;
    
    // DMRS Configuration
    uint8_t  dmrs_type;
    uint8_t  dmrs_additional_pos;
    uint16_t dmrs_symbols;
    
    // Transform precoding
    bool     transform_precoding;   ///< DFT-s-OFDM
    
    // MCS configuration
    uint8_t  mcs_table;
    uint8_t  max_mcs;
    uint8_t  min_mcs;
    
    // HARQ
    uint8_t  max_harq_retx;
    
    // Power control
    int8_t   p0_pusch;              ///< P0 nominal
    int8_t   alpha;                 ///< Path loss compensation factor (0-1)
    
    // UCI on PUSCH
    bool     enable_uci_on_pusch;
    uint8_t  beta_offset_ack;       ///< Beta offset for HARQ-ACK
    uint8_t  beta_offset_csi_part1;
    uint8_t  beta_offset_csi_part2;
};

//=============================================================================
// Time Domain Resource Allocation
//=============================================================================

struct PuschTimeDomainAllocation {
    uint8_t     config_index;
    uint8_t     mapping_type;       ///< A or B
    uint8_t     start_symbol;
    uint8_t     num_symbols;
    uint8_t     k2;                 ///< Slot offset from DCI
};

//=============================================================================
// PUSCH Grant
//=============================================================================

struct PuschGrant {
    Rnti        rnti;
    FrameTiming timing;             ///< Slot where UE should transmit
    
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
    
    // Layers
    uint8_t     num_layers;
    
    // Power control
    int8_t      tpc_command;        ///< TPC command (+/-1, +/-3 dB)
    int8_t      target_power_dbm;
    
    // UCI multiplexing
    bool        uci_on_pusch;
    uint8_t     harq_ack_bits;      ///< HARQ-ACK bits multiplexed
    uint8_t     csi_part1_bits;
    uint8_t     csi_part2_bits;
    
    // Frequency hopping
    bool        freq_hopping_enabled;
    uint16_t    second_hop_prb;
    
    [[nodiscard]] uint32_t num_res() const noexcept {
        uint32_t total_res = rb_allocation.num_rbs * symbol_allocation.num_symbols * 12;
        uint32_t dmrs_overhead = rb_allocation.num_rbs * __builtin_popcount(dmrs_symbols) * 4;
        return total_res - dmrs_overhead;
    }
};

//=============================================================================
// PUSCH Manager
//=============================================================================

class PuschManager {
public:
    explicit PuschManager(const PuschConfig& config = {});
    ~PuschManager();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    void configure(const PuschConfig& config);
    
    /**
     * @brief Add time domain allocation configuration
     */
    void add_time_domain_allocation(const PuschTimeDomainAllocation& tda);

    //=========================================================================
    // Grant Allocation
    //=========================================================================
    
    /**
     * @brief Allocate PUSCH grant for a UE
     * @param ue UE context
     * @param cell Cell context
     * @param bytes_requested Bytes UE reported in BSR
     * @param harq_process HARQ process to use
     * @return Grant if successful
     */
    std::optional<PuschGrant> allocate(
        UeContext& ue,
        CellContext& cell,
        uint32_t bytes_requested,
        HarqEntity<16>::Process* harq_process = nullptr);
    
    /**
     * @brief Allocate PUSCH for retransmission
     */
    std::optional<PuschGrant> allocate_retx(
        UeContext& ue,
        CellContext& cell,
        HarqEntity<16>::Process& harq_process);
    
    /**
     * @brief Allocate small PUSCH for SR response
     * Allocates minimum resources for UE to send BSR
     */
    std::optional<PuschGrant> allocate_sr_response(
        UeContext& ue,
        CellContext& cell);
    
    /**
     * @brief Get all grants for current slot
     */
    [[nodiscard]] const std::vector<PuschGrant>& 
    get_slot_grants() const noexcept;
    
    /**
     * @brief Clear grants for new slot
     */
    void reset_slot();

    //=========================================================================
    // MCS and TBS
    //=========================================================================
    
    /**
     * @brief Select MCS based on channel quality
     */
    uint8_t select_mcs(const UeContext& ue, uint16_t num_rbs) const;
    
    /**
     * @brief Calculate TBS for grant
     */
    uint32_t calculate_tbs(
        uint16_t num_rbs,
        uint8_t num_symbols,
        uint8_t mcs,
        uint8_t num_layers,
        uint16_t dmrs_symbols,
        uint8_t uci_bits = 0) const;
    
    /**
     * @brief Get minimum RBs for given bytes
     */
    uint16_t min_rbs_for_bytes(
        uint32_t bytes,
        uint8_t mcs,
        uint8_t num_symbols,
        uint8_t num_layers) const;
    
    /**
     * @brief Adjust MCS based on CRC feedback
     */
    void update_mcs_adaptation(Rnti rnti, bool crc_pass);

    //=========================================================================
    // Power Control
    //=========================================================================
    
    /**
     * @brief Calculate target TX power for UE
     */
    int8_t calculate_target_power(
        const UeContext& ue,
        uint16_t num_rbs) const;
    
    /**
     * @brief Generate TPC command based on power headroom
     */
    int8_t generate_tpc_command(const UeContext& ue) const;

    //=========================================================================
    // UCI on PUSCH
    //=========================================================================
    
    /**
     * @brief Calculate UCI overhead on PUSCH
     * @return Number of REs used for UCI
     */
    uint32_t calculate_uci_overhead(
        uint8_t harq_ack_bits,
        uint8_t csi_part1_bits,
        uint8_t csi_part2_bits,
        uint16_t num_rbs) const;
    
    /**
     * @brief Check if UCI can be multiplexed on PUSCH
     */
    bool can_multiplex_uci(
        const PuschGrant& grant,
        uint8_t harq_ack_bits,
        uint8_t csi_bits) const;

    //=========================================================================
    // Frequency Hopping
    //=========================================================================
    
    /**
     * @brief Calculate second hop PRB for frequency hopping
     */
    uint16_t calculate_second_hop_prb(
        uint16_t first_hop_prb,
        uint16_t num_rbs,
        uint16_t bwp_size) const;
    
    /**
     * @brief Check if frequency hopping should be enabled
     */
    bool should_enable_hopping(const UeContext& ue, uint16_t num_rbs) const;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint64_t total_grants;
        uint64_t new_tx_grants;
        uint64_t retx_grants;
        uint64_t sr_response_grants;
        uint64_t grant_failures;
        uint64_t total_bytes_granted;
        uint64_t total_rbs_used;
        uint64_t uci_multiplexed_grants;
        double   avg_mcs;
        double   avg_grant_size_rbs;
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
 * @brief Calculate PUSCH resource elements
 */
uint32_t calculate_pusch_res(
    uint16_t num_rbs,
    uint8_t num_symbols,
    uint16_t dmrs_symbols,
    uint8_t dmrs_type,
    bool transform_precoding);

/**
 * @brief Calculate effective TBS after UCI overhead
 */
uint32_t calculate_effective_tbs(
    uint32_t nominal_tbs,
    uint8_t harq_ack_bits,
    uint8_t csi_bits,
    uint8_t beta_offset_ack,
    uint8_t beta_offset_csi);

} // namespace nexgen::nr
