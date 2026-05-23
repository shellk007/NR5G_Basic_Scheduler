/**
 * @file phy_interface.hpp
 * @brief PHY Interface abstraction for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * This interface abstracts the L1-L2 communication, allowing the scheduler
 * to work with different PHY implementations (Intel FlexRAN, Marvell, etc.)
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace nexgen::nr {

//=============================================================================
// PHY Indication Types
//=============================================================================

struct CrcIndication {
    Rnti        rnti;
    HarqId      harq_id;
    bool        crc_pass;
    uint8_t     num_codeblocks;
    uint8_t     failed_codeblocks;
    int16_t     ul_sinr_db_x10;     ///< UL SINR * 10
    uint16_t    timing_advance;
    FrameTiming timing;
};

struct RxDataIndication {
    Rnti        rnti;
    HarqId      harq_id;
    LcId        lc_id;
    std::vector<uint8_t> data;
    FrameTiming timing;
};

struct UciIndication {
    Rnti        rnti;
    
    // HARQ-ACK
    bool        harq_ack_present;
    std::vector<bool> harq_ack;  // One per DL HARQ process
    
    // SR
    bool        sr_present;
    bool        sr_detected;
    
    // CSI
    bool        csi_present;
    uint8_t     rank_indicator;
    uint8_t     wideband_cqi;
    uint16_t    pmi;
    std::vector<uint8_t> subband_cqi;
    
    FrameTiming timing;
};

struct RachIndication {
    uint16_t    preamble_id;
    uint16_t    timing_advance;
    int16_t     preamble_power_db;
    uint8_t     beam_id;
    FrameTiming timing;
};

struct SrsIndication {
    Rnti        rnti;
    uint8_t     rank_indicator;
    uint16_t    timing_advance;
    std::vector<int16_t> wideband_snr;  // Per antenna
    FrameTiming timing;
};

//=============================================================================
// PHY Request Types
//=============================================================================

struct DlTtiRequest {
    FrameTiming timing;
    CellId      cell_id;
    
    struct PdcchPdu {
        Rnti            rnti;
        DciFormat       dci_format;
        AggregationLevel agg_level;
        uint8_t         cce_start;
        uint8_t         coreset_id;
        uint8_t         search_space_id;
        std::vector<uint8_t> dci_payload;
    };
    std::vector<PdcchPdu> pdcch_pdus;
    
    struct PdschPdu {
        Rnti            rnti;
        uint16_t        bwp_start;
        uint16_t        bwp_size;
        ResourceBlockRange rb_allocation;
        SymbolRange     symbol_allocation;
        uint8_t         mcs;
        uint8_t         mcs_table;
        uint8_t         rv;
        bool            ndi;
        HarqId          harq_id;
        uint8_t         num_layers;
        uint16_t        dmrs_symbols;
        uint8_t         dmrs_ports;
        uint32_t        tb_size;
        std::vector<uint8_t> tb_data;
    };
    std::vector<PdschPdu> pdsch_pdus;
    
    struct CsiRsPdu {
        uint16_t        start_rb;
        uint16_t        num_rbs;
        uint8_t         row;
        uint8_t         freq_domain;
        uint8_t         symb_l0;
        uint8_t         symb_l1;
        uint8_t         cdm_type;
        uint8_t         freq_density;
    };
    std::vector<CsiRsPdu> csirs_pdus;
    
    struct SsbPdu {
        uint16_t        pci;
        uint8_t         ssb_index;
        uint8_t         ssb_subcarrier_offset;
        uint16_t        ssb_offset_point_a;
        uint8_t         beam_id;
    };
    std::vector<SsbPdu> ssb_pdus;
};

struct UlTtiRequest {
    FrameTiming timing;
    CellId      cell_id;
    
    struct PuschPdu {
        Rnti            rnti;
        uint16_t        bwp_start;
        uint16_t        bwp_size;
        ResourceBlockRange rb_allocation;
        SymbolRange     symbol_allocation;
        uint8_t         mcs;
        uint8_t         mcs_table;
        uint8_t         rv;
        bool            ndi;
        HarqId          harq_id;
        uint8_t         num_layers;
        uint16_t        dmrs_symbols;
        uint8_t         dmrs_ports;
        uint32_t        tb_size;
        bool            uci_on_pusch;
    };
    std::vector<PuschPdu> pusch_pdus;
    
    struct PucchPdu {
        Rnti            rnti;
        uint8_t         format;
        uint16_t        start_rb;
        uint8_t         num_rbs;
        uint8_t         start_symbol;
        uint8_t         num_symbols;
        uint8_t         initial_cyclic_shift;
        uint8_t         time_domain_occ_index;
        bool            sr_flag;
        uint8_t         harq_ack_bits;
        uint8_t         csi_part1_bits;
    };
    std::vector<PucchPdu> pucch_pdus;
    
    struct SrsPdu {
        Rnti            rnti;
        uint16_t        start_rb;
        uint8_t         num_rbs;
        uint8_t         bandwidth_index;
        uint8_t         comb_size;
        uint8_t         comb_offset;
        uint8_t         cyclic_shift;
        uint8_t         frequency_position;
        uint8_t         frequency_shift;
        uint8_t         frequency_hopping;
        uint8_t         num_symbols;
        uint8_t         start_symbol;
        uint8_t         num_repetitions;
    };
    std::vector<SrsPdu> srs_pdus;
    
    struct PrachPdu {
        uint16_t        physical_root_sequence_index;
        uint8_t         num_occasions;
        uint8_t         format;
        uint8_t         num_preambles;
        uint8_t         start_preamble_index;
    };
    std::vector<PrachPdu> prach_pdus;
};

struct TxDataRequest {
    FrameTiming timing;
    CellId      cell_id;
    
    struct PduData {
        uint32_t pdu_index;
        uint32_t pdu_length;
        std::vector<uint8_t> data;
    };
    std::vector<PduData> pdu_list;
};

//=============================================================================
// PHY Interface
//=============================================================================

class IPhyInterface {
public:
    virtual ~IPhyInterface() = default;

    //=========================================================================
    // Slot Indication
    //=========================================================================
    
    using SlotCallback = std::function<void(CellId, const FrameTiming&)>;
    
    /**
     * @brief Register callback for slot indication
     * @param callback Function called at each slot boundary
     */
    virtual void register_slot_callback(SlotCallback callback) = 0;

    //=========================================================================
    // Downlink Operations
    //=========================================================================
    
    /**
     * @brief Send DL TTI request to PHY
     * @param request PDCCH/PDSCH/CSIRS/SSB PDUs for the slot
     * @return true if accepted
     */
    virtual bool send_dl_tti_request(const DlTtiRequest& request) = 0;
    
    /**
     * @brief Send TX data to PHY
     * @param request Transport block data
     * @return true if accepted
     */
    virtual bool send_tx_data_request(const TxDataRequest& request) = 0;

    //=========================================================================
    // Uplink Operations
    //=========================================================================
    
    /**
     * @brief Send UL TTI request to PHY
     * @param request PUSCH/PUCCH/SRS/PRACH configuration
     * @return true if accepted
     */
    virtual bool send_ul_tti_request(const UlTtiRequest& request) = 0;

    //=========================================================================
    // Indication Callbacks
    //=========================================================================
    
    using CrcCallback = std::function<void(CellId, const CrcIndication&)>;
    using RxDataCallback = std::function<void(CellId, const RxDataIndication&)>;
    using UciCallback = std::function<void(CellId, const UciIndication&)>;
    using RachCallback = std::function<void(CellId, const RachIndication&)>;
    using SrsCallback = std::function<void(CellId, const SrsIndication&)>;
    
    virtual void register_crc_callback(CrcCallback callback) = 0;
    virtual void register_rx_data_callback(RxDataCallback callback) = 0;
    virtual void register_uci_callback(UciCallback callback) = 0;
    virtual void register_rach_callback(RachCallback callback) = 0;
    virtual void register_srs_callback(SrsCallback callback) = 0;

    //=========================================================================
    // Configuration
    //=========================================================================
    
    struct PhyConfig {
        CellId      cell_id;
        uint32_t    pci;
        uint32_t    frequency_khz;
        uint16_t    bandwidth_mhz;
        uint8_t     scs;
        uint8_t     duplex_mode;
        uint8_t     num_tx_antennas;
        uint8_t     num_rx_antennas;
    };
    
    /**
     * @brief Configure PHY layer
     * @param config Cell configuration
     * @return true if successful
     */
    virtual bool configure(const PhyConfig& config) = 0;
    
    /**
     * @brief Start PHY processing
     * @return true if successful
     */
    virtual bool start() = 0;
    
    /**
     * @brief Stop PHY processing
     */
    virtual void stop() = 0;

    //=========================================================================
    // Timing
    //=========================================================================
    
    /**
     * @brief Get current PHY timing
     * @return Current frame/slot timing
     */
    virtual FrameTiming current_timing() const = 0;
    
    /**
     * @brief Get K0 (DL scheduling delay)
     */
    virtual uint8_t get_k0() const = 0;
    
    /**
     * @brief Get K1 (HARQ-ACK delay)
     */
    virtual uint8_t get_k1() const = 0;
    
    /**
     * @brief Get K2 (UL scheduling delay)
     */
    virtual uint8_t get_k2() const = 0;
};

//=============================================================================
// PHY Interface Factory
//=============================================================================

enum class PhyVendor : uint8_t {
    Simulation,     ///< Software simulation (testing)
    IntelFlexRAN,   ///< Intel FlexRAN
    Marvell,        ///< Marvell OCTEON
    Qualcomm,       ///< Qualcomm
    NVidia,         ///< NVidia
    Generic         ///< Generic FAPI implementation
};

/**
 * @brief Create PHY interface for specified vendor
 * @param vendor PHY implementation type
 * @return PHY interface instance
 */
std::unique_ptr<IPhyInterface> create_phy_interface(PhyVendor vendor);

} // namespace nexgen::nr
