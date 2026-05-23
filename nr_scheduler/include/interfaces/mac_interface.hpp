/**
 * @file mac_interface.hpp
 * @brief MAC Interface for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Defines the interface between the scheduler and upper MAC layer
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace nexgen::nr {

//=============================================================================
// MAC to Scheduler Events
//=============================================================================

/**
 * @brief UE Configuration request from MAC
 */
struct UeConfigRequest {
    Rnti        rnti;
    CellId      cell_id;
    SliceId     slice_id;
    
    // BWP Configuration
    std::vector<BwpConfig> dl_bwps;
    std::vector<BwpConfig> ul_bwps;
    
    // Logical Channels
    std::vector<LogicalChannelConfig> logical_channels;
    
    // DRX
    DrxConfig drx_config;
    
    // HARQ Configuration
    uint8_t num_dl_harq_processes;
    uint8_t num_ul_harq_processes;
    
    // Max aggregation level
    AggregationLevel max_aggregation_level;
    
    // MIMO
    uint8_t max_layers_dl;
    uint8_t max_layers_ul;
    
    // MCS limits
    uint8_t max_mcs_dl;
    uint8_t max_mcs_ul;
    
    // Power control
    int8_t p_cmax_dbm;
};

/**
 * @brief UE Release request from MAC
 */
struct UeReleaseRequest {
    Rnti    rnti;
    CellId  cell_id;
    uint8_t cause;  // Release cause
};

/**
 * @brief Downlink data arrival notification
 */
struct DlDataNotification {
    Rnti     rnti;
    CellId   cell_id;
    LcId     lc_id;
    uint32_t buffer_bytes;
    uint8_t  priority;
};

/**
 * @brief Buffer Status Report from UE
 */
struct BufferStatusReport {
    Rnti        rnti;
    CellId      cell_id;
    uint8_t     lcg_id;
    uint32_t    buffer_size_bytes;
    FrameTiming report_time;
};

/**
 * @brief Scheduling Request from UE
 */
struct SchedulingRequest {
    Rnti        rnti;
    CellId      cell_id;
    FrameTiming request_time;
};

/**
 * @brief Power Headroom Report from UE
 */
struct PowerHeadroomReport {
    Rnti        rnti;
    CellId      cell_id;
    int8_t      phr_value;  // dB
    int8_t      pcmax;      // dBm
    FrameTiming report_time;
};

/**
 * @brief Timing Advance Update
 */
struct TimingAdvanceUpdate {
    Rnti        rnti;
    CellId      cell_id;
    uint16_t    timing_advance;
    FrameTiming update_time;
};

//=============================================================================
// Scheduler to MAC Events
//=============================================================================

/**
 * @brief DL Scheduling Result - one per UE/slot
 */
struct DlSchedulingGrant {
    Rnti        rnti;
    CellId      cell_id;
    FrameTiming timing;
    
    // Resource allocation
    ResourceBlockRange rb_allocation;
    SymbolRange        symbol_allocation;
    
    // Transport block configuration
    TransportBlockConfig tb_config;
    HarqId              harq_id;
    
    // DCI
    DciFormat           dci_format;
    AggregationLevel    aggregation_level;
    
    // Per-LC allocations
    struct LcAllocation {
        LcId     lc_id;
        uint32_t bytes_allocated;
    };
    std::vector<LcAllocation> lc_allocations;
    
    // Beam
    std::optional<BeamId> beam_id;
};

/**
 * @brief UL Scheduling Result - one per UE/slot
 */
struct UlSchedulingGrant {
    Rnti        rnti;
    CellId      cell_id;
    FrameTiming timing;  // When UE should transmit
    
    // Resource allocation
    ResourceBlockRange rb_allocation;
    SymbolRange        symbol_allocation;
    
    // Transport block configuration
    TransportBlockConfig tb_config;
    HarqId              harq_id;
    
    // DCI
    DciFormat           dci_format;
    AggregationLevel    aggregation_level;
    
    // UCI on PUSCH
    bool include_harq_ack;
    bool include_csi;
    
    // Power control
    int8_t tpc_command;  // +/-1, +/-3 dB
    
    // Beam
    std::optional<BeamId> beam_id;
};

/**
 * @brief Paging scheduling notification
 */
struct PagingGrant {
    CellId      cell_id;
    FrameTiming timing;
    ResourceBlockRange rb_allocation;
    uint32_t    tb_size;
    std::vector<uint8_t> paging_message;
};

/**
 * @brief RAR scheduling notification
 */
struct RarGrant {
    CellId      cell_id;
    FrameTiming timing;
    uint16_t    ra_rnti;
    ResourceBlockRange rb_allocation;
    uint32_t    tb_size;
    
    struct RarContent {
        uint16_t preamble_id;
        uint16_t timing_advance;
        uint16_t temp_crnti;
        ResourceBlockRange ul_grant;
    };
    std::vector<RarContent> rar_list;
};

//=============================================================================
// MAC Interface
//=============================================================================

class IMacInterface {
public:
    virtual ~IMacInterface() = default;

    //=========================================================================
    // UE Management
    //=========================================================================
    
    using ConfigResult = std::function<void(Rnti, bool success, const std::string& error)>;
    
    /**
     * @brief Configure a new UE or modify existing
     * @param request UE configuration
     * @param callback Result callback
     */
    virtual void configure_ue(const UeConfigRequest& request, 
                               ConfigResult callback = nullptr) = 0;
    
    /**
     * @brief Release a UE
     * @param request Release request
     */
    virtual void release_ue(const UeReleaseRequest& request) = 0;

    //=========================================================================
    // Data Arrival
    //=========================================================================
    
    /**
     * @brief Notify scheduler of DL data availability
     * @param notification Buffer status per LC
     */
    virtual void notify_dl_data(const DlDataNotification& notification) = 0;
    
    /**
     * @brief Process BSR from UE
     * @param bsr Buffer status report
     */
    virtual void process_bsr(const BufferStatusReport& bsr) = 0;
    
    /**
     * @brief Process SR from UE
     * @param sr Scheduling request
     */
    virtual void process_sr(const SchedulingRequest& sr) = 0;
    
    /**
     * @brief Process PHR from UE
     * @param phr Power headroom report
     */
    virtual void process_phr(const PowerHeadroomReport& phr) = 0;

    //=========================================================================
    // CSI Reporting
    //=========================================================================
    
    /**
     * @brief Update CSI/CQI information for UE
     */
    virtual void update_csi(Rnti rnti, CellId cell_id, 
                            const ChannelQualityInfo& cqi) = 0;

    //=========================================================================
    // Scheduling Result Callbacks
    //=========================================================================
    
    using DlGrantCallback = std::function<void(const DlSchedulingGrant&)>;
    using UlGrantCallback = std::function<void(const UlSchedulingGrant&)>;
    using PagingCallback = std::function<void(const PagingGrant&)>;
    using RarCallback = std::function<void(const RarGrant&)>;
    
    virtual void register_dl_grant_callback(DlGrantCallback callback) = 0;
    virtual void register_ul_grant_callback(UlGrantCallback callback) = 0;
    virtual void register_paging_callback(PagingCallback callback) = 0;
    virtual void register_rar_callback(RarCallback callback) = 0;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct MacStatistics {
        uint64_t ues_configured;
        uint64_t dl_grants_sent;
        uint64_t ul_grants_sent;
        uint64_t bsr_received;
        uint64_t sr_received;
        uint64_t phr_received;
        uint64_t csi_updates;
    };
    
    virtual MacStatistics get_statistics() const = 0;
};

//=============================================================================
// Scheduler Event Handler
//=============================================================================

/**
 * @brief Interface for scheduler to receive MAC events
 * 
 * Implemented by the scheduler, called by MAC adapter
 */
class ISchedulerEventHandler {
public:
    virtual ~ISchedulerEventHandler() = default;
    
    // Slot processing
    virtual void on_slot_indication(CellId cell_id, const FrameTiming& timing) = 0;
    
    // UE events
    virtual void on_ue_config(const UeConfigRequest& config) = 0;
    virtual void on_ue_release(const UeReleaseRequest& release) = 0;
    
    // Data events
    virtual void on_dl_data(const DlDataNotification& notification) = 0;
    virtual void on_bsr(const BufferStatusReport& bsr) = 0;
    virtual void on_sr(const SchedulingRequest& sr) = 0;
    virtual void on_phr(const PowerHeadroomReport& phr) = 0;
    
    // CSI
    virtual void on_csi_update(Rnti rnti, CellId cell_id, 
                               const ChannelQualityInfo& cqi) = 0;
    
    // HARQ feedback
    virtual void on_harq_feedback(CellId cell_id, const HarqFeedback& feedback) = 0;
    
    // RACH
    virtual void on_rach_indication(CellId cell_id, const RachIndication& rach) = 0;
};

} // namespace nexgen::nr
