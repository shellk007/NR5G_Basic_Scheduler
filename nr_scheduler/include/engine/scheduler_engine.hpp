/**
 * @file scheduler_engine.hpp
 * @brief Main Scheduler Engine for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Central orchestration component that coordinates all scheduling operations
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include "../core/cell_context.hpp"
#include "../interfaces/phy_interface.hpp"
#include "../interfaces/mac_interface.hpp"

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

namespace nexgen::nr {

// Forward declarations
class IPolicyManager;
class IResourceAllocator;
class IQosEngine;
class IHarqController;
class ISliceOrchestrator;
class PdcchManager;
class PdschManager;
class PuschManager;
class PucchManager;

//=============================================================================
// Scheduler Configuration
//=============================================================================

struct SchedulerConfig {
    // Scheduling strategy
    enum class Strategy : uint8_t {
        RoundRobin,
        ProportionalFair,
        MaxThroughput,
        QoSAware,
        Custom
    } strategy = Strategy::ProportionalFair;
    
    // Timing parameters
    uint8_t scheduling_offset_slots = 2;  // K0/K2 for DL/UL
    uint8_t harq_ack_offset_slots = 4;    // K1 for HARQ-ACK
    
    // Resource limits
    uint16_t max_pdsch_per_tti = 16;
    uint16_t max_pusch_per_tti = 16;
    uint16_t max_pdcch_per_tti = 16;
    
    // Fairness parameters
    double alpha_fairness = 0.0;   // 0=max throughput, 1=proportional fair
    uint32_t history_window_ms = 1000;
    
    // QoS parameters
    bool enable_qos = true;
    double gbr_weight = 2.0;        // Higher weight for GBR traffic
    double delay_weight = 1.5;      // Weight for delay-sensitive traffic
    
    // HARQ parameters
    uint8_t max_harq_retx = 4;
    bool harq_retx_priority = true; // Prioritize retransmissions
    
    // Slicing
    bool enable_slicing = false;
    
    // Logging/Debug
    bool enable_scheduling_log = false;
    uint8_t log_level = 2;  // 0=error, 1=warn, 2=info, 3=debug
};

//=============================================================================
// Scheduling Decision
//=============================================================================

struct SchedulingDecision {
    Rnti rnti;
    
    enum class Type : uint8_t {
        None,
        NewTx,
        Retx
    } type = Type::None;
    
    // Resource allocation
    TimeFrequencyResource resources;
    TransportBlockConfig  tb_config;
    HarqId                harq_id;
    
    // Priority information
    double   priority_metric;
    uint32_t buffer_bytes;
    uint8_t  qos_priority;
    
    // For multiplexing
    std::vector<std::pair<LcId, uint32_t>> lc_bytes;  // LC -> bytes
    
    [[nodiscard]] bool is_valid() const noexcept {
        return type != Type::None && resources.num_res() > 0;
    }
};

//=============================================================================
// Scheduler Engine
//=============================================================================

class SchedulerEngine : public ISchedulerEventHandler {
public:
    explicit SchedulerEngine(const SchedulerConfig& config = {});
    ~SchedulerEngine() override;

    // Non-copyable, movable
    SchedulerEngine(const SchedulerEngine&) = delete;
    SchedulerEngine& operator=(const SchedulerEngine&) = delete;
    SchedulerEngine(SchedulerEngine&&) noexcept;
    SchedulerEngine& operator=(SchedulerEngine&&) noexcept;

    //=========================================================================
    // Initialization
    //=========================================================================
    
    /**
     * @brief Add a cell to be scheduled
     * @param config Cell configuration
     * @return true if successful
     */
    bool add_cell(const CellConfig& config);
    
    /**
     * @brief Remove a cell
     * @param cell_id Cell to remove
     */
    void remove_cell(CellId cell_id);
    
    /**
     * @brief Set PHY interface
     * @param phy PHY interface implementation
     */
    void set_phy_interface(std::shared_ptr<IPhyInterface> phy);
    
    /**
     * @brief Set custom policy manager
     * @param policy Custom scheduling policy
     */
    void set_policy_manager(std::unique_ptr<IPolicyManager> policy);
    
    /**
     * @brief Start scheduler operation
     * @return true if successful
     */
    bool start();
    
    /**
     * @brief Stop scheduler operation
     */
    void stop();

    //=========================================================================
    // ISchedulerEventHandler Implementation
    //=========================================================================
    
    void on_slot_indication(CellId cell_id, const FrameTiming& timing) override;
    void on_ue_config(const UeConfigRequest& config) override;
    void on_ue_release(const UeReleaseRequest& release) override;
    void on_dl_data(const DlDataNotification& notification) override;
    void on_bsr(const BufferStatusReport& bsr) override;
    void on_sr(const SchedulingRequest& sr) override;
    void on_phr(const PowerHeadroomReport& phr) override;
    void on_csi_update(Rnti rnti, CellId cell_id, 
                        const ChannelQualityInfo& cqi) override;
    void on_harq_feedback(CellId cell_id, const HarqFeedback& feedback) override;
    void on_rach_indication(CellId cell_id, const RachIndication& rach) override;

    //=========================================================================
    // Scheduling Output Callbacks
    //=========================================================================
    
    using DlResultCallback = std::function<void(CellId, const DlTtiRequest&)>;
    using UlResultCallback = std::function<void(CellId, const UlTtiRequest&)>;
    
    void register_dl_result_callback(DlResultCallback callback);
    void register_ul_result_callback(UlResultCallback callback);

    //=========================================================================
    // Statistics & Monitoring
    //=========================================================================
    
    struct Statistics {
        // Slot processing
        uint64_t slots_processed = 0;
        uint64_t slots_skipped = 0;
        double   avg_slot_processing_us = 0.0;
        double   max_slot_processing_us = 0.0;
        
        // DL scheduling
        uint64_t dl_bytes_scheduled = 0;
        uint64_t dl_ues_scheduled = 0;
        uint64_t dl_retransmissions = 0;
        double   dl_resource_utilization = 0.0;
        
        // UL scheduling
        uint64_t ul_bytes_scheduled = 0;
        uint64_t ul_ues_scheduled = 0;
        uint64_t ul_retransmissions = 0;
        double   ul_resource_utilization = 0.0;
        
        // QoS
        uint64_t qos_violations = 0;
        double   avg_delay_budget_margin_ms = 0.0;
        
        // Errors
        uint64_t allocation_failures = 0;
        uint64_t harq_timeouts = 0;
    };
    
    [[nodiscard]] Statistics get_statistics() const;
    void reset_statistics();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    [[nodiscard]] const SchedulerConfig& config() const noexcept;
    void update_config(const SchedulerConfig& config);

private:
    //=========================================================================
    // Internal Methods
    //=========================================================================
    
    void process_slot(CellContext& cell);
    
    // DL Scheduling Pipeline
    std::vector<SchedulingDecision> schedule_dl(CellContext& cell);
    void build_dl_tti_request(CellContext& cell, 
                               const std::vector<SchedulingDecision>& decisions);
    
    // UL Scheduling Pipeline
    std::vector<SchedulingDecision> schedule_ul(CellContext& cell);
    void build_ul_tti_request(CellContext& cell,
                               const std::vector<SchedulingDecision>& decisions);
    
    // UE Selection
    std::vector<std::shared_ptr<UeContext>> get_eligible_ues(
        CellContext& cell, bool is_downlink);
    
    // Priority calculation
    void calculate_priorities(std::vector<SchedulingDecision>& decisions,
                               CellContext& cell, bool is_downlink);
    
    // Resource allocation
    bool allocate_resources(SchedulingDecision& decision, 
                            CellContext& cell, bool is_downlink);
    
    // MCS selection
    uint8_t select_mcs(const UeContext& ue, bool is_downlink);
    
    // TBS calculation
    uint32_t calculate_tbs(uint8_t mcs, uint16_t num_rbs, uint8_t layers);

    //=========================================================================
    // Member Variables
    //=========================================================================
    
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//=============================================================================
// Policy Manager Interface
//=============================================================================

/**
 * @brief Interface for custom scheduling policies
 */
class IPolicyManager {
public:
    virtual ~IPolicyManager() = default;
    
    /**
     * @brief Calculate scheduling priority for a UE
     * @param ue UE context
     * @param cell Cell context
     * @param is_downlink true for DL, false for UL
     * @return Priority metric (higher = more urgent)
     */
    virtual double calculate_priority(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) = 0;
    
    /**
     * @brief Update historical data after scheduling
     * @param ue UE that was scheduled
     * @param bytes_scheduled Bytes allocated
     */
    virtual void update_history(
        const UeContext& ue,
        uint32_t bytes_scheduled) = 0;
    
    /**
     * @brief Called at start of each slot
     */
    virtual void on_slot_start(const FrameTiming& timing) = 0;
};

//=============================================================================
// Resource Allocator Interface
//=============================================================================

/**
 * @brief Interface for resource allocation strategies
 */
class IResourceAllocator {
public:
    virtual ~IResourceAllocator() = default;
    
    /**
     * @brief Allocate resources for a scheduling decision
     * @param decision Decision to allocate for (modified in place)
     * @param cell Cell context
     * @param is_downlink DL or UL allocation
     * @return true if allocation successful
     */
    virtual bool allocate(
        SchedulingDecision& decision,
        CellContext& cell,
        bool is_downlink) = 0;
    
    /**
     * @brief Get maximum allocatable resources for a UE
     * @param ue UE context
     * @param cell Cell context
     * @param is_downlink DL or UL
     * @return Maximum RBs that can be allocated
     */
    virtual uint16_t max_allocatable_rbs(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) = 0;
};

//=============================================================================
// Factory Functions
//=============================================================================

/**
 * @brief Create default policy manager
 */
std::unique_ptr<IPolicyManager> create_policy_manager(
    SchedulerConfig::Strategy strategy);

/**
 * @brief Create default resource allocator
 */
std::unique_ptr<IResourceAllocator> create_resource_allocator();

} // namespace nexgen::nr
