/**
 * @file slice_orchestrator.hpp
 * @brief Network Slice Orchestrator for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Manages network slicing with resource partitioning, isolation,
 * and SLA enforcement
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/cell_context.hpp"
#include <unordered_map>
#include <vector>
#include <memory>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// Slice Types
//=============================================================================

enum class SliceType : uint8_t {
    eMBB,       ///< Enhanced Mobile Broadband
    URLLC,      ///< Ultra-Reliable Low-Latency Communications
    mMTC,       ///< Massive Machine-Type Communications
    Custom      ///< Custom slice definition
};

//=============================================================================
// Slice SLA Configuration
//=============================================================================

struct SliceSlaConfig {
    // Resource guarantees
    uint16_t guaranteed_rbs;        ///< Minimum RBs per slot
    uint16_t max_rbs;               ///< Maximum RBs per slot (0=unlimited)
    double   resource_share;        ///< Share of total resources (0.0-1.0)
    
    // Rate guarantees
    uint64_t guaranteed_rate_bps;   ///< Minimum aggregate rate
    uint64_t max_rate_bps;          ///< Maximum aggregate rate
    
    // Latency
    uint32_t max_latency_ms;        ///< Maximum scheduling latency
    uint32_t target_latency_ms;     ///< Target scheduling latency
    
    // Reliability
    double   target_reliability;    ///< Target BLER (e.g., 0.001 = 99.9%)
    
    // Priority
    uint8_t  slice_priority;        ///< 1 (highest) to 255 (lowest)
    
    // Isolation
    bool     strict_isolation;      ///< No resource sharing across slices
    bool     preemption_capable;    ///< Can preempt lower priority slices
    bool     preemptible;           ///< Can be preempted by higher priority
};

//=============================================================================
// Slice Configuration
//=============================================================================

struct SliceConfig {
    SliceId         slice_id;
    std::string     slice_name;
    SliceType       type;
    
    // S-NSSAI
    uint8_t         sst;            ///< Slice/Service Type
    std::optional<uint32_t> sd;     ///< Slice Differentiator
    
    // SLA
    SliceSlaConfig  sla;
    
    // Associated QoS flows
    std::vector<uint8_t> allowed_5qi;
    
    // Cell associations
    std::vector<CellId> cells;
};

//=============================================================================
// Slice Runtime State
//=============================================================================

struct SliceState {
    SliceId         slice_id;
    bool            active;
    
    // Current resource usage
    uint16_t        current_rbs_dl;
    uint16_t        current_rbs_ul;
    uint64_t        current_rate_dl_bps;
    uint64_t        current_rate_ul_bps;
    
    // UE tracking
    uint32_t        connected_ues;
    uint32_t        active_ues;
    
    // Performance metrics
    double          avg_latency_ms;
    double          resource_utilization;
    uint32_t        sla_violations;
    
    // Sliding window statistics
    struct WindowStats {
        uint64_t bytes_dl;
        uint64_t bytes_ul;
        uint32_t ues_scheduled_dl;
        uint32_t ues_scheduled_ul;
        std::chrono::steady_clock::time_point window_start;
    } current_window;
};

//=============================================================================
// Slice Resource Allocation
//=============================================================================

struct SliceResourceBudget {
    SliceId     slice_id;
    
    // RB budget for current slot
    uint16_t    available_rbs_dl;
    uint16_t    available_rbs_ul;
    
    // Remaining capacity
    uint16_t    used_rbs_dl;
    uint16_t    used_rbs_ul;
    
    // Borrowed/lent resources
    int16_t     borrowed_rbs;   // Positive = borrowed, Negative = lent
    
    [[nodiscard]] uint16_t remaining_dl() const noexcept {
        return available_rbs_dl - used_rbs_dl;
    }
    
    [[nodiscard]] uint16_t remaining_ul() const noexcept {
        return available_rbs_ul - used_rbs_ul;
    }
    
    bool consume_dl(uint16_t rbs) {
        if (rbs <= remaining_dl()) {
            used_rbs_dl += rbs;
            return true;
        }
        return false;
    }
    
    bool consume_ul(uint16_t rbs) {
        if (rbs <= remaining_ul()) {
            used_rbs_ul += rbs;
            return true;
        }
        return false;
    }
};

//=============================================================================
// Slice Orchestrator
//=============================================================================

class SliceOrchestrator {
public:
    SliceOrchestrator();
    ~SliceOrchestrator();

    //=========================================================================
    // Slice Management
    //=========================================================================
    
    /**
     * @brief Create a new network slice
     * @param config Slice configuration
     * @return true if successful
     */
    bool create_slice(const SliceConfig& config);
    
    /**
     * @brief Update slice configuration
     * @param slice_id Slice to update
     * @param config New configuration
     * @return true if successful
     */
    bool update_slice(SliceId slice_id, const SliceConfig& config);
    
    /**
     * @brief Delete a slice
     * @param slice_id Slice to delete
     * @return true if successful
     */
    bool delete_slice(SliceId slice_id);
    
    /**
     * @brief Get slice configuration
     */
    std::optional<SliceConfig> get_slice_config(SliceId slice_id) const;
    
    /**
     * @brief Get all configured slices
     */
    std::vector<SliceId> get_all_slices() const;

    //=========================================================================
    // UE-Slice Association
    //=========================================================================
    
    /**
     * @brief Associate UE with a slice
     */
    bool associate_ue(Rnti rnti, SliceId slice_id);
    
    /**
     * @brief Disassociate UE from slice
     */
    void disassociate_ue(Rnti rnti);
    
    /**
     * @brief Get slice for a UE
     */
    std::optional<SliceId> get_ue_slice(Rnti rnti) const;
    
    /**
     * @brief Get all UEs in a slice
     */
    std::vector<Rnti> get_slice_ues(SliceId slice_id) const;

    //=========================================================================
    // Resource Allocation
    //=========================================================================
    
    /**
     * @brief Calculate resource budgets for all slices
     * Called at start of each slot
     * @param cell Cell context
     * @return Budget per slice
     */
    std::unordered_map<SliceId, SliceResourceBudget> 
    calculate_budgets(const CellContext& cell);
    
    /**
     * @brief Get resource budget for a specific slice
     */
    SliceResourceBudget get_budget(SliceId slice_id) const;
    
    /**
     * @brief Request additional resources (borrowing)
     * @param slice_id Requesting slice
     * @param rbs_needed Additional RBs needed
     * @param is_downlink DL or UL
     * @return RBs granted (may be less than requested)
     */
    uint16_t request_resources(SliceId slice_id, 
                                uint16_t rbs_needed, 
                                bool is_downlink);
    
    /**
     * @brief Return unused resources for sharing
     */
    void return_resources(SliceId slice_id, 
                          uint16_t rbs_unused, 
                          bool is_downlink);

    //=========================================================================
    // Scheduling Support
    //=========================================================================
    
    /**
     * @brief Get scheduling priority adjustment for a UE based on slice
     */
    double get_priority_boost(Rnti rnti) const;
    
    /**
     * @brief Check if scheduling is allowed for UE's slice
     */
    bool is_scheduling_allowed(Rnti rnti, uint16_t rbs, bool is_downlink) const;
    
    /**
     * @brief Order slices by scheduling priority
     */
    std::vector<SliceId> get_slices_by_priority() const;

    //=========================================================================
    // SLA Monitoring
    //=========================================================================
    
    /**
     * @brief Get current state of a slice
     */
    std::optional<SliceState> get_slice_state(SliceId slice_id) const;
    
    /**
     * @brief Check SLA compliance
     * @return List of slices violating SLA
     */
    std::vector<SliceId> check_sla_compliance() const;
    
    /**
     * @brief Report scheduling for a UE
     * Called after each scheduling decision
     */
    void report_scheduling(Rnti rnti, uint16_t rbs, 
                           uint32_t bytes, bool is_downlink);

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint32_t total_slices;
        uint32_t active_slices;
        uint64_t total_sla_violations;
        double   avg_resource_utilization;
        uint64_t resource_borrowing_events;
        double   isolation_efficiency;
    };
    
    [[nodiscard]] Statistics get_statistics() const;

    //=========================================================================
    // Slot Processing
    //=========================================================================
    
    /**
     * @brief Called at start of each slot
     */
    void on_slot_start(const FrameTiming& timing);
    
    /**
     * @brief Called at end of each slot
     */
    void on_slot_end(const FrameTiming& timing);

    //=========================================================================
    // Predefined Slice Templates
    //=========================================================================
    
    /**
     * @brief Create eMBB slice with default parameters
     */
    static SliceConfig create_embb_template(SliceId id, 
                                            const std::string& name);
    
    /**
     * @brief Create URLLC slice with default parameters
     */
    static SliceConfig create_urllc_template(SliceId id, 
                                             const std::string& name);
    
    /**
     * @brief Create mMTC slice with default parameters
     */
    static SliceConfig create_mmtc_template(SliceId id, 
                                            const std::string& name);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace nexgen::nr
