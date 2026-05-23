/**
 * @file qos_engine.hpp
 * @brief QoS Engine for NexGen 5G NR Scheduler
 * @version 1.0
 * 
 * Handles 5QI-based QoS management, GBR/MBR enforcement,
 * and delay budget tracking
 */

#pragma once

#include "../core/nr_types.hpp"
#include "../core/ue_context.hpp"
#include <chrono>
#include <unordered_map>
#include <deque>

namespace nexgen::nr {

//=============================================================================
// QoS Configuration
//=============================================================================

struct QosEngineConfig {
    // GBR enforcement
    double gbr_enforcement_strictness = 0.95;  // % of GBR to guarantee
    uint32_t gbr_measurement_window_ms = 100;
    
    // Delay budget
    double delay_margin_factor = 0.8;  // Schedule before X% of PDB
    uint32_t delay_check_granularity_ms = 1;
    
    // Rate shaping
    bool enable_mbr_enforcement = true;
    uint32_t token_bucket_depth_ms = 10;
    
    // Priority mapping
    bool use_dynamic_priority = true;
    double delay_urgency_weight = 2.0;
    double gbr_deficit_weight = 1.5;
};

//=============================================================================
// QoS Flow State
//=============================================================================

struct QosFlowState {
    uint8_t     qfi;
    QosFlowParams params;
    
    // Token bucket for rate shaping
    struct TokenBucket {
        int64_t  tokens;         // Available tokens (bytes)
        int64_t  bucket_size;    // Max tokens
        int64_t  fill_rate_bps;  // Tokens per second
        std::chrono::steady_clock::time_point last_update;
        
        void update(std::chrono::steady_clock::time_point now) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                now - last_update);
            int64_t new_tokens = (fill_rate_bps * elapsed.count()) / 1000000;
            tokens = std::min(tokens + new_tokens, bucket_size);
            last_update = now;
        }
        
        bool consume(int64_t bytes) {
            if (tokens >= bytes) {
                tokens -= bytes;
                return true;
            }
            return false;
        }
        
        [[nodiscard]] int64_t available() const { return tokens; }
    };
    
    TokenBucket dl_bucket;
    TokenBucket ul_bucket;
    
    // GBR tracking
    struct GbrTracker {
        std::deque<std::pair<std::chrono::steady_clock::time_point, uint32_t>> 
            history;  // (time, bytes)
        uint32_t window_ms = 100;
        
        void add_sample(uint32_t bytes) {
            auto now = std::chrono::steady_clock::now();
            history.emplace_back(now, bytes);
            
            // Prune old samples
            auto cutoff = now - std::chrono::milliseconds(window_ms);
            while (!history.empty() && history.front().first < cutoff) {
                history.pop_front();
            }
        }
        
        [[nodiscard]] uint64_t current_rate_bps() const {
            if (history.size() < 2) return 0;
            
            auto& oldest = history.front();
            auto& newest = history.back();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                newest.first - oldest.first);
            
            if (duration.count() == 0) return 0;
            
            uint64_t total_bytes = 0;
            for (const auto& [_, bytes] : history) {
                total_bytes += bytes;
            }
            
            return (total_bytes * 8000) / duration.count();  // bits per second
        }
    };
    
    GbrTracker dl_gbr_tracker;
    GbrTracker ul_gbr_tracker;
    
    // Delay tracking
    struct DelayTracker {
        uint32_t pdb_ms;
        std::chrono::steady_clock::time_point oldest_packet_arrival;
        bool has_pending_data = false;
        
        void packet_arrived() {
            if (!has_pending_data) {
                oldest_packet_arrival = std::chrono::steady_clock::now();
                has_pending_data = true;
            }
        }
        
        void packet_transmitted() {
            has_pending_data = false;
        }
        
        [[nodiscard]] double urgency() const {
            if (!has_pending_data) return 0.0;
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - oldest_packet_arrival);
            
            return static_cast<double>(elapsed.count()) / pdb_ms;
        }
        
        [[nodiscard]] bool is_critical() const {
            return urgency() > 0.8;  // 80% of PDB elapsed
        }
    };
    
    DelayTracker dl_delay_tracker;
    DelayTracker ul_delay_tracker;
    
    // Statistics
    uint64_t total_dl_bytes = 0;
    uint64_t total_ul_bytes = 0;
    uint32_t pdb_violations = 0;
    uint32_t gbr_violations = 0;
};

//=============================================================================
// QoS Priority Result
//=============================================================================

struct QosPriorityResult {
    double   base_priority;      // From 5QI priority level
    double   gbr_factor;         // GBR urgency multiplier
    double   delay_factor;       // Delay urgency multiplier
    double   final_priority;     // Combined priority
    
    bool     is_gbr;
    bool     is_delay_critical;
    uint32_t min_bytes_required; // Minimum bytes to schedule
    uint32_t max_bytes_allowed;  // Maximum bytes (MBR)
};

//=============================================================================
// QoS Engine
//=============================================================================

class QosEngine {
public:
    explicit QosEngine(const QosEngineConfig& config = {});
    ~QosEngine();

    //=========================================================================
    // Flow Management
    //=========================================================================
    
    /**
     * @brief Register a new QoS flow for a UE
     */
    void register_flow(Rnti rnti, const QosFlowParams& params);
    
    /**
     * @brief Remove a QoS flow
     */
    void remove_flow(Rnti rnti, uint8_t qfi);
    
    /**
     * @brief Remove all flows for a UE
     */
    void remove_ue_flows(Rnti rnti);

    //=========================================================================
    // Priority Calculation
    //=========================================================================
    
    /**
     * @brief Calculate QoS-aware scheduling priority
     * @param rnti UE identifier
     * @param qfi QoS Flow Identifier
     * @param is_downlink DL or UL
     * @param buffer_bytes Pending data in bytes
     * @return Priority result with all factors
     */
    QosPriorityResult calculate_priority(
        Rnti rnti,
        uint8_t qfi,
        bool is_downlink,
        uint32_t buffer_bytes);

    //=========================================================================
    // Rate Enforcement
    //=========================================================================
    
    /**
     * @brief Check if transmission is allowed under MBR
     * @return Max bytes allowed for this transmission
     */
    uint32_t get_allowed_bytes(Rnti rnti, uint8_t qfi, bool is_downlink);
    
    /**
     * @brief Report transmitted bytes for rate tracking
     */
    void report_transmission(Rnti rnti, uint8_t qfi, 
                              bool is_downlink, uint32_t bytes);

    //=========================================================================
    // Delay Tracking
    //=========================================================================
    
    /**
     * @brief Notify of new data arrival for delay tracking
     */
    void notify_data_arrival(Rnti rnti, uint8_t qfi, bool is_downlink);
    
    /**
     * @brief Notify of data transmission for delay tracking
     */
    void notify_data_transmitted(Rnti rnti, uint8_t qfi, bool is_downlink);
    
    /**
     * @brief Get flows with critical delay
     */
    std::vector<std::pair<Rnti, uint8_t>> get_delay_critical_flows() const;

    //=========================================================================
    // GBR Management
    //=========================================================================
    
    /**
     * @brief Get current rate for a GBR flow
     */
    uint64_t get_current_rate(Rnti rnti, uint8_t qfi, bool is_downlink) const;
    
    /**
     * @brief Get GBR deficit (how much below GBR)
     */
    int64_t get_gbr_deficit(Rnti rnti, uint8_t qfi, bool is_downlink) const;
    
    /**
     * @brief Get flows with GBR deficit
     */
    std::vector<std::pair<Rnti, uint8_t>> get_gbr_deficit_flows() const;

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        uint64_t total_flows;
        uint64_t gbr_flows;
        uint64_t delay_critical_events;
        uint64_t pdb_violations;
        uint64_t mbr_throttle_events;
        double   avg_delay_margin;
        double   gbr_satisfaction_ratio;
    };
    
    [[nodiscard]] Statistics get_statistics() const;
    void reset_statistics();

    //=========================================================================
    // Configuration
    //=========================================================================
    
    void update_config(const QosEngineConfig& config);
    [[nodiscard]] const QosEngineConfig& config() const;

    //=========================================================================
    // Periodic Processing
    //=========================================================================
    
    /**
     * @brief Called each slot for rate bucket updates
     */
    void on_slot_tick();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//=============================================================================
// 5QI Table
//=============================================================================

/**
 * @brief Get standard 5QI characteristics
 * @param fiveqi 5QI value (1-127)
 * @return QoS characteristics for the 5QI
 */
QosCharacteristics get_5qi_characteristics(uint8_t fiveqi);

/**
 * @brief Check if 5QI is for GBR traffic
 */
inline bool is_gbr_5qi(uint8_t fiveqi) {
    return fiveqi >= 1 && fiveqi <= 4;
}

/**
 * @brief Check if 5QI is delay-critical
 */
inline bool is_delay_critical_5qi(uint8_t fiveqi) {
    return fiveqi == 82 || fiveqi == 83 || fiveqi == 84 || fiveqi == 85;
}

} // namespace nexgen::nr
