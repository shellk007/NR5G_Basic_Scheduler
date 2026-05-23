/**
 * @file scheduler_engine.cpp
 * @brief Scheduler Engine Implementation for NexGen 5G NR Scheduler
 * @version 1.0
 */

#include "../include/engine/scheduler_engine.hpp"
#include "../include/channels/pdcch_manager.hpp"
#include "../include/channels/pdsch_manager.hpp"
#include "../include/channels/pusch_manager.hpp"
#include "../include/channels/pucch_manager.hpp"
#include "../include/services/qos_engine.hpp"
#include "../include/services/slice_orchestrator.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cmath>
#include <map>

namespace nexgen::nr {

//=============================================================================
// HARQ Timing Tracker
//=============================================================================

/**
 * @brief Tracks pending HARQ transmissions to match feedback to UE/process
 * 
 * When we schedule a PDSCH, we know when the UE should send HARQ-ACK (K1 slots later).
 * This tracker indexes by expected feedback slot so we can route UCI indications
 * back to the correct UE and HARQ process.
 */
class HarqTimingTracker {
public:
    struct PendingFeedback {
        CellId   cell_id;
        Rnti     rnti;
        HarqId   harq_id;
        bool     is_downlink;  // true for DL HARQ-ACK, false for UL CRC
        uint32_t tx_slot;      // When transmission occurred
    };
    
    /**
     * @brief Register an expected HARQ feedback
     * @param expected_slot Absolute slot when feedback is expected
     * @param info Pending feedback information
     */
    void register_pending(uint32_t expected_slot, const PendingFeedback& info) {
        std::lock_guard lock(mutex_);
        pending_[expected_slot].push_back(info);
        
        // Cleanup old entries (>100 slots old)
        auto it = pending_.begin();
        while (it != pending_.end() && it->first + 100 < expected_slot) {
            it = pending_.erase(it);
        }
    }
    
    /**
     * @brief Find pending feedback for a given slot and RNTI
     * @return Matching entry if found
     */
    std::optional<PendingFeedback> find_and_remove(
        uint32_t feedback_slot, 
        Rnti rnti,
        std::optional<HarqId> harq_id = std::nullopt) {
        
        std::lock_guard lock(mutex_);
        auto slot_it = pending_.find(feedback_slot);
        if (slot_it == pending_.end()) {
            return std::nullopt;
        }
        
        auto& entries = slot_it->second;
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->rnti == rnti && (!harq_id || it->harq_id == *harq_id)) {
                PendingFeedback result = *it;
                entries.erase(it);
                if (entries.empty()) {
                    pending_.erase(slot_it);
                }
                return result;
            }
        }
        return std::nullopt;
    }
    
    /**
     * @brief Get all pending feedbacks for a slot (for batch processing)
     */
    std::vector<PendingFeedback> get_all_for_slot(uint32_t feedback_slot) {
        std::lock_guard lock(mutex_);
        auto it = pending_.find(feedback_slot);
        if (it != pending_.end()) {
            return it->second;
        }
        return {};
    }
    
    /**
     * @brief Clear all pending entries (e.g., on UE release)
     */
    void clear_for_ue(Rnti rnti) {
        std::lock_guard lock(mutex_);
        for (auto& [slot, entries] : pending_) {
            entries.erase(
                std::remove_if(entries.begin(), entries.end(),
                    [rnti](const PendingFeedback& pf) { return pf.rnti == rnti; }),
                entries.end());
        }
    }

private:
    std::mutex mutex_;
    std::map<uint32_t, std::vector<PendingFeedback>> pending_;  // slot -> feedbacks
};

//=============================================================================
// Proportional Fair Policy Implementation
//=============================================================================

class ProportionalFairPolicy : public IPolicyManager {
public:
    explicit ProportionalFairPolicy(double alpha = 1.0, uint32_t window_ms = 1000)
        : alpha_(alpha), window_ms_(window_ms) {}

    double calculate_priority(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) override {
        
        uint32_t buffer = is_downlink ? ue.total_dl_buffer() : ue.total_ul_buffer();
        if (buffer == 0) return 0.0;
        
        // Get instantaneous achievable rate
        double inst_rate = estimate_instantaneous_rate(ue, cell, is_downlink);
        
        // Get historical average rate
        double avg_rate = get_average_rate(ue.rnti, is_downlink);
        if (avg_rate < 1.0) avg_rate = 1.0;  // Avoid division by zero
        
        // Proportional fair metric: R_inst / (R_avg)^alpha
        double pf_metric = inst_rate / std::pow(avg_rate, alpha_);
        
        // Boost for pending data
        double buffer_factor = std::log2(1.0 + buffer / 1000.0);
        
        return pf_metric * buffer_factor;
    }

    void update_history(const UeContext& ue, uint32_t bytes_scheduled) override {
        auto now = std::chrono::steady_clock::now();
        
        auto& history = rate_history_[ue.rnti];
        history.samples.emplace_back(now, bytes_scheduled);
        
        // Prune old samples
        auto cutoff = now - std::chrono::milliseconds(window_ms_);
        while (!history.samples.empty() && history.samples.front().first < cutoff) {
            history.samples.pop_front();
        }
        
        // Update cached average
        history.cached_avg_rate = compute_avg_rate(history);
    }

    void on_slot_start(const FrameTiming& /*timing*/) override {
        // Could decay rates here for fairness
    }

private:
    double estimate_instantaneous_rate(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) const {
        
        uint8_t cqi = is_downlink ? ue.effective_cqi() : 10;  // Assume moderate UL
        uint8_t mcs = ue.mcs_from_cqi();
        uint16_t max_rbs = cell.config().num_rbs / 4;  // Assume max 25% allocation
        
        return tbs_table_lookup(mcs, max_rbs, 1) * 1000.0;  // Bytes/slot to bps
    }

    double get_average_rate(Rnti rnti, bool /*is_downlink*/) const {
        auto it = rate_history_.find(rnti);
        if (it == rate_history_.end()) return 1.0;
        return it->second.cached_avg_rate;
    }

    double compute_avg_rate(const struct RateHistory& history) const {
        if (history.samples.size() < 2) return 1.0;
        
        uint64_t total_bytes = 0;
        for (const auto& [_, bytes] : history.samples) {
            total_bytes += bytes;
        }
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            history.samples.back().first - history.samples.front().first);
        
        if (duration.count() == 0) return 1.0;
        return (total_bytes * 8000.0) / duration.count();
    }

    struct RateHistory {
        std::deque<std::pair<std::chrono::steady_clock::time_point, uint32_t>> samples;
        double cached_avg_rate = 1.0;
    };

    double alpha_;
    uint32_t window_ms_;
    std::unordered_map<Rnti, RateHistory> rate_history_;
};

//=============================================================================
// Round Robin Policy Implementation
//=============================================================================

class RoundRobinPolicy : public IPolicyManager {
public:
    double calculate_priority(
        const UeContext& ue,
        const CellContext& /*cell*/,
        bool is_downlink) override {
        
        uint32_t buffer = is_downlink ? ue.total_dl_buffer() : ue.total_ul_buffer();
        if (buffer == 0) return 0.0;
        
        // Get last scheduled slot
        auto it = last_scheduled_.find(ue.rnti);
        if (it == last_scheduled_.end()) {
            return 1000.0;  // Never scheduled, high priority
        }
        
        // Priority based on how long since last scheduled
        return static_cast<double>(current_slot_ - it->second);
    }

    void update_history(const UeContext& ue, uint32_t /*bytes*/) override {
        last_scheduled_[ue.rnti] = current_slot_;
    }

    void on_slot_start(const FrameTiming& timing) override {
        current_slot_ = timing.absolute_slot();
    }

private:
    uint32_t current_slot_ = 0;
    std::unordered_map<Rnti, uint32_t> last_scheduled_;
};

//=============================================================================
// Max Throughput Policy Implementation
//=============================================================================

class MaxThroughputPolicy : public IPolicyManager {
public:
    double calculate_priority(
        const UeContext& ue,
        const CellContext& /*cell*/,
        bool is_downlink) override {
        
        uint32_t buffer = is_downlink ? ue.total_dl_buffer() : ue.total_ul_buffer();
        if (buffer == 0) return 0.0;
        
        // Simple: prioritize by CQI (higher CQI = higher spectral efficiency)
        return static_cast<double>(ue.effective_cqi()) * 
               std::min(buffer / 1000.0, 10.0);  // Cap buffer influence
    }

    void update_history(const UeContext& /*ue*/, uint32_t /*bytes*/) override {}
    void on_slot_start(const FrameTiming& /*timing*/) override {}
};

//=============================================================================
// QoS-Aware Policy Implementation
//=============================================================================

/**
 * @brief QoS-Aware policy that explicitly combines PF with QoS factors
 * 
 * Unlike ProportionalFair which only considers throughput fairness,
 * this policy integrates QoS metrics directly into the priority calculation.
 */
class QosAwarePolicy : public IPolicyManager {
public:
    explicit QosAwarePolicy(double alpha = 0.8, double qos_weight = 2.0)
        : pf_policy_(alpha), qos_weight_(qos_weight) {}

    double calculate_priority(
        const UeContext& ue,
        const CellContext& cell,
        bool is_downlink) override {
        
        // Base PF priority
        double pf_priority = pf_policy_.calculate_priority(ue, cell, is_downlink);
        
        // This policy explicitly boosts based on QoS characteristics
        // (The QoS engine will provide additional detail, but we apply
        //  a simpler heuristic here based on LC priority)
        double qos_boost = 1.0;
        
        for (const auto& [lc_id, lc_config] : ue.logical_channels) {
            // Higher priority LC (lower number) = higher boost
            double lc_boost = 17.0 / (lc_config.priority + 1);
            qos_boost = std::max(qos_boost, lc_boost);
            
            // GBR flows get additional boost
            if (lc_config.qos_params && 
                lc_config.qos_params->characteristics.resource_type == QosResourceType::GBR) {
                qos_boost *= 1.5;
            }
        }
        
        return pf_priority * std::pow(qos_boost, qos_weight_);
    }

    void update_history(const UeContext& ue, uint32_t bytes) override {
        pf_policy_.update_history(ue, bytes);
    }

    void on_slot_start(const FrameTiming& timing) override {
        pf_policy_.on_slot_start(timing);
    }

private:
    ProportionalFairPolicy pf_policy_;
    double qos_weight_;
};

//=============================================================================
// Default Resource Allocator
//=============================================================================

class DefaultResourceAllocator : public IResourceAllocator {
public:
    bool allocate(
        SchedulingDecision& decision,
        CellContext& cell,
        bool is_downlink) override {
        
        auto& slot = cell.current_slot();
        auto& grid = is_downlink ? slot.dl_grid : slot.ul_grid;
        
        // Get UE for MCS selection
        auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
        if (!ue) return false;
        
        // Select MCS based on channel quality (NOT hardcoded!)
        uint8_t mcs = ue->mcs_from_cqi();
        uint8_t num_layers = std::min<uint8_t>(ue->dl_channel_quality.rank_indicator, 
                                                 cell.config().max_layers);
        if (num_layers == 0) num_layers = 1;
        
        // Calculate number of available symbols (accounting for PDCCH/DMRS)
        uint8_t start_symbol = 2;  // After CORESET
        uint8_t num_symbols = 12;  // Typical PDSCH/PUSCH length
        
        // Calculate required RBs using proper TBS formula
        uint16_t required_rbs = min_rbs_for_tbs(
            decision.buffer_bytes, 
            mcs, 
            num_layers, 
            num_symbols,
            cell.config().num_rbs);
        
        // Cap at maximum allocatable
        required_rbs = std::min(required_rbs, 
            max_allocatable_rbs_internal(cell, is_downlink));
        
        if (required_rbs == 0) return false;
        
        // Find contiguous allocation
        auto range = grid.find_contiguous(required_rbs, start_symbol, num_symbols);
        if (!range) {
            // Try with fewer RBs if buffer can accept partial
            for (uint16_t rbs = required_rbs - 1; rbs >= 1; --rbs) {
                range = grid.find_contiguous(rbs, start_symbol, num_symbols);
                if (range) break;
            }
            if (!range) return false;
        }
        
        // Allocate
        TimeFrequencyResource res;
        res.rb_range = *range;
        res.symbol_range = {start_symbol, num_symbols};
        
        if (!grid.allocate(res)) {
            return false;
        }
        
        decision.resources = res;
        decision.tb_config.mcs_index = mcs;
        decision.tb_config.mcs_table = 0;  // 64QAM table
        decision.tb_config.num_layers = num_layers;
        decision.tb_config.tb_size_bytes = tbs_table_lookup(
            mcs, range->num_rbs, num_layers, num_symbols);
        
        return true;
    }

    uint16_t max_allocatable_rbs(
        const UeContext& /*ue*/,
        const CellContext& cell,
        bool is_downlink) override {
        return max_allocatable_rbs_internal(cell, is_downlink);
    }

private:
    uint16_t max_allocatable_rbs_internal(const CellContext& cell, bool is_downlink) const {
        auto& slot = cell.current_slot();
        auto& grid = is_downlink ? slot.dl_grid : slot.ul_grid;
        
        // Return maximum available contiguous block
        uint16_t max_rbs = 0;
        for (uint16_t rbs = cell.config().num_rbs; rbs > 0; --rbs) {
            if (grid.find_contiguous(rbs, 2, 12)) {
                max_rbs = rbs;
                break;
            }
        }
        return max_rbs;
    }
};

//=============================================================================
// Scheduler Engine Implementation
//=============================================================================

struct SchedulerEngine::Impl {
    SchedulerConfig config;
    
    // State
    std::atomic<bool> running{false};
    std::mutex mutex;
    
    // Cells
    std::unordered_map<CellId, std::unique_ptr<CellContext>> cells;
    
    // Components
    std::unique_ptr<IPolicyManager> policy_manager;
    std::unique_ptr<IResourceAllocator> resource_allocator;
    std::unique_ptr<QosEngine> qos_engine;
    std::unique_ptr<SliceOrchestrator> slice_orchestrator;
    
    // HARQ timing tracker for matching feedback to UE/process
    HarqTimingTracker harq_tracker;
    
    // Channel managers (per cell)
    std::unordered_map<CellId, std::unique_ptr<PdcchManager>> pdcch_managers;
    std::unordered_map<CellId, std::unique_ptr<PdschManager>> pdsch_managers;
    std::unordered_map<CellId, std::unique_ptr<PuschManager>> pusch_managers;
    std::unordered_map<CellId, std::unique_ptr<PucchManager>> pucch_managers;
    
    // Interfaces
    std::shared_ptr<IPhyInterface> phy_interface;
    
    // Callbacks
    DlResultCallback dl_result_callback;
    UlResultCallback ul_result_callback;
    
    // Statistics
    Statistics stats;
    std::chrono::steady_clock::time_point last_stats_reset;
    
    Impl(const SchedulerConfig& cfg) : config(cfg) {
        // Create default components
        policy_manager = create_policy_manager(cfg.strategy);
        resource_allocator = std::make_unique<DefaultResourceAllocator>();
        qos_engine = std::make_unique<QosEngine>();
        slice_orchestrator = std::make_unique<SliceOrchestrator>();
        
        last_stats_reset = std::chrono::steady_clock::now();
    }
};

SchedulerEngine::SchedulerEngine(const SchedulerConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

SchedulerEngine::~SchedulerEngine() {
    stop();
}

SchedulerEngine::SchedulerEngine(SchedulerEngine&&) noexcept = default;
SchedulerEngine& SchedulerEngine::operator=(SchedulerEngine&&) noexcept = default;

bool SchedulerEngine::add_cell(const CellConfig& config) {
    std::lock_guard lock(impl_->mutex);
    
    if (impl_->cells.count(config.cell_id)) {
        return false;  // Cell already exists
    }
    
    auto cell = std::make_unique<CellContext>(config);
    
    // Create channel managers for this cell
    impl_->pdcch_managers[config.cell_id] = std::make_unique<PdcchManager>();
    impl_->pdsch_managers[config.cell_id] = std::make_unique<PdschManager>();
    impl_->pusch_managers[config.cell_id] = std::make_unique<PuschManager>();
    impl_->pucch_managers[config.cell_id] = std::make_unique<PucchManager>();
    
    impl_->cells[config.cell_id] = std::move(cell);
    return true;
}

void SchedulerEngine::remove_cell(CellId cell_id) {
    std::lock_guard lock(impl_->mutex);
    
    impl_->cells.erase(cell_id);
    impl_->pdcch_managers.erase(cell_id);
    impl_->pdsch_managers.erase(cell_id);
    impl_->pusch_managers.erase(cell_id);
    impl_->pucch_managers.erase(cell_id);
}

void SchedulerEngine::set_phy_interface(std::shared_ptr<IPhyInterface> phy) {
    impl_->phy_interface = std::move(phy);
}

void SchedulerEngine::set_policy_manager(std::unique_ptr<IPolicyManager> policy) {
    impl_->policy_manager = std::move(policy);
}

bool SchedulerEngine::start() {
    if (impl_->running.exchange(true)) {
        return false;  // Already running
    }
    
    // Register PHY callbacks if available
    if (impl_->phy_interface) {
        impl_->phy_interface->register_slot_callback(
            [this](CellId cell_id, const FrameTiming& timing) {
                on_slot_indication(cell_id, timing);
            });
    }
    
    return true;
}

void SchedulerEngine::stop() {
    impl_->running = false;
}

void SchedulerEngine::on_slot_indication(CellId cell_id, const FrameTiming& timing) {
    auto start_time = std::chrono::steady_clock::now();
    
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(cell_id);
    if (it == impl_->cells.end()) {
        ++impl_->stats.slots_skipped;
        return;
    }
    
    auto& cell = *it->second;
    
    // Advance cell to new slot
    cell.advance_to_slot(timing);
    
    // Notify components
    impl_->policy_manager->on_slot_start(timing);
    impl_->slice_orchestrator->on_slot_start(timing);
    impl_->qos_engine->on_slot_tick();
    
    // Reset channel managers for new slot
    impl_->pdcch_managers[cell_id]->reset_slot();
    impl_->pdsch_managers[cell_id]->reset_slot();
    impl_->pusch_managers[cell_id]->reset_slot();
    impl_->pucch_managers[cell_id]->reset_slot();
    
    // Process the slot
    process_slot(cell);
    
    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    auto processing_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();
    
    ++impl_->stats.slots_processed;
    impl_->stats.avg_slot_processing_us = 
        (impl_->stats.avg_slot_processing_us * (impl_->stats.slots_processed - 1) + 
         processing_us) / impl_->stats.slots_processed;
    impl_->stats.max_slot_processing_us = 
        std::max(impl_->stats.max_slot_processing_us, 
                 static_cast<double>(processing_us));
}

void SchedulerEngine::process_slot(CellContext& cell) {
    // Schedule DL if allowed
    if (cell.current_slot().can_schedule_dl) {
        auto dl_decisions = schedule_dl(cell);
        if (!dl_decisions.empty()) {
            build_dl_tti_request(cell, dl_decisions);
        }
    }
    
    // Schedule UL if allowed
    if (cell.current_slot().can_schedule_ul) {
        auto ul_decisions = schedule_ul(cell);
        if (!ul_decisions.empty()) {
            build_ul_tti_request(cell, ul_decisions);
        }
    }
    
    // Update cell statistics
    cell.statistics().total_slots_processed++;
    cell.statistics().total_dl_bytes += cell.current_slot().dl_bytes_scheduled;
    cell.statistics().total_ul_bytes += cell.current_slot().ul_bytes_scheduled;
    
    impl_->slice_orchestrator->on_slot_end(cell.current_slot().timing);
}

std::vector<SchedulingDecision> SchedulerEngine::schedule_dl(CellContext& cell) {
    std::vector<SchedulingDecision> decisions;
    
    // Get eligible UEs
    auto eligible_ues = get_eligible_ues(cell, true);
    if (eligible_ues.empty()) {
        return decisions;
    }
    
    // Create decision for each eligible UE
    for (auto& ue : eligible_ues) {
        // Check for retransmissions first
        auto* retx_proc = ue->dl_harq.get_retx_process();
        if (retx_proc && impl_->config.harq_retx_priority) {
            SchedulingDecision decision;
            decision.rnti = ue->rnti;
            decision.type = SchedulingDecision::Type::Retx;
            decision.harq_id = retx_proc->id;
            decision.buffer_bytes = retx_proc->tb_config.tb_size_bytes;
            decisions.push_back(decision);
            continue;
        }
        
        // New transmission
        uint32_t buffer = ue->total_dl_buffer();
        if (buffer == 0) continue;
        
        SchedulingDecision decision;
        decision.rnti = ue->rnti;
        decision.type = SchedulingDecision::Type::NewTx;
        decision.buffer_bytes = buffer;
        
        // Get HARQ process
        auto* harq_proc = ue->dl_harq.get_free_process();
        if (!harq_proc) continue;
        decision.harq_id = harq_proc->id;
        
        decisions.push_back(decision);
    }
    
    // Calculate priorities
    calculate_priorities(decisions, cell, true);
    
    // Sort by priority (descending)
    std::sort(decisions.begin(), decisions.end(),
        [](const auto& a, const auto& b) {
            return a.priority_metric > b.priority_metric;
        });
    
    // Allocate resources
    std::vector<SchedulingDecision> successful;
    for (auto& decision : decisions) {
        if (successful.size() >= impl_->config.max_pdsch_per_tti) {
            break;
        }
        
        if (allocate_resources(decision, cell, true)) {
            successful.push_back(decision);
            
            // Update policy
            auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
            if (ue) {
                impl_->policy_manager->update_history(*ue, decision.tb_config.tb_size_bytes);
            }
        } else {
            ++impl_->stats.allocation_failures;
        }
    }
    
    return successful;
}

std::vector<SchedulingDecision> SchedulerEngine::schedule_ul(CellContext& cell) {
    std::vector<SchedulingDecision> decisions;
    
    auto eligible_ues = get_eligible_ues(cell, false);
    if (eligible_ues.empty()) {
        return decisions;
    }
    
    for (auto& ue : eligible_ues) {
        // Check for retransmissions first
        auto* retx_proc = ue->ul_harq.get_retx_process();
        if (retx_proc && impl_->config.harq_retx_priority) {
            SchedulingDecision decision;
            decision.rnti = ue->rnti;
            decision.type = SchedulingDecision::Type::Retx;
            decision.harq_id = retx_proc->id;
            decision.buffer_bytes = retx_proc->tb_config.tb_size_bytes;
            decisions.push_back(decision);
            continue;
        }
        
        // New transmission - check BSR/SR
        uint32_t buffer = ue->total_ul_buffer();
        if (buffer == 0 && !ue->ul_buffer_status.sr_pending) continue;
        
        SchedulingDecision decision;
        decision.rnti = ue->rnti;
        decision.type = SchedulingDecision::Type::NewTx;
        decision.buffer_bytes = std::max(buffer, 100u);  // Min grant for SR
        
        auto* harq_proc = ue->ul_harq.get_free_process();
        if (!harq_proc) continue;
        decision.harq_id = harq_proc->id;
        
        decisions.push_back(decision);
    }
    
    calculate_priorities(decisions, cell, false);
    
    std::sort(decisions.begin(), decisions.end(),
        [](const auto& a, const auto& b) {
            return a.priority_metric > b.priority_metric;
        });
    
    std::vector<SchedulingDecision> successful;
    for (auto& decision : decisions) {
        if (successful.size() >= impl_->config.max_pusch_per_tti) {
            break;
        }
        
        if (allocate_resources(decision, cell, false)) {
            successful.push_back(decision);
            
            auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
            if (ue) {
                impl_->policy_manager->update_history(*ue, decision.tb_config.tb_size_bytes);
            }
        } else {
            ++impl_->stats.allocation_failures;
        }
    }
    
    return successful;
}

std::vector<std::shared_ptr<UeContext>> SchedulerEngine::get_eligible_ues(
    CellContext& cell, bool is_downlink) {
    
    std::vector<std::shared_ptr<UeContext>> eligible;
    
    cell.ue_repository()->for_each([&](const std::shared_ptr<UeContext>& ue) {
        // Check DRX
        if (!ue->is_in_active_time(cell.current_slot().timing)) {
            return;
        }
        
        // Check buffer
        if (is_downlink) {
            if (ue->total_dl_buffer() == 0 && !ue->dl_harq.get_retx_process()) {
                return;
            }
        } else {
            if (ue->total_ul_buffer() == 0 && 
                !ue->ul_buffer_status.sr_pending &&
                !ue->ul_harq.get_retx_process()) {
                return;
            }
        }
        
        // Check slicing
        if (impl_->config.enable_slicing) {
            if (!impl_->slice_orchestrator->is_scheduling_allowed(
                    ue->rnti, 1, is_downlink)) {
                return;
            }
        }
        
        eligible.push_back(ue);
    });
    
    return eligible;
}

void SchedulerEngine::calculate_priorities(
    std::vector<SchedulingDecision>& decisions,
    CellContext& cell,
    bool is_downlink) {
    
    for (auto& decision : decisions) {
        auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
        if (!ue) {
            decision.priority_metric = 0.0;
            continue;
        }
        
        // Base priority from policy
        double base_priority = impl_->policy_manager->calculate_priority(
            *ue, cell, is_downlink);
        
        // QoS adjustment
        if (impl_->config.enable_qos) {
            // Find highest priority LC
            uint8_t best_qfi = 9;  // Default non-GBR
            for (const auto& [lc_id, lc_config] : ue->logical_channels) {
                if (lc_config.qos_params) {
                    best_qfi = std::min(best_qfi, lc_config.qos_params->qfi);
                }
            }
            
            auto qos_result = impl_->qos_engine->calculate_priority(
                ue->rnti, best_qfi, is_downlink, decision.buffer_bytes);
            
            decision.priority_metric = base_priority * qos_result.final_priority;
            decision.qos_priority = qos_result.is_delay_critical ? 1 : 2;
        } else {
            decision.priority_metric = base_priority;
            decision.qos_priority = 5;
        }
        
        // Retx priority boost
        if (decision.type == SchedulingDecision::Type::Retx) {
            decision.priority_metric *= 2.0;
        }
        
        // Slice priority boost
        if (impl_->config.enable_slicing) {
            double slice_boost = impl_->slice_orchestrator->get_priority_boost(ue->rnti);
            decision.priority_metric *= slice_boost;
        }
    }
}

bool SchedulerEngine::allocate_resources(
    SchedulingDecision& decision,
    CellContext& cell,
    bool is_downlink) {
    
    return impl_->resource_allocator->allocate(decision, cell, is_downlink);
}

void SchedulerEngine::build_dl_tti_request(
    CellContext& cell,
    const std::vector<SchedulingDecision>& decisions) {
    
    DlTtiRequest request;
    request.timing = cell.current_slot().timing;
    request.cell_id = cell.cell_id();
    
    // K1 timing: when HARQ-ACK is expected (typically 4 slots for FDD, variable for TDD)
    constexpr uint16_t k1_offset = 4;
    
    for (const auto& decision : decisions) {
        auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
        if (!ue) continue;
        
        // Build PDSCH PDU
        DlTtiRequest::PdschPdu pdsch;
        pdsch.rnti = decision.rnti;
        pdsch.rb_allocation = decision.resources.rb_range;
        pdsch.symbol_allocation = decision.resources.symbol_range;
        pdsch.mcs = decision.tb_config.mcs_index;
        pdsch.rv = decision.tb_config.rv_index;
        pdsch.ndi = decision.tb_config.ndi;
        pdsch.harq_id = decision.harq_id;
        pdsch.num_layers = decision.tb_config.num_layers;
        pdsch.tb_size = decision.tb_config.tb_size_bytes;
        
        request.pdsch_pdus.push_back(pdsch);
        
        // Register pending DL HARQ feedback (expected K1 slots later)
        uint32_t expected_feedback_slot = request.timing.advanced(k1_offset).absolute_slot();
        impl_->harq_tracker.register_pending(expected_feedback_slot, {
            cell.cell_id(),
            decision.rnti,
            decision.harq_id,
            true,  // is_downlink = true for DL HARQ-ACK
            request.timing.absolute_slot()
        });
        
        // Update statistics
        impl_->stats.dl_bytes_scheduled += decision.tb_config.tb_size_bytes;
        ++impl_->stats.dl_ues_scheduled;
        if (decision.type == SchedulingDecision::Type::Retx) {
            ++impl_->stats.dl_retransmissions;
        }
        
        cell.current_slot().dl_bytes_scheduled += decision.tb_config.tb_size_bytes;
        ++cell.current_slot().ues_scheduled_dl;
    }
    
    // Send to PHY
    if (impl_->phy_interface) {
        impl_->phy_interface->send_dl_tti_request(request);
    }
    
    // Notify callback
    if (impl_->dl_result_callback) {
        impl_->dl_result_callback(cell.cell_id(), request);
    }
}

void SchedulerEngine::build_ul_tti_request(
    CellContext& cell,
    const std::vector<SchedulingDecision>& decisions) {
    
    UlTtiRequest request;
    // Correctly compute K2 timing with wrap-around handling
    request.timing = cell.current_slot().timing.advanced(
        impl_->config.scheduling_offset_slots);
    request.cell_id = cell.cell_id();
    
    for (const auto& decision : decisions) {
        auto ue = cell.ue_repository()->find_by_rnti(decision.rnti);
        if (!ue) continue;
        
        UlTtiRequest::PuschPdu pusch;
        pusch.rnti = decision.rnti;
        pusch.rb_allocation = decision.resources.rb_range;
        pusch.symbol_allocation = decision.resources.symbol_range;
        pusch.mcs = decision.tb_config.mcs_index;
        pusch.rv = decision.tb_config.rv_index;
        pusch.ndi = decision.tb_config.ndi;
        pusch.harq_id = decision.harq_id;
        pusch.num_layers = decision.tb_config.num_layers;
        pusch.tb_size = decision.tb_config.tb_size_bytes;
        
        request.pusch_pdus.push_back(pusch);
        
        // Register pending UL HARQ feedback (CRC expected after PUSCH)
        // UL CRC typically arrives 1-2 slots after PUSCH transmission
        uint32_t expected_feedback_slot = request.timing.advanced(2).absolute_slot();
        impl_->harq_tracker.register_pending(expected_feedback_slot, {
            cell.cell_id(),
            decision.rnti,
            decision.harq_id,
            false,  // is_downlink = false for UL HARQ
            request.timing.absolute_slot()
        });
        
        impl_->stats.ul_bytes_scheduled += decision.tb_config.tb_size_bytes;
        ++impl_->stats.ul_ues_scheduled;
        if (decision.type == SchedulingDecision::Type::Retx) {
            ++impl_->stats.ul_retransmissions;
        }
        
        cell.current_slot().ul_bytes_scheduled += decision.tb_config.tb_size_bytes;
        ++cell.current_slot().ues_scheduled_ul;
    }
    
    if (impl_->phy_interface) {
        impl_->phy_interface->send_ul_tti_request(request);
    }
    
    if (impl_->ul_result_callback) {
        impl_->ul_result_callback(cell.cell_id(), request);
    }
}

// Event handlers
void SchedulerEngine::on_ue_config(const UeConfigRequest& config) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(config.cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->create_ue(config.rnti);
    if (!ue) return;
    
    // Apply configuration
    ue->slice_id = config.slice_id;
    ue->dl_bwps = config.dl_bwps;
    ue->ul_bwps = config.ul_bwps;
    ue->drx_config = config.drx_config;
    
    for (const auto& lc : config.logical_channels) {
        ue->logical_channels[lc.lc_id] = lc;
    }
    
    // Register with slice orchestrator
    if (impl_->config.enable_slicing) {
        impl_->slice_orchestrator->associate_ue(config.rnti, config.slice_id);
    }
}

void SchedulerEngine::on_ue_release(const UeReleaseRequest& release) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(release.cell_id);
    if (it == impl_->cells.end()) return;
    
    it->second->ue_repository()->remove_ue(release.rnti);
    impl_->qos_engine->remove_ue_flows(release.rnti);
    impl_->slice_orchestrator->disassociate_ue(release.rnti);
}

void SchedulerEngine::on_dl_data(const DlDataNotification& notification) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(notification.cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->find_by_rnti(notification.rnti);
    if (!ue) return;
    
    ue->update_dl_buffer(notification.lc_id, notification.buffer_bytes);
}

void SchedulerEngine::on_bsr(const BufferStatusReport& bsr) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(bsr.cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->find_by_rnti(bsr.rnti);
    if (!ue) return;
    
    ue->ul_buffer_status.lcg_buffer_bytes[bsr.lcg_id] = bsr.buffer_size_bytes;
    ue->ul_buffer_status.report_time = bsr.report_time;
}

void SchedulerEngine::on_sr(const SchedulingRequest& sr) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(sr.cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->find_by_rnti(sr.rnti);
    if (!ue) return;
    
    ue->ul_buffer_status.sr_pending = true;
}

void SchedulerEngine::on_phr(const PowerHeadroomReport& phr) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(phr.cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->find_by_rnti(phr.rnti);
    if (!ue) return;
    
    ue->phr = phr.phr_value;
    ue->p_cmax = phr.pcmax;
}

void SchedulerEngine::on_csi_update(Rnti rnti, CellId cell_id, 
                                     const ChannelQualityInfo& cqi) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->cells.find(cell_id);
    if (it == impl_->cells.end()) return;
    
    auto ue = it->second->ue_repository()->find_by_rnti(rnti);
    if (!ue) return;
    
    ue->dl_channel_quality = cqi;
}

void SchedulerEngine::on_harq_feedback(CellId cell_id, const HarqFeedback& feedback) {
    std::lock_guard lock(impl_->mutex);
    
    // Use the HARQ timing tracker to find the UE and process
    // The feedback.timing tells us when the feedback was received
    uint32_t feedback_slot = feedback.feedback_time.absolute_slot();
    
    // Try to find matching pending feedback entry
    // Note: In real implementation, the HarqFeedback would include RNTI from UCI
    // For now, we search all pending entries for this slot
    auto pending_list = impl_->harq_tracker.get_all_for_slot(feedback_slot);
    
    for (const auto& pending : pending_list) {
        if (pending.cell_id != cell_id) continue;
        if (pending.harq_id != feedback.harq_id) continue;
        
        // Found the matching entry
        auto it = impl_->cells.find(cell_id);
        if (it == impl_->cells.end()) continue;
        
        auto ue = it->second->ue_repository()->find_by_rnti(pending.rnti);
        if (!ue) continue;
        
        // Process the feedback
        ue->process_harq_feedback(pending.is_downlink, feedback);
        
        // Remove from tracker
        impl_->harq_tracker.find_and_remove(feedback_slot, pending.rnti, feedback.harq_id);
        
        // Update statistics
        if (!feedback.ack) {
            if (pending.is_downlink) {
                impl_->stats.dl_retransmissions++;
            } else {
                impl_->stats.ul_retransmissions++;
            }
        }
        
        return;  // Found and processed
    }
    
    // If we get here, we couldn't match the feedback
    // This might happen if the UE was released or timing drifted
    ++impl_->stats.harq_timeouts;
}

void SchedulerEngine::on_rach_indication(CellId /*cell_id*/, 
                                          const RachIndication& /*rach*/) {
    // Handle RACH - would trigger RAR scheduling
}

void SchedulerEngine::register_dl_result_callback(DlResultCallback callback) {
    impl_->dl_result_callback = std::move(callback);
}

void SchedulerEngine::register_ul_result_callback(UlResultCallback callback) {
    impl_->ul_result_callback = std::move(callback);
}

SchedulerEngine::Statistics SchedulerEngine::get_statistics() const {
    return impl_->stats;
}

void SchedulerEngine::reset_statistics() {
    impl_->stats = Statistics{};
    impl_->last_stats_reset = std::chrono::steady_clock::now();
}

const SchedulerConfig& SchedulerEngine::config() const noexcept {
    return impl_->config;
}

void SchedulerEngine::update_config(const SchedulerConfig& config) {
    std::lock_guard lock(impl_->mutex);
    impl_->config = config;
    impl_->policy_manager = create_policy_manager(config.strategy);
}

//=============================================================================
// Factory Functions
//=============================================================================

std::unique_ptr<IPolicyManager> create_policy_manager(
    SchedulerConfig::Strategy strategy) {
    
    switch (strategy) {
        case SchedulerConfig::Strategy::RoundRobin:
            return std::make_unique<RoundRobinPolicy>();
        case SchedulerConfig::Strategy::MaxThroughput:
            return std::make_unique<MaxThroughputPolicy>();
        case SchedulerConfig::Strategy::ProportionalFair:
            return std::make_unique<ProportionalFairPolicy>();
        case SchedulerConfig::Strategy::QoSAware:
            // QoS-Aware explicitly integrates QoS into priority calculation
            return std::make_unique<QosAwarePolicy>();
        case SchedulerConfig::Strategy::Custom:
        default:
            // Default to PF if custom not provided
            return std::make_unique<ProportionalFairPolicy>();
    }
}

std::unique_ptr<IResourceAllocator> create_resource_allocator() {
    return std::make_unique<DefaultResourceAllocator>();
}

} // namespace nexgen::nr
