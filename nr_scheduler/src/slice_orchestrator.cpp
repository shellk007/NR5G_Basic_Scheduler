/**
 * @file slice_orchestrator.cpp
 * @brief Slice Orchestrator Implementation for NexGen 5G NR Scheduler
 * @version 1.0
 */

#include "../include/services/slice_orchestrator.hpp"
#include <algorithm>
#include <mutex>

namespace nexgen::nr {

//=============================================================================
// Slice Orchestrator Implementation
//=============================================================================

struct SliceOrchestrator::Impl {
    mutable std::mutex mutex;
    
    // Slice configurations
    std::unordered_map<SliceId, SliceConfig> slice_configs;
    
    // Slice states
    std::unordered_map<SliceId, SliceState> slice_states;
    
    // UE to Slice mapping
    std::unordered_map<Rnti, SliceId> ue_slice_map;
    
    // Current slot budgets
    std::unordered_map<SliceId, SliceResourceBudget> current_budgets;
    
    // Shared resource pool
    uint16_t shared_pool_dl = 0;
    uint16_t shared_pool_ul = 0;
    
    // Statistics
    struct Stats {
        uint64_t total_sla_violations = 0;
        uint64_t resource_borrowing_events = 0;
        double total_utilization = 0.0;
        uint64_t utilization_samples = 0;
    } stats;
    
    // Current timing
    FrameTiming current_timing;
};

SliceOrchestrator::SliceOrchestrator()
    : impl_(std::make_unique<Impl>()) {}

SliceOrchestrator::~SliceOrchestrator() = default;

bool SliceOrchestrator::create_slice(const SliceConfig& config) {
    std::lock_guard lock(impl_->mutex);
    
    if (impl_->slice_configs.count(config.slice_id)) {
        return false;  // Already exists
    }
    
    impl_->slice_configs[config.slice_id] = config;
    
    // Initialize state
    SliceState state;
    state.slice_id = config.slice_id;
    state.active = true;
    impl_->slice_states[config.slice_id] = state;
    
    return true;
}

bool SliceOrchestrator::update_slice(SliceId slice_id, const SliceConfig& config) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->slice_configs.find(slice_id);
    if (it == impl_->slice_configs.end()) {
        return false;
    }
    
    it->second = config;
    return true;
}

bool SliceOrchestrator::delete_slice(SliceId slice_id) {
    std::lock_guard lock(impl_->mutex);
    
    // Check if any UEs are associated
    for (const auto& [rnti, sid] : impl_->ue_slice_map) {
        if (sid == slice_id) {
            return false;  // Can't delete slice with UEs
        }
    }
    
    impl_->slice_configs.erase(slice_id);
    impl_->slice_states.erase(slice_id);
    impl_->current_budgets.erase(slice_id);
    
    return true;
}

std::optional<SliceConfig> SliceOrchestrator::get_slice_config(SliceId slice_id) const {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->slice_configs.find(slice_id);
    if (it != impl_->slice_configs.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<SliceId> SliceOrchestrator::get_all_slices() const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<SliceId> slices;
    slices.reserve(impl_->slice_configs.size());
    
    for (const auto& [id, _] : impl_->slice_configs) {
        slices.push_back(id);
    }
    
    return slices;
}

bool SliceOrchestrator::associate_ue(Rnti rnti, SliceId slice_id) {
    std::lock_guard lock(impl_->mutex);
    
    if (!impl_->slice_configs.count(slice_id)) {
        return false;
    }
    
    impl_->ue_slice_map[rnti] = slice_id;
    
    // Update slice state
    auto& state = impl_->slice_states[slice_id];
    ++state.connected_ues;
    
    return true;
}

void SliceOrchestrator::disassociate_ue(Rnti rnti) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->ue_slice_map.find(rnti);
    if (it != impl_->ue_slice_map.end()) {
        auto& state = impl_->slice_states[it->second];
        if (state.connected_ues > 0) {
            --state.connected_ues;
        }
        impl_->ue_slice_map.erase(it);
    }
}

std::optional<SliceId> SliceOrchestrator::get_ue_slice(Rnti rnti) const {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->ue_slice_map.find(rnti);
    if (it != impl_->ue_slice_map.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<Rnti> SliceOrchestrator::get_slice_ues(SliceId slice_id) const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<Rnti> ues;
    
    for (const auto& [rnti, sid] : impl_->ue_slice_map) {
        if (sid == slice_id) {
            ues.push_back(rnti);
        }
    }
    
    return ues;
}

std::unordered_map<SliceId, SliceResourceBudget> 
SliceOrchestrator::calculate_budgets(const CellContext& cell) {
    std::lock_guard lock(impl_->mutex);
    
    impl_->current_budgets.clear();
    
    uint16_t total_rbs = cell.config().num_rbs;
    uint16_t allocated_dl = 0;
    uint16_t allocated_ul = 0;
    
    // First pass: allocate guaranteed resources
    for (const auto& [slice_id, config] : impl_->slice_configs) {
        SliceResourceBudget budget;
        budget.slice_id = slice_id;
        
        // Guaranteed allocation
        budget.available_rbs_dl = std::min(config.sla.guaranteed_rbs, 
                                            static_cast<uint16_t>(total_rbs - allocated_dl));
        budget.available_rbs_ul = std::min(config.sla.guaranteed_rbs,
                                            static_cast<uint16_t>(total_rbs - allocated_ul));
        
        allocated_dl += budget.available_rbs_dl;
        allocated_ul += budget.available_rbs_ul;
        
        budget.used_rbs_dl = 0;
        budget.used_rbs_ul = 0;
        budget.borrowed_rbs = 0;
        
        impl_->current_budgets[slice_id] = budget;
    }
    
    // Calculate shared pool
    impl_->shared_pool_dl = total_rbs - allocated_dl;
    impl_->shared_pool_ul = total_rbs - allocated_ul;
    
    // Second pass: distribute shared resources proportionally
    if (impl_->shared_pool_dl > 0 || impl_->shared_pool_ul > 0) {
        double total_share = 0.0;
        for (const auto& [_, config] : impl_->slice_configs) {
            total_share += config.sla.resource_share;
        }
        
        if (total_share > 0) {
            for (auto& [slice_id, budget] : impl_->current_budgets) {
                auto config_it = impl_->slice_configs.find(slice_id);
                if (config_it != impl_->slice_configs.end()) {
                    double share_ratio = config_it->second.sla.resource_share / total_share;
                    
                    uint16_t extra_dl = static_cast<uint16_t>(
                        impl_->shared_pool_dl * share_ratio);
                    uint16_t extra_ul = static_cast<uint16_t>(
                        impl_->shared_pool_ul * share_ratio);
                    
                    // Respect max_rbs limit
                    if (config_it->second.sla.max_rbs > 0) {
                        extra_dl = std::min(extra_dl, 
                            static_cast<uint16_t>(config_it->second.sla.max_rbs - 
                                                   budget.available_rbs_dl));
                        extra_ul = std::min(extra_ul,
                            static_cast<uint16_t>(config_it->second.sla.max_rbs - 
                                                   budget.available_rbs_ul));
                    }
                    
                    budget.available_rbs_dl += extra_dl;
                    budget.available_rbs_ul += extra_ul;
                }
            }
        }
    }
    
    return impl_->current_budgets;
}

SliceResourceBudget SliceOrchestrator::get_budget(SliceId slice_id) const {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->current_budgets.find(slice_id);
    if (it != impl_->current_budgets.end()) {
        return it->second;
    }
    
    return SliceResourceBudget{};
}

uint16_t SliceOrchestrator::request_resources(SliceId slice_id, 
                                               uint16_t rbs_needed, 
                                               bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto budget_it = impl_->current_budgets.find(slice_id);
    if (budget_it == impl_->current_budgets.end()) {
        return 0;
    }
    
    auto config_it = impl_->slice_configs.find(slice_id);
    if (config_it == impl_->slice_configs.end()) {
        return 0;
    }
    
    auto& budget = budget_it->second;
    const auto& config = config_it->second;
    
    // Check remaining budget
    uint16_t remaining = is_downlink ? budget.remaining_dl() : budget.remaining_ul();
    
    if (remaining >= rbs_needed) {
        return rbs_needed;  // Already have enough
    }
    
    // Need to borrow
    uint16_t to_borrow = rbs_needed - remaining;
    
    // Check if borrowing is allowed
    if (config.sla.strict_isolation) {
        return remaining;  // Can't borrow
    }
    
    // Find slices to borrow from
    uint16_t borrowed = 0;
    for (auto& [other_id, other_budget] : impl_->current_budgets) {
        if (other_id == slice_id) continue;
        
        auto other_config_it = impl_->slice_configs.find(other_id);
        if (other_config_it == impl_->slice_configs.end()) continue;
        
        // Check if other slice can be preempted
        if (!other_config_it->second.sla.preemptible) continue;
        if (config.sla.slice_priority >= other_config_it->second.sla.slice_priority) {
            continue;  // Can only preempt lower priority
        }
        
        // Borrow unused resources
        uint16_t other_remaining = is_downlink ? 
            other_budget.remaining_dl() : other_budget.remaining_ul();
        
        uint16_t to_take = std::min(to_borrow - borrowed, other_remaining);
        
        if (to_take > 0) {
            if (is_downlink) {
                other_budget.used_rbs_dl += to_take;
                budget.available_rbs_dl += to_take;
            } else {
                other_budget.used_rbs_ul += to_take;
                budget.available_rbs_ul += to_take;
            }
            
            budget.borrowed_rbs += to_take;
            borrowed += to_take;
            ++impl_->stats.resource_borrowing_events;
            
            if (borrowed >= to_borrow) break;
        }
    }
    
    return remaining + borrowed;
}

void SliceOrchestrator::return_resources(SliceId slice_id, 
                                          uint16_t rbs_unused, 
                                          bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->current_budgets.find(slice_id);
    if (it == impl_->current_budgets.end()) return;
    
    auto& budget = it->second;
    
    // Return to shared pool for this slot
    if (is_downlink) {
        impl_->shared_pool_dl += rbs_unused;
    } else {
        impl_->shared_pool_ul += rbs_unused;
    }
    
    // Reduce borrowed count if applicable
    if (budget.borrowed_rbs > 0) {
        budget.borrowed_rbs -= std::min(budget.borrowed_rbs, 
                                        static_cast<int16_t>(rbs_unused));
    }
}

double SliceOrchestrator::get_priority_boost(Rnti rnti) const {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->ue_slice_map.find(rnti);
    if (ue_it == impl_->ue_slice_map.end()) {
        return 1.0;
    }
    
    auto config_it = impl_->slice_configs.find(ue_it->second);
    if (config_it == impl_->slice_configs.end()) {
        return 1.0;
    }
    
    // Priority boost based on slice priority (1-255, lower = higher priority)
    return 256.0 / config_it->second.sla.slice_priority;
}

bool SliceOrchestrator::is_scheduling_allowed(Rnti rnti, uint16_t rbs, 
                                               bool is_downlink) const {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->ue_slice_map.find(rnti);
    if (ue_it == impl_->ue_slice_map.end()) {
        return true;  // No slice assigned, allow
    }
    
    auto budget_it = impl_->current_budgets.find(ue_it->second);
    if (budget_it == impl_->current_budgets.end()) {
        return true;
    }
    
    uint16_t remaining = is_downlink ? 
        budget_it->second.remaining_dl() : budget_it->second.remaining_ul();
    
    return remaining >= rbs;
}

std::vector<SliceId> SliceOrchestrator::get_slices_by_priority() const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<std::pair<SliceId, uint8_t>> slices;
    slices.reserve(impl_->slice_configs.size());
    
    for (const auto& [id, config] : impl_->slice_configs) {
        slices.emplace_back(id, config.sla.slice_priority);
    }
    
    // Sort by priority (lower value = higher priority)
    std::sort(slices.begin(), slices.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        });
    
    std::vector<SliceId> result;
    result.reserve(slices.size());
    for (const auto& [id, _] : slices) {
        result.push_back(id);
    }
    
    return result;
}

std::optional<SliceState> SliceOrchestrator::get_slice_state(SliceId slice_id) const {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->slice_states.find(slice_id);
    if (it != impl_->slice_states.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<SliceId> SliceOrchestrator::check_sla_compliance() const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<SliceId> violating;
    
    for (const auto& [slice_id, state] : impl_->slice_states) {
        auto config_it = impl_->slice_configs.find(slice_id);
        if (config_it == impl_->slice_configs.end()) continue;
        
        const auto& sla = config_it->second.sla;
        
        // Check latency
        if (state.avg_latency_ms > sla.max_latency_ms) {
            violating.push_back(slice_id);
            continue;
        }
        
        // Check rate (simplified)
        // In production, would track actual throughput
    }
    
    return violating;
}

void SliceOrchestrator::report_scheduling(Rnti rnti, uint16_t rbs, 
                                           uint32_t bytes, bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->ue_slice_map.find(rnti);
    if (ue_it == impl_->ue_slice_map.end()) return;
    
    SliceId slice_id = ue_it->second;
    
    // Update budget
    auto budget_it = impl_->current_budgets.find(slice_id);
    if (budget_it != impl_->current_budgets.end()) {
        if (is_downlink) {
            budget_it->second.used_rbs_dl += rbs;
        } else {
            budget_it->second.used_rbs_ul += rbs;
        }
    }
    
    // Update state
    auto state_it = impl_->slice_states.find(slice_id);
    if (state_it != impl_->slice_states.end()) {
        auto& state = state_it->second;
        
        if (is_downlink) {
            state.current_rbs_dl = rbs;
            state.current_window.bytes_dl += bytes;
            ++state.current_window.ues_scheduled_dl;
        } else {
            state.current_rbs_ul = rbs;
            state.current_window.bytes_ul += bytes;
            ++state.current_window.ues_scheduled_ul;
        }
    }
}

SliceOrchestrator::Statistics SliceOrchestrator::get_statistics() const {
    std::lock_guard lock(impl_->mutex);
    
    Statistics stats;
    stats.total_slices = impl_->slice_configs.size();
    stats.active_slices = impl_->slice_states.size();
    stats.total_sla_violations = impl_->stats.total_sla_violations;
    stats.resource_borrowing_events = impl_->stats.resource_borrowing_events;
    
    if (impl_->stats.utilization_samples > 0) {
        stats.avg_resource_utilization = impl_->stats.total_utilization / 
                                          impl_->stats.utilization_samples;
    }
    
    // Calculate isolation efficiency (1 - borrowed_resources / total_resources)
    double total_borrowed = 0;
    double total_used = 0;
    for (const auto& [_, budget] : impl_->current_budgets) {
        total_borrowed += std::abs(budget.borrowed_rbs);
        total_used += budget.used_rbs_dl + budget.used_rbs_ul;
    }
    
    stats.isolation_efficiency = (total_used > 0) ? 
        1.0 - (total_borrowed / total_used) : 1.0;
    
    return stats;
}

void SliceOrchestrator::on_slot_start(const FrameTiming& timing) {
    std::lock_guard lock(impl_->mutex);
    impl_->current_timing = timing;
    
    // Reset per-slot state
    for (auto& [_, state] : impl_->slice_states) {
        state.current_rbs_dl = 0;
        state.current_rbs_ul = 0;
    }
}

void SliceOrchestrator::on_slot_end(const FrameTiming& /*timing*/) {
    std::lock_guard lock(impl_->mutex);
    
    // Calculate utilization for this slot
    double slot_utilization = 0.0;
    for (const auto& [_, budget] : impl_->current_budgets) {
        if (budget.available_rbs_dl > 0) {
            slot_utilization += static_cast<double>(budget.used_rbs_dl) / 
                                budget.available_rbs_dl;
        }
    }
    
    if (!impl_->current_budgets.empty()) {
        slot_utilization /= impl_->current_budgets.size();
    }
    
    impl_->stats.total_utilization += slot_utilization;
    ++impl_->stats.utilization_samples;
    
    // Check SLA compliance
    auto violating = check_sla_compliance();
    impl_->stats.total_sla_violations += violating.size();
}

//=============================================================================
// Predefined Slice Templates
//=============================================================================

SliceConfig SliceOrchestrator::create_embb_template(SliceId id, 
                                                     const std::string& name) {
    SliceConfig config;
    config.slice_id = id;
    config.slice_name = name;
    config.type = SliceType::eMBB;
    config.sst = 1;  // eMBB SST
    
    config.sla.guaranteed_rbs = 10;
    config.sla.max_rbs = 0;  // Unlimited
    config.sla.resource_share = 0.5;
    config.sla.guaranteed_rate_bps = 10000000;  // 10 Mbps
    config.sla.max_rate_bps = 1000000000;       // 1 Gbps
    config.sla.max_latency_ms = 100;
    config.sla.target_latency_ms = 50;
    config.sla.target_reliability = 0.99;
    config.sla.slice_priority = 100;
    config.sla.strict_isolation = false;
    config.sla.preemption_capable = false;
    config.sla.preemptible = true;
    
    config.allowed_5qi = {6, 7, 8, 9};  // Non-GBR 5QIs
    
    return config;
}

SliceConfig SliceOrchestrator::create_urllc_template(SliceId id, 
                                                      const std::string& name) {
    SliceConfig config;
    config.slice_id = id;
    config.slice_name = name;
    config.type = SliceType::URLLC;
    config.sst = 2;  // URLLC SST
    
    config.sla.guaranteed_rbs = 20;
    config.sla.max_rbs = 50;
    config.sla.resource_share = 0.3;
    config.sla.guaranteed_rate_bps = 1000000;   // 1 Mbps
    config.sla.max_rate_bps = 10000000;         // 10 Mbps
    config.sla.max_latency_ms = 10;
    config.sla.target_latency_ms = 1;
    config.sla.target_reliability = 0.99999;    // 5 nines
    config.sla.slice_priority = 10;             // High priority
    config.sla.strict_isolation = true;
    config.sla.preemption_capable = true;
    config.sla.preemptible = false;
    
    config.allowed_5qi = {82, 83, 84, 85};  // Delay-critical 5QIs
    
    return config;
}

SliceConfig SliceOrchestrator::create_mmtc_template(SliceId id, 
                                                     const std::string& name) {
    SliceConfig config;
    config.slice_id = id;
    config.slice_name = name;
    config.type = SliceType::mMTC;
    config.sst = 3;  // mMTC SST
    
    config.sla.guaranteed_rbs = 5;
    config.sla.max_rbs = 30;
    config.sla.resource_share = 0.2;
    config.sla.guaranteed_rate_bps = 100000;    // 100 Kbps
    config.sla.max_rate_bps = 1000000;          // 1 Mbps
    config.sla.max_latency_ms = 1000;
    config.sla.target_latency_ms = 500;
    config.sla.target_reliability = 0.99;
    config.sla.slice_priority = 150;            // Lower priority
    config.sla.strict_isolation = false;
    config.sla.preemption_capable = false;
    config.sla.preemptible = true;
    
    config.allowed_5qi = {9};  // Best effort
    
    return config;
}

} // namespace nexgen::nr
