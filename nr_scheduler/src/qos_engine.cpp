/**
 * @file qos_engine.cpp
 * @brief QoS Engine Implementation for NexGen 5G NR Scheduler
 * @version 1.0
 */

#include "../include/services/qos_engine.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>

namespace nexgen::nr {

//=============================================================================
// QoS Engine Implementation
//=============================================================================

struct QosEngine::Impl {
    QosEngineConfig config;
    mutable std::mutex mutex;
    
    // Per-UE flow states
    std::unordered_map<Rnti, std::unordered_map<uint8_t, QosFlowState>> flows;
    
    // Statistics
    struct Stats {
        uint64_t total_flows = 0;
        uint64_t gbr_flows = 0;
        uint64_t delay_critical_events = 0;
        uint64_t pdb_violations = 0;
        uint64_t mbr_throttle_events = 0;
        double total_delay_margin = 0.0;
        uint64_t delay_samples = 0;
    } stats;
    
    explicit Impl(const QosEngineConfig& cfg) : config(cfg) {}
};

QosEngine::QosEngine(const QosEngineConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

QosEngine::~QosEngine() = default;

void QosEngine::register_flow(Rnti rnti, const QosFlowParams& params) {
    std::lock_guard lock(impl_->mutex);
    
    QosFlowState state;
    state.qfi = params.qfi;
    state.params = params;
    
    // Initialize token buckets
    auto now = std::chrono::steady_clock::now();
    
    auto init_bucket = [&](QosFlowState::TokenBucket& bucket, uint64_t rate_bps) {
        bucket.fill_rate_bps = rate_bps;
        bucket.bucket_size = (rate_bps * impl_->config.token_bucket_depth_ms) / 8000;
        bucket.tokens = bucket.bucket_size;  // Start full
        bucket.last_update = now;
    };
    
    if (params.characteristics.resource_type == QosResourceType::GBR ||
        params.characteristics.resource_type == QosResourceType::DelayC) {
        init_bucket(state.dl_bucket, params.mbr_dl_bps);
        init_bucket(state.ul_bucket, params.mbr_ul_bps);
        ++impl_->stats.gbr_flows;
    } else {
        // Non-GBR: large bucket, no hard rate limit
        init_bucket(state.dl_bucket, 1000000000);  // 1 Gbps
        init_bucket(state.ul_bucket, 1000000000);
    }
    
    // Initialize delay tracker
    state.dl_delay_tracker.pdb_ms = params.characteristics.packet_delay_budget_ms;
    state.ul_delay_tracker.pdb_ms = params.characteristics.packet_delay_budget_ms;
    
    // Initialize GBR tracker window
    state.dl_gbr_tracker.window_ms = impl_->config.gbr_measurement_window_ms;
    state.ul_gbr_tracker.window_ms = impl_->config.gbr_measurement_window_ms;
    
    impl_->flows[rnti][params.qfi] = std::move(state);
    ++impl_->stats.total_flows;
}

void QosEngine::remove_flow(Rnti rnti, uint8_t qfi) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->flows.find(rnti);
    if (it != impl_->flows.end()) {
        auto& ue_flows = it->second;
        auto flow_it = ue_flows.find(qfi);
        if (flow_it != ue_flows.end()) {
            if (flow_it->second.params.characteristics.resource_type == 
                QosResourceType::GBR) {
                --impl_->stats.gbr_flows;
            }
            ue_flows.erase(flow_it);
            --impl_->stats.total_flows;
        }
    }
}

void QosEngine::remove_ue_flows(Rnti rnti) {
    std::lock_guard lock(impl_->mutex);
    
    auto it = impl_->flows.find(rnti);
    if (it != impl_->flows.end()) {
        for (const auto& [qfi, flow] : it->second) {
            if (flow.params.characteristics.resource_type == QosResourceType::GBR) {
                --impl_->stats.gbr_flows;
            }
            --impl_->stats.total_flows;
        }
        impl_->flows.erase(it);
    }
}

QosPriorityResult QosEngine::calculate_priority(
    Rnti rnti,
    uint8_t qfi,
    bool is_downlink,
    uint32_t buffer_bytes) {
    
    std::lock_guard lock(impl_->mutex);
    
    QosPriorityResult result{};
    result.base_priority = 1.0;
    result.gbr_factor = 1.0;
    result.delay_factor = 1.0;
    result.is_gbr = false;
    result.is_delay_critical = false;
    result.max_bytes_allowed = buffer_bytes;
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) {
        result.final_priority = result.base_priority;
        return result;
    }
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) {
        result.final_priority = result.base_priority;
        return result;
    }
    
    auto& flow = flow_it->second;
    const auto& chars = flow.params.characteristics;
    
    // Base priority from 5QI priority level (lower value = higher priority)
    result.base_priority = 128.0 / chars.priority_level;
    
    // Check if GBR
    result.is_gbr = (chars.resource_type == QosResourceType::GBR ||
                     chars.resource_type == QosResourceType::DelayC);
    
    // GBR factor: boost priority if below guaranteed rate
    if (result.is_gbr) {
        auto& tracker = is_downlink ? flow.dl_gbr_tracker : flow.ul_gbr_tracker;
        uint64_t current_rate = tracker.current_rate_bps();
        uint64_t gbr = is_downlink ? flow.params.gbr_dl_bps : flow.params.gbr_ul_bps;
        
        if (current_rate < gbr * impl_->config.gbr_enforcement_strictness) {
            double deficit_ratio = 1.0 - (static_cast<double>(current_rate) / gbr);
            result.gbr_factor = 1.0 + deficit_ratio * impl_->config.gbr_deficit_weight;
            result.min_bytes_required = (gbr / 8000);  // 1ms worth of GBR
        }
    }
    
    // Delay factor: boost priority as packet approaches PDB
    auto& delay_tracker = is_downlink ? flow.dl_delay_tracker : flow.ul_delay_tracker;
    double urgency = delay_tracker.urgency();
    
    if (urgency > impl_->config.delay_margin_factor) {
        result.is_delay_critical = true;
        result.delay_factor = 1.0 + (urgency - impl_->config.delay_margin_factor) * 
                              impl_->config.delay_urgency_weight;
        ++impl_->stats.delay_critical_events;
    }
    
    // MBR enforcement
    if (impl_->config.enable_mbr_enforcement) {
        auto& bucket = is_downlink ? flow.dl_bucket : flow.ul_bucket;
        bucket.update(std::chrono::steady_clock::now());
        result.max_bytes_allowed = std::min(buffer_bytes, 
                                            static_cast<uint32_t>(bucket.available()));
        if (result.max_bytes_allowed < buffer_bytes) {
            ++impl_->stats.mbr_throttle_events;
        }
    }
    
    // Combine factors
    result.final_priority = result.base_priority * result.gbr_factor * result.delay_factor;
    
    // Track delay margin for statistics
    if (delay_tracker.has_pending_data) {
        impl_->stats.total_delay_margin += (1.0 - urgency);
        ++impl_->stats.delay_samples;
    }
    
    return result;
}

uint32_t QosEngine::get_allowed_bytes(Rnti rnti, uint8_t qfi, bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) {
        return UINT32_MAX;  // No restriction
    }
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) {
        return UINT32_MAX;
    }
    
    auto& flow = flow_it->second;
    auto& bucket = is_downlink ? flow.dl_bucket : flow.ul_bucket;
    
    bucket.update(std::chrono::steady_clock::now());
    return static_cast<uint32_t>(std::max(int64_t(0), bucket.available()));
}

void QosEngine::report_transmission(Rnti rnti, uint8_t qfi, 
                                     bool is_downlink, uint32_t bytes) {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) return;
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) return;
    
    auto& flow = flow_it->second;
    
    // Update token bucket
    auto& bucket = is_downlink ? flow.dl_bucket : flow.ul_bucket;
    bucket.update(std::chrono::steady_clock::now());
    bucket.tokens -= bytes;  // Can go negative
    
    // Update GBR tracker
    auto& tracker = is_downlink ? flow.dl_gbr_tracker : flow.ul_gbr_tracker;
    tracker.add_sample(bytes);
    
    // Update total stats
    if (is_downlink) {
        flow.total_dl_bytes += bytes;
    } else {
        flow.total_ul_bytes += bytes;
    }
}

void QosEngine::notify_data_arrival(Rnti rnti, uint8_t qfi, bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) return;
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) return;
    
    auto& flow = flow_it->second;
    auto& tracker = is_downlink ? flow.dl_delay_tracker : flow.ul_delay_tracker;
    tracker.packet_arrived();
}

void QosEngine::notify_data_transmitted(Rnti rnti, uint8_t qfi, bool is_downlink) {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) return;
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) return;
    
    auto& flow = flow_it->second;
    auto& tracker = is_downlink ? flow.dl_delay_tracker : flow.ul_delay_tracker;
    
    // Check for PDB violation before marking transmitted
    if (tracker.urgency() > 1.0) {
        ++impl_->stats.pdb_violations;
        ++flow.pdb_violations;
    }
    
    tracker.packet_transmitted();
}

std::vector<std::pair<Rnti, uint8_t>> QosEngine::get_delay_critical_flows() const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<std::pair<Rnti, uint8_t>> critical;
    
    for (const auto& [rnti, ue_flows] : impl_->flows) {
        for (const auto& [qfi, flow] : ue_flows) {
            if (flow.dl_delay_tracker.is_critical() || 
                flow.ul_delay_tracker.is_critical()) {
                critical.emplace_back(rnti, qfi);
            }
        }
    }
    
    return critical;
}

uint64_t QosEngine::get_current_rate(Rnti rnti, uint8_t qfi, bool is_downlink) const {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) return 0;
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) return 0;
    
    const auto& flow = flow_it->second;
    const auto& tracker = is_downlink ? flow.dl_gbr_tracker : flow.ul_gbr_tracker;
    return tracker.current_rate_bps();
}

int64_t QosEngine::get_gbr_deficit(Rnti rnti, uint8_t qfi, bool is_downlink) const {
    std::lock_guard lock(impl_->mutex);
    
    auto ue_it = impl_->flows.find(rnti);
    if (ue_it == impl_->flows.end()) return 0;
    
    auto flow_it = ue_it->second.find(qfi);
    if (flow_it == ue_it->second.end()) return 0;
    
    const auto& flow = flow_it->second;
    
    if (flow.params.characteristics.resource_type != QosResourceType::GBR &&
        flow.params.characteristics.resource_type != QosResourceType::DelayC) {
        return 0;
    }
    
    const auto& tracker = is_downlink ? flow.dl_gbr_tracker : flow.ul_gbr_tracker;
    uint64_t current = tracker.current_rate_bps();
    uint64_t gbr = is_downlink ? flow.params.gbr_dl_bps : flow.params.gbr_ul_bps;
    
    return static_cast<int64_t>(gbr) - static_cast<int64_t>(current);
}

std::vector<std::pair<Rnti, uint8_t>> QosEngine::get_gbr_deficit_flows() const {
    std::lock_guard lock(impl_->mutex);
    
    std::vector<std::pair<Rnti, uint8_t>> deficit;
    
    for (const auto& [rnti, ue_flows] : impl_->flows) {
        for (const auto& [qfi, flow] : ue_flows) {
            if (flow.params.characteristics.resource_type != QosResourceType::GBR &&
                flow.params.characteristics.resource_type != QosResourceType::DelayC) {
                continue;
            }
            
            uint64_t dl_gbr = flow.params.gbr_dl_bps;
            uint64_t ul_gbr = flow.params.gbr_ul_bps;
            
            if (flow.dl_gbr_tracker.current_rate_bps() < 
                dl_gbr * impl_->config.gbr_enforcement_strictness ||
                flow.ul_gbr_tracker.current_rate_bps() < 
                ul_gbr * impl_->config.gbr_enforcement_strictness) {
                deficit.emplace_back(rnti, qfi);
            }
        }
    }
    
    return deficit;
}

QosEngine::Statistics QosEngine::get_statistics() const {
    std::lock_guard lock(impl_->mutex);
    
    Statistics stats;
    stats.total_flows = impl_->stats.total_flows;
    stats.gbr_flows = impl_->stats.gbr_flows;
    stats.delay_critical_events = impl_->stats.delay_critical_events;
    stats.pdb_violations = impl_->stats.pdb_violations;
    stats.mbr_throttle_events = impl_->stats.mbr_throttle_events;
    
    if (impl_->stats.delay_samples > 0) {
        stats.avg_delay_margin = impl_->stats.total_delay_margin / 
                                  impl_->stats.delay_samples;
    }
    
    // Calculate GBR satisfaction
    uint64_t satisfied = 0;
    uint64_t total_gbr = 0;
    for (const auto& [_, ue_flows] : impl_->flows) {
        for (const auto& [_, flow] : ue_flows) {
            if (flow.params.characteristics.resource_type == QosResourceType::GBR ||
                flow.params.characteristics.resource_type == QosResourceType::DelayC) {
                ++total_gbr;
                
                uint64_t dl_gbr = flow.params.gbr_dl_bps;
                if (flow.dl_gbr_tracker.current_rate_bps() >= 
                    dl_gbr * impl_->config.gbr_enforcement_strictness) {
                    ++satisfied;
                }
            }
        }
    }
    
    stats.gbr_satisfaction_ratio = (total_gbr > 0) ? 
        static_cast<double>(satisfied) / total_gbr : 1.0;
    
    return stats;
}

void QosEngine::reset_statistics() {
    std::lock_guard lock(impl_->mutex);
    impl_->stats = Impl::Stats{};
}

void QosEngine::update_config(const QosEngineConfig& config) {
    std::lock_guard lock(impl_->mutex);
    impl_->config = config;
}

const QosEngineConfig& QosEngine::config() const {
    return impl_->config;
}

void QosEngine::on_slot_tick() {
    std::lock_guard lock(impl_->mutex);
    
    auto now = std::chrono::steady_clock::now();
    
    // Update all token buckets
    for (auto& [_, ue_flows] : impl_->flows) {
        for (auto& [_, flow] : ue_flows) {
            flow.dl_bucket.update(now);
            flow.ul_bucket.update(now);
        }
    }
}

//=============================================================================
// 5QI Table
//=============================================================================

QosCharacteristics get_5qi_characteristics(uint8_t fiveqi) {
    QosCharacteristics c{};
    c.fiveqi = fiveqi;
    
    // Standard 5QI values from 3GPP 23.501
    switch (fiveqi) {
        case 1:  // Conversational Voice
            c.resource_type = QosResourceType::GBR;
            c.priority_level = 20;
            c.packet_delay_budget_ms = 100;
            c.packet_error_rate = 1e-2;
            break;
            
        case 2:  // Conversational Video
            c.resource_type = QosResourceType::GBR;
            c.priority_level = 40;
            c.packet_delay_budget_ms = 150;
            c.packet_error_rate = 1e-3;
            break;
            
        case 3:  // Real-time Gaming
            c.resource_type = QosResourceType::GBR;
            c.priority_level = 30;
            c.packet_delay_budget_ms = 50;
            c.packet_error_rate = 1e-3;
            break;
            
        case 4:  // Non-conversational Video
            c.resource_type = QosResourceType::GBR;
            c.priority_level = 50;
            c.packet_delay_budget_ms = 300;
            c.packet_error_rate = 1e-6;
            break;
            
        case 5:  // IMS Signaling
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 10;
            c.packet_delay_budget_ms = 100;
            c.packet_error_rate = 1e-6;
            break;
            
        case 6:  // Video (Buffered)
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 60;
            c.packet_delay_budget_ms = 300;
            c.packet_error_rate = 1e-6;
            break;
            
        case 7:  // Voice, Video, Interactive Gaming
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 70;
            c.packet_delay_budget_ms = 100;
            c.packet_error_rate = 1e-3;
            break;
            
        case 8:  // Video (Buffered)
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 80;
            c.packet_delay_budget_ms = 300;
            c.packet_error_rate = 1e-6;
            break;
            
        case 9:  // Video (Buffered), TCP-based
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 90;
            c.packet_delay_budget_ms = 300;
            c.packet_error_rate = 1e-6;
            break;
            
        case 82:  // Discrete Automation
            c.resource_type = QosResourceType::DelayC;
            c.priority_level = 19;
            c.packet_delay_budget_ms = 10;
            c.packet_error_rate = 1e-4;
            break;
            
        case 83:  // Discrete Automation
            c.resource_type = QosResourceType::DelayC;
            c.priority_level = 22;
            c.packet_delay_budget_ms = 10;
            c.packet_error_rate = 1e-4;
            break;
            
        case 84:  // Intelligent Transport
            c.resource_type = QosResourceType::DelayC;
            c.priority_level = 24;
            c.packet_delay_budget_ms = 30;
            c.packet_error_rate = 1e-5;
            break;
            
        case 85:  // Electricity Distribution
            c.resource_type = QosResourceType::DelayC;
            c.priority_level = 21;
            c.packet_delay_budget_ms = 5;
            c.packet_error_rate = 1e-5;
            break;
            
        default:  // Default to Non-GBR best effort
            c.resource_type = QosResourceType::NonGBR;
            c.priority_level = 90;
            c.packet_delay_budget_ms = 300;
            c.packet_error_rate = 1e-6;
            break;
    }
    
    return c;
}

} // namespace nexgen::nr
