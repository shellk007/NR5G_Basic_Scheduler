/**
 * @file ue_context.hpp
 * @brief UE Context management for NexGen 5G NR Scheduler
 * @version 1.0
 */

#pragma once

#include "nr_types.hpp"
#include <unordered_map>
#include <memory>
#include <mutex>
#include <atomic>

namespace nexgen::nr {

//=============================================================================
// Logical Channel Configuration
//=============================================================================

struct LogicalChannelConfig {
    LcId        lc_id;
    LcgId       lcg_id;
    uint8_t     priority;           ///< 1 (highest) to 16 (lowest)
    uint32_t    prioritized_bit_rate_bps;
    uint32_t    bucket_size_duration_ms;
    
    // QoS mapping
    std::optional<QosFlowParams> qos_params;
};

//=============================================================================
// DRX Configuration
//=============================================================================

struct DrxConfig {
    bool        enabled;
    uint32_t    on_duration_ms;
    uint32_t    inactivity_timer_ms;
    uint32_t    harq_rtt_timer_ms;
    uint32_t    short_cycle_ms;
    uint32_t    long_cycle_ms;
    uint16_t    short_cycle_timer;
    uint16_t    slot_offset;
};

struct DrxState {
    bool        active_time;
    FrameTiming on_duration_start;
    FrameTiming inactivity_expiry;
    bool        in_short_cycle;
    uint16_t    short_cycle_count;
};

//=============================================================================
// HARQ Entity
//=============================================================================

template<size_t MaxProcesses = 16>
class HarqEntity {
public:
    struct Process {
        HarqId      id;
        HarqState   state{HarqState::Idle};
        uint8_t     tx_count{0};
        uint8_t     rv_index{0};
        bool        ndi{false};
        
        TransportBlockConfig tb_config;
        TimeFrequencyResource allocation;
        FrameTiming tx_time;
        FrameTiming feedback_expected;
        
        static constexpr uint8_t MAX_TX = 4;
        
        [[nodiscard]] bool can_retransmit() const noexcept {
            return tx_count < MAX_TX && state == HarqState::NackReceived;
        }
        
        void reset() noexcept {
            state = HarqState::Idle;
            tx_count = 0;
            rv_index = 0;
            ndi = !ndi;  // Toggle NDI
        }
        
        [[nodiscard]] uint8_t next_rv() const noexcept {
            static constexpr uint8_t rv_sequence[] = {0, 2, 3, 1};
            return rv_sequence[tx_count % 4];
        }
    };

    HarqEntity() {
        for (uint8_t i = 0; i < MaxProcesses; ++i) {
            processes_[i].id = i;
        }
    }

    [[nodiscard]] Process* get_free_process() noexcept {
        for (auto& proc : processes_) {
            if (proc.state == HarqState::Idle) {
                return &proc;
            }
        }
        return nullptr;
    }

    [[nodiscard]] Process* get_retx_process() noexcept {
        Process* oldest = nullptr;
        for (auto& proc : processes_) {
            if (proc.state == HarqState::NackReceived && proc.can_retransmit()) {
                if (!oldest || proc.tx_time.absolute_slot() < oldest->tx_time.absolute_slot()) {
                    oldest = &proc;
                }
            }
        }
        return oldest;
    }

    [[nodiscard]] Process* get_process(HarqId id) noexcept {
        if (id < MaxProcesses) {
            return &processes_[id];
        }
        return nullptr;
    }

    void process_feedback(const HarqFeedback& fb) {
        if (fb.harq_id >= MaxProcesses) return;
        
        auto& proc = processes_[fb.harq_id];
        if (proc.state != HarqState::Pending) return;
        
        if (fb.ack) {
            proc.reset();
        } else {
            proc.state = HarqState::NackReceived;
        }
    }

    [[nodiscard]] size_t pending_count() const noexcept {
        size_t count = 0;
        for (const auto& proc : processes_) {
            if (proc.state == HarqState::Pending || proc.state == HarqState::NackReceived) {
                ++count;
            }
        }
        return count;
    }

private:
    std::array<Process, MaxProcesses> processes_;
};

//=============================================================================
// UE Context
//=============================================================================

class UeContext {
public:
    // Identification
    Rnti        rnti;
    uint32_t    ue_id;
    SliceId     slice_id;
    
    // Configuration
    std::vector<BwpConfig>           dl_bwps;
    std::vector<BwpConfig>           ul_bwps;
    uint8_t                          active_dl_bwp_id{0};
    uint8_t                          active_ul_bwp_id{0};
    
    // Logical Channels
    std::unordered_map<LcId, LogicalChannelConfig> logical_channels;
    
    // DRX
    DrxConfig   drx_config;
    DrxState    drx_state;
    
    // HARQ Entities
    HarqEntity<16> dl_harq;
    HarqEntity<16> ul_harq;
    
    // Channel Quality
    ChannelQualityInfo dl_channel_quality;
    ChannelQualityInfo ul_channel_quality;
    
    // Buffer Status
    BufferStatus ul_buffer_status;
    std::unordered_map<LcId, uint32_t> dl_buffer_bytes;
    
    // Beam Information
    std::optional<BeamId> serving_beam_dl;
    std::optional<BeamId> serving_beam_ul;
    
    // Timing Advance
    uint16_t    timing_advance{0};
    FrameTiming ta_update_time;
    
    // Power Control
    int8_t      p_cmax{23};         ///< Max TX power in dBm
    int8_t      phr{0};             ///< Power Headroom Report

    //=========================================================================
    // Helper Methods
    //=========================================================================

    [[nodiscard]] const BwpConfig* active_dl_bwp() const noexcept {
        for (const auto& bwp : dl_bwps) {
            if (bwp.bwp_id == active_dl_bwp_id) {
                return &bwp;
            }
        }
        return nullptr;
    }

    [[nodiscard]] const BwpConfig* active_ul_bwp() const noexcept {
        for (const auto& bwp : ul_bwps) {
            if (bwp.bwp_id == active_ul_bwp_id) {
                return &bwp;
            }
        }
        return nullptr;
    }

    [[nodiscard]] uint32_t total_dl_buffer() const noexcept {
        uint32_t total = 0;
        for (const auto& [_, bytes] : dl_buffer_bytes) {
            total += bytes;
        }
        return total;
    }

    [[nodiscard]] uint32_t total_ul_buffer() const noexcept {
        return ul_buffer_status.total_buffer();
    }

    [[nodiscard]] bool has_pending_data() const noexcept {
        return total_dl_buffer() > 0 || total_ul_buffer() > 0 || 
               ul_buffer_status.sr_pending;
    }

    [[nodiscard]] bool is_in_active_time(const FrameTiming& now) const noexcept {
        if (!drx_config.enabled) return true;
        return drx_state.active_time;
    }

    [[nodiscard]] uint8_t effective_cqi() const noexcept {
        return dl_channel_quality.wideband_cqi;
    }

    [[nodiscard]] uint8_t mcs_from_cqi() const noexcept {
        // Simple CQI to MCS mapping
        static constexpr uint8_t cqi_to_mcs[] = {
            0, 0, 1, 3, 5, 7, 9, 11, 13, 15, 18, 20, 22, 24, 26, 28
        };
        uint8_t cqi = effective_cqi();
        return (cqi < 16) ? cqi_to_mcs[cqi] : 28;
    }

    void update_dl_buffer(LcId lc_id, uint32_t bytes) {
        if (bytes == 0) {
            dl_buffer_bytes.erase(lc_id);
        } else {
            dl_buffer_bytes[lc_id] = bytes;
        }
    }

    void update_ul_buffer(const BufferStatus& bsr) {
        ul_buffer_status = bsr;
    }

    void process_harq_feedback(bool is_dl, const HarqFeedback& fb) {
        if (is_dl) {
            dl_harq.process_feedback(fb);
        } else {
            ul_harq.process_feedback(fb);
        }
    }
};

//=============================================================================
// UE Repository
//=============================================================================

class UeRepository {
public:
    using UePtr = std::shared_ptr<UeContext>;

    [[nodiscard]] UePtr create_ue(Rnti rnti) {
        std::lock_guard lock(mutex_);
        
        auto ue = std::make_shared<UeContext>();
        ue->rnti = rnti;
        ue->ue_id = next_ue_id_++;
        
        ue_by_rnti_[rnti] = ue;
        ue_by_id_[ue->ue_id] = ue;
        
        return ue;
    }

    void remove_ue(Rnti rnti) {
        std::lock_guard lock(mutex_);
        
        auto it = ue_by_rnti_.find(rnti);
        if (it != ue_by_rnti_.end()) {
            ue_by_id_.erase(it->second->ue_id);
            ue_by_rnti_.erase(it);
        }
    }

    [[nodiscard]] UePtr find_by_rnti(Rnti rnti) const {
        std::lock_guard lock(mutex_);
        
        auto it = ue_by_rnti_.find(rnti);
        return (it != ue_by_rnti_.end()) ? it->second : nullptr;
    }

    [[nodiscard]] UePtr find_by_id(uint32_t ue_id) const {
        std::lock_guard lock(mutex_);
        
        auto it = ue_by_id_.find(ue_id);
        return (it != ue_by_id_.end()) ? it->second : nullptr;
    }

    template<typename Func>
    void for_each(Func&& func) const {
        std::lock_guard lock(mutex_);
        
        for (const auto& [_, ue] : ue_by_rnti_) {
            func(ue);
        }
    }

    [[nodiscard]] size_t count() const noexcept {
        std::lock_guard lock(mutex_);
        return ue_by_rnti_.size();
    }

private:
    mutable std::mutex mutex_;
    std::atomic<uint32_t> next_ue_id_{1};
    std::unordered_map<Rnti, UePtr> ue_by_rnti_;
    std::unordered_map<uint32_t, UePtr> ue_by_id_;
};

} // namespace nexgen::nr
