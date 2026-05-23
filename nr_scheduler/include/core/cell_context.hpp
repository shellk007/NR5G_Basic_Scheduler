/**
 * @file cell_context.hpp
 * @brief Cell Context and Resource Grid for NexGen 5G NR Scheduler
 * @version 1.0
 */

#pragma once

#include "nr_types.hpp"
#include "ue_context.hpp"
#include <bitset>
#include <vector>
#include <optional>

namespace nexgen::nr {

//=============================================================================
// Cell Configuration
//=============================================================================

enum class DuplexMode : uint8_t {
    FDD,
    TDD
};

struct TddSlotConfig {
    uint8_t num_dl_symbols;
    uint8_t num_guard_symbols;
    uint8_t num_ul_symbols;
    
    [[nodiscard]] bool is_downlink_only() const noexcept {
        return num_ul_symbols == 0 && num_guard_symbols == 0;
    }
    
    [[nodiscard]] bool is_uplink_only() const noexcept {
        return num_dl_symbols == 0 && num_guard_symbols == 0;
    }
    
    [[nodiscard]] bool is_special() const noexcept {
        return !is_downlink_only() && !is_uplink_only();
    }
};

struct TddPattern {
    uint8_t periodicity_slots;
    std::vector<TddSlotConfig> slot_configs;
    
    [[nodiscard]] const TddSlotConfig& get_config(uint8_t slot_in_pattern) const {
        return slot_configs[slot_in_pattern % slot_configs.size()];
    }
};

struct CellConfig {
    CellId              cell_id;
    uint32_t            pci;                    ///< Physical Cell ID
    uint32_t            frequency_khz;
    uint16_t            bandwidth_mhz;
    uint16_t            num_rbs;
    DuplexMode          duplex_mode;
    NumerologyConfig    numerology;
    
    // TDD Configuration (only if TDD)
    std::optional<TddPattern> tdd_pattern;
    
    // CORESET/SearchSpace
    uint8_t             coreset0_num_rbs;
    uint8_t             coreset0_num_symbols;
    
    // Max UEs and Allocations
    uint16_t            max_ues;
    uint8_t             max_pdsch_per_slot;
    uint8_t             max_pusch_per_slot;
    uint8_t             max_pdcch_per_slot;
    
    // MIMO Configuration
    uint8_t             num_tx_antennas;
    uint8_t             num_rx_antennas;
    uint8_t             max_layers;
    
    // Power Settings
    int8_t              ref_signal_power_dbm;
    
    [[nodiscard]] bool is_fdd() const noexcept { return duplex_mode == DuplexMode::FDD; }
    [[nodiscard]] bool is_tdd() const noexcept { return duplex_mode == DuplexMode::TDD; }
};

//=============================================================================
// Resource Grid
//=============================================================================

class ResourceGrid {
public:
    static constexpr uint16_t MAX_RBS = 273;
    static constexpr uint8_t  MAX_SYMBOLS = 14;
    
    using RbBitmap = std::bitset<MAX_RBS>;

    explicit ResourceGrid(uint16_t num_rbs = MAX_RBS, uint8_t num_symbols = MAX_SYMBOLS)
        : num_rbs_(num_rbs), num_symbols_(num_symbols) {
        reset();
    }

    void reset() noexcept {
        for (auto& symbol : rb_allocation_) {
            symbol.reset();
        }
    }

    [[nodiscard]] bool is_available(const TimeFrequencyResource& res) const noexcept {
        for (uint8_t sym = res.symbol_range.start_symbol; 
             sym <= res.symbol_range.end_symbol(); ++sym) {
            for (uint16_t rb = res.rb_range.start_rb; 
                 rb <= res.rb_range.end_rb(); ++rb) {
                if (rb_allocation_[sym].test(rb)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool allocate(const TimeFrequencyResource& res) noexcept {
        if (!is_available(res)) {
            return false;
        }
        
        for (uint8_t sym = res.symbol_range.start_symbol; 
             sym <= res.symbol_range.end_symbol(); ++sym) {
            for (uint16_t rb = res.rb_range.start_rb; 
                 rb <= res.rb_range.end_rb(); ++rb) {
                rb_allocation_[sym].set(rb);
            }
        }
        return true;
    }

    void deallocate(const TimeFrequencyResource& res) noexcept {
        for (uint8_t sym = res.symbol_range.start_symbol; 
             sym <= res.symbol_range.end_symbol(); ++sym) {
            for (uint16_t rb = res.rb_range.start_rb; 
                 rb <= res.rb_range.end_rb(); ++rb) {
                rb_allocation_[sym].reset(rb);
            }
        }
    }

    [[nodiscard]] std::optional<ResourceBlockRange> find_contiguous(
        uint16_t num_rbs, 
        uint8_t start_symbol, 
        uint8_t num_symbols) const noexcept {
        
        uint16_t contiguous = 0;
        uint16_t start = 0;
        
        for (uint16_t rb = 0; rb < num_rbs_; ++rb) {
            bool available = true;
            for (uint8_t sym = start_symbol; sym < start_symbol + num_symbols; ++sym) {
                if (rb_allocation_[sym].test(rb)) {
                    available = false;
                    break;
                }
            }
            
            if (available) {
                if (contiguous == 0) start = rb;
                ++contiguous;
                
                if (contiguous >= num_rbs) {
                    return ResourceBlockRange{start, num_rbs};
                }
            } else {
                contiguous = 0;
            }
        }
        
        return std::nullopt;
    }

    [[nodiscard]] uint16_t available_rbs(uint8_t symbol) const noexcept {
        return static_cast<uint16_t>(num_rbs_ - rb_allocation_[symbol].count());
    }

    [[nodiscard]] uint16_t total_available_res() const noexcept {
        uint16_t total = 0;
        for (uint8_t sym = 0; sym < num_symbols_; ++sym) {
            total += available_rbs(sym);
        }
        return total;
    }

    [[nodiscard]] const RbBitmap& get_allocation(uint8_t symbol) const noexcept {
        return rb_allocation_[symbol];
    }

private:
    uint16_t num_rbs_;
    uint8_t  num_symbols_;
    std::array<RbBitmap, MAX_SYMBOLS> rb_allocation_;
};

//=============================================================================
// CCE (Control Channel Element) Pool
//=============================================================================

class CcePool {
public:
    static constexpr uint8_t MAX_CCES = 128;

    explicit CcePool(uint8_t num_cces = MAX_CCES) : num_cces_(num_cces) {
        reset();
    }

    void reset() noexcept {
        cce_bitmap_.reset();
        allocated_count_ = 0;
    }

    [[nodiscard]] std::optional<uint8_t> allocate(AggregationLevel al) noexcept {
        uint8_t al_value = static_cast<uint8_t>(al);
        
        // Find aligned starting position
        for (uint8_t start = 0; start + al_value <= num_cces_; start += al_value) {
            bool available = true;
            for (uint8_t i = 0; i < al_value && available; ++i) {
                if (cce_bitmap_.test(start + i)) {
                    available = false;
                }
            }
            
            if (available) {
                for (uint8_t i = 0; i < al_value; ++i) {
                    cce_bitmap_.set(start + i);
                }
                allocated_count_ += al_value;
                return start;
            }
        }
        
        return std::nullopt;
    }

    void deallocate(uint8_t start_cce, AggregationLevel al) noexcept {
        uint8_t al_value = static_cast<uint8_t>(al);
        for (uint8_t i = 0; i < al_value && start_cce + i < num_cces_; ++i) {
            if (cce_bitmap_.test(start_cce + i)) {
                cce_bitmap_.reset(start_cce + i);
                --allocated_count_;
            }
        }
    }

    [[nodiscard]] uint8_t available() const noexcept {
        return num_cces_ - allocated_count_;
    }

    [[nodiscard]] bool can_allocate(AggregationLevel al) const noexcept {
        return available() >= static_cast<uint8_t>(al);
    }

private:
    uint8_t num_cces_;
    uint8_t allocated_count_{0};
    std::bitset<MAX_CCES> cce_bitmap_;
};

//=============================================================================
// Slot Context
//=============================================================================

struct SlotContext {
    FrameTiming timing;
    
    // Resource grids for this slot
    ResourceGrid dl_grid;
    ResourceGrid ul_grid;
    CcePool      cce_pool;
    
    // TDD Configuration for this slot
    bool can_schedule_dl;
    bool can_schedule_ul;
    TddSlotConfig tdd_config;
    
    // Scheduling limits
    uint8_t remaining_pdsch;
    uint8_t remaining_pusch;
    uint8_t remaining_pdcch;
    
    // Statistics
    uint32_t dl_bytes_scheduled{0};
    uint32_t ul_bytes_scheduled{0};
    uint16_t ues_scheduled_dl{0};
    uint16_t ues_scheduled_ul{0};
    
    void initialize(const CellConfig& cfg, const FrameTiming& t) {
        timing = t;
        dl_grid = ResourceGrid(cfg.num_rbs, cfg.numerology.symbols_per_slot);
        ul_grid = ResourceGrid(cfg.num_rbs, cfg.numerology.symbols_per_slot);
        cce_pool = CcePool(cfg.coreset0_num_rbs * cfg.coreset0_num_symbols);
        
        remaining_pdsch = cfg.max_pdsch_per_slot;
        remaining_pusch = cfg.max_pusch_per_slot;
        remaining_pdcch = cfg.max_pdcch_per_slot;
        
        dl_bytes_scheduled = 0;
        ul_bytes_scheduled = 0;
        ues_scheduled_dl = 0;
        ues_scheduled_ul = 0;
        
        // Determine DL/UL capability based on TDD pattern
        if (cfg.is_fdd()) {
            can_schedule_dl = true;
            can_schedule_ul = true;
            tdd_config = {14, 0, 14};
        } else if (cfg.tdd_pattern) {
            uint8_t slot_in_pattern = t.slot % cfg.tdd_pattern->periodicity_slots;
            tdd_config = cfg.tdd_pattern->get_config(slot_in_pattern);
            can_schedule_dl = tdd_config.num_dl_symbols > 0;
            can_schedule_ul = tdd_config.num_ul_symbols > 0;
        }
    }
};

//=============================================================================
// Cell Context
//=============================================================================

class CellContext {
public:
    explicit CellContext(const CellConfig& config) 
        : config_(config), ue_repo_(std::make_shared<UeRepository>()) {
    }

    [[nodiscard]] const CellConfig& config() const noexcept { return config_; }
    [[nodiscard]] CellId cell_id() const noexcept { return config_.cell_id; }
    
    [[nodiscard]] std::shared_ptr<UeRepository> ue_repository() const noexcept {
        return ue_repo_;
    }

    [[nodiscard]] SlotContext& current_slot() noexcept { return current_slot_; }
    [[nodiscard]] const SlotContext& current_slot() const noexcept { return current_slot_; }

    void advance_to_slot(const FrameTiming& timing) {
        current_slot_.initialize(config_, timing);
    }

    //=========================================================================
    // Statistics
    //=========================================================================
    
    struct Statistics {
        std::atomic<uint64_t> total_dl_bytes{0};
        std::atomic<uint64_t> total_ul_bytes{0};
        std::atomic<uint64_t> total_slots_processed{0};
        std::atomic<uint32_t> dl_retransmissions{0};
        std::atomic<uint32_t> ul_retransmissions{0};
        std::atomic<uint32_t> scheduling_failures{0};
    };

    [[nodiscard]] const Statistics& statistics() const noexcept { return stats_; }
    [[nodiscard]] Statistics& statistics() noexcept { return stats_; }

private:
    CellConfig config_;
    std::shared_ptr<UeRepository> ue_repo_;
    SlotContext current_slot_;
    Statistics stats_;
};

} // namespace nexgen::nr
