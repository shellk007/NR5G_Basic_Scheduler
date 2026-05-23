/**
 * @file test_scheduler_engine.cpp
 * @brief Unit tests for NexGen Scheduler Engine
 */

#include <gtest/gtest.h>
#include "engine/scheduler_engine.hpp"
#include "core/cell_context.hpp"

namespace nexgen::nr::test {

class SchedulerEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.strategy = SchedulerConfig::Strategy::ProportionalFair;
        config_.enable_qos = true;
        config_.enable_slicing = false;
        
        scheduler_ = std::make_unique<SchedulerEngine>(config_);
        
        // Add test cell
        CellConfig cell_cfg;
        cell_cfg.cell_id = 1;
        cell_cfg.pci = 100;
        cell_cfg.num_rbs = 106;  // 40 MHz
        cell_cfg.duplex_mode = DuplexMode::TDD;
        cell_cfg.numerology = NumerologyConfig::from_scs(SubcarrierSpacing::kHz30);
        cell_cfg.max_ues = 32;
        cell_cfg.max_pdsch_per_slot = 8;
        cell_cfg.max_pusch_per_slot = 8;
        
        scheduler_->add_cell(cell_cfg);
    }

    void TearDown() override {
        scheduler_->stop();
    }

    SchedulerConfig config_;
    std::unique_ptr<SchedulerEngine> scheduler_;
};

TEST_F(SchedulerEngineTest, StartStop) {
    EXPECT_TRUE(scheduler_->start());
    scheduler_->stop();
}

TEST_F(SchedulerEngineTest, AddRemoveCell) {
    CellConfig cfg;
    cfg.cell_id = 2;
    cfg.num_rbs = 52;
    cfg.duplex_mode = DuplexMode::FDD;
    cfg.numerology = NumerologyConfig::from_scs(SubcarrierSpacing::kHz15);
    
    EXPECT_TRUE(scheduler_->add_cell(cfg));
    EXPECT_FALSE(scheduler_->add_cell(cfg));  // Duplicate
    
    scheduler_->remove_cell(2);
    EXPECT_TRUE(scheduler_->add_cell(cfg));  // Can add again
}

TEST_F(SchedulerEngineTest, UeConfiguration) {
    scheduler_->start();
    
    UeConfigRequest ue_cfg;
    ue_cfg.rnti = 0x1234;
    ue_cfg.cell_id = 1;
    ue_cfg.slice_id = 1;
    ue_cfg.num_dl_harq_processes = 16;
    ue_cfg.num_ul_harq_processes = 16;
    
    // Add BWP
    BwpConfig bwp;
    bwp.bwp_id = 0;
    bwp.start_rb = 0;
    bwp.num_rbs = 106;
    bwp.scs = SubcarrierSpacing::kHz30;
    bwp.is_active = true;
    ue_cfg.dl_bwps.push_back(bwp);
    ue_cfg.ul_bwps.push_back(bwp);
    
    // Add LC
    LogicalChannelConfig lc;
    lc.lc_id = 4;
    lc.lcg_id = 1;
    lc.priority = 5;
    ue_cfg.logical_channels.push_back(lc);
    
    scheduler_->on_ue_config(ue_cfg);
    
    // Verify by sending data
    DlDataNotification notif;
    notif.rnti = 0x1234;
    notif.cell_id = 1;
    notif.lc_id = 4;
    notif.buffer_bytes = 1000;
    
    scheduler_->on_dl_data(notif);
}

TEST_F(SchedulerEngineTest, SlotProcessing) {
    scheduler_->start();
    
    // Configure UE
    UeConfigRequest ue_cfg;
    ue_cfg.rnti = 0x1001;
    ue_cfg.cell_id = 1;
    
    BwpConfig bwp{0, 0, 106, SubcarrierSpacing::kHz30, true};
    ue_cfg.dl_bwps.push_back(bwp);
    ue_cfg.ul_bwps.push_back(bwp);
    
    LogicalChannelConfig lc{4, 1, 5};
    ue_cfg.logical_channels.push_back(lc);
    
    scheduler_->on_ue_config(ue_cfg);
    
    // Add DL data
    DlDataNotification notif{0x1001, 1, 4, 5000};
    scheduler_->on_dl_data(notif);
    
    // Process slot
    FrameTiming timing{0, 0, 0};
    timing.slots_per_frame = 20;
    scheduler_->on_slot_indication(1, timing);
    
    // Check statistics
    auto stats = scheduler_->get_statistics();
    EXPECT_EQ(stats.slots_processed, 1);
    EXPECT_GT(stats.dl_bytes_scheduled, 0);
}

TEST_F(SchedulerEngineTest, BSRProcessing) {
    scheduler_->start();
    
    UeConfigRequest ue_cfg;
    ue_cfg.rnti = 0x2001;
    ue_cfg.cell_id = 1;
    BwpConfig bwp{0, 0, 106, SubcarrierSpacing::kHz30, true};
    ue_cfg.dl_bwps.push_back(bwp);
    ue_cfg.ul_bwps.push_back(bwp);
    scheduler_->on_ue_config(ue_cfg);
    
    // Send BSR
    BufferStatusReport bsr;
    bsr.rnti = 0x2001;
    bsr.cell_id = 1;
    bsr.lcg_id = 1;
    bsr.buffer_size_bytes = 10000;
    bsr.report_time = {0, 0, 0};
    
    scheduler_->on_bsr(bsr);
    
    // Process slot
    FrameTiming timing{0, 1, 0};
    timing.slots_per_frame = 20;
    scheduler_->on_slot_indication(1, timing);
    
    auto stats = scheduler_->get_statistics();
    EXPECT_GT(stats.ul_bytes_scheduled, 0);
}

TEST_F(SchedulerEngineTest, MultiUeScheduling) {
    scheduler_->start();
    
    // Configure multiple UEs
    for (uint16_t i = 0; i < 4; ++i) {
        UeConfigRequest ue_cfg;
        ue_cfg.rnti = 0x1000 + i;
        ue_cfg.cell_id = 1;
        BwpConfig bwp{0, 0, 106, SubcarrierSpacing::kHz30, true};
        ue_cfg.dl_bwps.push_back(bwp);
        scheduler_->on_ue_config(ue_cfg);
        
        // Add DL data
        DlDataNotification notif;
        notif.rnti = 0x1000 + i;
        notif.cell_id = 1;
        notif.lc_id = 4;
        notif.buffer_bytes = 10000;
        scheduler_->on_dl_data(notif);
    }
    
    // Process slot
    FrameTiming timing{0, 0, 0};
    timing.slots_per_frame = 20;
    scheduler_->on_slot_indication(1, timing);
    
    auto stats = scheduler_->get_statistics();
    EXPECT_EQ(stats.slots_processed, 1);
    EXPECT_GE(stats.dl_ues_scheduled, 1);
}

TEST_F(SchedulerEngineTest, PolicyChange) {
    SchedulerConfig new_config = scheduler_->config();
    new_config.strategy = SchedulerConfig::Strategy::RoundRobin;
    
    scheduler_->update_config(new_config);
    
    EXPECT_EQ(scheduler_->config().strategy, SchedulerConfig::Strategy::RoundRobin);
}

TEST_F(SchedulerEngineTest, Statistics) {
    scheduler_->start();
    
    auto stats = scheduler_->get_statistics();
    EXPECT_EQ(stats.slots_processed, 0);
    EXPECT_EQ(stats.allocation_failures, 0);
    
    scheduler_->reset_statistics();
    stats = scheduler_->get_statistics();
    EXPECT_EQ(stats.slots_processed, 0);
}

// Resource Grid Tests
class ResourceGridTest : public ::testing::Test {
protected:
    ResourceGrid grid_{106, 14};
};

TEST_F(ResourceGridTest, BasicAllocation) {
    TimeFrequencyResource res;
    res.rb_range = {0, 10};
    res.symbol_range = {2, 12};
    
    EXPECT_TRUE(grid_.is_available(res));
    EXPECT_TRUE(grid_.allocate(res));
    EXPECT_FALSE(grid_.is_available(res));
}

TEST_F(ResourceGridTest, FindContiguous) {
    auto range = grid_.find_contiguous(20, 2, 12);
    ASSERT_TRUE(range.has_value());
    EXPECT_EQ(range->num_rbs, 20);
    
    // Allocate some RBs
    TimeFrequencyResource res{{0, 50}, {2, 12}};
    grid_.allocate(res);
    
    // Find remaining
    range = grid_.find_contiguous(50, 2, 12);
    ASSERT_TRUE(range.has_value());
    EXPECT_EQ(range->start_rb, 50);
}

TEST_F(ResourceGridTest, Reset) {
    TimeFrequencyResource res{{0, 100}, {0, 14}};
    grid_.allocate(res);
    
    EXPECT_FALSE(grid_.is_available(res));
    
    grid_.reset();
    EXPECT_TRUE(grid_.is_available(res));
}

// HARQ Entity Tests
class HarqEntityTest : public ::testing::Test {
protected:
    HarqEntity<16> harq_;
};

TEST_F(HarqEntityTest, GetFreeProcess) {
    auto* proc = harq_.get_free_process();
    ASSERT_NE(proc, nullptr);
    EXPECT_EQ(proc->state, HarqState::Idle);
}

TEST_F(HarqEntityTest, ProcessFeedback) {
    auto* proc = harq_.get_free_process();
    proc->state = HarqState::Pending;
    
    HarqFeedback fb;
    fb.harq_id = proc->id;
    fb.ack = true;
    
    harq_.process_feedback(fb);
    EXPECT_EQ(proc->state, HarqState::Idle);
}

TEST_F(HarqEntityTest, RetransmissionProcess) {
    auto* proc = harq_.get_free_process();
    proc->state = HarqState::NackReceived;
    proc->tx_count = 1;
    
    auto* retx = harq_.get_retx_process();
    ASSERT_NE(retx, nullptr);
    EXPECT_EQ(retx->id, proc->id);
}

} // namespace nexgen::nr::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
