/**
 * @file test_slice_orchestrator.cpp
 * @brief Unit tests for Slice Orchestrator
 */

#include <gtest/gtest.h>
#include "services/slice_orchestrator.hpp"

namespace nexgen::nr::test {

class SliceOrchestratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        orchestrator_ = std::make_unique<SliceOrchestrator>();
    }

    std::unique_ptr<SliceOrchestrator> orchestrator_;
};

TEST_F(SliceOrchestratorTest, CreateSlice) {
    SliceConfig config;
    config.slice_id = 1;
    config.slice_name = "Test Slice";
    config.type = SliceType::eMBB;
    config.sst = 1;
    config.sla.guaranteed_rbs = 10;
    config.sla.resource_share = 0.5;
    
    EXPECT_TRUE(orchestrator_->create_slice(config));
    EXPECT_FALSE(orchestrator_->create_slice(config));  // Duplicate
}

TEST_F(SliceOrchestratorTest, GetSliceConfig) {
    SliceConfig config;
    config.slice_id = 1;
    config.slice_name = "Test";
    config.sla.guaranteed_rbs = 20;
    
    orchestrator_->create_slice(config);
    
    auto retrieved = orchestrator_->get_slice_config(1);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved->slice_name, "Test");
    EXPECT_EQ(retrieved->sla.guaranteed_rbs, 20);
    
    EXPECT_FALSE(orchestrator_->get_slice_config(99).has_value());
}

TEST_F(SliceOrchestratorTest, DeleteSlice) {
    SliceConfig config;
    config.slice_id = 1;
    orchestrator_->create_slice(config);
    
    EXPECT_TRUE(orchestrator_->delete_slice(1));
    EXPECT_FALSE(orchestrator_->get_slice_config(1).has_value());
}

TEST_F(SliceOrchestratorTest, AssociateUe) {
    SliceConfig config;
    config.slice_id = 1;
    orchestrator_->create_slice(config);
    
    EXPECT_TRUE(orchestrator_->associate_ue(0x1234, 1));
    EXPECT_FALSE(orchestrator_->associate_ue(0x5678, 99));  // Invalid slice
    
    auto slice = orchestrator_->get_ue_slice(0x1234);
    ASSERT_TRUE(slice.has_value());
    EXPECT_EQ(*slice, 1);
}

TEST_F(SliceOrchestratorTest, DisassociateUe) {
    SliceConfig config;
    config.slice_id = 1;
    orchestrator_->create_slice(config);
    orchestrator_->associate_ue(0x1234, 1);
    
    orchestrator_->disassociate_ue(0x1234);
    
    EXPECT_FALSE(orchestrator_->get_ue_slice(0x1234).has_value());
}

TEST_F(SliceOrchestratorTest, GetSliceUes) {
    SliceConfig config;
    config.slice_id = 1;
    orchestrator_->create_slice(config);
    
    orchestrator_->associate_ue(0x1000, 1);
    orchestrator_->associate_ue(0x1001, 1);
    orchestrator_->associate_ue(0x1002, 1);
    
    auto ues = orchestrator_->get_slice_ues(1);
    EXPECT_EQ(ues.size(), 3);
}

TEST_F(SliceOrchestratorTest, DeleteSliceWithUes) {
    SliceConfig config;
    config.slice_id = 1;
    orchestrator_->create_slice(config);
    orchestrator_->associate_ue(0x1234, 1);
    
    // Should fail because UE is associated
    EXPECT_FALSE(orchestrator_->delete_slice(1));
    
    orchestrator_->disassociate_ue(0x1234);
    EXPECT_TRUE(orchestrator_->delete_slice(1));
}

TEST_F(SliceOrchestratorTest, PriorityBoost) {
    SliceConfig urllc;
    urllc.slice_id = 1;
    urllc.sla.slice_priority = 10;
    orchestrator_->create_slice(urllc);
    
    SliceConfig embb;
    embb.slice_id = 2;
    embb.sla.slice_priority = 100;
    orchestrator_->create_slice(embb);
    
    orchestrator_->associate_ue(0x1000, 1);
    orchestrator_->associate_ue(0x2000, 2);
    
    double urllc_boost = orchestrator_->get_priority_boost(0x1000);
    double embb_boost = orchestrator_->get_priority_boost(0x2000);
    
    EXPECT_GT(urllc_boost, embb_boost);
}

TEST_F(SliceOrchestratorTest, SlicesByPriority) {
    SliceConfig low;
    low.slice_id = 1;
    low.sla.slice_priority = 200;
    orchestrator_->create_slice(low);
    
    SliceConfig high;
    high.slice_id = 2;
    high.sla.slice_priority = 10;
    orchestrator_->create_slice(high);
    
    SliceConfig mid;
    mid.slice_id = 3;
    mid.sla.slice_priority = 100;
    orchestrator_->create_slice(mid);
    
    auto ordered = orchestrator_->get_slices_by_priority();
    ASSERT_EQ(ordered.size(), 3);
    EXPECT_EQ(ordered[0], 2);  // Highest priority first
    EXPECT_EQ(ordered[1], 3);
    EXPECT_EQ(ordered[2], 1);
}

TEST_F(SliceOrchestratorTest, UrllcTemplate) {
    auto urllc = SliceOrchestrator::create_urllc_template(1, "Factory URLLC");
    
    EXPECT_EQ(urllc.type, SliceType::URLLC);
    EXPECT_EQ(urllc.sst, 2);
    EXPECT_TRUE(urllc.sla.strict_isolation);
    EXPECT_TRUE(urllc.sla.preemption_capable);
    EXPECT_FALSE(urllc.sla.preemptible);
    EXPECT_LE(urllc.sla.max_latency_ms, 10);
}

TEST_F(SliceOrchestratorTest, EmbbTemplate) {
    auto embb = SliceOrchestrator::create_embb_template(2, "Consumer eMBB");
    
    EXPECT_EQ(embb.type, SliceType::eMBB);
    EXPECT_EQ(embb.sst, 1);
    EXPECT_FALSE(embb.sla.strict_isolation);
    EXPECT_TRUE(embb.sla.preemptible);
}

TEST_F(SliceOrchestratorTest, MmtcTemplate) {
    auto mmtc = SliceOrchestrator::create_mmtc_template(3, "IoT mMTC");
    
    EXPECT_EQ(mmtc.type, SliceType::mMTC);
    EXPECT_EQ(mmtc.sst, 3);
    EXPECT_GT(mmtc.sla.max_latency_ms, 500);
}

TEST_F(SliceOrchestratorTest, SlotProcessing) {
    SliceConfig config;
    config.slice_id = 1;
    config.sla.guaranteed_rbs = 10;
    orchestrator_->create_slice(config);
    
    FrameTiming timing{0, 0, 0};
    orchestrator_->on_slot_start(timing);
    orchestrator_->on_slot_end(timing);
    
    auto stats = orchestrator_->get_statistics();
    EXPECT_EQ(stats.total_slices, 1);
}

} // namespace nexgen::nr::test
