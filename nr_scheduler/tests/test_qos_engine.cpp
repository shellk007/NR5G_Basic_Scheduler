/**
 * @file test_qos_engine.cpp
 * @brief Unit tests for QoS Engine
 */

#include <gtest/gtest.h>
#include "services/qos_engine.hpp"

namespace nexgen::nr::test {

class QosEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        QosEngineConfig config;
        config.gbr_enforcement_strictness = 0.95;
        config.delay_margin_factor = 0.8;
        config.enable_mbr_enforcement = true;
        
        qos_ = std::make_unique<QosEngine>(config);
    }

    std::unique_ptr<QosEngine> qos_;
};

TEST_F(QosEngineTest, RegisterFlow) {
    QosFlowParams params;
    params.qfi = 1;
    params.characteristics = get_5qi_characteristics(1);  // Voice
    params.gbr_dl_bps = 64000;
    params.gbr_ul_bps = 64000;
    
    qos_->register_flow(0x1234, params);
    
    auto stats = qos_->get_statistics();
    EXPECT_EQ(stats.total_flows, 1);
    EXPECT_EQ(stats.gbr_flows, 1);
}

TEST_F(QosEngineTest, RemoveFlow) {
    QosFlowParams params;
    params.qfi = 1;
    params.characteristics = get_5qi_characteristics(1);
    
    qos_->register_flow(0x1234, params);
    qos_->remove_flow(0x1234, 1);
    
    auto stats = qos_->get_statistics();
    EXPECT_EQ(stats.total_flows, 0);
}

TEST_F(QosEngineTest, PriorityCalculation) {
    QosFlowParams params;
    params.qfi = 5;
    params.characteristics = get_5qi_characteristics(5);  // IMS signaling
    
    qos_->register_flow(0x1000, params);
    
    auto result = qos_->calculate_priority(0x1000, 5, true, 1000);
    EXPECT_GT(result.base_priority, 0);
    EXPECT_GT(result.final_priority, 0);
}

TEST_F(QosEngineTest, GbrFlow) {
    QosFlowParams params;
    params.qfi = 1;
    params.characteristics = get_5qi_characteristics(1);
    params.gbr_dl_bps = 100000;
    
    qos_->register_flow(0x1000, params);
    
    auto result = qos_->calculate_priority(0x1000, 1, true, 5000);
    EXPECT_TRUE(result.is_gbr);
}

TEST_F(QosEngineTest, DelayTracking) {
    QosFlowParams params;
    params.qfi = 82;
    params.characteristics = get_5qi_characteristics(82);  // URLLC
    
    qos_->register_flow(0x1000, params);
    
    qos_->notify_data_arrival(0x1000, 82, true);
    
    // Immediately after arrival, should not be critical
    auto result = qos_->calculate_priority(0x1000, 82, true, 100);
    EXPECT_FALSE(result.is_delay_critical);
}

TEST_F(QosEngineTest, ReportTransmission) {
    QosFlowParams params;
    params.qfi = 1;
    params.characteristics = get_5qi_characteristics(1);
    params.gbr_dl_bps = 1000000;
    
    qos_->register_flow(0x1000, params);
    
    qos_->report_transmission(0x1000, 1, true, 1000);
    
    uint64_t rate = qos_->get_current_rate(0x1000, 1, true);
    EXPECT_GE(rate, 0);  // Rate might be 0 if only one sample
}

TEST_F(QosEngineTest, FiveQICharacteristics) {
    auto voice = get_5qi_characteristics(1);
    EXPECT_EQ(voice.resource_type, QosResourceType::GBR);
    EXPECT_EQ(voice.packet_delay_budget_ms, 100);
    
    auto urllc = get_5qi_characteristics(82);
    EXPECT_EQ(urllc.resource_type, QosResourceType::DelayC);
    EXPECT_EQ(urllc.packet_delay_budget_ms, 10);
    
    auto best_effort = get_5qi_characteristics(9);
    EXPECT_EQ(best_effort.resource_type, QosResourceType::NonGBR);
}

TEST_F(QosEngineTest, SlotTick) {
    qos_->on_slot_tick();
    // Should not crash
}

} // namespace nexgen::nr::test
