/**
 * @file test_resource_grid.cpp
 * @brief Unit tests for Resource Grid and CCE Pool
 */

#include <gtest/gtest.h>
#include "core/cell_context.hpp"

namespace nexgen::nr::test {

class ResourceGridTest : public ::testing::Test {
protected:
    ResourceGrid grid_{106, 14};  // 40 MHz, 14 symbols
};

TEST_F(ResourceGridTest, InitialState) {
    // All resources should be available initially
    EXPECT_EQ(grid_.total_available_res(), 106 * 14);
    
    for (uint8_t sym = 0; sym < 14; ++sym) {
        EXPECT_EQ(grid_.available_rbs(sym), 106);
    }
}

TEST_F(ResourceGridTest, SimpleAllocation) {
    TimeFrequencyResource res;
    res.rb_range = {0, 10};
    res.symbol_range = {0, 14};
    
    EXPECT_TRUE(grid_.is_available(res));
    EXPECT_TRUE(grid_.allocate(res));
    EXPECT_FALSE(grid_.is_available(res));
}

TEST_F(ResourceGridTest, OverlappingAllocation) {
    TimeFrequencyResource res1{{0, 20}, {0, 7}};
    TimeFrequencyResource res2{{10, 20}, {3, 4}};  // Overlaps with res1
    
    EXPECT_TRUE(grid_.allocate(res1));
    EXPECT_FALSE(grid_.is_available(res2));
    EXPECT_FALSE(grid_.allocate(res2));
}

TEST_F(ResourceGridTest, NonOverlappingAllocation) {
    TimeFrequencyResource res1{{0, 50}, {0, 7}};
    TimeFrequencyResource res2{{50, 50}, {0, 7}};  // Adjacent, no overlap
    
    EXPECT_TRUE(grid_.allocate(res1));
    EXPECT_TRUE(grid_.is_available(res2));
    EXPECT_TRUE(grid_.allocate(res2));
}

TEST_F(ResourceGridTest, FindContiguous) {
    // Allocate some RBs
    TimeFrequencyResource used{{20, 30}, {2, 12}};
    grid_.allocate(used);
    
    // Find 15 contiguous RBs
    auto range = grid_.find_contiguous(15, 2, 12);
    ASSERT_TRUE(range.has_value());
    EXPECT_EQ(range->num_rbs, 15);
    
    // Should find at start (0-19)
    EXPECT_EQ(range->start_rb, 0);
}

TEST_F(ResourceGridTest, FindContiguousLarge) {
    // Try to find more RBs than available
    auto range = grid_.find_contiguous(200, 0, 14);
    EXPECT_FALSE(range.has_value());
}

TEST_F(ResourceGridTest, FindContiguousFragmented) {
    // Create fragmentation: allocate every other 10-RB block
    for (uint16_t i = 0; i < 100; i += 20) {
        TimeFrequencyResource res{{i, 10}, {0, 14}};
        grid_.allocate(res);
    }
    
    // Should find 10-RB gaps
    auto range = grid_.find_contiguous(10, 0, 14);
    ASSERT_TRUE(range.has_value());
    EXPECT_EQ(range->num_rbs, 10);
    
    // Should not find 15-RB contiguous
    range = grid_.find_contiguous(15, 0, 14);
    EXPECT_FALSE(range.has_value());
}

TEST_F(ResourceGridTest, Deallocate) {
    TimeFrequencyResource res{{0, 50}, {0, 7}};
    
    grid_.allocate(res);
    EXPECT_FALSE(grid_.is_available(res));
    
    grid_.deallocate(res);
    EXPECT_TRUE(grid_.is_available(res));
}

TEST_F(ResourceGridTest, Reset) {
    // Allocate everything
    TimeFrequencyResource full{{0, 106}, {0, 14}};
    grid_.allocate(full);
    
    EXPECT_EQ(grid_.total_available_res(), 0);
    
    grid_.reset();
    
    EXPECT_EQ(grid_.total_available_res(), 106 * 14);
}

TEST_F(ResourceGridTest, SymbolRanges) {
    // Allocate in first 7 symbols
    TimeFrequencyResource res1{{0, 50}, {0, 7}};
    grid_.allocate(res1);
    
    // Same RBs should be available in last 7 symbols
    TimeFrequencyResource res2{{0, 50}, {7, 7}};
    EXPECT_TRUE(grid_.is_available(res2));
    EXPECT_TRUE(grid_.allocate(res2));
}

// CCE Pool Tests
class CcePoolTest : public ::testing::Test {
protected:
    CcePool pool_{64};  // 64 CCEs
};

TEST_F(CcePoolTest, InitialState) {
    EXPECT_EQ(pool_.available(), 64);
}

TEST_F(CcePoolTest, AllocateAL1) {
    auto cce = pool_.allocate(AggregationLevel::AL1);
    ASSERT_TRUE(cce.has_value());
    EXPECT_EQ(pool_.available(), 63);
}

TEST_F(CcePoolTest, AllocateAL8) {
    auto cce = pool_.allocate(AggregationLevel::AL8);
    ASSERT_TRUE(cce.has_value());
    EXPECT_EQ(*cce % 8, 0);  // Should be aligned
    EXPECT_EQ(pool_.available(), 56);
}

TEST_F(CcePoolTest, AllocateAL16) {
    auto cce = pool_.allocate(AggregationLevel::AL16);
    ASSERT_TRUE(cce.has_value());
    EXPECT_EQ(*cce % 16, 0);  // Should be aligned
    EXPECT_EQ(pool_.available(), 48);
}

TEST_F(CcePoolTest, MultipleAllocations) {
    // Allocate 4 AL16 (uses all 64 CCEs)
    for (int i = 0; i < 4; ++i) {
        auto cce = pool_.allocate(AggregationLevel::AL16);
        ASSERT_TRUE(cce.has_value());
    }
    
    EXPECT_EQ(pool_.available(), 0);
    
    // Should fail to allocate more
    auto cce = pool_.allocate(AggregationLevel::AL1);
    EXPECT_FALSE(cce.has_value());
}

TEST_F(CcePoolTest, Deallocate) {
    auto cce = pool_.allocate(AggregationLevel::AL8);
    ASSERT_TRUE(cce.has_value());
    
    pool_.deallocate(*cce, AggregationLevel::AL8);
    EXPECT_EQ(pool_.available(), 64);
}

TEST_F(CcePoolTest, CanAllocate) {
    EXPECT_TRUE(pool_.can_allocate(AggregationLevel::AL16));
    
    // Allocate most CCEs
    for (int i = 0; i < 3; ++i) {
        pool_.allocate(AggregationLevel::AL16);
    }
    
    // 16 CCEs left
    EXPECT_TRUE(pool_.can_allocate(AggregationLevel::AL16));
    EXPECT_TRUE(pool_.can_allocate(AggregationLevel::AL8));
    
    pool_.allocate(AggregationLevel::AL8);
    
    // 8 CCEs left
    EXPECT_FALSE(pool_.can_allocate(AggregationLevel::AL16));
    EXPECT_TRUE(pool_.can_allocate(AggregationLevel::AL8));
}

TEST_F(CcePoolTest, Reset) {
    for (int i = 0; i < 4; ++i) {
        pool_.allocate(AggregationLevel::AL16);
    }
    
    EXPECT_EQ(pool_.available(), 0);
    
    pool_.reset();
    
    EXPECT_EQ(pool_.available(), 64);
}

// Frame Timing Tests
class FrameTimingTest : public ::testing::Test {};

TEST_F(FrameTimingTest, AbsoluteSlot) {
    FrameTiming t{100, 5, 0};
    t.slots_per_frame = 20;
    
    EXPECT_EQ(t.absolute_slot(), 100 * 20 + 5);
}

TEST_F(FrameTimingTest, Diff) {
    FrameTiming t1{100, 10, 0};
    FrameTiming t2{100, 5, 0};
    t1.slots_per_frame = 20;
    t2.slots_per_frame = 20;
    
    EXPECT_EQ(t1.diff(t2, 20), 5);
    EXPECT_EQ(t2.diff(t1, 20), -5);
}

TEST_F(FrameTimingTest, DiffWrapAround) {
    FrameTiming t1{5, 0, 0};
    FrameTiming t2{1020, 0, 0};
    t1.slots_per_frame = 20;
    t2.slots_per_frame = 20;
    
    // t1 is 9 frames after t2 (wrapping at 1024)
    int64_t diff = t1.diff(t2, 20);
    EXPECT_GT(diff, 0);  // t1 is "after" t2
}

// Numerology Tests
class NumerologyTest : public ::testing::Test {};

TEST_F(NumerologyTest, FromSCS15kHz) {
    auto cfg = NumerologyConfig::from_scs(SubcarrierSpacing::kHz15);
    EXPECT_EQ(cfg.slots_per_frame, 10);
    EXPECT_EQ(cfg.symbols_per_slot, 14);
}

TEST_F(NumerologyTest, FromSCS30kHz) {
    auto cfg = NumerologyConfig::from_scs(SubcarrierSpacing::kHz30);
    EXPECT_EQ(cfg.slots_per_frame, 20);
}

TEST_F(NumerologyTest, FromSCS120kHz) {
    auto cfg = NumerologyConfig::from_scs(SubcarrierSpacing::kHz120);
    EXPECT_EQ(cfg.slots_per_frame, 80);
}

// Resource Block Range Tests
class ResourceBlockRangeTest : public ::testing::Test {};

TEST_F(ResourceBlockRangeTest, EndRb) {
    ResourceBlockRange r{10, 20};
    EXPECT_EQ(r.end_rb(), 29);
}

TEST_F(ResourceBlockRangeTest, Overlaps) {
    ResourceBlockRange r1{10, 20};
    ResourceBlockRange r2{25, 10};  // 25-34, overlaps with 10-29
    ResourceBlockRange r3{30, 10};  // 30-39, no overlap
    
    EXPECT_TRUE(r1.overlaps(r2));
    EXPECT_FALSE(r1.overlaps(r3));
}

TEST_F(ResourceBlockRangeTest, Adjacent) {
    ResourceBlockRange r1{0, 50};
    ResourceBlockRange r2{50, 50};
    
    EXPECT_FALSE(r1.overlaps(r2));  // Adjacent but not overlapping
}

} // namespace nexgen::nr::test
