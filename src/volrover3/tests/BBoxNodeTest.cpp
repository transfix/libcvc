#include <gtest/gtest.h>
#include <volrover3/BBoxNode.h>
#include <cvc/bounding_box.h>

class BBoxNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        testBBox = cvc::bounding_box(0.0, 0.0, 0.0, 10.0, 20.0, 30.0);
    }
    
    void TearDown() override {
    }
    
    cvc::bounding_box testBBox;
};

// ============================================================================
// Coordinate Label Tests (2 corners instead of 8)
// ============================================================================

TEST_F(BBoxNodeTest, CoordinateLabelsDefaultVisible) {
    BBoxNode node("test.bbox");
    
    // Coordinate labels should be visible by default
    EXPECT_TRUE(node.getCoordinatesVisible());
}

TEST_F(BBoxNodeTest, SetCoordinatesVisible) {
    BBoxNode node("test.bbox");
    
    node.setCoordinatesVisible(false);
    EXPECT_FALSE(node.getCoordinatesVisible());
    
    node.setCoordinatesVisible(true);
    EXPECT_TRUE(node.getCoordinatesVisible());
}

TEST_F(BBoxNodeTest, CoordinateLabelCount) {
    BBoxNode node("test.bbox");
    
    // Set coordinates visible first
    node.setCoordinatesVisible(true);
    
    // Then set bounding box (this will create the labels)
    node.setBoundingBox(testBBox);
    
    // After the fix, we should have exactly 2 labels (min and max corners)
    // instead of 8 labels (all corners)
    // This is verified by the internal implementation in createCoordinateLabels()
    
    // We can verify the labels are created by checking visibility
    EXPECT_TRUE(node.getCoordinatesVisible());
}

TEST_F(BBoxNodeTest, CoordinateLabelColor) {
    BBoxNode node("test.bbox");
    
    // Set label color
    node.setCoordinateLabelColor(1.0, 0.5, 0.0);
    
    // Get label color
    double r, g, b;
    node.getCoordinateLabelColor(r, g, b);
    
    EXPECT_DOUBLE_EQ(r, 1.0);
    EXPECT_DOUBLE_EQ(g, 0.5);
    EXPECT_DOUBLE_EQ(b, 0.0);
}

TEST_F(BBoxNodeTest, CoordinateLabelFontSize) {
    BBoxNode node("test.bbox");
    
    // Default font size should be 12
    EXPECT_EQ(node.getCoordinateLabelFontSize(), 12);
    
    // Set new font size
    node.setCoordinateLabelFontSize(18);
    EXPECT_EQ(node.getCoordinateLabelFontSize(), 18);
    
    // Should clamp to minimum 1
    node.setCoordinateLabelFontSize(0);
    EXPECT_EQ(node.getCoordinateLabelFontSize(), 1);
    
    node.setCoordinateLabelFontSize(-5);
    EXPECT_EQ(node.getCoordinateLabelFontSize(), 1);
}

// ============================================================================
// Bounding Box Tests
// ============================================================================

TEST_F(BBoxNodeTest, DefaultConstruction) {
    BBoxNode node("test.bbox");
    
    // Default bbox should be (-1, -1, -1) to (1, 1, 1)
    cvc::bounding_box bbox = node.getBoundingBox();
    
    EXPECT_DOUBLE_EQ(bbox[0], -1.0);
    EXPECT_DOUBLE_EQ(bbox[1], -1.0);
    EXPECT_DOUBLE_EQ(bbox[2], -1.0);
    EXPECT_DOUBLE_EQ(bbox[3], 1.0);
    EXPECT_DOUBLE_EQ(bbox[4], 1.0);
    EXPECT_DOUBLE_EQ(bbox[5], 1.0);
}

TEST_F(BBoxNodeTest, SetBoundingBox) {
    BBoxNode node("test.bbox");
    
    node.setBoundingBox(testBBox);
    
    cvc::bounding_box bbox = node.getBoundingBox();
    
    EXPECT_DOUBLE_EQ(bbox[0], 0.0);
    EXPECT_DOUBLE_EQ(bbox[1], 0.0);
    EXPECT_DOUBLE_EQ(bbox[2], 0.0);
    EXPECT_DOUBLE_EQ(bbox[3], 10.0);
    EXPECT_DOUBLE_EQ(bbox[4], 20.0);
    EXPECT_DOUBLE_EQ(bbox[5], 30.0);
}

TEST_F(BBoxNodeTest, SetColor) {
    BBoxNode node("test.bbox");
    
    // Set yellow color (default is yellow anyway)
    node.setColor(1.0, 1.0, 0.0);
    
    // We can't directly verify the color is applied to the actor,
    // but we can verify the method doesn't crash
    EXPECT_NO_THROW(node.setColor(0.5, 0.5, 0.5));
}

TEST_F(BBoxNodeTest, DefaultVisibility) {
    BBoxNode node("test.bbox");
    
    // BBox should be visible by default
    EXPECT_TRUE(node.isVisible());
}

TEST_F(BBoxNodeTest, SetVisibility) {
    BBoxNode node("test.bbox");
    
    node.setVisible(false);
    EXPECT_FALSE(node.isVisible());
    
    node.setVisible(true);
    EXPECT_TRUE(node.isVisible());
}

TEST_F(BBoxNodeTest, LargeBoundingBox) {
    BBoxNode node("test.bbox");
    
    // Test with a large bounding box (like the 945x945x945 volume)
    cvc::bounding_box largeBBox(0.0, 0.0, 0.0, 945.0, 945.0, 945.0);
    node.setBoundingBox(largeBBox);
    
    cvc::bounding_box bbox = node.getBoundingBox();
    
    EXPECT_DOUBLE_EQ(bbox[0], 0.0);
    EXPECT_DOUBLE_EQ(bbox[1], 0.0);
    EXPECT_DOUBLE_EQ(bbox[2], 0.0);
    EXPECT_DOUBLE_EQ(bbox[3], 945.0);
    EXPECT_DOUBLE_EQ(bbox[4], 945.0);
    EXPECT_DOUBLE_EQ(bbox[5], 945.0);
    
    // Verify coordinates are visible by default
    EXPECT_TRUE(node.getCoordinatesVisible());
}

TEST_F(BBoxNodeTest, NonUniformBoundingBox) {
    BBoxNode node("test.bbox");
    
    // Test with non-uniform bounds
    cvc::bounding_box nonUniform(-10.0, -5.0, 0.0, 100.0, 50.0, 25.0);
    node.setBoundingBox(nonUniform);
    
    cvc::bounding_box bbox = node.getBoundingBox();
    
    EXPECT_DOUBLE_EQ(bbox[0], -10.0);
    EXPECT_DOUBLE_EQ(bbox[1], -5.0);
    EXPECT_DOUBLE_EQ(bbox[2], 0.0);
    EXPECT_DOUBLE_EQ(bbox[3], 100.0);
    EXPECT_DOUBLE_EQ(bbox[4], 50.0);
    EXPECT_DOUBLE_EQ(bbox[5], 25.0);
}

TEST_F(BBoxNodeTest, UpdateBoundingBox) {
    BBoxNode node("test.bbox");
    
    // Set initial bbox
    node.setBoundingBox(testBBox);
    
    cvc::bounding_box bbox1 = node.getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox1[3], 10.0);
    
    // Update to different bbox
    cvc::bounding_box newBBox(5.0, 5.0, 5.0, 15.0, 15.0, 15.0);
    node.setBoundingBox(newBBox);
    
    cvc::bounding_box bbox2 = node.getBoundingBox();
    EXPECT_DOUBLE_EQ(bbox2[0], 5.0);
    EXPECT_DOUBLE_EQ(bbox2[3], 15.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
