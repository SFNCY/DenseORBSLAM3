#include <gtest/gtest.h>

TEST(ExampleTest, BasicAssertion) {
    EXPECT_EQ(1 + 1, 2);
}

TEST(ExampleTest, StringOperation) {
    std::string s = "Hello";
    EXPECT_EQ(s.length(), 5u);
}

TEST(ExampleTest, VectorOperation) {
    std::vector<int> v = {1, 2, 3};
    EXPECT_EQ(v.size(), 3u);
    EXPECT_EQ(v[0], 1);
}