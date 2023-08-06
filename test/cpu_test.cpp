#include <gtest/gtest.h>

TEST(CPU, DummyTest) {

  EXPECT_TRUE(true);

  EXPECT_EQ(2 + 2, 4);

  EXPECT_STREQ("hello", "hello");

  EXPECT_FALSE(false);
}
