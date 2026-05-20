//
// Created by user on 5/20/26.
//

#include <gtest/gtest.h>

TEST(MyTest, AlwaysPasses) {
    SUCCEED();

    SUCCEED() << "This test always passes.";
}