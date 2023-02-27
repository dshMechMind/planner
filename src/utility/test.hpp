#pragma once

#include <iostream>

namespace planner
{
class TestPrint
{
public:
    TestPrint() = default;
    void print()
    { std::cout << "TestPrint: plannerLib test!!!!!!!!!!!!" << std::endl; }
};
}
