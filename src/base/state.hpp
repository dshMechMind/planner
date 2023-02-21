#pragma once

#include <vector>
#include <iostream>
#include "../utility/class_forward.hpp"
#include "../utility/export.hpp"

namespace planner {
PLANNER_STRUCT_FORWARD(State)

struct EXPORT State
{
    State(const std::vector<double>& v) : values(v) {}
    State() = default;
    
    std::vector<double> values;

    void push_back(double v) { values.push_back(v); }
    void resize(std::size_t size) { values.resize(size); }

    double operator[](std::size_t index) const { return values[index]; }

    void printState(const std::string& desc = "" ) const
    {
        std::cout << desc << std::endl;
        for (const auto& v : values)
            std::cout << v << ", ";
        std::cout << std::endl;
    }

    std::size_t size() const { return values.size(); }

};
}
