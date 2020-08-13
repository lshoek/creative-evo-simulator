#pragma once
#include <vector>

class VectorUtils 
{
public:
    static std::vector<double> flatten(const std::vector<std::vector<double>>& full)
    {
        std::vector<double> accum;
        for (auto sub : full) {
            accum.insert(end(accum), std::begin(sub), std::end(sub));
        }
        return accum;
    }
};
