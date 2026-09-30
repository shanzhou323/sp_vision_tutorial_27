#include "statistics.hpp"

void Statistics::onProduced()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ++produced_;
}
void Statistics::onProcessed()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ++processed_;
}
void Statistics::onSaved()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ++saved_;
}
void Statistics::onCorrupted()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ++corrupted_;
}
StatisticsSnapshot Statistics::snapshot() const
{
    std::lock_guard<std::mutex> lock(mtx_);
    return {produced_, processed_, saved_, corrupted_};
}