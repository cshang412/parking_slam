#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <mutex>

namespace parking_slam
{

/**
 * @brief
 *
 * @tparam Key
 * @tparam T
 */
template <typename Key, typename T>
class ConcurrentMap
{
public:
    /**
     * @brief Construct a new Concurrent Map object
     *
     */
    ConcurrentMap() = default;

    /**
     * @brief Destroy the Concurrent Map object
     *
     */
    ~ConcurrentMap() = default;

    /**
     * @brief
     *
     * @param key
     * @return T&
     */
    T& operator[](const Key& key)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data[key];
    }

    /**
     * @brief
     *
     * @param key
     * @return T&
     */
    T& operator[](Key&& key)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data[key];
    }

    /**
     * @brief
     *
     * @param key
     * @return T&
     */
    T& at(const Key& key)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.at(key);
    }

    /**
     * @brief
     *
     * @param key
     * @return const T&
     */
    const T& at(const Key& key) const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.at(key);
    }

    /**
     * @brief
     *
     * @return auto
     */
    auto begin()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.begin();
    }

    /**
     * @brief
     *
     * @return auto
     */
    auto end()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.end();
    }

    /**
     * @brief
     *
     * @return size_t
     */
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.size();
    }

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _data.empty();
    }

    /**
     * @brief
     *
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _data.clear();
    }

private:
    /**
     * @brief
     *
     */
    std::map<Key, T>   _data;
    mutable std::mutex _data_mutex{};
};

}  // namespace parking_slam
