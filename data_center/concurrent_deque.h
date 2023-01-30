#pragma once

#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

namespace parking_slam
{

/**
 * @brief
 *
 * @tparam T
 */
template <typename T>
class ConcurrentDeque
{
public:
    /**
     * @brief Construct a new Concurrent Deque object
     *
     * @param max_size
     */
    ConcurrentDeque(const size_t& max_size = 1000) : _max_size(max_size)
    {
    }

    // TODO @dabing: 删除拷贝构造函数, 增加专用拷贝函数;
    ConcurrentDeque(const ConcurrentDeque<T>& c)
    {
        std::lock_guard<std::mutex> lock_outer(c._data_mutex);
        std::lock_guard<std::mutex> lock_inner(this->_data_mutex);
        this->_max_size = c._max_size;
        this->_deque    = c._deque;
    }
    // ConcurrentDeque& copy() {
    //     std::lock_guard<std::mutex> lock_inner(this->_data_mutex);
    //     ConcurrentDeque<T> c(this->_max_size);
    //     std::lock_guard<std::mutex> lock_outer(c._data_mutex);
    //     c._deque = this->_deque;
    //     return c;
    // }

    /**
     * @brief Destroy the Concurrent Deque object
     *
     */
    ~ConcurrentDeque() = default;

    /**
     * @brief
     *
     * @param data
     */
    void insert(const T& data)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _deque.push_back(data);
        if (_deque.size() > _max_size)
        {
            _deque.pop_front();
        }
    }

    /**
     * @brief
     *
     * @param pos
     * @return T&
     */
    T& operator[](const size_t& pos)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque[pos];
    }

    /**
     * @brief
     *
     * @param pos
     * @return const T&
     */
    const T& operator[](const size_t& pos) const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque[pos];
    }

    /**
     * @brief
     *
     * @param pos
     * @return T&
     */
    T& at(const size_t& pos)
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.at(pos);
    }

    /**
     * @brief
     *
     * @param pos
     * @return const T&
     */
    const T& at(const size_t& pos) const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.at(pos);
    }

    /**
     * @brief
     *
     * @return T&
     */
    T& front()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.front();
    }

    /**
     * @brief
     *
     * @return auto
     */
    auto begin()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.begin();
    }

    /**
     * @brief
     *
     * @return auto
     */
    auto end()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.end();
    }

    /**
     * @brief
     *
     */
    void pop_front()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        if (!_deque.empty())
        {
            _deque.pop_front();
        }
        else
        {
            // do nothing
        }
    }

    /**
     * @brief
     *
     */
    void pop_back()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        if (!_deque.empty())
        {
            _deque.pop_back();
        }
        else
        {
            // do nothing
        }
    }

    /**
     * @brief
     *
     * @return const T&
     */
    const T& front() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.front();
    }

    /**
     * @brief
     *
     * @return const T&
     */
    const T& back() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.back();
    }

    /**
     * @brief
     *
     * @return size_t
     */
    size_t size() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        return _deque.size();
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
        return _deque.empty();
    }

    /**
     * @brief
     *
     * @param data
     * @return true
     * @return false
     */
    bool latest(T& data) const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        if (_deque.empty)
        {
            return false;
        }

        data = _deque.back();
        return true;
    }

    /**
     * @brief
     *
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _deque.clear();
    }

    /**
     * @brief
     *
     * @return std::vector<T>
     */
    std::vector<T> to_vector() const
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        std::vector<T>              data_vec;
        data_vec.resize(_deque.size());
        for (size_t i = 0; i < _deque.size(); ++i)
        {
            data_vec[i] = _deque.at(i);
        }

        return data_vec;
    }

private:
    /**
     * @brief
     *
     */
    std::deque<T>      _deque;
    size_t             _max_size;
    mutable std::mutex _data_mutex{};
};

}  // namespace parking_slam
