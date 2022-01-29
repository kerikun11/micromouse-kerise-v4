/*!
@file concurrent_queue.hpp
@author JongYoon Lim
@brief A simple C++11 Concurrent Queue based on std::queue.
@see https://github.com/jlim262/concurrent_queue
 */
#pragma once
#ifndef LIME62_CONCURRENT_QUEUE_H
#define LIME62_CONCURRENT_QUEUE_H

#include <queue>
#include <atomic>
#include <mutex>
#include <condition_variable>

/*!
@brief namespace for JongYoon Lim
@see https://github.com/lime62
*/
namespace lime62 {
    /*!
    @brief A simple C++11 Concurrent Queue based on std::queue.

     Supports waiting operations for retrieving an element when it's empty.
     It's interrupted by calling interrupt().

    @ref std::queue class
    */
    template<typename T, typename Container=std::deque<T> >
    class concurrent_queue {
    public:
        typedef typename std::queue<T>::size_type size_type;
        typedef typename std::queue<T>::reference reference;
        typedef typename std::queue<T>::const_reference const_reference;

        ~concurrent_queue() {
            interrupt();
        }

        void interrupt() {
            // interrupted_ = true;
            condition_variable_.notify_one();
        }

        void push(T const &e) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                queue_.push(e);
            }
            condition_variable_.notify_one();
        }

        template<typename... _Args>
        void emplace(_Args&&... __args) {
            {
                std::unique_lock<std::mutex> lock(this->mutex_);
                queue_.emplace(std::forward<_Args>(__args)...);
            }
            condition_variable_.notify_one();
        }

        bool empty() {
            std::unique_lock<std::mutex> lock(this->mutex_);
            return queue_.empty();
        }

        void pop() {
            std::unique_lock<std::mutex> lock(this->mutex_);
            if (!queue_.empty())
                queue_.pop();
        }

        void front_pop(T& ret) {
            std::unique_lock<std::mutex> lock(this->mutex_);
            wait(lock);
            ret = queue_.front();
            queue_.pop();
        }

        size_type size() {
            std::unique_lock<std::mutex> lock(this->mutex_);
            return queue_.size();
        }

        reference front() {
            std::unique_lock<std::mutex> lock(this->mutex_);
            wait(lock);
            return queue_.front();
        }

        const_reference front() const {
            std::unique_lock<std::mutex> lock(this->mutex_);
            wait(lock);
            return queue_.front();
        }

        reference back() {
            std::unique_lock<std::mutex> lock(this->mutex_);
            wait(lock);
            return queue_.back();
        }

        const_reference back() const {
            std::unique_lock<std::mutex> lock(this->mutex_);
            wait(lock);
            return queue_.back();
        }

        void swap(concurrent_queue& q) {
            throw std::exception("Not supported");
        }

    protected:
        std::queue<T, Container> queue_;
        std::mutex mutex_;
        std::condition_variable condition_variable_;
        // std::atomic_bool interrupted_;

    private:
        void wait(std::unique_lock<std::mutex>& lock) {
            // interrupted_ = false;
            while (queue_.empty()) {
                condition_variable_.wait(lock);
                // if (interrupted_)
                //     throw std::runtime_error("Interrupted");
            }
        }
    };
}
#endif //LIME62_CONCURRENT_QUEUE_H
