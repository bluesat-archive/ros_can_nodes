/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros_node_lib/callback_queue.h"
#include <boost/scope_exit.hpp>
#include <chrono>
#include <boost/make_shared.hpp>

namespace roscan {

void CallbackQueue::enable() {
    std::lock_guard<std::mutex> lock{mutex_};
    enabled_ = true;
    condition_.notify_all();
}

void CallbackQueue::disable() {
    std::lock_guard<std::mutex> lock{mutex_};
    enabled_ = false;
    condition_.notify_all();
}

void CallbackQueue::clear() {
    std::lock_guard<std::mutex> lock{mutex_};
    callbacks_.clear();
}

bool CallbackQueue::isEmpty() {
    std::lock_guard<std::mutex> lock{mutex_};
    return callbacks_.empty() && calling_ == 0;
}

bool CallbackQueue::isEnabled() {
    std::lock_guard<std::mutex> lock{mutex_};
    return enabled_;
}

void CallbackQueue::setupTLS() {
    if (!tls_.get()) {
        tls_.reset(new TLS{});
    }
}

void CallbackQueue::addCallback(const CallbackInterfacePtr& callback, const uint64_t removal_id) {
    CallbackInfo info;
    info.callback = callback;
    info.removal_id = removal_id;

    {
        std::lock_guard<std::mutex> lock{id_info_mutex_};

        if (id_info_.find(removal_id) == id_info_.end()) {
            IDInfoPtr id_info{boost::make_shared<IDInfo>()};
            id_info->id = removal_id;
            id_info_.insert(std::make_pair(removal_id, id_info));
        }
    }

    {
        std::lock_guard<std::mutex> lock{mutex_};

        if (!enabled_) {
            return;
        }
        callbacks_.push_back(info);
    }
    condition_.notify_one();
}

CallbackQueue::IDInfoPtr CallbackQueue::getIDInfo(const uint64_t id) {
    std::lock_guard<std::mutex> lock{id_info_mutex_};
    const auto it = id_info_.find(id);
    return it != id_info_.end() ? it->second : IDInfoPtr();
}

void CallbackQueue::removeByID(const uint64_t removal_id) {
    setupTLS();

    {
        IDInfoPtr id_info;
        {
            std::lock_guard<std::mutex> lock{id_info_mutex_};
            const auto it = id_info_.find(removal_id);
            if (it != id_info_.end()) {
                id_info = it->second;
            } else {
                return;
            }
        }

        // If we're being called from within a callback from our queue, we must unlock the shared lock we already own
        // here so that we can take a unique lock.  We'll re-lock it later.
        if (tls_->calling_in_this_thread == id_info->id) {
            id_info->calling_rw_mutex.unlock_shared();
        }

        {
            std::unique_lock<std::shared_timed_mutex> rw_lock{id_info->calling_rw_mutex};
            std::lock_guard<std::mutex> lock{mutex_};
            callbacks_.erase(std::remove_if(callbacks_.begin(), callbacks_.end(), [removal_id](const auto& info){ return removal_id == info.removal_id; }), callbacks_.end());
        }

        if (tls_->calling_in_this_thread == id_info->id) {
            id_info->calling_rw_mutex.lock_shared();
        }
    }

    // If we're being called from within a callback, we need to remove the callbacks that match the id that have already been
    // popped off the queue
    for (auto& info: tls_->callbacks) {
        if (info.removal_id == removal_id) {
            info.marked_for_removal = true;
        }
    }

    {
        std::lock_guard<std::mutex> lock{id_info_mutex_};
        id_info_.erase(removal_id);
    }
}

CallbackQueue::CallOneResult CallbackQueue::callOne(const ros::WallDuration timeout) {
    setupTLS();
    auto tls = tls_.get();

    CallbackInfo cb_info;

    {
        std::unique_lock<std::mutex> lock{mutex_};

        if (!enabled_) {
            return Disabled;
        }

        if (callbacks_.empty()) {
            if (!timeout.isZero()) {
                condition_.wait_for(lock, std::chrono::nanoseconds(timeout.toNSec()));
            }

            if (callbacks_.empty()) {
                return Empty;
            }

            if (!enabled_) {
                return Disabled;
            }
        }

        const auto it = std::find_if(callbacks_.cbegin(), callbacks_.cend(), [](const auto& info){ return info.callback->ready(); });
        if (it != callbacks_.cend()) {
            cb_info = *it;
            callbacks_.erase(it);
        }
        callbacks_.erase(std::remove_if(callbacks_.begin(), callbacks_.end(), [](const auto& info){ return info.marked_for_removal; }), callbacks_.end());

        if (!cb_info.callback) {
            return TryAgain;
        }
        ++calling_;
    }

    auto was_empty = tls->callbacks.empty();
    tls->callbacks.push_back(cb_info);
    if (was_empty) {
        tls->cb_it = tls->callbacks.begin();
    }

    auto res = callOneCB(tls);
    if (res != Empty) {
        std::lock_guard<std::mutex> lock{mutex_};
        --calling_;
    }
    return res;
}

void CallbackQueue::callAvailable(const ros::WallDuration timeout) {
    setupTLS();
    auto tls = tls_.get();

    {
        std::unique_lock<std::mutex> lock{mutex_};

        if (!enabled_) {
            return;
        }

        if (callbacks_.empty()) {
            if (!timeout.isZero()) {
                condition_.wait_for(lock, std::chrono::nanoseconds(timeout.toNSec()));
            }

            if (callbacks_.empty() || !enabled_) {
                return;
            }
        }

        auto was_empty = tls->callbacks.empty();

        tls->callbacks.insert(tls->callbacks.end(), callbacks_.begin(), callbacks_.end());
        callbacks_.clear();

        calling_ += tls->callbacks.size();

        if (was_empty) {
            tls->cb_it = tls->callbacks.begin();
        }
    }

    size_t called = 0;

    while (!tls->callbacks.empty()) {
        if (callOneCB(tls) != Empty) {
            ++called;
        }
    }

    {
        std::lock_guard<std::mutex> lock{mutex_};
        calling_ -= called;
    }
}

CallbackQueue::CallOneResult CallbackQueue::callOneCB(TLS *const tls) {
    // Check for a recursive call.  If recursive, increment the current iterator.  Otherwise
    // set the iterator it the beginning of the thread-local callbacks
    if (tls->calling_in_this_thread == 0xffffffffffffffffULL) {
        tls->cb_it = tls->callbacks.begin();
    }

    if (tls->cb_it == tls->callbacks.end()) {
        return Empty;
    }

    auto info = *tls->cb_it;
    auto& cb = info.callback;

    auto id_info = getIDInfo(info.removal_id);
    if (id_info) {
        std::shared_lock<std::shared_timed_mutex> rw_lock{id_info->calling_rw_mutex};

        auto last_calling = tls->calling_in_this_thread;
        tls->calling_in_this_thread = id_info->id;

        auto result = CallbackInterface::Invalid;

        {
            // Ensure that thread id gets restored, even if callback throws.
            // This is done with RAII rather than try-catch so that the source
            // of the original exception is not masked in a crash report.
            BOOST_SCOPE_EXIT(&tls, &last_calling) {
                tls->calling_in_this_thread = last_calling;
            }
            BOOST_SCOPE_EXIT_END

            tls->cb_it = tls->callbacks.erase(tls->cb_it);
            if (!info.marked_for_removal) {
                result = cb->call();
            }
        }

        // Push TryAgain callbacks to the back of the shared queue
        if (result == CallbackInterface::TryAgain && !info.marked_for_removal) {
            std::lock_guard<std::mutex> lock{mutex_};
            callbacks_.push_back(info);
            return TryAgain;
        }
        return Called;
    } else {
        tls->cb_it = tls->callbacks.erase(tls->cb_it);
    }
    return Called;
}

} // namespace roscan
