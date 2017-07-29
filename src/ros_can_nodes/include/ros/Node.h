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

#ifndef ROSCPP_INIT_H
#define ROSCPP_INIT_H

#include "ros/forwards.h"
#include "ros/spinner.h"
#include "common.h"

#include "ros/names.h"
#include "ros/xmlrpc_manager.h"
#include "ros/poll_manager.h"
#include "ros/connection_manager.h"
#include "ros/topic_manager.h"
#include "ros/service_manager.h"
#include "ros/network.h"
#include "ros/file_log.h"
#include "ros/callback_queue.h"
#include "ros/param.h"
#include "ros/rosout_appender.h"
#include "ros/subscribe_options.h"
#include "ros/transport/transport_tcp.h"
#include "ros/internal_timer_manager.h"
#include "XmlRpcSocket.h"

#include "roscpp/GetLoggers.h"
#include "roscpp/SetLoggerLevel.h"
#include "roscpp/Empty.h"

#include <ros/console.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>

#include <algorithm>

#include <signal.h>

#include <cstdlib>

namespace ros
{

    class ROSCPP_DECL Node {
        private:
            CallbackQueuePtr g_global_queue;
            ROSOutAppender* g_rosout_appender;
            CallbackQueuePtr g_internal_callback_queue;

            bool g_initialized;
            bool g_started;
            bool g_atexit_registered;
            bool g_ok;
            uint32_t g_init_options;
            bool g_shutdown_requested;
            volatile bool g_shutting_down;
            boost::recursive_mutex g_shutting_down_mutex;
            boost::thread g_internal_queue_thread;
            boost::mutex g_start_mutex;

            std::string name_;
            std::string namespace_;

        public:
            /**
             * \brief Returns the name of the current node.
             */
            ROSCPP_DECL const std::string& getName() const;
            /**
             * \brief Returns the namespace of the current node.
             */
            ROSCPP_DECL const std::string& getNamespace() const;

            /** @brief Get the list of topics advertised by this node
             *
             * @param[out] topics The advertised topics
             */
            ROSCPP_DECL void getAdvertisedTopics(V_string& topics);

            /** @brief Get the list of topics subscribed to by this node
             *
             * @param[out] The subscribed topics
             */
            ROSCPP_DECL void getSubscribedTopics(V_string& topics);

            void checkForShutdown();
            /**
             * \brief Returns whether or not ros::shutdown() has been (or is being) called
             */
            bool isShuttingDown();


            bool getLoggers(roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response& resp);


            bool setLoggerLevel(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response&);

            bool closeAllConnections(roscpp::Empty::Request&, roscpp::Empty::Response&);

            void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);

            CallbackQueuePtr getInternalCallbackQueue();

            void basicSigintHandler(int sig);

            void internalCallbackQueueThreadFunc();

            void start(const std::string& name);


            void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
            /** \brief Enter simple event loop
             *
             * This method enters a loop, processing callbacks.  This method should only be used
             * if the NodeHandle API is being used.
             *
             * This method is mostly useful when your node does all of its work in
             * subscription callbacks.  It will not process any callbacks that have been assigned to
             * custom queues.
             *
             */
            void spin();

            /** \brief Enter simple event loop
             *
             * This method enters a loop, processing callbacks.  This method should only be used
             * if the NodeHandle API is being used.
             *
             * This method is mostly useful when your node does all of its work in
             * subscription callbacks.  It will not process any callbacks that have been assigned to
             * custom queues.
             *
             * \param spinner a spinner to use to call callbacks.  Two default implementations are available,
             * SingleThreadedSpinner and MultiThreadedSpinner
             */
            void spin(Spinner& spinner);
            /**
             * \brief Process a single round of callbacks.
             *
             * This method is useful if you have your own loop running and would like to process
             * any callbacks that are available.  This is equivalent to calling callAvailable() on the
             * global CallbackQueue.  It will not process any callbacks that have been assigned to
             * custom queues.
             */
            void spinOnce();

            /**
             * \brief Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown(), or similar.
             */
            void waitForShutdown();

            /** \brief Check whether it's time to exit.
             *
             * ok() becomes false once ros::shutdown() has been called and is finished
             *
             * \return true if we're still OK, false if it's time to exit
             */
            bool ok();
            /**
             * \brief Disconnects everything and unregisters from the master.  It is generally not
             * necessary to call this function, as the node will automatically shutdown when all
             * NodeHandles destruct.  However, if you want to break out of a spin() loop explicitly,
             * this function allows that.
             */
            void shutdown();

            /**
             * \brief Request that the node shut itself down from within a ROS thread
             *
             * This method signals a ROS thread to call shutdown().
             */
            void requestShutdown();

            /**
             * \brief Actually starts the internals of the node (spins up threads, starts the network polling and xmlrpc loops,
             * connects to internal subscriptions like /clock, starts internal service servers, etc.).
             *
             * Usually unnecessary to call manually, as it is automatically called by the creation of the first NodeHandle if
             * the node has not already been started.  If you would like to prevent the automatic shutdown caused by the last
             * NodeHandle going out of scope, call this before any NodeHandle has been created (e.g. immediately after init())
             */
            void start();
            /**
             * \brief Returns whether or not the node has been started through ros::start()
             */
            bool isStarted();

            /**
             * \brief Returns a pointer to the global default callback queue.
             *
             * This is the queue that all callbacks get added to unless a different one is specified, either in the NodeHandle
             * or in the individual NodeHandle::subscribe()/NodeHandle::advertise()/etc. functions.
             */
            CallbackQueue* getGlobalCallbackQueue();
    };

}

#endif
