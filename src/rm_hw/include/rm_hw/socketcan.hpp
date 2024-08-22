#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>
#include <boost/function.hpp>

namespace can  {
    class SocketCAN {
    private:
        ifreq interface_request_{};
        sockaddr_can address_{};
        pthread_t receiver_thread_id_{};
    public:
        /**
         * @brief CAN socket file descriptor
         */
        int sock_id_ = -1;
        /**
         * @brief Request for the child thread to terminate
         */
        bool terminate_receiver_thread_ = false;
        bool receiver_thread_running_ = false;

    SocketCAN() = default;
    ~SocketCAN();

    /**
     * @brief  Open and bind socket
     * 
     * @param interface bus's name
     * @param handler pointer to a function which shall 
     * 
     * @return \c true if it was successful, \c false otherwise
     */
    bool open(const std::string& interface, boost::function<void(const can_frame& frame)> handler);
    /**
     * @brief Close and unbind the socket
     * 
     */
    void close();
    /**
     * @brief Returns whether the socket is open or closed
     * 
     * @return \c true if the socket is open, \c false otherwise
     */
    bool is_open() const;
    /**
     * @brief Sends the referenced frame to the CAN bus
     * 
     * @param frame referenced frame which you want to send
     * 
     */
    void write(can_frame& frame) const;
    /**
     * @brief Starts a new thread, which listens to the CAN bus
     * 
     */
    bool start_receiver_thread(int thread_priority);
    /**
     * @brief Pointer to the function which shall be called when a new frame is received
     * 
     */
    boost::function<void(const can_frame& frame)> reception_handler_;
    };
} //namespace can