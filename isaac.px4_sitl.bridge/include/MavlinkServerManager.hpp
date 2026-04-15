#pragma once

#include <map>
#include <memory>
#include <atomic>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include "../../../../../PX4-Autopilot/build/px4_sitl_default/mavlink/common/mavlink.h"

namespace isaac {
namespace px4_sitl {
namespace bridge {

class MavlinkTcpConnection
{
public:
    MavlinkTcpConnection(int port) : port_(port), server_fd_(-1), client_fd_(-1) {}
    ~MavlinkTcpConnection() { shutdown(); }

    bool init(void)
    {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0)
            return false;

        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))
            return false;

        address_.sin_family = AF_INET;
        address_.sin_addr.s_addr = INADDR_ANY;
        address_.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr *)&address_, sizeof(address_)) < 0)
            return false;

        if (listen(server_fd_, 1) < 0)
            return false;

        if (fcntl(server_fd_, F_SETFL, O_NONBLOCK) < 0)
            return false;
        return true;
    }

    bool connect(void)
    {
        socklen_t addrlen = sizeof(address_);
        if (client_fd_ < 0)
        {
            client_fd_ = accept(server_fd_, (struct sockaddr *)&address_, &addrlen);
            if (client_fd_ < 0)
            {
                /** @note Resource will be temporary unavailable until PX4 is ready */
                // CARB_LOG_ERROR("Accept failed: %s", strerror(errno));
                return false;
            }

            if (fcntl(client_fd_, F_SETFL, O_NONBLOCK) < 0)
            {
                // CARB_LOG_ERROR("Failed to set non-blocking mode: %s", strerror(errno));
                close(client_fd_);
                client_fd_ = -1;
                return false;
            }

            CARB_LOG_WARN("Client connected on port %d", port_);
        }
        return true;
    }

    bool send_data(const uint8_t *data, size_t length)
    {
        if (client_fd_ < 0)
            return false;

        return send(client_fd_, data, length, 0) == static_cast<ssize_t>(length);
    }

    bool receive_data(uint8_t *buffer, size_t max_length, size_t &bytes_received)
    {
        if (client_fd_ < 0)
            return false;

        ssize_t result = recv(client_fd_, buffer, max_length, 0);
        if (result > 0)
        {
            bytes_received = static_cast<size_t>(result);
            return true;
        }
        else if (result == 0 || (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
        {
            close(client_fd_);
            client_fd_ = -1;
            return false;
        }
        return true;
    }

    void shutdown(void)
    {
        if (client_fd_ >= 0)
            close(client_fd_);
        if (server_fd_ >= 0)
            close(server_fd_);
        client_fd_ = server_fd_ = -1;
    }

    bool isConnected(void) const
    {
        return client_fd_ >= 0;
    }

private:
    const int port_;
    int server_fd_, client_fd_;
    struct sockaddr_in address_;
};

class MavlinkServerManager
{
public:
    static MavlinkServerManager &getInstance()
    {
        static MavlinkServerManager instance;
        return instance;
    }

    bool createServerSocket(int instance_id)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        int port = BASE_PORT + instance_id;

        auto connection = std::make_shared<MavlinkTcpConnection>(port);
        if (!connection->init())
            return false;

        connections_[instance_id] = connection;
        return true;
    }

    bool getServerSocket(int instance_id)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = connections_.find(instance_id);
        return it != connections_.end();
    }

    bool connect(int instance_id)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = connections_.find(instance_id);
        if (it != connections_.end())
        {
            return it->second->connect();
        }
        return false;
    }

    bool isConnected(int instance_id)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = connections_.find(instance_id);
        if (it != connections_.end())
        {
            return it->second->isConnected();
        }
        return false;
    }

    bool send(int instance_id, const uint8_t *data, size_t length)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = connections_.find(instance_id);
        if (it != connections_.end())
        {
            return it->second->send_data(data, length);
        }
        return false;
    }

    bool receive(int instance_id, uint8_t *buffer, size_t max_length, size_t &bytes_received)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = connections_.find(instance_id);
        if (it != connections_.end())
        {
            return it->second->receive_data(buffer, max_length, bytes_received);
        }
        return false;
    }

    void removeAllConnections(void)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!connections_.empty())
            connections_.clear();
    }

private:
    // Delete copy constructor and assignment operator
    MavlinkServerManager() = default;
    MavlinkServerManager(const MavlinkServerManager &) = delete;
    void operator=(const MavlinkServerManager &) = delete;
    static constexpr int BASE_PORT = 4560;

    std::map<int, std::shared_ptr<MavlinkTcpConnection>> connections_;
    std::mutex mtx_;
};

} // bridge
} // px4_sitl
} // isaac