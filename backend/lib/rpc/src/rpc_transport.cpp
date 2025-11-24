#include "robot_module_kassow/rpc/rpc_transport.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <chrono>
#include <cstring>
#include <sstream>
#include <system_error>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <intrin.h>
#pragma comment(lib, "Ws2_32.lib")
using socket_len_t = int;
using socket_send_recv_t = int;
static constexpr int INVALID_SOCKET_FD = INVALID_SOCKET;
#elif defined(__APPLE__)
#include <arpa/inet.h>
#include <libkern/OSByteOrder.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
using socket_len_t = socklen_t;
using socket_send_recv_t = ssize_t;
static constexpr int INVALID_SOCKET_FD = -1;
#else
#include <arpa/inet.h>
#include <endian.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
using socket_len_t = socklen_t;
using socket_send_recv_t = ssize_t;
static constexpr int INVALID_SOCKET_FD = -1;
#endif

namespace aergo::robot::kassow::rpc
{
    namespace
    {
        constexpr size_t FRAME_HEADER_SIZE = 1 + 3 * sizeof(uint32_t); // kind + request_id + payload_size + blob_size
        constexpr uint32_t MAX_FRAME_SIZE = 64 * 1024 * 1024; // 64 MiB guard rail

        constexpr size_t REQUEST_WIRE_SIZE = 1 + sizeof(uint64_t) + sizeof(uint64_t); // req_type + feature + action_id
        constexpr size_t RESPONSE_WIRE_SIZE = 1 + sizeof(uint64_t); // resp_type + action_id
        constexpr size_t STATUS_WIRE_SIZE = sizeof(uint64_t); // feature
        constexpr size_t FINISHED_WIRE_SIZE = sizeof(uint64_t) + 1; // action_id + success

        uint32_t toNetwork32(uint32_t value)
        {
#ifdef _WIN32
            return _byteswap_ulong(value);
#else
            return htonl(value);
#endif
        }

        uint32_t fromNetwork32(uint32_t value)
        {
#ifdef _WIN32
            return _byteswap_ulong(value);
#else
            return ntohl(value);
#endif
        }

        uint64_t toNetwork64(uint64_t value)
        {
#if defined(_WIN32)
            return _byteswap_uint64(value);
#elif defined(__APPLE__)
            return OSSwapHostToBigInt64(value);
#else
            return htobe64(value);
#endif
        }

        uint64_t fromNetwork64(uint64_t value)
        {
#if defined(_WIN32)
            return _byteswap_uint64(value);
#elif defined(__APPLE__)
            return OSSwapBigToHostInt64(value);
#else
            return be64toh(value);
#endif
        }

        template <typename FrameHeaderT>
        std::array<uint8_t, FRAME_HEADER_SIZE> serializeHeader(const FrameHeaderT& header)
        {
            std::array<uint8_t, FRAME_HEADER_SIZE> data{};
            data[0] = static_cast<uint8_t>(header.kind);
            uint32_t net_request = toNetwork32(header.request_id);
            uint32_t net_payload = toNetwork32(header.payload_size);
            uint32_t net_blob = toNetwork32(header.blob_size);
            std::memcpy(data.data() + 1, &net_request, sizeof(uint32_t));
            std::memcpy(data.data() + 1 + sizeof(uint32_t), &net_payload, sizeof(uint32_t));
            std::memcpy(data.data() + 1 + 2 * sizeof(uint32_t), &net_blob, sizeof(uint32_t));
            return data;
        }

        template <typename FrameHeaderT>
        bool deserializeHeader(const uint8_t* data, FrameHeaderT& out)
        {
            out.kind = static_cast<RpcMessageKind>(data[0]);
            uint32_t net_request{};
            uint32_t net_payload{};
            uint32_t net_blob{};
            std::memcpy(&net_request, data + 1, sizeof(uint32_t));
            std::memcpy(&net_payload, data + 1 + sizeof(uint32_t), sizeof(uint32_t));
            std::memcpy(&net_blob, data + 1 + 2 * sizeof(uint32_t), sizeof(uint32_t));
            out.request_id = fromNetwork32(net_request);
            out.payload_size = fromNetwork32(net_payload);
            out.blob_size = fromNetwork32(net_blob);
            return true;
        }

        std::string lastErrorString()
        {
#ifdef _WIN32
            int err = WSAGetLastError();
            return std::system_category().message(err);
#else
            return std::system_category().message(errno);
#endif
        }

        std::array<std::byte, REQUEST_WIRE_SIZE> serializeRequest(const Request& req)
        {
            std::array<std::byte, REQUEST_WIRE_SIZE> buf{};
            buf[0] = static_cast<std::byte>(req.req_type);
            uint64_t feature_net = toNetwork64(static_cast<uint64_t>(req.feature));
            uint64_t action_net = toNetwork64(req.action_id);
            std::memcpy(buf.data() + 1, &feature_net, sizeof(uint64_t));
            std::memcpy(buf.data() + 1 + sizeof(uint64_t), &action_net, sizeof(uint64_t));
            return buf;
        }

        bool deserializeRequest(const std::byte* data, size_t size, Request& out)
        {
            if (size != REQUEST_WIRE_SIZE)
            {
                return false;
            }
            out.req_type = static_cast<aergo::module::helpers::robot_interface::ReqType>(static_cast<uint8_t>(data[0]));
            uint64_t feature_net{};
            uint64_t action_net{};
            std::memcpy(&feature_net, data + 1, sizeof(uint64_t));
            std::memcpy(&action_net, data + 1 + sizeof(uint64_t), sizeof(uint64_t));
            out.feature = static_cast<aergo::module::helpers::robot_interface::RobotFeature>(fromNetwork64(feature_net));
            out.action_id = fromNetwork64(action_net);
            return true;
        }

        std::array<std::byte, RESPONSE_WIRE_SIZE> serializeResponse(const Response& resp)
        {
            std::array<std::byte, RESPONSE_WIRE_SIZE> buf{};
            buf[0] = static_cast<std::byte>(resp.resp_type);
            uint64_t action_net = toNetwork64(resp.action_id);
            std::memcpy(buf.data() + 1, &action_net, sizeof(uint64_t));
            return buf;
        }

        bool deserializeResponse(const std::byte* data, size_t size, Response& out)
        {
            if (size != RESPONSE_WIRE_SIZE)
            {
                return false;
            }
            out.resp_type = static_cast<aergo::module::helpers::robot_interface::RespType>(static_cast<uint8_t>(data[0]));
            uint64_t action_net{};
            std::memcpy(&action_net, data + 1, sizeof(uint64_t));
            out.action_id = fromNetwork64(action_net);
            return true;
        }

        std::array<std::byte, STATUS_WIRE_SIZE> serializeStatus(const StatusMessage& status)
        {
            std::array<std::byte, STATUS_WIRE_SIZE> buf{};
            uint64_t feature_net = toNetwork64(static_cast<uint64_t>(status.feature));
            std::memcpy(buf.data(), &feature_net, sizeof(uint64_t));
            return buf;
        }

        bool deserializeStatus(const std::byte* data, size_t size, StatusMessage& out)
        {
            if (size != STATUS_WIRE_SIZE)
            {
                return false;
            }
            uint64_t feature_net{};
            std::memcpy(&feature_net, data, sizeof(uint64_t));
            out.feature = static_cast<aergo::module::helpers::robot_interface::RobotFeature>(fromNetwork64(feature_net));
            return true;
        }

        std::array<std::byte, FINISHED_WIRE_SIZE> serializeFinished(const FinishedMessage& finished)
        {
            std::array<std::byte, FINISHED_WIRE_SIZE> buf{};
            uint64_t action_net = toNetwork64(finished.action_id);
            std::memcpy(buf.data(), &action_net, sizeof(uint64_t));
            buf[sizeof(uint64_t)] = finished.success ? std::byte{1} : std::byte{0};
            return buf;
        }

        bool deserializeFinished(const std::byte* data, size_t size, FinishedMessage& out)
        {
            if (size != FINISHED_WIRE_SIZE)
            {
                return false;
            }
            uint64_t action_net{};
            std::memcpy(&action_net, data, sizeof(uint64_t));
            out.action_id = fromNetwork64(action_net);
            out.success = data[sizeof(uint64_t)] != std::byte{0};
            return true;
        }
    } // namespace


    TcpSocket::TcpSocket() = default;
    TcpSocket::~TcpSocket()
    {
        close();
    }

    TcpSocket::TcpSocket(TcpSocket&& other) noexcept
    {
        socket_fd_ = other.socket_fd_;
        other.socket_fd_.reset();
    }

    TcpSocket& TcpSocket::operator=(TcpSocket&& other) noexcept
    {
        if (this != &other)
        {
            close();
            socket_fd_ = other.socket_fd_;
            other.socket_fd_.reset();
        }
        return *this;
    }

    bool TcpSocket::initializeSockets(const RpcLogger* logger)
    {
        TcpSocket tmp;
        return tmp.ensureInitialized(logger);
    }

    bool TcpSocket::ensureInitialized(const RpcLogger* logger)
    {
#ifdef _WIN32
        static bool initialized = false;
        if (initialized)
        {
            return true;
        }
        WSADATA wsaData{};
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0)
        {
            if (logger)
            {
                auto msg = "WSAStartup failed: " + std::to_string(result);
                logger->log(RpcLogType::ERROR, msg.c_str());
            }
            return false;
        }
        initialized = true;
#endif
        (void)logger;
        return true;
    }

    void TcpSocket::logError(const RpcLogger* logger, std::string_view context) const
    {
        if (!logger)
        {
            return;
        }
        std::ostringstream oss;
        oss << context << ": " << lastErrorString();
        logger->log(RpcLogType::ERROR, oss.str().c_str());
    }

    bool TcpSocket::connect(const std::string& host, uint16_t port, const RpcLogger* logger)
    {
        close();
        if (!ensureInitialized(logger))
        {
            return false;
        }

        struct addrinfo hints {};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        struct addrinfo* result = nullptr;
        std::string port_str = std::to_string(port);
        int res = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &result);
        if (res != 0)
        {
            if (logger)
            {
                logger->log(RpcLogType::ERROR, ("getaddrinfo failed: " + std::string(gai_strerror(res))).c_str());
            }
            return false;
        }

        bool connected = false;
        for (struct addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next)
        {
            int fd = ::socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
            if (fd == INVALID_SOCKET_FD)
            {
                continue;
            }

            if (::connect(fd, ptr->ai_addr, static_cast<socket_len_t>(ptr->ai_addrlen)) == 0)
            {
                socket_fd_ = fd;
                connected = true;
                break;
            }
#ifdef _WIN32
            closesocket(fd);
#else
            ::close(fd);
#endif
        }

        freeaddrinfo(result);

        if (!connected && logger)
        {
            logError(logger, "connect failed");
        }

        return connected;
    }

    bool TcpSocket::adopt(int socket_fd, const RpcLogger* logger)
    {
        close();
        if (!ensureInitialized(logger))
        {
            return false;
        }
        socket_fd_ = socket_fd;
        return true;
    }

    void TcpSocket::close()
    {
        if (socket_fd_.has_value())
        {
#ifdef _WIN32
            closesocket(*socket_fd_);
#else
            ::close(*socket_fd_);
#endif
            socket_fd_.reset();
        }
    }

    bool TcpSocket::waitForData(std::chrono::milliseconds timeout, const RpcLogger* logger)
    {
        if (!socket_fd_.has_value())
        {
            return false;
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(*socket_fd_, &readfds);

        struct timeval tv {};
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

        int ret = select(*socket_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(*socket_fd_, &readfds))
        {
            return true;
        }
        if (ret < 0)
        {
            logError(logger, "select failed");
            close();
        }
        return false;
    }

    bool TcpSocket::sendAll(const uint8_t* data, size_t size, const RpcLogger* logger)
    {
        if (!socket_fd_.has_value())
        {
            return false;
        }

        size_t total_sent = 0;
        while (total_sent < size)
        {
            socket_send_recv_t sent = ::send(*socket_fd_, reinterpret_cast<const char*>(data + total_sent), static_cast<socket_send_recv_t>(size - total_sent), 0);
            if (sent <= 0)
            {
                logError(logger, "send failed");
                close();
                return false;
            }
            total_sent += static_cast<size_t>(sent);
        }
        return true;
    }

    bool TcpSocket::recvAll(uint8_t* data, size_t size, std::chrono::milliseconds timeout, const RpcLogger* logger)
    {
        if (!socket_fd_.has_value())
        {
            return false;
        }

        size_t total_recv = 0;
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (total_recv < size)
        {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
            {
                return false;
            }

            auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
            if (!waitForData(remaining_ms, logger))
            {
                return false;
            }

            socket_send_recv_t recvd = ::recv(*socket_fd_, reinterpret_cast<char*>(data + total_recv), static_cast<socket_send_recv_t>(size - total_recv), 0);
            if (recvd <= 0)
            {
                logError(logger, "recv failed");
                close();
                return false;
            }
            total_recv += static_cast<size_t>(recvd);
        }

        return true;
    }

    int TcpSocket::nativeHandle() const noexcept
    {
        return socket_fd_.value_or(INVALID_SOCKET_FD);
    }


    RpcClient::RpcClient(const RpcLogger* logger)
        : logger_(logger), next_request_id_(1)
    {}

    RpcClient::~RpcClient()
    {
        disconnect();
    }

    void RpcClient::log(RpcLogType type, std::string_view msg) const
    {
        if (logger_)
        {
            logger_->log(type, std::string(msg).c_str());
        }
    }

    bool RpcClient::connect(const std::string& host, uint16_t port)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        pending_request_id_.reset();
        return socket_.connect(host, port, logger_);
    }

    void RpcClient::disconnect()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        pending_request_id_.reset();
        socket_.close();
    }

    bool RpcClient::isConnected() const noexcept
    {
        return socket_.isConnected();
    }

    void RpcClient::setStatusCallback(StatusCallback cb)
    {
        status_cb_ = std::move(cb);
    }

    void RpcClient::setFinishedCallback(FinishedCallback cb)
    {
        finished_cb_ = std::move(cb);
    }

    bool RpcClient::writeFrame(const RpcFrameHeader& header, Span<const std::byte> payload, Span<const std::byte> blob)
    {
        auto header_buf = serializeHeader(header);
        if (!socket_.sendAll(header_buf.data(), header_buf.size(), logger_))
        {
            return false;
        }
        if (!payload.empty() && !socket_.sendAll(reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), logger_))
        {
            return false;
        }
        if (!blob.empty() && !socket_.sendAll(reinterpret_cast<const uint8_t*>(blob.data()), blob.size(), logger_))
        {
            return false;
        }
        return true;
    }

    bool RpcClient::readFrame(RpcFrameHeader& header, std::vector<std::byte>& payload, std::vector<std::byte>& blob, std::chrono::milliseconds timeout)
    {
        std::array<uint8_t, FRAME_HEADER_SIZE> header_raw{};
        if (!socket_.recvAll(header_raw.data(), header_raw.size(), timeout, logger_))
        {
            return false;
        }

        if (!deserializeHeader(header_raw.data(), header))
        {
            log(RpcLogType::ERROR, "Failed to deserialize RPC frame header");
            return false;
        }

        if (header.payload_size > MAX_FRAME_SIZE || header.blob_size > MAX_FRAME_SIZE)
        {
            log(RpcLogType::ERROR, "RPC frame exceeds maximum size");
            return false;
        }

        payload.resize(header.payload_size);
        blob.resize(header.blob_size);

        if (header.payload_size > 0)
        {
            if (!socket_.recvAll(reinterpret_cast<uint8_t*>(payload.data()), header.payload_size, timeout, logger_))
            {
                return false;
            }
        }
        if (header.blob_size > 0)
        {
            if (!socket_.recvAll(reinterpret_cast<uint8_t*>(blob.data()), header.blob_size, timeout, logger_))
            {
                return false;
            }
        }

        return true;
    }

    void RpcClient::dispatchAsync(const RpcFrameHeader& header, Span<const std::byte> payload, Span<const std::byte> blob)
    {
        switch (header.kind)
        {
            case RpcMessageKind::STATUS:
            {
                if (payload.size() == STATUS_WIRE_SIZE && status_cb_)
                {
                    StatusMessage status{};
                    if (deserializeStatus(payload.data(), payload.size(), status))
                    {
                        status_cb_(status, blob);
                    }
                    else
                    {
                        log(RpcLogType::WARNING, "Received status message with invalid payload");
                    }
                }
                break;
            }
            case RpcMessageKind::FINISHED:
            {
                if (payload.size() == FINISHED_WIRE_SIZE && finished_cb_)
                {
                    FinishedMessage finished{};
                    if (deserializeFinished(payload.data(), payload.size(), finished))
                    {
                        finished_cb_(finished, blob);
                    }
                    else
                    {
                        log(RpcLogType::WARNING, "Received finished message with invalid payload");
                    }
                }
                break;
            }
            default:
                log(RpcLogType::WARNING, "Received unexpected message while dispatching async data");
                break;
        }
    }

    bool RpcClient::sendRequest(const Request& request,
                                Span<const std::byte> request_blob,
                                Response& out_response,
                                std::vector<std::byte>& out_response_blob,
                                std::chrono::milliseconds timeout)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (!socket_.isConnected())
        {
            log(RpcLogType::ERROR, "RPC client is not connected");
            return false;
        }

        pending_request_id_ = next_request_id_++;

        RpcFrameHeader header{
            .kind = RpcMessageKind::REQUEST,
            .request_id = *pending_request_id_,
            .payload_size = static_cast<uint32_t>(REQUEST_WIRE_SIZE),
            .blob_size = static_cast<uint32_t>(request_blob.size())
        };

        auto payload_arr = serializeRequest(request);
        Span<const std::byte> payload(payload_arr.data(), payload_arr.size());
        if (!writeFrame(header, payload, request_blob))
        {
            pending_request_id_.reset();
            return false;
        }

        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            RpcFrameHeader incoming{};
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (!readFrame(incoming, payload_buffer_, blob_buffer_, remaining))
            {
                break;
            }

            if (incoming.kind == RpcMessageKind::RESPONSE)
            {
                if (!pending_request_id_.has_value() || incoming.request_id != *pending_request_id_)
                {
                    std::ostringstream oss;
                    oss << "Received response with unexpected request id: " << incoming.request_id
                        << ", expected: " << (pending_request_id_.has_value() ? std::to_string(*pending_request_id_) : std::string("none"));
                    log(RpcLogType::WARNING, oss.str());
                    continue;
                }

                if (payload_buffer_.size() != RESPONSE_WIRE_SIZE)
                {
                    log(RpcLogType::ERROR, "Invalid response payload size");
                    pending_request_id_.reset();
                    return false;
                }

                Response response_deserialized;
                if (!deserializeResponse(payload_buffer_.data(), payload_buffer_.size(), response_deserialized))
                {
                    log(RpcLogType::ERROR, "Failed to parse response payload");
                    pending_request_id_.reset();
                    return false;
                }

                out_response = response_deserialized;
                out_response_blob.resize(blob_buffer_.size());
                std::memcpy(out_response_blob.data(), blob_buffer_.data(), blob_buffer_.size());
                pending_request_id_.reset();
                return true;
            }

            // Async message while waiting for response
            dispatchAsync(incoming,
                          Span<const std::byte>(payload_buffer_.data(), payload_buffer_.size()),
                          Span<const std::byte>(blob_buffer_.data(), blob_buffer_.size()));
        }

        pending_request_id_.reset();
        return false;
    }

    bool RpcClient::pollOnce(std::chrono::milliseconds timeout)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (!socket_.isConnected())
        {
            return false;
        }

        if (!socket_.waitForData(timeout, logger_))
        {
            return false; // timeout
        }

        RpcFrameHeader header{};
        if (!readFrame(header, payload_buffer_, blob_buffer_, timeout))
        {
            return false;
        }

        if (header.kind == RpcMessageKind::RESPONSE)
        {
            log(RpcLogType::WARNING, "Received stray response without pending request, dropping");
            return false;
        }

        dispatchAsync(header,
                      Span<const std::byte>(payload_buffer_.data(), payload_buffer_.size()),
                      Span<const std::byte>(blob_buffer_.data(), blob_buffer_.size()));
        return true;
    }


    RpcServer::RpcServer(const RpcLogger* logger)
        : logger_(logger), listen_fd_(INVALID_SOCKET_FD)
    {}

    RpcServer::~RpcServer()
    {
        stop();
    }

    void RpcServer::log(RpcLogType type, std::string_view msg) const
    {
        if (logger_)
        {
            logger_->log(type, std::string(msg).c_str());
        }
    }

    bool RpcServer::start(uint16_t port)
    {
        stop();

        if (!TcpSocket::initializeSockets(logger_))
        {
            return false;
        }

        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_fd_ == INVALID_SOCKET_FD)
        {
            log(RpcLogType::ERROR, "Failed to create listen socket");
            return false;
        }

        int opt = 1;
        setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&opt), sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            log(RpcLogType::ERROR, "Failed to bind listen socket");
            stop();
            return false;
        }

        if (::listen(listen_fd_, 1) < 0)
        {
            log(RpcLogType::ERROR, "Failed to listen on socket");
            stop();
            return false;
        }

        log(RpcLogType::INFO, "RPC server listening");
        return true;
    }

    void RpcServer::stop()
    {
        client_.close();
        if (listen_fd_ != INVALID_SOCKET_FD)
        {
#ifdef _WIN32
            closesocket(listen_fd_);
#else
            ::close(listen_fd_);
#endif
            listen_fd_ = INVALID_SOCKET_FD;
        }
    }

    bool RpcServer::ensureClient()
    {
        if (client_.isConnected())
        {
            return true;
        }
        if (listen_fd_ == INVALID_SOCKET_FD)
        {
            log(RpcLogType::ERROR, "RPC server is not listening");
            return false;
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_fd_, &readfds);

        struct timeval tv {};
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int ret = select(listen_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(listen_fd_, &readfds))
        {
            sockaddr_storage addr{};
            socket_len_t len = sizeof(addr);
            int new_fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
            if (new_fd != INVALID_SOCKET_FD)
            {
                client_.adopt(new_fd, logger_);
                log(RpcLogType::INFO, "Accepted RPC client connection");
                return true;
            }
        }

        return client_.isConnected();
    }

    void RpcServer::setRequestHandler(std::function<void(const IncomingRequest&)> handler)
    {
        request_handler_ = std::move(handler);
    }

    bool RpcServer::pollOnce(std::chrono::milliseconds timeout)
    {
        if (!ensureClient())
        {
            return false;
        }

        if (!client_.waitForData(timeout, logger_))
        {
            return false;
        }

        RpcFrameHeader header{};

        std::array<uint8_t, FRAME_HEADER_SIZE> header_raw{};
        if (!client_.recvAll(header_raw.data(), header_raw.size(), timeout, logger_))
        {
            return false;
        }

        if (!deserializeHeader(header_raw.data(), header))
        {
            log(RpcLogType::ERROR, "RPC server failed to deserialize header");
            return false;
        }

        if (header.payload_size > MAX_FRAME_SIZE || header.blob_size > MAX_FRAME_SIZE)
        {
            log(RpcLogType::ERROR, "RPC server frame too large");
            return false;
        }

        payload_buffer_.resize(header.payload_size);
        blob_buffer_.resize(header.blob_size);
        if (header.payload_size > 0)
        {
            if (!client_.recvAll(reinterpret_cast<uint8_t*>(payload_buffer_.data()), header.payload_size, timeout, logger_))
            {
                return false;
            }
        }
        if (header.blob_size > 0)
        {
            if (!client_.recvAll(reinterpret_cast<uint8_t*>(blob_buffer_.data()), header.blob_size, timeout, logger_))
            {
                return false;
            }
        }

        if (header.kind == RpcMessageKind::REQUEST && payload_buffer_.size() == REQUEST_WIRE_SIZE)
        {
            IncomingRequest incoming_request;
            if (!deserializeRequest(payload_buffer_.data(), payload_buffer_.size(), incoming_request.request))
            {
                log(RpcLogType::WARNING, "Failed to parse request payload");
                return false;
            }
            incoming_request.request_id = header.request_id;
            incoming_request.blob = Span<const std::byte>(blob_buffer_.data(), blob_buffer_.size());

            if (request_handler_)
            {
                request_handler_(incoming_request);
                return true;
            }

            log(RpcLogType::WARNING, "RPC server received request but no handler is set");
            return false;
        }

        log(RpcLogType::WARNING, "RPC server received unsupported frame");
        return false;
    }

    bool RpcServer::sendResponse(uint32_t request_id, const Response& response, Span<const std::byte> blob)
    {
        if (!client_.isConnected())
        {
            return false;
        }

        RpcFrameHeader header{
            .kind = RpcMessageKind::RESPONSE,
            .request_id = request_id,
            .payload_size = static_cast<uint32_t>(RESPONSE_WIRE_SIZE),
            .blob_size = static_cast<uint32_t>(blob.size())
        };

        auto payload_arr = serializeResponse(response);
        Span<const std::byte> payload(payload_arr.data(), payload_arr.size());
        auto header_buf = serializeHeader(header);
        if (!client_.sendAll(header_buf.data(), header_buf.size(), logger_))
        {
            return false;
        }
        if (!payload.empty() && !client_.sendAll(reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), logger_))
        {
            return false;
        }
        if (!blob.empty() && !client_.sendAll(reinterpret_cast<const uint8_t*>(blob.data()), blob.size(), logger_))
        {
            return false;
        }
        return true;
    }

    bool RpcServer::sendStatusMessage(const StatusMessage& status, Span<const std::byte> blob)
    {
        if (!client_.isConnected())
        {
            return false;
        }

        RpcFrameHeader header{
            .kind = RpcMessageKind::STATUS,
            .request_id = 0,
            .payload_size = static_cast<uint32_t>(STATUS_WIRE_SIZE),
            .blob_size = static_cast<uint32_t>(blob.size())
        };

        auto payload_arr = serializeStatus(status);
        Span<const std::byte> payload(payload_arr.data(), payload_arr.size());
        auto header_buf = serializeHeader(header);
        if (!client_.sendAll(header_buf.data(), header_buf.size(), logger_))
        {
            return false;
        }
        if (!client_.sendAll(reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), logger_))
        {
            return false;
        }
        if (!blob.empty() && !client_.sendAll(reinterpret_cast<const uint8_t*>(blob.data()), blob.size(), logger_))
        {
            return false;
        }
        return true;
    }

    bool RpcServer::sendFinishedMessage(const FinishedMessage& finished, Span<const std::byte> blob)
    {
        if (!client_.isConnected())
        {
            return false;
        }

        RpcFrameHeader header{
            .kind = RpcMessageKind::FINISHED,
            .request_id = finished.action_id ? static_cast<uint32_t>(finished.action_id) : 0u,
            .payload_size = static_cast<uint32_t>(FINISHED_WIRE_SIZE),
            .blob_size = static_cast<uint32_t>(blob.size())
        };

        auto payload_arr = serializeFinished(finished);
        Span<const std::byte> payload(payload_arr.data(), payload_arr.size());
        auto header_buf = serializeHeader(header);
        if (!client_.sendAll(header_buf.data(), header_buf.size(), logger_))
        {
            return false;
        }
        if (!client_.sendAll(reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), logger_))
        {
            return false;
        }
        if (!blob.empty() && !client_.sendAll(reinterpret_cast<const uint8_t*>(blob.data()), blob.size(), logger_))
        {
            return false;
        }
        return true;
    }
}
