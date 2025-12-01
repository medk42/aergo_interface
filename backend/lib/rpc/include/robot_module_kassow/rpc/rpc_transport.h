#pragma once

#include "module_helpers/robot_interface/message_types.h"
#include "module_helpers/robot_interface/cpp17_utils.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aergo::robot::kassow::rpc
{
    using Request = aergo::module::helpers::robot_interface::Request;
    using Response = aergo::module::helpers::robot_interface::Response;
    using StatusMessage = aergo::module::helpers::robot_interface::StatusMessage;
    using FinishedMessage = aergo::module::helpers::robot_interface::FinishedMessage;

    enum class RpcLogType { INFO, WARNING, ERROR };

    class RpcLogger
    {
    public:
        virtual ~RpcLogger() = default;
        virtual void log(RpcLogType type, const char* message) const noexcept = 0;
    };

    enum class RpcMessageKind : uint8_t
    {
        REQUEST = 0,
        RESPONSE = 1,
        STATUS = 2,
        FINISHED = 3
    };

    struct RpcFrameHeader
    {
        RpcMessageKind kind{};
        uint32_t request_id{};
        uint32_t payload_size{};
        uint32_t blob_size{};
    };

    class TcpSocket
    {
    public:
        TcpSocket();
        ~TcpSocket();

        static bool initializeSockets(const RpcLogger* logger);

        TcpSocket(const TcpSocket&) = delete;
        TcpSocket& operator=(const TcpSocket&) = delete;

        TcpSocket(TcpSocket&& other) noexcept;
        TcpSocket& operator=(TcpSocket&& other) noexcept;

        bool connect(const std::string& host, uint16_t port, const RpcLogger* logger);
        bool adopt(int socket_fd, const RpcLogger* logger);
        void close();

        bool isConnected() const noexcept { return socket_fd_.has_value(); }

        bool sendAll(const uint8_t* data, size_t size, const RpcLogger* logger);
        bool recvAll(uint8_t* data, size_t size, std::chrono::milliseconds timeout, const RpcLogger* logger);
        bool waitForData(std::chrono::milliseconds timeout, const RpcLogger* logger);

        int nativeHandle() const noexcept;

        bool setNoDelay(bool no_delay, const RpcLogger* logger);

    private:
        bool ensureInitialized(const RpcLogger* logger);
        void logError(const RpcLogger* logger, std::string_view context) const;

        std::optional<int> socket_fd_;
    };

    class RpcClient
    {
    public:
        using StatusCallback = std::function<void(const StatusMessage&, Span<const std::byte>)>;
        using FinishedCallback = std::function<void(const FinishedMessage&, Span<const std::byte>)>;

        explicit RpcClient(const RpcLogger* logger);
        ~RpcClient();

        bool connect(const std::string& host, uint16_t port);
        void disconnect();
        bool isConnected() const noexcept;

        void setStatusCallback(StatusCallback cb);
        void setFinishedCallback(FinishedCallback cb);

        bool sendRequest(const Request& request,
                         Span<const std::byte> request_blob,
                         Response& out_response,
                         std::vector<std::byte>& out_response_blob,
                         std::chrono::milliseconds timeout);

        bool pollOnce(std::chrono::milliseconds timeout);

    private:
        bool readFrame(RpcFrameHeader& header, std::vector<std::byte>& payload, std::vector<std::byte>& blob, std::chrono::milliseconds timeout);
        bool writeFrame(const RpcFrameHeader& header, Span<const std::byte> payload, Span<const std::byte> blob);
        void dispatchAsync(const RpcFrameHeader& header, Span<const std::byte> payload, Span<const std::byte> blob);
        void log(RpcLogType type, std::string_view msg) const;

        const RpcLogger* logger_;
        TcpSocket socket_;
        std::mutex socket_mutex_;
        uint32_t next_request_id_;
        std::optional<uint32_t> pending_request_id_;
        StatusCallback status_cb_;
        FinishedCallback finished_cb_;

        std::vector<std::byte> payload_buffer_;
        std::vector<std::byte> blob_buffer_;
    };

    class RpcServer
    {
    public:
        struct IncomingRequest
        {
            uint32_t request_id{};
            Request request{};
            Span<const std::byte> blob;
        };

        explicit RpcServer(const RpcLogger* logger);
        ~RpcServer();

        bool start(uint16_t port);
        void stop();
        void setRequestHandler(std::function<void(const IncomingRequest&)> handler);
        bool pollOnce(std::chrono::milliseconds timeout);

        bool sendResponse(uint32_t request_id, const Response& response, Span<const std::byte> blob);
        bool sendStatusMessage(const StatusMessage& status, Span<const std::byte> blob);
        bool sendFinishedMessage(const FinishedMessage& finished, Span<const std::byte> blob);

    private:
        bool ensureClient();
        void log(RpcLogType type, std::string_view msg) const;

        const RpcLogger* logger_;
        int listen_fd_;
        TcpSocket client_;
        std::function<void(const IncomingRequest&)> request_handler_;
        std::vector<std::byte> payload_buffer_;
        std::vector<std::byte> blob_buffer_;
    };
}
