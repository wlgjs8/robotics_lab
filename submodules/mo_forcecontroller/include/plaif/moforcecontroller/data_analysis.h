#pragma once

#include <map>
#include <string>
#include <memory>
#include <mutex>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

class DebugDataCommunicator {
    public:
        // 싱글톤 접근자
        static DebugDataCommunicator& Instance();
    
        // 네트워킹
        bool OpenDebugMode(int port, const std::string& ip = "127.0.0.1");
        void CloseDebugMode();
        bool IsOpen() const;
    
        // 타입 별칭
        using Map1D = std::map<std::string, double>;
        using Map2D = std::map<std::string, Map1D>; // group -> (key -> value)
    
        // 업데이트 API
        void Update(const std::string& key, double value);   // 1단 키 추가/갱신
        void UpdateBulk(const Map1D& flat);                   // 1단 일괄
        void UpdateBulk(const Map2D& nested);                 // 2단(그룹) 일괄
    
        bool Erase(const std::string& key);                   // 최상위 키 제거(그룹/단일)
        void Clear();                                         // 전체 제거
    
        // 스냅샷(JSON 반환: 중첩 유지)
        nlohmann::json Snapshot() const;
    
        // 전송 API
        bool SendSnapshot();                                  // 내부 JSON 스냅샷 전송
        bool Send(const Map1D& flat);                         // 1단 맵 전송
        bool Send(const Map2D& nested);                       // 2단 맵 전송
        bool Send(const nlohmann::json& j);                   // 임의 JSON 전송
    
        // 복사/대입 금지
        DebugDataCommunicator(const DebugDataCommunicator&) = delete;
        DebugDataCommunicator& operator=(const DebugDataCommunicator&) = delete;
    
    private:
        DebugDataCommunicator() = default;
        ~DebugDataCommunicator();
    
        // helper
        static nlohmann::json BuildJson(const Map1D& flat);
        static nlohmann::json BuildJson(const Map2D& nested);
        bool send_json(const nlohmann::json& j);
    
        // 데이터(중첩 가능)
        mutable std::mutex mtx_;
        nlohmann::json data_;  // 내부 상태: object로 유지
    
        // 네트워킹
        std::unique_ptr<boost::asio::io_context> io_ctx_;
        std::unique_ptr<boost::asio::ip::udp::socket> socket_;
        boost::asio::ip::udp::endpoint endpoint_;
    };