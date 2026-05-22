#include "plaif/moforcecontroller/data_analysis.h"

/*
// 1) 1단(평탄) 업데이트
DebugDataCommunicator::Instance().Update("K", K);
DebugDataCommunicator::Instance().Update("B", B);

// 2) 2단(그룹) 업데이트
DebugDataCommunicator::Map2D bulk2 = {
    {"Thread Process Time", {
        {"Timer Thread", program_time.time[__PROGRAM_TIMER_THREAD__]},
        {"Planning Thread", program_time.time[__PLANNING_THREAD__]},
        {"Recive Thread", program_time.time[__RECIVEDATA_THREAD__]},
        {"Send Thread",   program_time.time[__SENDDATA_THREAD__]},
        {"Control Thread",program_time.time[__CONTROL_THREAD__]}
    }},
    {"Position Controller Output", {
        {"Joint1", position_controller_result.position[0]},
        {"Joint2", position_controller_result.position[1]},
        {"Joint3", position_controller_result.position[2]},
        {"Joint4", position_controller_result.position[3]},
        {"Joint5", position_controller_result.position[4]},
        {"Joint6", position_controller_result.position[5]}
    }}
};
DebugDataCommunicator::Instance().UpdateBulk(bulk2);

// 3) 전송
DebugDataCommunicator::Instance().SendSnapshot();        // 내부 누적 JSON 전송
DebugDataCommunicator::Instance().Send(bulk2);           // 임시 nested 페이로드 전송

------------------------------------------------------
*/

DebugDataCommunicator& DebugDataCommunicator::Instance() {
    static DebugDataCommunicator instance;
    return instance;
}

bool DebugDataCommunicator::OpenDebugMode(int port, const std::string& ip) {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
        io_ctx_ = std::make_unique<boost::asio::io_context>();

        auto addr = boost::asio::ip::make_address(ip);
        endpoint_ = boost::asio::ip::udp::endpoint(addr, static_cast<unsigned short>(port));

        socket_ = std::make_unique<boost::asio::ip::udp::socket>(*io_ctx_, boost::asio::ip::udp::v4());
        return true;
    } catch (...) {
        socket_.reset();
        io_ctx_.reset();
        return false;
    }
}

void DebugDataCommunicator::CloseDebugMode() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (socket_) {
        boost::system::error_code ec;
        socket_->close(ec);
        socket_.reset();
    }
    io_ctx_.reset();
}

bool DebugDataCommunicator::IsOpen() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<bool>(socket_);
}

// ===== 업데이트 =====

void DebugDataCommunicator::Update(const std::string& key, double value) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data_.is_object()) data_ = nlohmann::json::object();
    data_[key] = value; // (최상위) 키 추가/갱신
}

void DebugDataCommunicator::UpdateBulk(const Map1D& flat) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data_.is_object()) data_ = nlohmann::json::object();
    for (const auto& [k, v] : flat) {
        data_[k] = v;
    }
}

void DebugDataCommunicator::UpdateBulk(const Map2D& nested) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data_.is_object()) data_ = nlohmann::json::object();

    for (const auto& [group, kvs] : nested) {
        nlohmann::json& g = data_[group];
        if (!g.is_object()) g = nlohmann::json::object();
        for (const auto& [k, v] : kvs) {
            g[k] = v;
        }
    }
}

bool DebugDataCommunicator::Erase(const std::string& key) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!data_.is_object()) return false;
    return data_.erase(key) > 0;
}

void DebugDataCommunicator::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    data_.clear();
    data_ = nlohmann::json::object();
}

nlohmann::json DebugDataCommunicator::Snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

// ===== 전송 =====

bool DebugDataCommunicator::SendSnapshot() {
    nlohmann::json snap;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        snap = data_;
    }
    return send_json(snap);
}

bool DebugDataCommunicator::Send(const Map1D& flat) {
    return send_json(BuildJson(flat));
}

bool DebugDataCommunicator::Send(const Map2D& nested) {
    return send_json(BuildJson(nested));
}

bool DebugDataCommunicator::Send(const nlohmann::json& j) {
    return send_json(j);
}

// ===== helper =====

nlohmann::json DebugDataCommunicator::BuildJson(const Map1D& flat) {
    // std::map<string,double> -> JSON object
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [k, v] : flat) j[k] = v;
    return j;
}

nlohmann::json DebugDataCommunicator::BuildJson(const Map2D& nested) {
    // std::map<string, std::map<string,double>> -> JSON object of objects
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [group, kvs] : nested) {
        nlohmann::json obj = nlohmann::json::object();
        for (const auto& [k, v] : kvs) obj[k] = v;
        j[group] = std::move(obj);
    }
    return j;
}

bool DebugDataCommunicator::send_json(const nlohmann::json& j) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!socket_) return false;

    const std::string payload = j.dump();
    boost::system::error_code ec;
    socket_->send_to(boost::asio::buffer(payload), endpoint_, 0, ec);
    return !ec;
}

DebugDataCommunicator::~DebugDataCommunicator() {
    CloseDebugMode();
}