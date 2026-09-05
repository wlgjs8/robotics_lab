#include "rb_servo/kinematics/dh_calibration.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <regex>
#include <sstream>

namespace rb_servo {

DhTable rb5850eNominalDh() {
    return DhTable{{
        {0.0, 169.20, 0.00, -90.0},
        {-90.0, -148.40, 425.00, 0.0},
        {0.0, 148.40, 392.00, 0.0},
        {90.0, -110.70, 0.00, 90.0},
        {0.0, 110.70, 0.00, -90.0},
        {0.0, -96.70, 0.00, 90.0},
    }};
}

const DhSlotLayout& rb5850eLinkParameterLayout() {
    static const DhSlotLayout layout{8, {
        {0, 2, DhField::Alpha},
        {1, 3, DhField::Alpha},
        {2, 1, DhField::D},
        {3, 2, DhField::A},
        {4, 3, DhField::A},
        {5, 4, DhField::D},
    }};
    return layout;
}

std::vector<double> parseLinkParameterPayload(const std::string& payload) {
    std::vector<double> values;
    const std::size_t lb = payload.find('[');
    const std::size_t rb = payload.find(']', lb == std::string::npos ? 0 : lb);
    std::string body;
    if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        body = payload.substr(lb + 1, rb - lb - 1);
    } else {
        const std::size_t k = payload.find("link_parameter");
        body = k == std::string::npos ? payload : payload.substr(k + std::strlen("link_parameter"));
    }
    const char* c = body.c_str();
    while (*c) {
        char* e = nullptr;
        const double v = std::strtod(c, &e);
        if (e == c) { ++c; continue; }
        values.push_back(v);
        c = e;
    }
    return values;
}

namespace {
double& cell(DhRow& r, DhField f) {
    switch (f) {
        case DhField::ThetaOffset: return r.theta_offset_deg;
        case DhField::D: return r.d_mm;
        case DhField::A: return r.a_mm;
        case DhField::Alpha: return r.alpha_deg;
    }
    return r.d_mm;
}
double cellValue(const DhRow& r, DhField f) {
    switch (f) {
        case DhField::ThetaOffset: return r.theta_offset_deg;
        case DhField::D: return r.d_mm;
        case DhField::A: return r.a_mm;
        case DhField::Alpha: return r.alpha_deg;
    }
    return r.d_mm;
}
const char* fieldName(DhField f) {
    switch (f) {
        case DhField::ThetaOffset: return "theta_offset";
        case DhField::D: return "d";
        case DhField::A: return "a";
        case DhField::Alpha: return "alpha";
    }
    return "?";
}
bool isAngle(DhField f) { return f == DhField::Alpha || f == DhField::ThetaOffset; }
std::string fmt(double v) {
    char b[64];
    std::snprintf(b, sizeof(b), "%.10g", v);
    return b;
}
}  // namespace

std::string adoptLinkParameter(const std::vector<double>& raw, const DhTable& nominal,
                               const DhSlotLayout& layout, double max_abs_delta_mm,
                               double max_abs_delta_deg, DhAdoption* out) {
    if (out == nullptr) return "adoptLinkParameter: no output";
    if (static_cast<int>(raw.size()) != layout.count) {
        return "link_parameter has " + std::to_string(raw.size()) + " values but the RB5-850E layout is " +
               std::to_string(layout.count) + " values - unknown firmware layout, nothing adopted";
    }
    for (double v : raw) {
        if (!std::isfinite(v)) return "link_parameter carries a non-finite value";
    }
    std::vector<bool> mapped(raw.size(), false);
    DhAdoption a;
    a.table = nominal;
    a.raw = raw;
    for (const DhSlot& s : layout.slots) {
        if (s.slot < 0 || s.slot >= static_cast<int>(raw.size()) || s.joint < 1 || s.joint > 6) {
            return "link_parameter layout is malformed";
        }
        mapped[static_cast<std::size_t>(s.slot)] = true;
        const double target = raw[static_cast<std::size_t>(s.slot)];
        const double nom = cellValue(nominal[static_cast<std::size_t>(s.joint - 1)], s.field);
        const double delta = target - nom;
        const double limit = isAngle(s.field) ? max_abs_delta_deg : max_abs_delta_mm;
        if (std::fabs(delta) > limit) {
            return std::string("link_parameter slot ") + std::to_string(s.slot) + " (J" + std::to_string(s.joint) +
                   "." + fieldName(s.field) + ") = " + fmt(target) + " is " + fmt(delta) +
                   (isAngle(s.field) ? " deg" : " mm") + " from nominal " + fmt(nom) + ", beyond the " +
                   fmt(limit) + " limit - refusing the whole table";
        }
        cell(a.table[static_cast<std::size_t>(s.joint - 1)], s.field) = target;
        if (std::fabs(delta) > 1e-9) ++a.changed_cells;
        if (isAngle(s.field)) {
            a.max_abs_delta_deg = std::max(a.max_abs_delta_deg, std::fabs(delta));
        } else {
            a.max_abs_delta_mm = std::max(a.max_abs_delta_mm, std::fabs(delta));
        }
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (!mapped[i] && std::fabs(raw[i]) > 1e-9) {
            return "link_parameter slot " + std::to_string(i) + " is not in the RB5-850E layout but carries " +
                   fmt(raw[i]) + " - an unknown calibration arrived, nothing adopted";
        }
    }
    *out = std::move(a);
    return "";
}

namespace {
struct JointEdit {
    const char* joint;
    double dx, dy, dz;      // m
    double droll, dpitch, dyaw;   // rad
};
constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;
}  // namespace

std::string patchUrdfWithDh(const std::string& urdf_text, const std::string& joint_prefix,
                            const DhTable& nominal, const DhTable& calibrated,
                            std::string* out_text, int* edits) {
    if (out_text == nullptr) return "patchUrdfWithDh: no output";
    if (edits != nullptr) *edits = 0;
    const auto d = [&](int j, DhField f) {
        return cellValue(calibrated[static_cast<std::size_t>(j - 1)], f) -
               cellValue(nominal[static_cast<std::size_t>(j - 1)], f);
    };
    const JointEdit plan[] = {
        {"base_joint", 0.0, 0.0, d(1, DhField::D) / 1000.0, 0.0, 0.0, 0.0},
        {"elbow_joint", 0.0, 0.0, d(2, DhField::A) / 1000.0, 0.0, 0.0, d(2, DhField::Alpha) * kDegToRad},
        {"wrist1_joint", 0.0, 0.0, d(3, DhField::A) / 1000.0, 0.0, 0.0, d(3, DhField::Alpha) * kDegToRad},
        {"wrist2_joint", 0.0, d(4, DhField::D) / 1000.0, 0.0, 0.0, 0.0, 0.0},
    };
    // Cells this rewrite does not carry: refuse rather than drop them silently.
    for (int j = 1; j <= 6; ++j) {
        for (DhField f : {DhField::ThetaOffset, DhField::D, DhField::A, DhField::Alpha}) {
            const bool carried =
                (j == 1 && f == DhField::D) || (j == 2 && (f == DhField::A || f == DhField::Alpha)) ||
                (j == 3 && (f == DhField::A || f == DhField::Alpha)) || (j == 4 && f == DhField::D);
            if (!carried && std::fabs(d(j, f)) > 1e-12) {
                return std::string("DH cell J") + std::to_string(j) + "." + fieldName(f) +
                       " differs from nominal but this URDF rewrite does not carry it";
            }
        }
    }
    std::string text = urdf_text;
    int count = 0;
    for (const JointEdit& e : plan) {
        const bool any = std::fabs(e.dx) > 0 || std::fabs(e.dy) > 0 || std::fabs(e.dz) > 0 ||
                         std::fabs(e.droll) > 0 || std::fabs(e.dpitch) > 0 || std::fabs(e.dyaw) > 0;
        const std::string name = joint_prefix + e.joint;
        const std::regex joint_re("<joint\\s+name=\"" + name + "\"[^>]*>([\\s\\S]*?)</joint>");
        std::smatch jm;
        std::sregex_iterator it(text.begin(), text.end(), joint_re), end;
        int matches = 0;
        for (auto i = it; i != end; ++i) ++matches;
        if (matches != 1) {
            return "URDF joint \"" + name + "\" found " + std::to_string(matches) + " times (need exactly 1)";
        }
        if (!std::regex_search(text, jm, joint_re)) return "URDF joint \"" + name + "\" vanished";
        const std::string body = jm[1].str();
        const std::regex origin_re("<origin\\s+xyz=\"([^\"]*)\"\\s+rpy=\"([^\"]*)\"\\s*/>");
        std::smatch om;
        if (!std::regex_search(body, om, origin_re)) {
            return "URDF joint \"" + name + "\" has no <origin xyz=.. rpy=../>";
        }
        if (!any) continue;
        double v[6];
        {
            std::istringstream a(om[1].str());
            std::istringstream b(om[2].str());
            if (!(a >> v[0] >> v[1] >> v[2]) || !(b >> v[3] >> v[4] >> v[5])) {
                return "URDF joint \"" + name + "\" origin is not six numbers";
            }
        }
        v[0] += e.dx; v[1] += e.dy; v[2] += e.dz;
        v[3] += e.droll; v[4] += e.dpitch; v[5] += e.dyaw;
        const std::string origin = "<origin xyz=\"" + fmt(v[0]) + " " + fmt(v[1]) + " " + fmt(v[2]) +
                                   "\" rpy=\"" + fmt(v[3]) + " " + fmt(v[4]) + " " + fmt(v[5]) + "\" />";
        const std::size_t body_pos = static_cast<std::size_t>(jm.position(1));
        const std::size_t origin_pos = body_pos + static_cast<std::size_t>(om.position(0));
        text.replace(origin_pos, static_cast<std::size_t>(om.length(0)), origin);
        ++count;
    }
    *out_text = std::move(text);
    if (edits != nullptr) *edits = count;
    return "";
}

std::string absolutizeMeshPaths(const std::string& urdf_text, const std::string& urdf_dir) {
    const std::regex re("filename=\"([^\"]*)\"");
    std::string out;
    std::size_t last = 0;
    for (std::sregex_iterator it(urdf_text.begin(), urdf_text.end(), re), end; it != end; ++it) {
        const std::smatch& m = *it;
        const std::string rel = m[1].str();
        std::string abs = rel;
        if (!rel.empty() && rel.rfind("package://", 0) != 0 && rel.rfind("file://", 0) != 0 && rel[0] != '/') {
            abs = (std::filesystem::path(urdf_dir) / rel).lexically_normal().string();
        }
        out.append(urdf_text, last, static_cast<std::size_t>(m.position(0)) - last);
        out += "filename=\"" + abs + "\"";
        last = static_cast<std::size_t>(m.position(0) + m.length(0));
    }
    out.append(urdf_text, last, std::string::npos);
    return out;
}

std::uint64_t fnv1a64(const std::string& s) {
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

std::string formatDhTable(const DhTable& t) {
    std::ostringstream os;
    for (std::size_t i = 0; i < t.size(); ++i) {
        os << "J" << (i + 1) << "{theta_off " << fmt(t[i].theta_offset_deg) << " d " << fmt(t[i].d_mm)
           << " a " << fmt(t[i].a_mm) << " alpha " << fmt(t[i].alpha_deg) << "}";
        if (i + 1 < t.size()) os << " ";
    }
    return os.str();
}

std::string formatDhDelta(const DhTable& nominal, const DhTable& calibrated) {
    std::ostringstream os;
    bool any = false;
    for (std::size_t i = 0; i < nominal.size(); ++i) {
        for (DhField f : {DhField::ThetaOffset, DhField::D, DhField::A, DhField::Alpha}) {
            const double dv = cellValue(calibrated[i], f) - cellValue(nominal[i], f);
            if (std::fabs(dv) <= 1e-9) continue;
            if (any) os << ", ";
            os << "J" << (i + 1) << "." << fieldName(f) << " " << (dv >= 0 ? "+" : "") << fmt(dv)
               << (isAngle(f) ? " deg" : " mm");
            any = true;
        }
    }
    return any ? os.str() : "none (box table == nominal)";
}

}  // namespace rb_servo
