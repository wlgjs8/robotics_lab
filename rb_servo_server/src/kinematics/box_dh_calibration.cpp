#include "rb_servo/kinematics/box_dh_calibration.hpp"

#include "rb_servo/robot/rbpodo_backend.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace rb_servo {
namespace {

std::string readFile(const std::string& path, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "cannot read " + path;
        return "";
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool writeFile(const std::string& path, const std::string& text, std::string* error) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error) *error = "cannot write " + path;
        return false;
    }
    out << text;
    return static_cast<bool>(out);
}

std::string resolveExisting(const std::string& path) {
    namespace fs = std::filesystem;
    if (fs::exists(path)) return fs::absolute(path).lexically_normal().string();
    return "";
}

std::string jsonArray(const std::vector<double>& v) {
    std::ostringstream os;
    os << "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) os << ", ";
        os << v[i];
    }
    os << "]";
    return os.str();
}

std::string jsonTable(const DhTable& t) {
    std::ostringstream os;
    os << "[";
    for (std::size_t i = 0; i < t.size(); ++i) {
        if (i) os << ", ";
        os << "{\"joint\": " << (i + 1) << ", \"theta_offset_deg\": " << t[i].theta_offset_deg
           << ", \"d_mm\": " << t[i].d_mm << ", \"a_mm\": " << t[i].a_mm << ", \"alpha_deg\": " << t[i].alpha_deg << "}";
    }
    os << "]";
    return os.str();
}

std::string nowIso() {
    const std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&t));
    return buf;
}

}  // namespace

BoxDhCalibrationResult runBoxDhCalibration(DualArmConfig& config) {
    namespace fs = std::filesystem;
    BoxDhCalibrationResult r;
    const KinematicsCalibrationConfig& cal = config.kinematics.calibration;
    if (cal.source != "box") {
        r.ok = true;
        r.applied = false;
        std::cerr << "[INFO] box DH calibration: kinematics.calibration.source=" << cal.source
                  << " -> URDF nominal kinematics (no box query)\n";
        return r;
    }
    if (!config.kinematics.enable || config.kinematics.provider != "pinocchio") {
        r.error = "kinematics.calibration.source=box needs kinematics.enable=true and provider=pinocchio";
        return r;
    }
    const DhTable nominal = rb5850eNominalDh();
    const DhSlotLayout& layout = rb5850eLinkParameterLayout();
    const BackendConfig* backends[2] = {&config.left_robot, &config.right_robot};
    const char* names[2] = {"left", "right"};
    std::array<DhAdoption, 2> adopted{};
    for (int i = 0; i < 2; ++i) {
        const BackendConfig& b = *backends[i];
        if (b.backend_type != BackendType::Rbpodo) {
            r.error = std::string(names[i]) + " arm backend is not rbpodo; the box DH cannot be read";
            return r;
        }
        std::vector<double> values;
        std::string raw, err;
        std::cerr << "[INFO] box DH calibration: probing " << names[i] << " box " << b.ip
                  << " for get_link_parameter() (timeout " << cal.probe_timeout_sec << " s)\n";
        if (!RbpodoBackend::probeLinkParameter(b, cal.probe_timeout_sec, &values, &raw, &err)) {
            r.error = std::string(names[i]) + " box (" + b.ip + "): " + err;
            return r;
        }
        std::cerr << "[INFO] box DH calibration: " << names[i] << " link_parameter (" << values.size()
                  << " values): " << jsonArray(values) << "\n";
        const std::string adopt_err = adoptLinkParameter(values, nominal, layout, cal.max_abs_delta_mm,
                                                         cal.max_abs_delta_deg, &adopted[static_cast<std::size_t>(i)]);
        if (!adopt_err.empty()) {
            r.error = std::string(names[i]) + " box: " + adopt_err + " (raw reply: \"" + raw + "\")";
            return r;
        }
        r.raw[static_cast<std::size_t>(i)] = values;
        r.table[static_cast<std::size_t>(i)] = adopted[static_cast<std::size_t>(i)].table;
        std::cerr << "[INFO] box DH calibration: " << names[i] << " adopted "
                  << adopted[static_cast<std::size_t>(i)].changed_cells << " cell(s) off nominal: "
                  << formatDhDelta(nominal, adopted[static_cast<std::size_t>(i)].table) << "\n";
    }

    // ---- write the runtime URDFs --------------------------------------------
    std::string err;
    const fs::path out_dir = fs::absolute(cal.output_dir).lexically_normal();
    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec) {
        r.error = "cannot create " + out_dir.string() + ": " + ec.message();
        return r;
    }
    const std::string single_src = resolveExisting(config.kinematics.urdf);
    if (single_src.empty()) {
        r.error = "kinematics.urdf not found: " + config.kinematics.urdf;
        return r;
    }
    const std::string single_text = readFile(single_src, &err);
    if (single_text.empty()) {
        r.error = err;
        return r;
    }
    const std::string single_dir = fs::path(single_src).parent_path().string();
    std::array<std::string, 2> single_out{};
    for (int i = 0; i < 2; ++i) {
        std::string patched;
        int edits = 0;
        const std::string perr = patchUrdfWithDh(single_text, "", nominal, r.table[static_cast<std::size_t>(i)],
                                                 &patched, &edits);
        if (!perr.empty()) {
            r.error = std::string(names[i]) + " single-arm URDF: " + perr;
            return r;
        }
        patched = absolutizeMeshPaths(patched, single_dir);
        const fs::path p = out_dir / (std::string("rb5_850e_") + names[i] + "_calibrated.urdf");
        if (!writeFile(p.string(), patched, &err)) {
            r.error = err;
            return r;
        }
        single_out[static_cast<std::size_t>(i)] = p.string();
        std::cerr << "[INFO] box DH calibration: wrote " << p.string() << " (" << edits
                  << " joint origin(s) rewritten, fnv1a64 " << std::hex << fnv1a64(patched) << std::dec << ")\n";
    }
    r.urdf_left = single_out[0];
    r.urdf_right = single_out[1];

    // ---- the GUI's arm model: the same four joints, drawn calibrated ----------
    if (!cal.gui_arm_urdf.empty()) {
        const std::string gui_src = resolveExisting(cal.gui_arm_urdf);
        if (gui_src.empty()) {
            r.error = "kinematics.calibration.gui_arm_urdf not found: " + cal.gui_arm_urdf;
            return r;
        }
        const std::string gui_text = readFile(gui_src, &err);
        if (gui_text.empty()) {
            r.error = err;
            return r;
        }
        const std::string gui_dir = fs::path(gui_src).parent_path().string();
        const std::string stem = fs::path(gui_src).stem().string();
        for (int i = 0; i < 2; ++i) {
            std::string patched;
            int edits = 0;
            const std::string perr = patchUrdfWithDh(gui_text, "", nominal, r.table[static_cast<std::size_t>(i)],
                                                     &patched, &edits);
            if (!perr.empty()) {
                r.error = std::string(names[i]) + " GUI arm URDF: " + perr;
                return r;
            }
            patched = absolutizeMeshPaths(patched, gui_dir);
            const fs::path p = out_dir / (stem + "_" + names[i] + "_calibrated.urdf");
            if (!writeFile(p.string(), patched, &err)) {
                r.error = err;
                return r;
            }
            (i == 0 ? config.kinematics.gui_urdf_left : config.kinematics.gui_urdf_right) = p.string();
            std::cerr << "[INFO] box DH calibration: wrote " << p.string() << " (GUI arm model, " << edits
                      << " joint origin(s) rewritten)\n";
        }
    } else {
        std::cerr << "[INFO] box DH calibration: kinematics.calibration.gui_arm_urdf unset; the GUI keeps"
                     " its nominal arm model (the collision overlay is calibrated)\n";
    }

    if (config.safety.self_collision.enable) {
        auto& mesh = config.safety.self_collision.mesh;
        const std::string dual_src = resolveExisting(mesh.unified_urdf);
        if (dual_src.empty()) {
            r.error = "safety.self_collision.mesh.unified_urdf not found: " + mesh.unified_urdf;
            return r;
        }
        std::string dual_text = readFile(dual_src, &err);
        if (dual_text.empty()) {
            r.error = err;
            return r;
        }
        int edits_total = 0;
        const std::string prefixes[2] = {mesh.left_prefix, mesh.right_prefix};
        for (int i = 0; i < 2; ++i) {
            std::string patched;
            int edits = 0;
            const std::string perr = patchUrdfWithDh(dual_text, prefixes[i], nominal,
                                                     r.table[static_cast<std::size_t>(i)], &patched, &edits);
            if (!perr.empty()) {
                r.error = std::string(names[i]) + " unified URDF: " + perr;
                return r;
            }
            dual_text = std::move(patched);
            edits_total += edits;
        }
        dual_text = absolutizeMeshPaths(dual_text, fs::path(dual_src).parent_path().string());
        const fs::path p = out_dir / "dual_rb5_850e_calibrated.urdf";
        if (!writeFile(p.string(), dual_text, &err)) {
            r.error = err;
            return r;
        }
        r.urdf_dual = p.string();
        std::cerr << "[INFO] box DH calibration: wrote " << p.string() << " (" << edits_total
                  << " joint origin(s) rewritten, fnv1a64 " << std::hex << fnv1a64(dual_text) << std::dec
                  << "); the collision monitor and the GUI manifest use this file\n";
        mesh.unified_urdf = p.string();
    }

    // ---- rewire the config: every consumer is built from the calibrated files ----
    config.kinematics.urdf_left = r.urdf_left;
    config.kinematics.urdf_right = r.urdf_right;
    config.left_robot.require_link_parameter = true;
    config.left_robot.expected_link_parameter = r.raw[0];
    config.right_robot.require_link_parameter = true;
    config.right_robot.expected_link_parameter = r.raw[1];

    // ---- manifest -------------------------------------------------------------
    {
        std::ostringstream js;
        js << "{\n  \"schema\": \"robotics_lab.box_dh_calibration.v1\",\n"
           << "  \"time\": \"" << nowIso() << "\",\n"
           << "  \"layout\": \"rb5_850e get_link_parameter 8 values: J2.alpha J3.alpha J1.d J2.a J3.a J4.d 0 0\",\n"
           << "  \"nominal\": " << jsonTable(nominal) << ",\n"
           << "  \"arms\": {\n";
        for (int i = 0; i < 2; ++i) {
            js << "    \"" << names[i] << "\": {\"ip\": \"" << backends[i]->ip << "\", \"raw\": "
               << jsonArray(r.raw[static_cast<std::size_t>(i)]) << ", \"table\": "
               << jsonTable(r.table[static_cast<std::size_t>(i)]) << ", \"delta\": \""
               << formatDhDelta(nominal, r.table[static_cast<std::size_t>(i)]) << "\", \"urdf\": \""
               << single_out[static_cast<std::size_t>(i)] << "\"}" << (i == 0 ? ",\n" : "\n");
        }
        js << "  },\n  \"unified_urdf\": \"" << r.urdf_dual << "\",\n"
           << "  \"gui_urdf_left\": \"" << config.kinematics.gui_urdf_left << "\",\n"
           << "  \"gui_urdf_right\": \"" << config.kinematics.gui_urdf_right << "\"\n}\n";
        const fs::path mp = out_dir / "dh_calibration.json";
        if (!writeFile(mp.string(), js.str(), &err)) {
            r.error = err;
            return r;
        }
        r.manifest_path = mp.string();
    }
    std::cerr << "[INFO] box DH calibration: APPLIED to IK/FK (per-arm URDF), the collision monitor and the"
                 " GUI manifest; manifest " << r.manifest_path << "\n";
    r.ok = true;
    r.applied = true;
    return r;
}

}  // namespace rb_servo
