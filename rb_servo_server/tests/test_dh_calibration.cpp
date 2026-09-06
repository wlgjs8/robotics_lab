// THE BOX DH CALIBRATION, offline half (2026-09-05): the RB5-850E table and slot
// layout, adoption rules, and the URDF rewrite every consumer is built from.
#include "rb_servo/kinematics/dh_calibration.hpp"

#include <cmath>
#include <fstream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

#define RB_CHECK(cond)                                                       \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__  \
                      << ": " #cond << "\n";                                 \
            return false;                                                    \
        }                                                                    \
    } while (0)

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

const std::string kUrdf =
    "<robot name=\"t\">\n"
    "  <joint name=\"PREFIXbase_joint\" type=\"revolute\">\n"
    "    <parent link=\"PREFIXlink0\" /><child link=\"PREFIXlink1\" />\n"
    "    <origin xyz=\"0.0 0.0 0.1692\" rpy=\"0.0 0.0 0.0\" />\n"
    "    <axis xyz=\"0.0 0.0 1.0\" />\n"
    "  </joint>\n"
    "  <joint name=\"PREFIXshoulder_joint\" type=\"revolute\">\n"
    "    <origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\" />\n"
    "  </joint>\n"
    "  <joint name=\"PREFIXelbow_joint\" type=\"revolute\">\n"
    "    <origin xyz=\"0.0 0.0 0.425\" rpy=\"0.0 0.0 0.0\" />\n"
    "  </joint>\n"
    "  <joint name=\"PREFIXwrist1_joint\" type=\"revolute\">\n"
    "    <origin xyz=\"0.0 0.0 0.392\" rpy=\"0.0 0.0 0.0\" />\n"
    "  </joint>\n"
    "  <joint name=\"PREFIXwrist2_joint\" type=\"revolute\">\n"
    "    <origin xyz=\"0.0 -0.1107 0.1107\" rpy=\"0.0 0.0 0.0\" />\n"
    "  </joint>\n"
    "  <link name=\"PREFIXlink1\"><collision><geometry>\n"
    "    <mesh filename=\"../meshes/robots/rb5_850e/collision/link1_hull.stl\" /></geometry></collision></link>\n"
    "</robot>\n";

std::string withPrefix(const std::string& prefix) {
    return std::regex_replace(kUrdf, std::regex("PREFIX"), prefix);
}

// The six origin numbers of a named joint.
bool origin(const std::string& urdf, const std::string& joint, double out[6]) {
    const std::regex re("<joint\\s+name=\"" + joint + "\"[^>]*>[\\s\\S]*?<origin\\s+xyz=\"([^\"]*)\"\\s+rpy=\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(urdf, m, re)) return false;
    std::istringstream a(m[1].str()), b(m[2].str());
    return static_cast<bool>(a >> out[0] >> out[1] >> out[2]) && static_cast<bool>(b >> out[3] >> out[4] >> out[5]);
}

std::vector<double> nominalRaw() {
    // J2.alpha J3.alpha J1.d J2.a J3.a J4.d 0 0
    return {0.0, 0.0, 169.2, 425.0, 392.0, -110.7, 0.0, 0.0};
}

bool testNominalTableAndLayout() {
    const rb_servo::DhTable t = rb_servo::rb5850eNominalDh();
    RB_CHECK(near(t[0].d_mm, 169.2) && near(t[0].alpha_deg, -90.0));
    RB_CHECK(near(t[1].a_mm, 425.0) && near(t[1].theta_offset_deg, -90.0) && near(t[1].d_mm, -148.4));
    RB_CHECK(near(t[2].a_mm, 392.0));
    RB_CHECK(near(t[3].d_mm, -110.7) && near(t[3].alpha_deg, 90.0));
    RB_CHECK(near(t[5].d_mm, -96.7));
    const rb_servo::DhSlotLayout& L = rb_servo::rb5850eLinkParameterLayout();
    RB_CHECK(L.count == 8 && L.slots.size() == 6);
    RB_CHECK(L.slots[2].joint == 1 && L.slots[2].field == rb_servo::DhField::D);
    RB_CHECK(L.slots[0].joint == 2 && L.slots[0].field == rb_servo::DhField::Alpha);
    return true;
}

bool testParsePayload() {
    const auto v = rb_servo::parseLinkParameterPayload("link_parameter = [0.01, -0.02, 165.5, 425.1, 391.9, -110.6, 0, 0]");
    RB_CHECK(v.size() == 8 && near(v[2], 165.5) && near(v[5], -110.6));
    // THE REAL BOX (2026-09-06, left arm): the key is bracketed too.
    const auto box = rb_servo::parseLinkParameterPayload(
        "info[link_parameter][-0.0582, -0.0449, 163.9439, 425.5847, 392.3901, -111.2781, 0.0000, 0.0000]");
    RB_CHECK(box.size() == 8 && near(box[0], -0.0582) && near(box[2], 163.9439) && near(box[5], -111.2781) && near(box[7], 0.0));
    const auto w = rb_servo::parseLinkParameterPayload("link_parameter 1 2 3");
    RB_CHECK(w.size() == 3 && near(w[2], 3.0));
    RB_CHECK(rb_servo::parseLinkParameterPayload("link_parameter = []").empty());
    RB_CHECK(rb_servo::parseLinkParameterPayload("").empty());
    return true;
}

bool testAdoptionAcceptsAndRefuses() {
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    const auto& L = rb_servo::rb5850eLinkParameterLayout();
    rb_servo::DhAdoption a;
    // The measured box answer of 2026-09-04: J1.d 165.5 vs 169.2 nominal.
    std::vector<double> raw = nominalRaw();
    raw[2] = 165.5;
    RB_CHECK(rb_servo::adoptLinkParameter(raw, nom, L, 10.0, 2.0, &a).empty());
    RB_CHECK(a.changed_cells == 1 && near(a.table[0].d_mm, 165.5) && near(a.max_abs_delta_mm, 3.7, 1e-9));
    RB_CHECK(near(a.table[1].a_mm, 425.0));                       // untouched cells stay nominal
    RB_CHECK(near(a.table[1].d_mm, -148.4));                      // cells outside the layout untouched
    // The real left box of 2026-09-06: six calibrated cells, all inside the limits.
    RB_CHECK(rb_servo::adoptLinkParameter({-0.0582, -0.0449, 163.9439, 425.5847, 392.3901, -111.2781, 0.0, 0.0},
                                          nom, L, 10.0, 2.0, &a).empty());
    RB_CHECK(a.changed_cells == 6 && near(a.table[0].d_mm, 163.9439) && near(a.table[1].alpha_deg, -0.0582) &&
             near(a.table[3].d_mm, -111.2781) && near(a.max_abs_delta_mm, 5.2561, 1e-6));
    // skip_cells keeps a reported cell nominal, logs it, still validates the rest.
    RB_CHECK(rb_servo::adoptLinkParameter({-0.0582, -0.0449, 163.9439, 425.5847, 392.3901, -111.2781, 0.0, 0.0},
                                          nom, L, 10.0, 2.0, &a, {"J1.d"}).empty());
    RB_CHECK(a.changed_cells == 5 && near(a.table[0].d_mm, 169.2) && near(a.table[1].a_mm, 425.5847));
    RB_CHECK(a.skipped.size() == 1 && a.skipped[0].find("J1.d") == 0);
    RB_CHECK(!rb_servo::adoptLinkParameter(nominalRaw(), nom, L, 10.0, 2.0, &a, {"J9.d"}).empty());   // bad name
    int j = 0;
    rb_servo::DhField f = rb_servo::DhField::D;
    RB_CHECK(rb_servo::parseDhCellName("J3.alpha", &j, &f) && j == 3 && f == rb_servo::DhField::Alpha);
    RB_CHECK(!rb_servo::parseDhCellName("J3.beta", &j, &f));
    // Exactly nominal: adopted, nothing changed.
    RB_CHECK(rb_servo::adoptLinkParameter(nominalRaw(), nom, L, 10.0, 2.0, &a).empty() && a.changed_cells == 0);
    // Six values: an unknown layout.
    const std::vector<double> full = nominalRaw();
    std::vector<double> six(full.begin(), full.begin() + 6);
    RB_CHECK(!rb_servo::adoptLinkParameter(six, nom, L, 10.0, 2.0, &a).empty());
    // A nonzero unmapped slot: an unknown calibration.
    raw = nominalRaw();
    raw[6] = 1.5;
    RB_CHECK(!rb_servo::adoptLinkParameter(raw, nom, L, 10.0, 2.0, &a).empty());
    // Beyond the limits: garbage.
    raw = nominalRaw();
    raw[3] = 440.0;
    RB_CHECK(!rb_servo::adoptLinkParameter(raw, nom, L, 10.0, 2.0, &a).empty());
    raw = nominalRaw();
    raw[0] = 3.0;
    RB_CHECK(!rb_servo::adoptLinkParameter(raw, nom, L, 10.0, 2.0, &a).empty());
    // Refused tables leave the previous adoption alone (a is untouched by refusals).
    RB_CHECK(a.changed_cells == 0);
    return true;
}

bool testUrdfPatchNominalIsIdentity() {
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    std::string out;
    int edits = -1;
    const std::string u = withPrefix("");
    RB_CHECK(rb_servo::patchUrdfWithDh(u, "", nom, nom, &out, &edits).empty());
    RB_CHECK(edits == 0 && out == u);
    return true;
}

bool testUrdfPatchCarriesTheSixCells() {
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    rb_servo::DhTable cal = nom;
    cal[0].d_mm = 165.5;        // J1.d  -> base_joint z
    cal[1].a_mm = 425.4;        // J2.a  -> elbow_joint z
    cal[1].alpha_deg = 0.05;    // J2.alpha -> elbow_joint yaw
    cal[2].a_mm = 391.8;        // J3.a  -> wrist1_joint z
    cal[2].alpha_deg = -0.02;   // J3.alpha -> wrist1_joint yaw
    cal[3].d_mm = -110.5;       // J4.d  -> wrist2_joint y
    for (const std::string prefix : {std::string(""), std::string("dual_rb5_850e_left_")}) {
        std::string out;
        int edits = 0;
        const std::string u = withPrefix(prefix);
        const std::string err = rb_servo::patchUrdfWithDh(u, prefix, nom, cal, &out, &edits);
        RB_CHECK(err.empty());
        RB_CHECK(edits == 4);
        double o[6];
        RB_CHECK(origin(out, prefix + "base_joint", o) && near(o[2], 0.1655, 1e-9) && near(o[0], 0.0) && near(o[5], 0.0));
        RB_CHECK(origin(out, prefix + "elbow_joint", o) && near(o[2], 0.4254, 1e-9) && near(o[5], 0.05 * M_PI / 180.0, 1e-9));
        RB_CHECK(origin(out, prefix + "wrist1_joint", o) && near(o[2], 0.3918, 1e-9) && near(o[5], -0.02 * M_PI / 180.0, 1e-9));
        RB_CHECK(origin(out, prefix + "wrist2_joint", o) && near(o[1], -0.1105, 1e-9) && near(o[2], 0.1107, 1e-9));
        RB_CHECK(origin(out, prefix + "shoulder_joint", o) && near(o[2], 0.0));   // untouched
        // Everything outside the four origins is byte-identical.
        RB_CHECK(out.find("<axis xyz=\"0.0 0.0 1.0\" />") != std::string::npos);
        RB_CHECK(out.find("link1_hull.stl") != std::string::npos);
    }
    // A dual URDF: only the named prefix's chain moves.
    {
        const std::string dual = withPrefix("dual_rb5_850e_left_") + withPrefix("dual_rb5_850e_right_");
        std::string out;
        int edits = 0;
        RB_CHECK(rb_servo::patchUrdfWithDh(dual, "dual_rb5_850e_left_", nom, cal, &out, &edits).empty() && edits == 4);
        double o[6];
        RB_CHECK(origin(out, "dual_rb5_850e_left_base_joint", o) && near(o[2], 0.1655, 1e-9));
        RB_CHECK(origin(out, "dual_rb5_850e_right_base_joint", o) && near(o[2], 0.1692, 1e-9));
    }
    return true;
}

bool testUrdfPatchRefusals() {
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    rb_servo::DhTable cal = nom;
    cal[0].d_mm = 165.5;
    std::string out;
    int edits = 0;
    // A joint the rewrite needs is missing.
    RB_CHECK(!rb_servo::patchUrdfWithDh(withPrefix("other_"), "", nom, cal, &out, &edits).empty());
    // A cell the rewrite does not carry differs: refuse instead of dropping it.
    rb_servo::DhTable bad = nom;
    bad[4].d_mm = 111.0;   // J5.d
    RB_CHECK(!rb_servo::patchUrdfWithDh(withPrefix(""), "", nom, bad, &out, &edits).empty());
    return true;
}

bool testAbsolutizeMeshPathsAndHash() {
    const std::string in = "<mesh filename=\"../meshes/a.stl\" /><mesh filename=\"package://x/b.stl\" /><mesh filename=\"/abs/c.stl\" />";
    const std::string out = rb_servo::absolutizeMeshPaths(in, "/repo/descriptions/urdf");
    RB_CHECK(out.find("filename=\"/repo/descriptions/meshes/a.stl\"") != std::string::npos);
    RB_CHECK(out.find("filename=\"package://x/b.stl\"") != std::string::npos);
    RB_CHECK(out.find("filename=\"/abs/c.stl\"") != std::string::npos);
    RB_CHECK(rb_servo::fnv1a64("a") != rb_servo::fnv1a64("b"));
    RB_CHECK(rb_servo::fnv1a64("same") == rb_servo::fnv1a64("same"));
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    rb_servo::DhTable cal = nom;
    cal[0].d_mm = 165.5;
    RB_CHECK(rb_servo::formatDhDelta(nom, cal).find("J1.d -3.7 mm") != std::string::npos);
    RB_CHECK(rb_servo::formatDhDelta(nom, nom).find("none") != std::string::npos);
    return true;
}

// THE SHIPPED FILES: the rewrite must find the four joints in rb5_850e.urdf and in
// both chains of the unified dual URDF, and Pinocchio must still build the same
// model (same nq) from the patched text.
bool testShippedUrdfsPatchAndStillBuild() {
#ifndef RB_SERVO_SOURCE_DIR
    std::cout << "  (RB_SERVO_SOURCE_DIR not set; skipping the shipped-URDF case)\n";
    return true;
#else
    const std::string root = RB_SERVO_SOURCE_DIR;
    const rb_servo::DhTable nom = rb_servo::rb5850eNominalDh();
    rb_servo::DhTable cal = nom;
    cal[0].d_mm = 165.5;
    cal[1].alpha_deg = 0.05;
    cal[3].d_mm = -110.5;
    struct Case { const char* file; std::vector<std::string> prefixes; };
    const Case cases[] = {
        {"/descriptions/urdf/rb5_850e.urdf", {""}},
        {"/descriptions/urdf/dual_rb5_850e_ver3.urdf", {"dual_rb5_850e_left_", "dual_rb5_850e_right_"}},
        {"/descriptions/urdf/rb5_850e_pika_articulated.urdf", {""}},
    };
    for (const Case& c : cases) {
        std::ifstream in(root + c.file);
        if (!in) {
            std::cout << "  (missing " << c.file << "; skipping)\n";
            continue;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        std::string text = ss.str();
        pinocchio::Model before;
        pinocchio::urdf::buildModelFromXML(text, before);
        for (const std::string& prefix : c.prefixes) {
            std::string out;
            int edits = 0;
            const std::string err = rb_servo::patchUrdfWithDh(text, prefix, nom, cal, &out, &edits);
            if (!err.empty()) std::cerr << "patch error: " << err << "\n";
            RB_CHECK(err.empty());
            RB_CHECK(edits == 3);
            text = out;
            double o[6];
            RB_CHECK(origin(text, prefix + "base_joint", o) && near(o[2], 0.1655, 1e-9));
            RB_CHECK(origin(text, prefix + "elbow_joint", o) && near(o[5], 0.05 * M_PI / 180.0, 1e-9));
            RB_CHECK(origin(text, prefix + "wrist2_joint", o) && near(o[1], -0.1105, 1e-9));
        }
        const std::string abs = rb_servo::absolutizeMeshPaths(text, root + "/descriptions/urdf");
        RB_CHECK(abs.find("filename=\"../") == std::string::npos);
        pinocchio::Model after;
        pinocchio::urdf::buildModelFromXML(abs, after);
        RB_CHECK(after.nq == before.nq && after.njoints == before.njoints);
        // Nominal in -> byte-identical out, on the shipped file too.
        std::string same;
        int zero = -1;
        RB_CHECK(rb_servo::patchUrdfWithDh(ss.str(), c.prefixes[0], nom, nom, &same, &zero).empty());
        RB_CHECK(zero == 0 && same == ss.str());
    }
    return true;
#endif
}

}  // namespace

int main() {
    if (!testNominalTableAndLayout()) return 1;
    if (!testParsePayload()) return 1;
    if (!testAdoptionAcceptsAndRefuses()) return 1;
    if (!testUrdfPatchNominalIsIdentity()) return 1;
    if (!testUrdfPatchCarriesTheSixCells()) return 1;
    if (!testUrdfPatchRefusals()) return 1;
    if (!testAbsolutizeMeshPathsAndHash()) return 1;
    if (!testShippedUrdfsPatchAndStillBuild()) return 1;
    std::cout << "dh_calibration tests passed\n";
    return 0;
}
