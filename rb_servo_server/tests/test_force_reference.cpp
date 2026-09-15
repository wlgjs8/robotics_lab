#include "rb_servo/experimental/force_reference.hpp"
#include <Eigen/Geometry>
#include "rb_servo/experimental/kinematic_acceleration.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rb_servo::experimental;
namespace {
constexpr double dt = 0.002;
double test_mass_kg = 4;
using V = Eigen::Vector3d;
void require(bool value, const std::string& reason) {
    if (!value) throw std::runtime_error(reason);
}
ForceReferenceConfig config() {
    // Development profile. 8 N is an explicit recovery target below the user's
    // 10 N test criterion; 20 N is NOT promoted to the operating configuration.
    return {10, 8, 3, test_mass_kg, 500, 44400, 0.6, 5, 2000};
}
struct Plant {
    ForceReference controller;
    std::deque<V> fifo;
    V measured{V::Zero()}, velocity{V::Zero()}, force{V::Zero()};
    V normal{V::UnitX()};
    double stiffness{4400}, wall_position{0.02}, tool_mass{0.814};
    double ripple_n{0}, time{0};
    double compensation_mass_scale{1}, position_noise_m{0};
    int pose_sample_period_ticks{1}, sensor_tick{0};
    V held_observation{V::Zero()};
    bool wall{true};
    ForceReferenceResult result{};
    KinematicAcceleration motion;
    std::uint64_t sequence{3}, stamp{1000000000};
    double maximum_acceleration{0}, maximum_jerk{0}, peak_force{0};
    Plant(int delay, const ForceReferenceConfig& c = config())
        : controller(c, dt), fifo(delay, V::Zero()) {
        controller.reset({});
        motion.update(1,stamp-4000000,V::Zero(),0.05);
        motion.update(2,stamp-2000000,V::Zero(),0.05);
        motion.update(3,stamp,V::Zero(),0.05);
    }
    void tick(const V& goal, const V& goal_velocity) {
        const V old_velocity = velocity;
        const V previous = measured;
        fifo.push_back(controller.state().position);
        measured = fifo.front(); fifo.pop_front();
        velocity = (measured - previous) / dt;
        const V acceleration = (velocity - old_velocity) / dt;
        force = -stiffness * std::max(0.0, normal.dot(measured) - wall_position) * normal;
        if (!wall) force.setZero();
        const V sensed = force - tool_mass * acceleration +
            ripple_n * std::sin(2 * 3.141592653589793 * 50 * time) * normal;
        const V noisy=measured+position_noise_m*
            V(std::sin(2*3.141592653589793*73*time),
              std::sin(2*3.141592653589793*41*time),
              std::sin(2*3.141592653589793*113*time));
        // A synthetic staggered-coordinate readback, not an identified RB5
        // encoder/FK model. Packets stay fresh while components can be held.
        for(int axis=0;axis<3;++axis) {
            if ((sensor_tick+axis)%pose_sample_period_ticks==0) held_observation[axis]=noisy[axis];
        }
        ++sensor_tick;
        const V observed=held_observation;
        require(motion.update(++sequence,stamp+=2000000,observed,0.05), "acceleration estimate");
        const auto old = controller.state();
        result = controller.step(goal, goal_velocity, observed,
            sensed+compensation_mass_scale*tool_mass* *motion.value());
        require(result.valid, result.reason);
        const auto& c = controller.config();
        const double a = (result.state.velocity-old.velocity).cwiseAbs().maxCoeff()/dt;
        const double j = (result.state.acceleration-old.acceleration).cwiseAbs().maxCoeff()/dt;
        maximum_acceleration = std::max(maximum_acceleration, a);
        maximum_jerk = std::max(maximum_jerk, j);
        peak_force = std::max(peak_force, force.norm());
        require(result.state.velocity.cwiseAbs().maxCoeff() <= c.max_velocity_m_s + 1e-8,
                "velocity bound");
        require(a <= c.max_acceleration_m_s2 + 1e-7, "acceleration discontinuity");
        require(j <= c.max_jerk_m_s3 + 1e-6, "jerk discontinuity");
        time += dt;
    }
};

void testInvalidInput() {
    auto c = config();
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid = c; invalid.mass_kg = bad;
        bool refused = false;
        try { ForceReference value(invalid, dt); } catch (const std::invalid_argument&) { refused = true; }
        require(refused, "invalid dynamics accepted");
    }
    c.recovery_force_n = c.sustained_force_n;
    bool refused = false;
    try { ForceReference value(c, dt); } catch (const std::invalid_argument&) { refused = true; }
    require(refused, "10 N equilibrium is not the current recovery requirement");
    ForceReference controller(config(), dt);
    require(!controller.step(V::Zero(), V::Zero(), V::Zero(), V::Zero()).valid, "unseeded move");
    controller.reset({});
    const V bad = V::Constant(std::numeric_limits<double>::quiet_NaN());
    require(!controller.step(V::Zero(), V::Zero(), V::Zero(), bad).valid, "nonfinite wrench");
    require(controller.state().position.isZero(0), "invalid input mutated state");
    require(!controller.step(V::Zero(),V::Zero(),V::Zero(),V::Constant(1e308)).valid,
            "overflowing wrench accepted");
    controller.clear();
    require(!controller.initialized(), "clear retained an executable state");
}

void testAcquisitionAcceleration() {
    KinematicAcceleration estimate;
    const V acceleration(2,-3,1), velocity(0.1,0.2,-0.1), origin(1,2,3);
    const auto position=[&](double time)->V { return origin+velocity*time+0.5*acceleration*time*time; };
    require(!estimate.update(1,1000000000,position(0),0.05), "unwarmed estimate exposed");
    require(!estimate.update(2,1002000000,position(0.002),0.05), "two points imply acceleration");
    require(estimate.update(3,1005000000,position(0.005),0.05), "nonuniform sample rejected");
    require((*estimate.value()-acceleration).norm()<1e-7, "nonuniform acceleration incorrect");
    require(estimate.update(3,1005000000,position(0.005),0.05), "cached sample lost estimate");
    require(estimate.update(4,1009000000,position(0.009),0.05), "duplicate corrupted history");
    require((*estimate.value()-acceleration).norm()<1e-7, "duplicate entered difference stencil");
    require(!estimate.update(5,1100000000,position(0.1),0.05)&&!estimate.value(),
            "stale gap retained acceleration");
    require(!estimate.update(1,1000000000,position(0),0.05), "new acquisition epoch not reset");
    estimate.update(2,1002000000,position(0.002),0.05);
    require(estimate.update(3,1005000000,position(0.005),0.05), "new epoch failed to warm");
    require(!estimate.update(4,1009000000,V::Constant(std::numeric_limits<double>::quiet_NaN()),0.05),
            "nonfinite pose accepted");
    require(!estimate.value(), "invalid pose retained acceleration");
    require(!estimate.update(1,1000000000,position(0),std::numeric_limits<double>::infinity()),
            "unbounded freshness setting accepted");
}

void testFreeMotion() {
    for (double speed : {0.03, 0.1, 0.3}) {
        Plant plant(9); plant.wall = false; plant.ripple_n = 1;
        double low=1e9, high=-1e9;
        for (int i=0;i<2500;++i) {
            const V velocity(speed, 0, 0);
            plant.tick(velocity*plant.time, velocity);
            if (i>=2000) {
                low=std::min(low, plant.controller.state().velocity.x());
                high=std::max(high, plant.controller.state().velocity.x());
            }
        }
        require(std::abs(plant.controller.state().position.x()-speed*plant.time)<0.002,
                "free-space tracking lost");
        require(high-low<0.001, "free-space self-excited ripple");
        require(plant.controller.state().position.x()>0.1, "hidden displacement fence");
    }
}

void testPredictionContinuityAtUnresolvedForce() {
    for (double noise : {0.0,3.0})
    for (const V& direction : std::vector<V>{V::UnitX(),-V::UnitY(),V(1,2,3).normalized()}) {
        auto parameters=config();parameters.noise_force_n=noise;
        for (double epsilon : {1e-7,1e-5,1e-3}) {
            ForceReference reference(parameters,dt);
            ForceReferenceState seed;
            seed.position=0.01*direction; // one centimeter of in-flight closing motion
            reference.reset(seed);
            auto at_boundary=reference.step(seed.position,V::Zero(),V::Zero(),-noise*direction);
            require(at_boundary.valid&&at_boundary.predicted_force.isZero(0),"unresolved normal predicted force");
            reference.reset(seed);
            auto above=reference.step(seed.position,V::Zero(),V::Zero(),-(noise+epsilon)*direction);
            require(above.valid&&above.predicted_force.norm()<2*epsilon,
                    "noise-boundary crossing enabled a full stiffness prediction");
        }
    }
}

void testContactEnvelope() {
    double worst_peak=0, worst_tail=0;
    for (double stiffness : {1000.,4400.,15000.,44400.})
    for (int delay : {3,9,13})
    for (double speed : {0.03,0.1,0.15}) {
        Plant plant(delay); plant.stiffness=stiffness;
        double low=1e9,high=0;
        for (int i=0;i<5000;++i) {
            const V velocity(speed,0,0);
            plant.tick(velocity*plant.time, velocity);
            if (i>=4500) { low=std::min(low,plant.force.norm());high=std::max(high,plant.force.norm()); }
        }
        std::cout << "contact k=" << stiffness << " delay=" << delay*dt
                  << " v=" << speed << " tail=" << low << ".." << high
                  << " peak=" << plant.peak_force << '\n';
        require(high<config().sustained_force_n, "sustained force exceeds test criterion");
        require(high-low<0.2, "persistent contact oscillation");
        worst_peak=std::max(worst_peak,plant.peak_force);worst_tail=std::max(worst_tail,high-low);
    }
    // Peak is reported separately: these tests do not certify an impact ceiling.
    std::cout << "envelope peak=" << worst_peak << " tail_pp=" << worst_tail << '\n';
}

void testDirectionsAndTangentialMotion() {
    for (const V& normal : std::vector<V>{V::UnitX(),-V::UnitY(),V::UnitZ(),V(1,2,3).normalized()}) {
        Plant plant(9);plant.normal=normal;
        const V tangent=normal.unitOrthogonal();
        const V velocity=0.1*normal+0.01*tangent;
        for(int i=0;i<4000;++i)plant.tick(velocity*plant.time,velocity);
        std::cout << "direction=" << normal.transpose() << " force=" << plant.force.norm()
                  << " velocity=" << plant.controller.state().velocity.transpose() << '\n';
        require(plant.force.norm()<10, "direction-specific force failure");
        require(std::abs(tangent.dot(plant.controller.state().velocity)-0.01)<0.001,
                "force constraint removed tangential motion");
    }
}

void testHoldAndRelease() {
    std::vector<double> release_speeds;
    for (int holding_ticks : {1500,4500}) {
        Plant plant(9);
        const V goal(0.1,0,0);
        for(int i=0;i<holding_ticks;++i)plant.tick(goal,V::Zero());
        require(plant.force.norm()<10, "held goal sustains excessive force");
        plant.wall=false;
        double maximum=0;
        for(int i=0;i<1500;++i) {
            plant.tick(goal,V::Zero());
            maximum=std::max(maximum,plant.controller.state().velocity.norm());
        }
        require((plant.controller.state().position-goal).norm()<0.001, "release failed to reach valid goal");
        release_speeds.push_back(maximum);
    }
    require(std::abs(release_speeds[0]-release_speeds[1])<0.005,
            "holding duration accumulated release motion");
}

// A reproducible audit, deliberately separate from the ideal-model regression.
// A successful process exit means the audit was written, NOT that the candidate
// is fit for hardware. Every case reports its own criteria and physical peak.
void writeRobustnessAudit(const std::string& path, bool staggered=false) {
    std::ofstream csv(path);
    require(csv.good(), "cannot open robustness audit output");
    csv << std::setprecision(12)
        << "wall,stiffness_n_m,delay_sec,speed_m_s,compensation_mass_scale,position_noise_m,pose_sample_period_ticks,"
           "valid,peak_force_n,tail_force_min_n,tail_force_max_n,tail_force_pp_n,"
           "above_10_duration_sec,longest_above_10_sec,tail_velocity_pp_m_s,"
           "free_tracking_error_m,sustained_pass,ripple_pass,reason\n";
    int cases=0,accepted=0;
    for (const bool wall : {false,true})
    for (const double stiffness : {1000.,4400.,15000.,44400.})
    for (const int delay : {3,9,13})
    for (const double speed : {0.03,0.15})
    for (const double mass_scale : {0.8,1.0,1.2})
    for (const double noise : {0.,1e-6,5e-6}) {
        if (!wall && stiffness!=4400.) continue;
        if (staggered && (mass_scale!=1.0 || noise!=0.0)) continue;
        Plant plant(delay);
        plant.wall=wall;plant.stiffness=stiffness;plant.ripple_n=1;
        plant.compensation_mass_scale=mass_scale;plant.position_noise_m=noise;
        plant.pose_sample_period_ticks=staggered?2:1;
        // An oblique contact exercises all Cartesian axes and normal estimation.
        plant.normal=V(1,2,3).normalized();
        const V velocity=speed*plant.normal;
        double force_low=1e100,force_high=0,velocity_low=1e100,velocity_high=-1e100;
        double above=0,longest=0,streak=0;
        bool valid=true;
        std::string reason="completed";
        try {
            for(int i=0;i<5000;++i) {
                plant.tick(velocity*plant.time,velocity);
                if (plant.force.norm()>=10) { above+=dt;streak+=dt;longest=std::max(longest,streak); }
                else streak=0;
                if (i>=4500) {
                    force_low=std::min(force_low,plant.force.norm());
                    force_high=std::max(force_high,plant.force.norm());
                    const double v=plant.normal.dot(plant.controller.state().velocity);
                    velocity_low=std::min(velocity_low,v);velocity_high=std::max(velocity_high,v);
                }
            }
        } catch(const std::exception& error) { valid=false;reason=error.what(); }
        const double error=(plant.controller.state().position-velocity*plant.time).norm();
        const bool sustained=valid && force_high<10;
        // Sensor ripple is reported, not hidden by judging only the final tick.
        const bool ripple=valid && (wall ? force_high-force_low<0.2 :
            velocity_high-velocity_low<0.001 && error<0.002);
        ++cases;if (sustained&&ripple)++accepted;
        csv << wall << ',' << stiffness << ',' << delay*dt << ',' << speed << ',' << mass_scale << ','
            << noise << ',' << plant.pose_sample_period_ticks << ',' << valid << ',' << plant.peak_force << ',' << force_low << ',' << force_high << ','
            << force_high-force_low << ',' << above << ',' << longest << ',' << velocity_high-velocity_low << ','
            << (wall ? 0 : error) << ',' << sustained << ',' << ripple << ',' << reason << '\n';
    }
    require(csv.good(), "failed writing robustness audit");
    std::cout << "robustness audit: " << accepted << '/' << cases
              << " meet both criteria; no physical impact ceiling certified\n";
}
}

int main(int argc, char** argv) {
    try {
        if (argc==3 && std::string(argv[1])=="--audit-csv") {
            writeRobustnessAudit(argv[2]);
            return 0;
        }
        if (argc==3 && std::string(argv[1])=="--staggered-audit-csv") {
            writeRobustnessAudit(argv[2],true);
            return 0;
        }
        if(argc==2)test_mass_kg=std::stod(argv[1]);
        testInvalidInput();testAcquisitionAcceleration();testPredictionContinuityAtUnresolvedForce();
        testFreeMotion();testContactEnvelope();
        testDirectionsAndTangentialMotion();testHoldAndRelease();
        std::cout << "force reference: all checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "force reference: " << error.what() << '\n';
        return 1;
    }
}
