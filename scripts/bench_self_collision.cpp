// In-loop timing: existing Eigen CAPSULE path vs URDF MESH(+gripper) coal path.
// Mirrors what would run inside the 500 Hz (2 ms) servo_j safety gate.
//
// build/run: see scripts/bench_self_collision.sh
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/BVH/BVH_model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

#include "rb_servo/math/capsule_distance.hpp"

using rb_servo::math::Vector3;
namespace pin = pinocchio;

static const std::string WS = "/home/plaif/workspace";
static const std::string UNIFIED =
    WS + "/robotics_lab/rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf";
static const std::string PIKA =
    WS + "/robotics_lab/rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool/pika_gripper.STL";

struct Cap { std::string frame; Vector3 p0, p1; double r; };

// config.hpp defaultRb3ArmCapsules() (16); index 0 = link0
static std::vector<Cap> armCaps() {
  return {
    {"link0",{0.0696,-0.0007,0.0256},{-0.0623,0.0023,0.0359},0.0566},
    {"link1",{-0.0052,-0.0212,-0.0681},{-0.0010,0.0093,0.0579},0.0647},
    {"link2",{-0.0003,-0.1552,-0.0497},{0.0011,-0.0772,0.0790},0.0804},
    {"link2",{0.0003,-0.1174,0.0812},{-0.0015,-0.1194,0.2131},0.0376},
    {"link2",{-0.0028,-0.0744,0.2178},{0.0002,-0.1501,0.3285},0.0706},
    {"link3",{-0.0001,0.0175,-0.0464},{0.0009,-0.0355,0.0669},0.0624},
    {"link4",{0.0014,-0.1107,0.2140},{-0.0026,-0.0893,0.3132},0.0405},
    {"link4",{0.0407,0.0077,0.0764},{-0.0407,0.0076,0.0765},0.0490},
    {"link4",{0.0003,-0.1457,0.3534},{0.0005,-0.0296,0.3459},0.0525},
    {"link4",{0.0003,-0.1362,0.2054},{-0.0001,0.0451,0.1032},0.0513},
    {"link5",{0.0034,-0.0138,0.0650},{-0.0017,0.0081,-0.0486},0.0399},
    {"link6",{0.0018,0.0347,0.0784},{-0.0175,-0.0385,0.0756},0.0307},
    {"attachment_site",{0,0,0},{0,0,0.1000},0.0350},
    {"attachment_site",{0,0.0607,0.0810},{0,0.0940,0.0810},0.0280},
    {"attachment_site",{-0.1039,0,0.1305},{0.1039,0,0.1305},0.0317},
    {"attachment_site",{-0.0594,0,0.2286},{0.0594,0,0.2286},0.0220},
  };
}
static std::vector<Cap> standCaps() {
  return {
    {"stand",{-0.15,0.06,0.005},{0.15,0.06,0.005},0.0602},
    {"stand",{-0.15,-0.06,0.005},{0.15,-0.06,0.005},0.0602},
    {"stand",{0,0,0},{0,0,0.43},0.0943},
    {"stand",{0,0.0159,0.3941},{0,-0.1432,0.5532},0.0943},
    {"stand",{-0.105,-0.1404,0.6034},{0.105,-0.1404,0.6034},0.053},
    {"stand",{-0.105,-0.1934,0.6564},{0.105,-0.1934,0.6564},0.053},
    {"stand",{-0.105,-0.1934,0.5504},{0.105,-0.1934,0.5504},0.053},
    {"stand",{-0.105,-0.2464,0.6034},{0.105,-0.2464,0.6034},0.053},
    {"stand",{-0.25,-0.2136,0.6766},{0.25,-0.2135,0.6766},0.0388},
    {"stand",{-0.25,-0.2666,0.6235},{0.25,-0.2666,0.6235},0.0388},
    {"stand",{-0.2562,-0.2172,0.6838},{-0.0992,-0.1062,0.5728},0.0403},
    {"stand",{-0.2562,-0.2738,0.6272},{-0.0992,-0.1628,0.5162},0.0403},
    {"stand",{0.0992,-0.1062,0.5728},{0.2562,-0.2172,0.6838},0.0403},
    {"stand",{0.0992,-0.1628,0.5162},{0.2562,-0.2738,0.6272},0.0403},
  };
}

int main() {
  pin::Model model;
  pin::urdf::buildModel(UNIFIED, model);
  pin::Data data(model);

  const char* L[6]={"dual_rb5_850e_left_base_joint","dual_rb5_850e_left_shoulder_joint",
    "dual_rb5_850e_left_elbow_joint","dual_rb5_850e_left_wrist1_joint",
    "dual_rb5_850e_left_wrist2_joint","dual_rb5_850e_left_wrist3_joint"};
  const char* R[6]={"dual_rb5_850e_right_base_joint","dual_rb5_850e_right_shoulder_joint",
    "dual_rb5_850e_right_elbow_joint","dual_rb5_850e_right_wrist1_joint",
    "dual_rb5_850e_right_wrist2_joint","dual_rb5_850e_right_wrist3_joint"};
  Eigen::VectorXd q = pin::neutral(model);
  const double D=M_PI/180.0;
  double init[6]={0,-30,80,0,60,0};
  for(int i=0;i<6;i++){ q[model.joints[model.getJointId(L[i])].idx_q()]=init[i]*D;
                        q[model.joints[model.getJointId(R[i])].idx_q()]=init[i]*D; }

  // ---------------- CAPSULE path (existing Eigen seg-seg) ------------------
  auto arm=armCaps(); auto st=standCaps();
  // precompute endpoint lists + frame ids
  struct WC{ Vector3 a,b; double r; };
  std::vector<pin::FrameIndex> lf(arm.size()), rf(arm.size()), sf(st.size());
  for(size_t i=0;i<arm.size();++i){ lf[i]=model.getFrameId("dual_rb5_850e_left_"+arm[i].frame);
                                    rf[i]=model.getFrameId("dual_rb5_850e_right_"+arm[i].frame); }
  for(size_t i=0;i<st.size();++i) sf[i]=model.getFrameId(st[i].frame);
  std::vector<int> ignore={0}; // stack_sim.yaml stand_ignore_bones

  auto capsuleCycle=[&](double& outmin){
    pin::forwardKinematics(model,data,q);
    pin::updateFramePlacements(model,data);
    std::vector<WC> lc(arm.size()), rc(arm.size()), sc(st.size());
    for(size_t i=0;i<arm.size();++i){
      const auto& Tl=data.oMf[lf[i]]; lc[i]={Tl.act(arm[i].p0),Tl.act(arm[i].p1),arm[i].r};
      const auto& Tr=data.oMf[rf[i]]; rc[i]={Tr.act(arm[i].p0),Tr.act(arm[i].p1),arm[i].r};
    }
    for(size_t i=0;i<st.size();++i){ const auto& Ts=data.oMf[sf[i]]; sc[i]={Ts.act(st[i].p0),Ts.act(st[i].p1),st[i].r}; }
    double m=1e9;
    for(size_t i=0;i<lc.size();++i)for(size_t j=0;j<rc.size();++j)
      m=std::min(m,rb_servo::math::capsuleCapsuleDistance(lc[i].a,lc[i].b,lc[i].r,rc[j].a,rc[j].b,rc[j].r));
    auto vsStand=[&](std::vector<WC>&ac){ for(size_t i=0;i<ac.size();++i){
      if(std::find(ignore.begin(),ignore.end(),(int)i)!=ignore.end())continue;
      for(size_t j=0;j<sc.size();++j)
        m=std::min(m,rb_servo::math::capsuleCapsuleDistance(ac[i].a,ac[i].b,ac[i].r,sc[j].a,sc[j].b,sc[j].r)); } };
    vsStand(lc); vsStand(rc); outmin=m;
  };

  // ---------------- MESH path (coal convex + gripper) ---------------------
  pin::GeometryModel gm;
  pin::urdf::buildGeom(model, UNIFIED, pin::COLLISION, gm, std::vector<std::string>{
      WS+"/robotics_lab/rb_servo_server/descriptions/urdf"});
  for(auto& go: gm.geometryObjects){
    auto bvh=std::dynamic_pointer_cast<coal::BVHModelBase>(go.geometry);
    if(bvh){ bvh->buildConvexRepresentation(false); if(bvh->convex) go.geometry=bvh->convex; }
  }
  // attach pika gripper convex hull to each attachment_site
  coal::MeshLoader loader;
  auto pbvh=loader.load(PIKA, coal::Vec3s(0.001,0.001,0.001));
  pbvh->buildConvexRepresentation(false);
  auto phull=pbvh->convex;
  Eigen::AngleAxisd rz(M_PI/2,Eigen::Vector3d::UnitZ());
  for(const std::string side: {std::string("left"),std::string("right")}){
    auto fid=model.getFrameId("dual_rb5_850e_"+side+"_attachment_site");
    const auto& fr=model.frames[fid];
    pin::SE3 place = fr.placement * pin::SE3(rz.toRotationMatrix(), Eigen::Vector3d::Zero());
    gm.addGeometryObject(pin::GeometryObject(side+"_pika",fr.parentJoint,fr.parentFrame,place,phull));
  }
  // classify + curated pairs (left-right, arm-stand minus link0)
  auto nm=[&](size_t i){return gm.geometryObjects[i].name;};
  auto has=[&](size_t i,const char*s){return nm(i).find(s)!=std::string::npos;};
  std::vector<size_t> li,ri,si;
  for(size_t i=0;i<gm.geometryObjects.size();++i){
    if(has(i,"left"))li.push_back(i); else if(has(i,"right"))ri.push_back(i); else si.push_back(i);
  }
  gm.removeAllCollisionPairs();
  for(auto a:li)for(auto b:ri)gm.addCollisionPair(pin::CollisionPair(a,b));
  for(auto&grp:{li,ri})for(auto a:grp){ if(has(a,"link0"))continue; for(auto b:si)gm.addCollisionPair(pin::CollisionPair(a,b)); }
  pin::GeometryData gd(gm);

  auto meshCycle=[&](double& outmin){
    pin::computeDistances(model,data,gm,gd,q);
    double m=1e9; for(size_t k=0;k<gm.collisionPairs.size();++k) m=std::min(m,gd.distanceResults[k].min_distance);
    outmin=m;
  };
  // enforcement-only: boolean "any pair closer than margin", stop at first hit
  for(auto& req: gd.collisionRequests){ req.security_margin=0.003; req.num_max_contacts=1; }
  auto meshCollideCycle=[&](double& outviol){
    bool hit=pin::computeCollisions(model,data,gm,gd,q,/*stopAtFirstCollision=*/true);
    outviol = hit?1.0:0.0;
  };

  // ------------------------- timing --------------------------------------
  const int N=20000; double cmin=0,mmin=0;
  capsuleCycle(cmin); meshCycle(mmin); // warm
  size_t cpairs=arm.size()*arm.size()+2*(arm.size()-ignore.size())*st.size();
  printf("unified model nq=%d  | capsule pairs~%zu  mesh pairs=%zu  mesh geoms=%zu\n",
         model.nq,cpairs,gm.collisionPairs.size(),gm.geometryObjects.size());
  printf("min clearance @init: capsule=%.1f mm   mesh=%.1f mm\n",cmin*1000,mmin*1000);

  // fixed-pose mean (cache-friendly best case)
  auto benchFixed=[&](auto fn){ double s; auto t0=std::chrono::high_resolution_clock::now();
    for(int i=0;i<N;i++) fn(s);
    auto t1=std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double,std::micro>(t1-t0).count()/N; };
  double tc=benchFixed(capsuleCycle), tm=benchFixed(meshCycle);
  printf("\n--- FIXED pose (init), mean over %d iters ---\n",N);
  printf("CAPSULE (existing Eigen seg-seg): %8.2f us/cycle  (%.2f%% of 2ms)\n",tc,tc/2000*100);
  printf("MESH+gripper (coal convex GJK)  : %8.2f us/cycle  (%.2f%% of 2ms)\n",tm,tm/2000*100);
  printf("ratio mesh/capsule = %.1fx\n",tm/tc);

  // worst-case sweep: per-cycle time across many distinct random poses
  // (GJK iteration count varies with geometry, so the 2ms guarantee needs max)
  auto setRandom=[&](unsigned seed){ // cheap LCG, deterministic
    auto rnd=[&](){ seed=seed*1664525u+1013904223u; return ((seed>>9)&0x7fffff)/8388607.0*2-1; };
    for(int i=0;i<6;i++){ q[model.joints[model.getJointId(L[i])].idx_q()]=rnd()*100*D;
                          q[model.joints[model.getJointId(R[i])].idx_q()]=rnd()*100*D; } };
  auto sweep=[&](auto fn,int P){ std::vector<double> ts; ts.reserve(P); double s;
    for(int p=0;p<P;p++){ setRandom(p+1);
      auto t0=std::chrono::high_resolution_clock::now(); fn(s);
      auto t1=std::chrono::high_resolution_clock::now();
      ts.push_back(std::chrono::duration<double,std::micro>(t1-t0).count()); }
    std::sort(ts.begin(),ts.end());
    double sum=0; for(double v:ts)sum+=v;
    return std::array<double,4>{sum/P, ts[(size_t)(P*0.5)], ts[(size_t)(P*0.99)], ts.back()}; };
  const int P=4000;
  auto cs=sweep(capsuleCycle,P), ms=sweep(meshCycle,P), mb=sweep(meshCollideCycle,P);
  printf("\n--- WORST-CASE sweep across %d random poses (us/cycle) ---\n",P);
  printf("%-34s %8s %8s %8s %8s %8s\n","path","mean","median","p99","max","max%2ms");
  printf("%-34s %8.1f %8.1f %8.1f %8.1f %7.1f%%\n","CAPSULE (Eigen, clearance)",cs[0],cs[1],cs[2],cs[3],cs[3]/2000*100);
  printf("%-34s %8.1f %8.1f %8.1f %8.1f %7.1f%%\n","MESH (coal, full clearance)",ms[0],ms[1],ms[2],ms[3],ms[3]/2000*100);
  printf("%-34s %8.1f %8.1f %8.1f %8.1f %7.1f%%\n","MESH (coal, boolean stopAtFirst)",mb[0],mb[1],mb[2],mb[3],mb[3]/2000*100);
  return 0;
}
