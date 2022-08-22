#include <cstdlib>

#if defined(WIN32)
#pragma warning(disable:4996)
#include <GL/glut.h>
#ifdef NDEBUG
#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#endif // NDEBUG

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#elif defined(__APPLE__) || defined(MACOSX)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <GLUT/glut.h>
#else // MACOSX
#include <GL/glut.h>
#endif // unix

#define GLM_FORCE_SWIZZLE
#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/ext/quaternion_geometric.hpp"
//#include "glm/ext/quaternion_trigonometric.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/norm.hpp"
//#include "glm/ext.hpp"
#include "glm/gtx/string_cast.hpp"
#include "glm/gtx/closest_point.hpp"
#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"
#include <vector>
#include <iostream>
#include <cstdint>
#include <algorithm>
#include <cfloat>

#define USE_TEST_SCENE (0)
#define USE_DEBUG      (1)
#define USE_DOUBLE     (1)
#define USE_CAPTURE    (0)
#define USE_TW_SCENE   (1)
#define USE_OBB_COL    (0)

#if !(USE_DOUBLE)
typedef float     Float;
typedef glm::vec3 Vec3;
typedef glm::quat Quat;
typedef glm::mat3 Mat3;
typedef glm::mat4 Mat4;
#else
typedef double       Float;
typedef glm::f64vec3 Vec3;
typedef glm::dquat   Quat;
typedef glm::dmat3   Mat3;
typedef glm::dmat4   Mat4;
#endif

namespace {
  Float PI_                      = (Float)3.14159265358979323846;
  Float max_rotation_per_substep = (Float)0.5;
#if 1
  Float gravity                  = (Float)-10.0;
#else
  Float gravity                  = (Float)0.0;
#endif
  Float fixed_dt                 = (Float)(1.0 / 60.0);
  glm::vec4 box_vtx[8] = {
    {+0.5f, +0.5f, +0.5f, 0.0f},
    {+0.5f, +0.5f, -0.5f, 0.0f},
    {-0.5f, +0.5f, +0.5f, 0.0f},
    {-0.5f, +0.5f, -0.5f, 0.0f},
    {+0.5f, -0.5f, +0.5f, 0.0f},
    {+0.5f, -0.5f, -0.5f, 0.0f},
    {-0.5f, -0.5f, +0.5f, 0.0f},
    {-0.5f, -0.5f, -0.5f, 0.0f},
  };
  float cam_clamp_pos_min_x = -3.0f;
  float cam_clamp_pos_max_x = +3.0f;
  float cam_clamp_pos_min_y = -0.0f;
  float cam_clamp_pos_max_y = +5.5f;
  float cam_clamp_pos_min_z = +0.9f;
  float cam_clamp_pos_max_z = +5.0f;
  float cam_clamp_tgt_min_x = -3.0f;
  float cam_clamp_tgt_max_x = +3.0f;
  float cam_clamp_tgt_min_y = -5.5f;
  float cam_clamp_tgt_max_y = +5.5f;
  Vec3  barrier_pos((Float)0.0, (Float)-2.0, (Float)0.0);
  float barrier_radius = 4.0f;
};

struct Energy {
  Float k;
  Float u;
  Energy(Float in_k, Float in_u) : k(in_k), u(in_u) {}
};

struct Body;

struct BoundingSphere {
  Vec3* position;
  Float radius;
  Body* body;
};

struct OBB {
  Vec3 position;
  Vec3 size; // half size
  Mat3 orientation;
};

struct Sphere {
  Vec3  position;
  Float radius;
};

struct Plane {
  Vec3  normal;
  Float distance;
};

struct Contact {
  Vec3  position;
  Vec3  normal;
  Float distance;
};

struct BodyPairContact {
  Body* body0;
  Body* body1;
  Contact contact;
};

struct DebugInfo {
  bool    show_depth;
  GLfloat dof;
  GLfloat focus;
  std::vector<Energy> energies;
  DebugInfo() : show_depth(false),  dof(0.1f), focus(0.0f), energies() {}
};

struct Collision {
  bool use_ground;
  bool use_self_contact;
  bool show_contact;
  bool show_bounding;
  float safety_multiplier;
  Collision() : use_ground(true), use_self_contact(false), show_contact(false), show_bounding(false), safety_multiplier((Float)2.0) {}
};

struct Camera {
  glm::vec3  pos;
  glm::vec3  tgt;
  bool       is_zoom;
  bool       is_move;
  glm::ivec2 zoom_start;
  glm::ivec2 move_start;
  Camera() :
#if USE_TEST_SCENE
    pos(0.0f, 1.6f, 15.0f),
#else
    pos(0.0f, 3.0f,  4.0f),
#endif
    tgt(0.0f, 3.0f, 0.0f), is_zoom(false), is_move(false), zoom_start(), move_start() {
  }
  void handle_zoom(float d) {
    float scale = 1.0f - 0.03f * d;
    float dist = glm::length(tgt - pos);
    if ((d > (Float)0.0 && dist < 0.2f) || (d < (Float)0.0 && dist > 5.0f)) {
      return;
    }
    pos = tgt + (pos - tgt) * scale;
  }
  void handle_motion(int dx, int dy) {
    //printf("motion %d, %d \n", dx, dy);
    float scale = 0.01f;
    float prev = glm::length(tgt - pos);
    glm::vec3 dir_z = tgt - pos;
    glm::vec3 dir_x(dir_z.z, 0.0f, -dir_z.x); 
    dir_x = glm::normalize(dir_x);
    glm::vec3 dir_y = glm::cross(dir_z, dir_x);
    dir_y = glm::normalize(dir_y);
    pos += dir_x * scale * (float)dx;
    pos += dir_y * scale * (float)dy;
    dir_z = tgt - pos;
    dir_z = glm::normalize(dir_z);
    float delta = glm::length(tgt - pos) - prev;
    pos += dir_z * -delta;
  }
  void clamp() {
    pos.x = glm::clamp(pos.x, cam_clamp_pos_min_x, cam_clamp_pos_max_x);
    pos.y = glm::clamp(pos.y, cam_clamp_pos_min_y, cam_clamp_pos_max_y);
    pos.z = glm::clamp(pos.z, cam_clamp_pos_min_z, cam_clamp_pos_max_z);
    tgt.x = glm::clamp(tgt.x, cam_clamp_tgt_min_x, cam_clamp_tgt_max_x);
    tgt.y = glm::clamp(tgt.y, cam_clamp_tgt_min_y, cam_clamp_tgt_max_y);
  }
  void reset() {
    pos = glm::vec3(0.0f, 3.0f, 4.0f);
    tgt = glm::vec3(0.0f, 3.0f, 0.0f);
  }
};

struct Params {
  int     num_objs;
  GLfloat compliance;
  int     substeps;
  GLfloat rot_damping;
  GLfloat pos_damping;
  int     joint_type;
  bool    has_swing_limit;
  GLfloat min_swing_angle;
  GLfloat max_swing_angle;
  GLfloat swing_limit_compliance;
  bool    has_twist_limit;
  GLfloat min_twist_angle;
  GLfloat max_twist_angle;
  GLfloat twist_limit_compliance;
  bool    show_joint;
  bool    show_force;
  Params() : num_objs(100), compliance(0.0f), substeps(40), rot_damping(1000.0f), pos_damping(1000.0f), joint_type(/*Joint::Type::Spherical*/0), has_swing_limit(false), min_swing_angle(-2.0f), max_swing_angle(+2.0f), swing_limit_compliance((Float)0.0), has_twist_limit(false), min_twist_angle(-2.0f), max_twist_angle(+2.0f), twist_limit_compliance((Float)0.0), show_joint(false), show_force(false) {}
};

void get_quat_axis0(Vec3* axis, const Quat& q) {
  Float x2 = q.x * (Float)2.0;
  Float w2 = q.w * (Float)2.0;
  axis->x = ( q.w * w2) - (Float)1.0 + q.x * x2;
  axis->y = ( q.z * w2) + q.y * x2;
  axis->z = (-q.y * w2) + q.z * x2;
  // TODO replace glm::gtx::quaternion::axis()
  //glm::vec3 ret = glm::gtx::quaternion::axis(q);
}

void get_quat_axis1(Vec3* axis, const Quat& q) {
  Float y2 = q.y * (Float)2.0;
  Float w2 = q.w * (Float)2.0;
  axis->x = (-q.z * w2) + q.x * y2;
  axis->y = ( q.w * w2) - (Float)1.0 + q.y * y2;
  axis->z = ( q.x * w2) + q.z * y2;
  // TODO replace glm::gtx::quaternion::axis()
}

void get_quat_axis2(Vec3* axis, const Quat& q) {
  Float z2 = q.z * (Float)2.0;
  Float w2 = q.w * (Float)2.0;
  axis->x = ( q.y * w2) + q.x * z2;
  axis->y = (-q.x * w2) + q.y * z2;
  axis->z = ( q.w * w2) - (Float)1.0 + q.z * z2;
  // TODO replace glm::gtx::quaternion::axis()
}

Vec3 apply_quaternion(Vec3& in_v, const Quat& in_q) {
  Vec3 v = in_v;
  Quat q = in_q;
  Quat i;
  i.x =  q.w * v.x + q.y * v.z - q.z * v.y;
  i.y =  q.w * v.y + q.z * v.x - q.x * v.z;
  i.z =  q.w * v.z + q.x * v.y - q.y * v.x;
  i.w = -q.x * v.x - q.y * v.y - q.z * v.z;
  Vec3 ret;
  ret.x = i.x * q.w + i.w * -q.x + i.y * -q.z - i.z * -q.y;
  ret.y = i.y * q.w + i.w * -q.y + i.z * -q.x - i.x * -q.z;
  ret.z = i.z * q.w + i.w * -q.z + i.x * -q.y - i.y * -q.x;
  return ret;
}

struct Pose {
  Vec3 p;
  Quat q;
  Pose() : p(), q((Float)1.0, (Float)0.0, (Float)0.0, (Float)0.0) {}
  void rotate(Vec3* v) {
    *v = apply_quaternion(*v, q);
  }
  void transform_pose(Pose* in_pose) {
    in_pose->q = q * in_pose->q;
    rotate(&in_pose->p);
    in_pose->p += p;
  }
  void inv_rotate(Vec3* v) {
    Quat inv = glm::conjugate(q);
    *v = apply_quaternion(*v, inv);
  }
  void inv_transform(Vec3* v) {
    *v -= p;
    inv_rotate(v);
  }
};

struct Body{
  Pose  pose;
  Pose  prev_pose;
  Vec3  vel;
  Vec3  omega;
  Vec3  size;
  Float inv_mass;
  Vec3  inv_inertia;
  BoundingSphere sphere;
  Body(const Pose& p) : pose(p), prev_pose(p), vel(), omega(), size(), inv_mass((Float)1.0), inv_inertia(1.0, 1.0, 1.0), sphere() {
  }
  void set_box(Vec3 in_size, Float density = 1.0) {
    size = in_size;
    Float mass = size.x * size.y * size.z * density;
    inv_mass = (Float)1.0 / mass;
    mass /= (Float)12.0; // rotational inertia of cuboid (https://en.wikipedia.org/wiki/List_of_moments_of_inertia)
    inv_inertia.x = (Float)1.0 / (size.y * size.y + size.z * size.z) / mass;
    inv_inertia.y = (Float)1.0 / (size.z * size.z + size.x * size.x) / mass;
    inv_inertia.z = (Float)1.0 / (size.x * size.x + size.y * size.y) / mass;
    sphere.position = &pose.p;
    sphere.radius = (Float)0.5 * sqrt( (size.x * size.x) + (size.y * size.y) + (size.z * size.z) ); // circumscribed sphere
	sphere.body = this;
  }
  void apply_rotation(Vec3& rot, Float scale = 1.0f) {
    Float phi = glm::length(rot);
    if ((phi * scale) > max_rotation_per_substep) {
      scale = max_rotation_per_substep / phi;
    }
    Quat dq = Quat((Float)0.0, rot.x * scale, rot.y * scale, rot.z * scale); // constructor(wxyz), internal(xyzw)
    dq *= pose.q;
    pose.q += (Float)0.5 * dq;                                                   // q <- q + h/2[ωx, ωy, ωz, 0]q
    pose.q = glm::normalize(pose.q);                                             // q <- q / |q|
  }
  void integrate(Float dt, Float gravity) {
    prev_pose = pose;      // x_prev <- x
    vel.y += gravity * dt; // v      <- v + hf_ext/m
    pose.p += vel * dt;    // x      <- x + hv
    apply_rotation(omega, dt);
  }
  void update(Float dt) {
    vel = pose.p - prev_pose.p;
    vel *= 1.0f / dt;                               // v   <- (x - x_prev) / h
    Quat dq = pose.q * glm::conjugate(prev_pose.q); // Δq <- q (q_prev)^-1
    omega.x = dq.x * 2.0f / dt;
    omega.y = dq.y * 2.0f / dt;
    omega.z = dq.z * 2.0f / dt;                     // ω <- 2[Δq_x, Δq_y, Δq_z] / h
    if (dq.w < 0.0) {                               // ω <- Δq_w >= 0 ? ω : -ω
      omega.x = -omega.x;
      omega.y = -omega.y;
      omega.z = -omega.z;
    }
  }
  Vec3 get_velocity_at(const Vec3& pos) {
    Vec3 local_vel((Float)0.0, (Float)0.0, (Float)0.0);
    local_vel = pos - pose.p;
    local_vel = glm::cross(local_vel, omega);
    local_vel = vel - local_vel;
    return local_vel;
  }
  Float get_inverse_mass(const Vec3& normal, Vec3* pos = nullptr) {
    Vec3 n;
    if (pos == nullptr) {
      n = normal;
    } else {
      n = *pos - pose.p;
      n = glm::cross(n, normal);
    }
    pose.inv_rotate(&n);
    Float w = n.x * n.x * inv_inertia.x +
              n.y * n.y * inv_inertia.y +
              n.z * n.z * inv_inertia.z;
    if (pos != nullptr) {
      w += inv_mass;
    }
    return w;
  }
  void apply_correction(const Vec3& corr, Vec3* pos = nullptr, bool velocity_level = false) {
    Vec3 dq;
    if (pos == nullptr) {
      dq = corr;
    } else {
      if (velocity_level) {
        vel    += corr * inv_mass;
      } else {
        pose.p += corr * inv_mass;
      }
      dq = *pos - pose.p;
      dq = glm::cross(dq, corr);
    }
    pose.inv_rotate(&dq);
    dq.x = inv_inertia.x * dq.x;
    dq.y = inv_inertia.y * dq.y;
    dq.z = inv_inertia.z * dq.z;
    pose.rotate(&dq);
    if (velocity_level) {
      omega += dq;
    } else {
      apply_rotation(dq);
    }
  }
  Float get_kinetic_energy(int substeps) { // 1/2 mv^2 + 1/2 Iω^2
    Float     m = 1.0f / (Float)inv_mass;
    Float   sdt = fixed_dt / substeps;
    Vec3      v = (pose.p - prev_pose.p) * (1.0f / sdt); // m/s
    Vec3      o = omega * (Float)1.0 / sdt;                    // rad/s
    Float     K = (1.0f / 2.0f) * m * glm::length(v) * glm::length(v);       // 1/2 mv^2
#if 0
    Vec3      I = 1.0f / inv_inertia;
    K += (1.0f / 2.0f) * glm::length(I) * glm::length(o) * glm::length(o);
#else
    Vec3      iv = (Float)1.0 / inv_inertia;
    Mat3      I(iv.x, 0.0f, 0.0f,
                0.0f, iv.y, 0.0f,
                0.0f, 0.0f, iv.z);
    Quat      cq = glm::conjugate(pose.q);
    Mat3      Ip = glm::mat3_cast(pose.q) * I * glm::mat3_cast(cq); // I' = R I R' eq.(10) in http://www-geoph.eps.s.u-tokyo.ac.jp/~s52605/planetology/tensor_of_inertia.pdf
    Float scalar_product = 0.0f;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        scalar_product += Ip[i][j] * o[i] * o[j];
      }
    }
    K += (1.0f / 2.0f) * scalar_product; // 1/2 I' ω_ij ω_ji
#endif
    return K;
  }
  Float get_potential_energy() {
    Float m = 1.0f / inv_mass;
    return m * -1.0f * gravity * pose.p.y;
  }
};

struct SPlane {
  glm::vec4 v;
};

struct Light {
  glm::vec4 v;
};

struct Shadow {
  float v[4][4];
};

struct Context;

class Scene {
public:
  enum {
    eChain_Of_100_Bodies,
    eTwistedRope,
    eKnot,
    eNum,
  };
  virtual void Update(Context& context, Float dt) = 0;
  virtual void Render(float alpha = 1.0f) = 0;
  virtual ~Scene() {}
};

struct Joint;
struct Raycaster;

struct Context {
  std::uint32_t       frame;
  DebugInfo           debug_info;
  Collision           collision;
  Params              params;
  SPlane              floor;
  Light               light;
  Shadow              floor_shadow;
  GLint               window_w;
  GLint               window_h;
  GLint               vp[4];
  GLdouble            modelview_mtx[16];
  GLdouble            proj_mtx[16];
  std::vector<Body*>  bodies;
  std::vector<Joint*> joints;
  std::vector<BoundingSphere*> spheres;
  std::vector<BodyPairContact> contacts;
  std::vector<BodyPairContact> self_contacts;
  Joint*              grab_joint;
  GLint               grab_idx;
  Vec3                grab_point;
  float               grab_distance;
  float               grab_depth;
  Scene*              scene;
  int                 scene_num;
  Camera              camera;
  bool                paused;
  Raycaster*          raycaster;
    Context() : frame(0), debug_info(), collision(), params(), floor(), light(), floor_shadow(), window_w(0), window_h(0), vp(), modelview_mtx(), proj_mtx(), bodies(), joints(), spheres(), contacts(), self_contacts(), grab_joint(nullptr), grab_idx(-1), grab_point(), grab_distance(0.0f), grab_depth(0.0f), scene(nullptr), scene_num(Scene::eChain_Of_100_Bodies/*Scene::eTwistedRope*/ /*Scene::eKnot*/), camera(), paused(false), raycaster(nullptr) {}
};

Context g_Context;

struct Ray {
  glm::vec3 origin;
  glm::vec3 direction;
  Ray() : origin(), direction() {}
};

void calc_world_coord(glm::f64vec3* w, int x, int y, double z) {
  GLint realy = g_Context.vp[3] - (GLint)y - 1;
//  printf ("Coordinates at cursor are (%4d, %4d)\n", x, realy);
  gluUnProject((GLdouble)x, (GLdouble)realy, z,
                g_Context.modelview_mtx, g_Context.proj_mtx, g_Context.vp, &w->x, &w->y, &w->z);
}

void closest_point(Vec3* out, const OBB& obb, const Vec3& p) {
  *out = obb.position;
  Vec3 dir = p - obb.position;
  for (int i = 0; i < 3; i++) {
    const Float* o = glm::value_ptr(obb.orientation);
    int idx = i * 3;
    Vec3  axis(o[idx + 0], o[idx + 1], o[idx + 2]);
    Float distance = glm::dot(dir, axis);
    if (distance > obb.size[i]) {
      distance = obb.size[i];
    }
    if (distance < -obb.size[i]) {
      distance = -obb.size[i];
    }
    *out += (axis * distance);
  }
}

bool obb_sphere_intersection(Vec3* closest_p, const OBB& obb, const Sphere& sphere) {
  closest_point(closest_p, obb, sphere.position);
  Vec3 dist = sphere.position - *closest_p;
  Float dist_sq = glm::dot(dist, dist);
  Float radius_sq = sphere.radius * sphere.radius;
  return (dist_sq < radius_sq);
}

bool obb_plane_intersection(const OBB& obb, const Plane& plane) {
  const Float* o = glm::value_ptr(obb.orientation);
  Vec3 rot[] = {
    Vec3(o[0], o[1], o[2]),
    Vec3(o[3], o[4], o[5]),
    Vec3(o[6], o[7], o[8]),
  };
  Vec3  normal = plane.normal;
  Float plen   = obb.size.x * fabs(glm::dot(normal, rot[0])) +
                 obb.size.y * fabs(glm::dot(normal, rot[1])) +
                 obb.size.z * fabs(glm::dot(normal, rot[2]));
  Float dist   = glm::dot(plane.normal, obb.position) - plane.distance;
  return (fabs(dist) <= plen);
}

bool obb_obb_intersection() {
  return false;
}

bool ray_obb_intersection(glm::vec3& ray_origin, glm::vec3& ray_direction, glm::vec3& aabb_min, glm::vec3& aabb_max, glm::mat4& model_matrix, float* intersection_distance) {
  float t_min = 0.0f;
  float t_max = 100000.0f;
  glm::vec3 obb_position_worldspace(model_matrix[3].x, model_matrix[3].y, model_matrix[3].z);
  glm::vec3 delta = obb_position_worldspace - ray_origin;
  {
    glm::vec3 x_axis(model_matrix[0].x, model_matrix[0].y, model_matrix[0].z);
    float e = glm::dot(x_axis, delta);
    float f = glm::dot(ray_direction, x_axis);
    if ( fabs(f) > 0.001f ){ // Standard case
      float t1 = (e + aabb_min.x) / f; // Intersection with the "left" plane
      float t2 = (e + aabb_max.x) / f; // Intersection with the "right" plane
      if (t1 > t2){ // if wrong order
        float w = t1;
        t1 = t2;
        t2 = w; // swap t1 and t2
      }
      if ( t2 < t_max ) { t_max = t2; }
      if ( t1 > t_min ) { t_min = t1; }
      if (t_max < t_min ) {
        return false;
      }
    } else {
      if(-e+aabb_min.x > 0.0f || -e+aabb_max.x < 0.0f) {
        return false;
      }
    }
  }
  {
    glm::vec3 y_axis(model_matrix[1].x, model_matrix[1].y, model_matrix[1].z);
    float e = glm::dot(y_axis, delta);
    float f = glm::dot(ray_direction, y_axis);
    if ( fabs(f) > 0.001f ){ // Standard case
      float t1 = (e + aabb_min.y) / f;
      float t2 = (e + aabb_max.y) / f;
      if (t1 > t2){ // if wrong order
        float w = t1;
        t1 = t2;
        t2 = w; // swap t1 and t2
      }
      if ( t2 < t_max ) { t_max = t2; }
      if ( t1 > t_min ) { t_min = t1; }
      if (t_max < t_min ) {
        return false;
      }
    } else {
      if(-e+aabb_min.y > 0.0f || -e+aabb_max.y < 0.0f) {
        return false;
      }
    }
  }
  {
    glm::vec3 z_axis(model_matrix[2].x, model_matrix[2].y, model_matrix[2].z);
    float e = glm::dot(z_axis, delta);
    float f = glm::dot(ray_direction, z_axis);
    if ( fabs(f) > 0.001f ){ // Standard case
      float t1 = (e + aabb_min.z) / f;
      float t2 = (e + aabb_max.z) / f;
      if (t1 > t2){ // if wrong order
        float w = t1;
        t1 = t2;
        t2 = w; // swap t1 and t2
      }
      if ( t2 < t_max ) { t_max = t2; }
      if ( t1 > t_min ) { t_min = t1; }
      if (t_max < t_min ) {
        return false;
      }
    } else {
      if(-e+aabb_min.z > 0.0f || -e+aabb_max.z < 0.0f) {
        return false;
      }
    }
  }
  *intersection_distance = t_min;
  return true;
}

struct Raycaster {
  Ray ray;
  Raycaster() {}
  void set_from_camera(int x, int y, bool is_read_depth, float* depth) {
    Context& ctx = g_Context;
    ray.origin = ctx.camera.pos;
    float z = *depth; // original 0.5 fixed
    if (is_read_depth) {
      glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z); // or use depth
      *depth = z;
    }
    glm::f64vec3 w;
    calc_world_coord(&w, x, y, (double)z);
    glm::vec3 end((float)w.x, (float)w.y, (float)w.z);
    ray.direction = glm::normalize(end - ray.origin);
  }
  float intersect_objects(Vec3* out, glm::mat4& mtx) {
    Context& ctx = g_Context;
    float d = 0.0f;
    glm::vec3 aabb_min(-0.5f, -0.5f, -0.5f);
    glm::vec3 aabb_max(+0.5f, +0.5f, +0.5f);
    if (ray_obb_intersection(ray.origin, ray.direction, aabb_min, aabb_max, mtx, &d)) {
      *out = Vec3(ray.origin + ray.direction * d);
    }
    return d;
  }
};

struct Material {
  GLfloat ambient[4];
  GLfloat diffuse[4];
  GLfloat specular[4];
  GLfloat shininess;
};

// emerald
Material mat_emerald = {
  { 0.0215f,  0.1745f,   0.0215f,  1.0f },
  { 0.07568f, 0.61424f,  0.07568f, 1.0f },
  { 0.633f,   0.727811f, 0.633f,   1.0f },
  76.8f
};

// jade
Material mat_jade = {
  { 0.135f,     0.2225f,   0.1575f,   1.0f },
  { 0.54f,      0.89f,     0.63f,     1.0f },
  { 0.316228f,  0.316228f, 0.316228f, 1.0f },
  12.8f
};

// obsidian
Material mat_obsidian = {
  { 0.05375f, 0.05f,      0.06625f,  1.0f },
  { 0.18275f, 0.17f,      0.22525f,  1.0f },
  { 0.332741f, 0.328634f, 0.346435f, 1.0f },
  38.4f
};

// pearl
Material mat_pearl = {
  { 0.25f,     0.20725f,  0.20725f,  1.0f },
  { 1.0f,      0.829f,    0.829f,    1.0f },
  { 0.296648f, 0.296648f, 0.296648f, 1.0f },
  10.24f
};

// ruby
Material mat_ruby  = {
  { 0.1745f,   0.01175f,  0.01175f,  1.0f },
  { 0.61424f,  0.04136f,  0.04136f,  1.0f },
  { 0.727811f, 0.626959f, 0.626959f, 1.0f },
  76.8f
};

// turquoise
Material mat_turquoise = {
  { 0.1f,      0.18725f, 0.1745f,   1.0f },
  { 0.396f,    0.74151f, 0.69102f,  1.0f },
  { 0.297254f, 0.30829f, 0.306678f, 1.0f },
  12.8f,
};

// brass
Material mat_brass = {
  { 0.329412f,  0.223529f, 0.027451f, 1.0f },
  { 0.780392f,  0.568627f, 0.113725f, 1.0f },
  { 0.992157f,  0.941176f, 0.807843f, 1.0f },
  27.89743616f,
};

// bronze
Material mat_bronze = {
  { 0.2125f,   0.1275f,   0.054f,    1.0f },
  { 0.714f,    0.4284f,   0.18144f,  1.0f },
  { 0.393548f, 0.271906f, 0.166721f, 1.0f },
  25.6f,
};

// chrome
Material mat_chrome = {
  { 0.25f,     0.25f,     0.25f,     1.0f },
  { 0.4f,      0.4f,      0.4f,      1.0f },
  { 0.774597f, 0.774597f, 0.774597f, 1.0f },
  76.8f,
};

// copper
Material mat_copper = {
  { 0.19125f,  0.0735f,   0.0225f,   1.0f },
  { 0.7038f,   0.27048f,  0.0828f,   1.0f },
  { 0.256777f, 0.137622f, 0.086014f, 1.0f },
  12.8f,
};

// gold
Material mat_gold = {
  { 0.24725f,  0.1995f,   0.0745f,    1.0f },
  { 0.75164f,  0.60648f,  0.22648f,   1.0f },
  { 0.628281f, 0.555802f, 0.366065f,  1.0f },
  51.2f,
};

// silver
Material mat_silver = {
  { 0.19225f,  0.19225f,  0.19225f,  1.0f },
  { 0.50754f,  0.50754f,  0.50754f,  1.0f },
  { 0.508273f, 0.508273f, 0.508273f, 1.0f },
  51.2f,
};

// plastic(black)
Material mat_plastic_black = {
  { 0.0f,  0.0f,  0.0f,  1.0f },
  { 0.01f, 0.01f, 0.01f, 1.0f },
  { 0.50f, 0.50f, 0.50f, 1.0f },
  32.0f,
};

// plastic(cyan)
Material mat_plastic_cyan = {
  { 0.0f, 0.1f,        0.06f,    1.0f },
  { 0.0f, 0.50980392f, 0.50980392f, 1.0f },
  { 0.50196078f, 0.50196078f, 0.50196078f, 1.0f },
  32.0f,
};

// rubber(black)
Material mat_rubbr_black = {
  { 0.02f, 0.02f, 0.02f, 1.0f },
  { 0.01f, 0.01f, 0.01f, 1.0f },
  { 0.4f,  0.4f,  0.4f,  1.0f },
  10.0f,
};

// rubber(red)
Material mat_rubbr_red = {
  { 0.05f, 0.0f,  0.0f,  1.0f },
  { 0.5f,  0.4f,  0.4f,  1.0f },
  { 0.7f,  0.04f, 0.04f, 1.0f },
  10.0f,
};

// white
Material mat_white = {
  { 1.0f, 1.0f, 1.0f, 1.0f },
  { 1.0f, 1.0f, 1.0f, 1.0f },
  { 1.0f, 1.0f, 1.0f, 1.0f },
  32.0f,
};


/*

	//13:Plastic(Cyan)
        0.0,   0.1,    0.06,    1.0,
	0.0,   0.50980392,0.50980392,1.0,
        0.50196078,0.50196078,0.50196078,1.0,
        32,

	//14:Plastic(Green)
        0.0,     0.0,   0.0,  1.0,
        0.1,     0.35,  0.1,  1.0,
        0.45,    0.55,  0.45, 1.0,
        32,

	//15:Plastic(Red)
        0.0,     0.0,     0.0,  1.0,
        0.5,     0.0,     0.0,  1.0,
        0.7,     0.6,     0.6,  1.0,
        32,

	//16:Plastic(White)
        0.0,   0.0,     0.0,  1.0,
        0.55,  0.55,    0.55, 1.0,
        0.70,  0.70,    0.70, 1.0,
        32,

	//17:Plastic(Yellow)
        0.0,  0.0,     0.0,  1.0,
        0.5,  0.5,     0.0,  1.0,
        0.60, 0.60,    0.50, 1.0,
        32,

	//18:Rubber(Black)
        0.02,   0.02,    0.02, 1.0,
        0.01,   0.01,    0.01, 1.0,
        0.4,    0.4,     0.4,  1.0,
        10.0,

	//19:Rubber(Cyan)
        0.0,     0.05,    0.05, 1.0,
        0.4,     0.5,     0.5,  1.0,
        0.04,    0.7,     0.7,  1.0,
        10.0,

	//20:Rubber(Green)
        0.0,    0.05,    0.0,  1.0,
        0.4,    0.5,     0.4,  1.0,
        0.04,   0.7,     0.04, 1.0,
        10.0,

        //22:Rubber(Red)
        0.05,     0.0,     0.0,  1.0,
        0.5,      0.4,     0.4,  1.0,
        0.7,      0.04,    0.04, 1.0,
        10.0,

	//23:Rubber(White)
        0.05,   0.05,    0.05, 1.0,
        0.5,    0.5,     0.5,  1.0,
        0.7,    0.7,     0.7,  1.0,
        10.0,

	//24:Rubber(Yellow)
        0.05,  0.05,    0.0,  1.0,
        0.5,   0.5,     0.4,  1.0,
        0.7,   0.7,     0.04, 1.0,
        10.0

 */

struct Joint{
  enum Type {
    Spherical,
    Hinge,
    Fixed,
  };
  int type;//Type type;
  Body* body0;
  Body* body1;
  Pose local_pose0;
  Pose local_pose1;
  Pose global_pose0;
  Pose global_pose1;
  Float compliance;
  Float rot_damping;
  Float pos_damping;
  bool  has_swing_limit;
  Float min_swing_angle;
  Float max_swing_angle;
  Float swing_limit_compliance;
  bool  has_twist_limit;
  Float min_twist_angle;
  Float max_twist_angle;
  Float twist_limit_compliance;
  Float force;
  Joint(/*Joint::Type*/ int in_type, Body* in_body0, Body* in_body1, const Pose& in_local_pose0, const Pose& in_local_pose1)
    : type(in_type), body0(in_body0), body1(in_body1), local_pose0(in_local_pose0), local_pose1(in_local_pose1),
      global_pose0(in_local_pose0), global_pose1(in_local_pose1), compliance(g_Context.params.compliance), rot_damping(g_Context.params.rot_damping),
      pos_damping(g_Context.params.pos_damping), has_swing_limit(g_Context.params.has_swing_limit),
      min_swing_angle((Float)g_Context.params.min_swing_angle * PI_),
      max_swing_angle((Float)g_Context.params.max_swing_angle * PI_),
      swing_limit_compliance((Float)g_Context.params.swing_limit_compliance), has_twist_limit(g_Context.params.has_twist_limit),
      min_twist_angle((Float)g_Context.params.min_twist_angle * PI_),
      max_twist_angle((Float)g_Context.params.max_twist_angle * PI_),
      twist_limit_compliance((Float)g_Context.params.twist_limit_compliance), force((Float)0.0) {
  }
  void update_global_poses() {
    global_pose0 = local_pose0;
    if (body0) {
      body0->pose.transform_pose(&global_pose0);
    }
    global_pose1 = local_pose1;
    if (body1) {
      body1->pose.transform_pose(&global_pose1);
    }
  }
  void limit_angle(Body* in_body0, Body* in_body1, const Vec3& n, const Vec3& a, Vec3 b, Float min_ang, Float max_ang, Float compliance, Float dt, Float max_corr = PI_) {
    Vec3 c = glm::cross(a, b);
    Float phi = std::asin(glm::dot(c, n));
    if (glm::dot(a, b) < (Float)0.0) {
      phi = PI_ - phi;
    }
    if (phi >  PI_) {
      phi -= (Float)2.0 * PI_;
    }
    if (phi < -PI_) {
      phi += (Float)2.0 * PI_;
    }
    if (phi < min_ang || phi > max_ang) {
      phi = std::min(std::max(min_ang, phi), max_ang);
      Quat q = glm::angleAxis(phi, n);
      Vec3 omega = a;
      omega = apply_quaternion(omega, q);
      omega = glm::cross(omega, b);
      phi   = glm::length(omega);
      if (phi > max_corr) {
        omega *= (max_corr / phi);
      }
      apply_body_pair_correction(body0, body1, omega, compliance, dt);
    }
  }
  void apply_body_pair_correction(Body* in_body0, Body* in_body1, Vec3 corr, Float in_compliance, Float dt, Vec3* pos0 = nullptr, Vec3* pos1 = nullptr, bool velocity_level = false) {
    Float c = glm::length(corr);
    if (c == (Float)0.0) {
      return;
    }
    Vec3 normal = glm::normalize(corr);
    Float w0 = in_body0 ? in_body0->get_inverse_mass(normal, pos0) : (Float)0.0;
    Float w1 = in_body1 ? in_body1->get_inverse_mass(normal, pos1) : (Float)0.0;
    Float w  = w0 + w1;
    if (w == (Float)0.0) {
      return;
    }
    Float lambda = -c / (w + compliance / dt / dt); // eq.(4)
    force = lambda / dt / dt; // eq.(10)
    normal *= -lambda;
    if (in_body0) {
      in_body0->apply_correction(normal, pos0, velocity_level);
    }
    if (in_body1) {
      normal *= -1.0f;
      in_body1->apply_correction(normal, pos1, velocity_level);
    }
  }
  void solve_pos(Float dt) {
    update_global_poses();
    if (type == Type::Fixed) { // orientation
      Quat q = global_pose0.q;
      q = glm::conjugate(q);
      q = global_pose1.q * q;                         // eq.(19)
      Vec3 omega(2.0f * q.x, 2.0f * q.y, 2.0f * q.z); // eq.(20)
      if (q.w < 0.0f) {
        omega *= (Float)-1.0; // original js bug(?)
      }
      apply_body_pair_correction(body0, body1, omega, compliance, dt);
    }
    if (type == Type::Hinge) {
      Vec3 a0;
      Vec3 b0;
      Vec3 c0;
      Vec3 a1;
      get_quat_axis0(&a0, global_pose0.q);
      get_quat_axis1(&b0, global_pose0.q);
      get_quat_axis2(&c0, global_pose0.q);
      get_quat_axis0(&a1, global_pose1.q);
      a0 = glm::cross(a0, a1);
      apply_body_pair_correction(body0, body1, a0, (Float)0.0, dt);
      if (has_swing_limit) {
        update_global_poses();
        Vec3 n;
        get_quat_axis0(&n, global_pose0.q);
        Vec3 b00;
        Vec3 b11;
        get_quat_axis0(&b00, global_pose0.q);
        get_quat_axis1(&b11, global_pose1.q);
        limit_angle(body0, body1, n, b00, b11, min_swing_angle, max_swing_angle, swing_limit_compliance, dt);
      }
    }
    if (type == Type::Spherical) {
      if (has_swing_limit) {
        update_global_poses();
        Vec3 a0;
        Vec3 a1;
        get_quat_axis0(&a0, global_pose0.q);
        get_quat_axis0(&a1, global_pose1.q);
        Vec3 n = glm::cross(a0, a1);
        if (glm::length(n) > (Float)0.0) {
          n = glm::normalize(n);
        }
        limit_angle(body0, body1, n, a0, a1, min_swing_angle, max_swing_angle, swing_limit_compliance, dt);
      }
      if (has_twist_limit) {
        update_global_poses();
        Vec3 n0;
        Vec3 n1;
        get_quat_axis0(&n0, global_pose0.q);
        get_quat_axis0(&n1, global_pose1.q);
        Vec3 n = n0 + n1;
        if (glm::length(n) > (Float)0.0) {
          n = glm::normalize(n);
        }
        Vec3 a0;
        get_quat_axis1(&a0, global_pose0.q);
        a0 += n * glm::dot(-n, a0);
        if (glm::length(a0) > (Float)0.0) {
          a0 = glm::normalize(a0);
        }
        Vec3 a1;
        get_quat_axis1(&a1, global_pose1.q);
        a1 += n * glm::dot(-n, a1);
        if (glm::length(a1) > (Float)0.0) {
          a1 = glm::normalize(a1);
        }
        Float max_corr = glm::dot(n0, n1) > (Float)-0.5 ? (Float)2.0 * PI_ : (Float)1.0 * dt;
        limit_angle(body0, body1, n, a0, a1, min_twist_angle, max_twist_angle, twist_limit_compliance, dt, max_corr);
      }
    }
    update_global_poses();     // position
    Vec3 corr = global_pose1.p - global_pose0.p;
    apply_body_pair_correction(body0, body1, corr, compliance, dt, &global_pose0.p, &global_pose1.p);
  }
  void solve_vel(Float dt) {
    if (rot_damping > 0.0) {
      Vec3 omega((Float)0.0, (Float)0.0, (Float)0.0);
      if (body0) {
        omega -= body0->omega;
      }
      if (body1) {
        omega += body1->omega;
      }
      omega *= std::min((Float)1.0, rot_damping * dt);
      apply_body_pair_correction(body0, body1, omega, (Float)0.0, dt, nullptr, nullptr, true);
    }
    if (pos_damping > 0.0) {
      update_global_poses();
      Vec3 vel(0.0f, 0.0f, 0.0f);
      if (body0) {
        vel -= body0->get_velocity_at(global_pose0.p);
      }
      if (body1) {
        vel += body1->get_velocity_at(global_pose1.p);
      }
      vel *= std::min((Float)1.0, pos_damping * dt);;
      apply_body_pair_correction(body0, body1, vel, (Float)0.0, dt, &global_pose0.p, &global_pose1.p, true);
    }
  }
};

void write_ppm(GLubyte* buff, GLenum format) {
  int w = glutGet(GLUT_WINDOW_WIDTH);
  int h = glutGet(GLUT_WINDOW_HEIGHT);
  int  pix_sz = (format == GL_RGBA) ? 4 : 1;
  char suffix[256];
  sprintf(suffix, (format == GL_RGBA) ? "screen.ppm" : "depth.ppm");
  char filename[1024];
  sprintf(filename, "%08d_%s", g_Context.frame, suffix);
  FILE *fp = fopen(filename, "wb");
  if (fp) {
    fprintf(fp, "P%d\n", (format == GL_RGBA) ? 6 : 5); // 5:Portable graymap(Binary), 6:Portable pixmap(Binary)
    fprintf(fp, "%u %u\n", w, h);
    fprintf(fp, "255\n");
    for(int y = 0; y < h; y++) {
      for(int x = 0; x < w; x++) {
        int index = (h - y - 1) * w * pix_sz + (x * pix_sz);
        if (format == GL_RGBA) {
          int r = buff[index];
          int g = buff[index + 1];
          int b = buff[index + 2];
          int a = buff[index + 3]; // not use here
          putc(r, fp); // binary
          putc(g, fp);
          putc(b, fp);
        } else {
          putc(buff[index], fp);
        }
      }
    }
    fclose(fp);
  }
}

void write_image(GLenum format) {
  GLsizei w = glutGet(GLUT_WINDOW_WIDTH);
  GLsizei h = glutGet(GLUT_WINDOW_HEIGHT);
  GLubyte* buff = (GLubyte*)malloc((size_t)w * (size_t)h * 4); // w x h * RGBA
  glReadBuffer(GL_FRONT/*GL_BACK*/);
  glReadPixels(0, 0, w, h, format, GL_UNSIGNED_BYTE, buff);
  write_ppm(buff, format);
  free(buff);
}

void render_string(std::string& str, int w, int h, GLfloat x0, GLfloat y0) {
  glDisable(GL_LIGHTING);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, w, h, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRasterPos2f(x0, y0);
  int size = (int)str.size();
  for(int i = 0; i < size; ++i){
    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
  }
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

void render_pipe(GLfloat width, GLfloat length, int slice, GLfloat color[]) {
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
  GLUquadricObj* q;
  q = gluNewQuadric();
  gluQuadricDrawStyle(q, GLU_FILL);
  gluQuadricNormals(q, GLU_SMOOTH);
  gluCylinder(q, width, width, length, slice, 1); // quad, base, top, height, slice, stacks
  glPushMatrix();
    glTranslatef(0.0f, 0.0f, length);
    gluDisk(q, 0.0f, width, slice, 1); // quad, inner, outer, slices, loops(top)
  glPopMatrix();
  gluQuadricOrientation(q, GLU_INSIDE);
  gluDisk(q, 0.0f, width, slice, 1); // quad, inner, outer, slices, loops(bottom)
  gluDeleteQuadric(q); 
}

void render_pipe(const glm::vec3& start, const glm::vec3& end, GLfloat width, int slice, GLfloat color[]) {
  glm::vec3 vec    = end - start;
  GLfloat   length = glm::length(vec);
  GLfloat   ax;
  if (fabs(vec.z) < FLT_EPSILON) {
    ax = 57.2957795f * acos( vec.x / length ); // rotation angle in x-y plane
    if (vec.y <= 0.0f)
      ax = -ax;
  } else {
    ax = 57.2957795f * acos( vec.z / length ); // rotation angle
    if (vec.z <= 0.0f)
      ax = -ax;
  }
  GLfloat rx = -vec.y * vec.z;
  GLfloat ry =  vec.x * vec.z;
  glPushMatrix();
    glTranslatef(start.x, start.y, start.z);
    if (fabs(vec.z) < FLT_EPSILON) {
      glRotatef(90.0f,  0.0f, 1.0f, 0.0f); // Rotate & align with x axis
      glRotatef(   ax, -1.0f, 0.0f, 0.0f); // Rotate to point 2 in x-y plane
    } else {
      glRotatef(   ax,   rx,    ry, 0.0f); // Rotate about rotation vector
    }
    render_pipe(width, length, slice, color);
  glPopMatrix();
}

void render_arrow(const glm::vec3& start, const glm::vec3& end, GLfloat width, int slice, GLfloat height, GLfloat color[]) {
  render_pipe(start, end, width, slice, color);
  glm::vec3 vec        = end - start;
  float     vec_length = glm::length(vec);
  if (vec_length > FLT_MIN) {
    static const glm::vec3 init(0.0f, 0.0f, 1.0f); // glutSolidCone() +z
    glm::vec3 normalized_vec = glm::normalize(vec);
    glm::vec3 diff = normalized_vec - init;
    if (glm::length(diff) > FLT_MIN) {
      glm::vec3 rot_axis  = glm::normalize(glm::cross(init, normalized_vec));
      float     rot_angle = std::acos(glm::dot(init, normalized_vec)) * 57.295f; // 360.0f / (2.0f * 3.14f)
      glm::vec3 cone_pos = end;
      if (vec_length > height){
        cone_pos = start + (vec_length - height) * normalized_vec; // offset cone height
        glPushMatrix();
          glTranslatef(cone_pos.x, cone_pos.y, cone_pos.z);
          glRotatef(rot_angle, rot_axis.x, rot_axis.y, rot_axis.z);
          glutSolidCone(height * 0.25, height, 4, 4); // base, height, slices, stacks
        glPopMatrix();
      }
    }
  }
}

void render_sphere(const glm::vec3& pos, float radius, int slices, GLfloat color[]) {
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
  glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(radius, slices, slices); // radius, slices, stacks
  glPopMatrix();
}

void set_material(const Material& mat) {
  glMaterialfv(GL_FRONT, GL_AMBIENT,   mat.ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR,  mat.specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, &mat.shininess);
}

void set_material(const Material& in_mat, float alpha) {
  Material mat = in_mat;
  mat.ambient[3]   = alpha;
  mat.diffuse[3]   = alpha;
  mat.specular[3]  = alpha;
  set_material(mat);
}

void render_sphere(const glm::vec3& pos, float radius, int slices) {
  glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(radius, slices, slices); // radius, slices, stacks
  glPopMatrix();
}

void render_floor(GLfloat w, GLfloat d, int num_w, int num_d) {
  static const GLfloat color[][4] = { { 0.6f, 0.6f, 0.6f, 1.0f },   // white
                                      { 0.3f, 0.3f, 0.3f, 1.0f } }; // gray
  GLfloat center_w = (w * num_w) / 2.0f;
  GLfloat center_d = (d * num_d) / 2.0f;
  glPushMatrix();
    glNormal3f(0.0f, 1.0f, 0.0f); // up vector
    glBegin(GL_QUADS);
    for (int j = 0; j < num_d; ++j) {
      GLfloat dj  = d  * j;
      GLfloat djd = dj + d;
      for (int i = 0; i < num_w; ++i) {
        GLfloat wi  = w  * i;
        GLfloat wiw = wi + w;
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color[(i + j) & 1]);
        glVertex3f(wi  - center_w,  0.0, dj  - center_d);
        glVertex3f(wi  - center_w,  0.0, djd - center_d);
        glVertex3f(wiw - center_w,  0.0, djd - center_d);
        glVertex3f(wiw - center_w,  0.0, dj  - center_d);
      }
    }
    glEnd();
  glPopMatrix();
}

void init_imgui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGLUT_Init();
  ImGui_ImplGLUT_InstallFuncs();
  ImGui_ImplOpenGL2_Init();
}

void create_objects() {
  static const Vec3      object_size(0.02f, 0.04f, 0.02f);
  static const Vec3      last_object_size(0.2f, 0.04f, 0.2f);
  Vec3      last_size = object_size;
  Body* last_body = nullptr;
  auto& ctx = g_Context;
  for(int i = 0; i < ctx.params.num_objs; i++) {
    Pose pose;
    pose.p.x = (Float)0.0;
    Float offset = (100 - ctx.params.num_objs) * object_size.y * (Float)1.4 + (Float)0.1;
    pose.p.y = ((ctx.params.num_objs * object_size.y + last_object_size.y) * (Float)1.4 + offset) - i * object_size.y;
    pose.p.z = (Float)0.0;
    Body* body      = new Body(pose);
    Vec3      sz = (i == ctx.params.num_objs - 1) ? last_object_size : object_size;
    body->set_box(sz);
    Vec3      z_axis(0.0f, 0.0f, 1.0f);
#if 0
    Vec3      x_axis(1.0f, 0.0f, 0.0f);
    Vec3      y_axis(0.0f, 1.0f, 0.0f);
    Quat      q = glm::angleAxis(glm::degrees((Float)45.0), z_axis);
    Vec3      euler = glm::eulerAngles(q);
    body->omega = euler;
#endif
    ctx.bodies.push_back(body);
    Pose joint_pose0;
    Pose joint_pose1;
    joint_pose0.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
    joint_pose1.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
    Float s = ((i % 2) == 0) ? (Float)-0.5 : (Float)0.5;
    joint_pose0.p = Vec3(s * sz.x,         (Float)0.5 * sz.y,        s * sz.z);
    joint_pose1.p = Vec3(s * last_size.x, (Float)-0.5 * last_size.y, s * last_size.z);
    if (!last_body) {
      joint_pose1 = joint_pose0;
      joint_pose1.p += pose.p;
    }
    Joint* joint = new Joint(/*Joint::Type::Spherical*/ctx.params.joint_type, body, last_body, joint_pose0, joint_pose1);
    joint->compliance = ctx.params.compliance;
    joint->rot_damping  = ctx.params.rot_damping;
    joint->pos_damping  = ctx.params.pos_damping;
    joint->has_swing_limit = ctx.params.has_swing_limit;       // swing
    joint->min_swing_angle = ctx.params.min_swing_angle * PI_;
    joint->max_swing_angle = ctx.params.max_swing_angle * PI_;
    joint->has_twist_limit = ctx.params.has_twist_limit;       // twist
    joint->min_twist_angle = ctx.params.min_twist_angle * PI_;
    joint->max_twist_angle = ctx.params.max_twist_angle * PI_;
    ctx.joints.push_back(joint);
    last_body = body;
    last_size = sz;
  }
}
void create_objects2() {
  static const Vec3 object_size((Float)0.02, (Float)0.04, (Float)0.02);
  Vec3  last_size = object_size;
  Body* last_body = nullptr;
  auto& ctx = g_Context;
  Vec3      z_axis((Float)0.0, (Float)0.0, (Float)1.0);
  for(int i = 0; i < ctx.params.num_objs; i++) {
    Pose pose;
    Float radian = ((Float)i / (Float)30.0) * (Float)2.0 * PI_;
    radian = (radian > (Float)2.0 * PI_) ? radian - (Float)2.0 * PI_ : radian;
    pose.p.x = (Float)cos(radian) * (Float)0.1;
    Float offset = (100 - ctx.params.num_objs) * object_size.y * (Float)1.4 + (Float)0.1;
    pose.p.y = ((ctx.params.num_objs * object_size.y + object_size.y) * (Float)1.4 + offset) - i * object_size.y;
    pose.p.z = (Float)sin(radian) * (Float)0.1;
    Body* body   = new Body(pose);
    Vec3      sz = object_size;
    body->set_box(sz);
    ctx.bodies.push_back(body);
  }
  last_body = nullptr;
  for(int i = 0; i < ctx.params.num_objs; i++) {
    Pose joint_pose0;
    Pose joint_pose1;
    Body* body      = ctx.bodies[i];
    Vec3  vec_half  = (i == 0) ? Vec3((Float)0.0, (Float)0.0, (Float)0.0) : (body->pose.p - last_body->pose.p) * (Float)0.5;
#if 0
    joint_pose0.q = prev_body->pose.q;
    joint_pose1.q = next_body->pose.q;
#else
    joint_pose0.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
    joint_pose1.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
#endif
    joint_pose0.p = -vec_half;
    joint_pose1.p = +vec_half;
    if (!last_body) {
      joint_pose1 = joint_pose0;
      joint_pose1.p += body->pose.p;
    }
    Joint* joint = new Joint(/*Joint::Type::Fixed*/Joint::Type::Spherical, body, last_body, joint_pose0, joint_pose1);
    joint->compliance  = ctx.params.compliance;
    joint->rot_damping = ctx.params.rot_damping;
    joint->pos_damping = ctx.params.pos_damping;
    joint->has_swing_limit = ctx.params.has_swing_limit;       // swing
    joint->min_swing_angle = ctx.params.min_swing_angle * PI_;
    joint->max_swing_angle = ctx.params.max_swing_angle * PI_;
    joint->has_twist_limit = ctx.params.has_twist_limit;       // twist
    joint->min_twist_angle = ctx.params.min_twist_angle * PI_;
    joint->max_twist_angle = ctx.params.max_twist_angle * PI_;
    ctx.joints.push_back(joint);
    last_body = body;
  }
  for(auto& body : ctx.bodies) {
    auto* sphere = new BoundingSphere();
    *sphere = body->sphere;
    ctx.spheres.push_back(sphere);
  }
}

void read_objects() {
  auto& ctx = g_Context;
  std::string s;
  for(auto& body : ctx.bodies) {
    std::getline(std::cin, s);
    glm::f64vec3 p;
    if (std::sscanf(s.c_str(), "dvec3(%lf, %lf, %lf)", &p.x, &p.y, &p.z) != 3) {
      std::sscanf(s.c_str(), "fvec3(%lf, %lf, %lf)",  &p.x, &p.y, &p.z);
    }
    body->pose.p = (Vec3)p;
    std::getline(std::cin, s);
    glm::dquat q;
    if (std::sscanf(s.c_str(), "dquat(%lf, {%lf, %lf, %lf})", &q.w, &q.x, &q.y, &q.z) != 4) {
      std::sscanf(s.c_str(), "fquat(%lf, {%lf, %lf, %lf})", &q.w, &q.x, &q.y, &q.z);
    }
    body->pose.q = (Quat)q;
  }
  for(auto& joint : ctx.joints) {
    if (joint != g_Context.grab_joint) {
      std::getline(std::cin, s);
      glm::f64vec3 p;
      if (std::sscanf(s.c_str(), "dvec3(%lf, %lf, %lf)", &p.x, &p.y, &p.z) !=3 ) {
        std::sscanf(s.c_str(), "fvec3(%lf, %lf, %lf)",  &p.x, &p.y, &p.z);
      }
      joint->local_pose0.p = (Vec3)p;
      std::getline(std::cin, s);
      glm::dquat q;
      if (std::sscanf(s.c_str(), "dquat(%lf, {%lf, %lf, %lf)", &q.w, &q.x, &q.y, &q.z) != 4) {
        std::sscanf(s.c_str(), "fquat(%lf, {%lf, %lf, %lf})", &q.w, &q.x, &q.y, &q.z);
      }
      joint->local_pose0.q = (Quat)q;
      std::getline(std::cin, s);
      if (std::sscanf(s.c_str(), "dvec3(%lf, %lf, %lf)", &p.x, &p.y, &p.z) !=3 ) {
        std::sscanf(s.c_str(), "fvec3(%lf, %lf, %lf)",  &p.x, &p.y, &p.z);
      }
      joint->local_pose1.p = (Vec3)p;
      std::getline(std::cin, s);
      if (std::sscanf(s.c_str(), "dquat(%lf, {%lf, %lf, %lf)", &q.w, &q.x, &q.y, &q.z) != 4) {
        std::sscanf(s.c_str(), "fquat(%lf, {%lf, %lf, %lf})", &q.w, &q.x, &q.y, &q.z);
      }
      joint->local_pose1.q = (Quat)q;
    }
  }
}

void write_objects() {
  auto& ctx = g_Context;
  for(auto& body : ctx.bodies) {
    std::cout << glm::to_string(body->pose.p) << std::endl;
    std::cout << glm::to_string(body->pose.q) << std::endl;
  }
  for(auto& joint : ctx.joints) {
    if (joint != g_Context.grab_joint) {
      std::cout << glm::to_string(joint->local_pose0.p) << std::endl;
      std::cout << glm::to_string(joint->local_pose0.q) << std::endl;
      std::cout << glm::to_string(joint->local_pose1.p) << std::endl;
      std::cout << glm::to_string(joint->local_pose1.q) << std::endl;
    }
  }
}

void destroy_objects() {
  auto& ctx = g_Context;
  int sphere_sz = (int)ctx.spheres.size();
  for(int i = 0; i < sphere_sz; i++) {
    if (ctx.spheres[i]) {
      delete ctx.spheres[i];
      ctx.spheres[i] = nullptr;
    }
  }
  int joint_sz = (int)ctx.joints.size();
  for(int i = 0; i < joint_sz; i++) {
    if (ctx.joints[i]) {
      delete ctx.joints[i];
      ctx.joints[i] = nullptr;
    }
  }
  int body_sz = (int)ctx.bodies.size();
  for(int i = 0; i < body_sz; i++) {
    if (ctx.bodies[i]) {
      delete ctx.bodies[i];
      ctx.bodies[i] = nullptr;
    }
  }
  ctx.spheres.clear();
  ctx.spheres.shrink_to_fit();
  ctx.joints.clear();
  ctx.joints.shrink_to_fit();
  ctx.bodies.clear();
  ctx.bodies.shrink_to_fit();
}

void finalize_imgui() {
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGLUT_Shutdown();
  ImGui::DestroyContext();
}

void finalize(void) {
  Context& ctx = g_Context;
  if (ctx.scene) {
      delete ctx.scene;
      ctx.scene = nullptr;
  }
  if (ctx.raycaster) {
    delete ctx.raycaster;
    ctx.raycaster = nullptr;
  }
  finalize_imgui();
  return;
}

void find_plane(SPlane* plane, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
  glm::vec3 vec0 = v1 - v0;
  glm::vec3 vec1 = v2 - v0;
  plane->v[0] =   vec0.y * vec1.z - vec0.z * vec1.y;
  plane->v[1] = -(vec0.x * vec1.z - vec0.z * vec1.x);
  plane->v[2] =   vec0.x * vec1.y - vec0.y * vec1.x;
  plane->v[3] = -(plane->v[0] * v0.x + plane->v[1] * v0.y + plane->v[2] * v0.z);
}

void calc_shadow_matrix(Shadow* shadow, const SPlane& plane, const Light& light) {
  GLfloat dot = glm::dot(plane.v, light.v);
  shadow->v[0][0] = dot - light.v[0] * plane.v[0];
  shadow->v[1][0] = 0.f - light.v[0] * plane.v[1];
  shadow->v[2][0] = 0.f - light.v[0] * plane.v[2];
  shadow->v[3][0] = 0.f - light.v[0] * plane.v[3];

  shadow->v[0][1] = 0.f - light.v[1] * plane.v[0];
  shadow->v[1][1] = dot - light.v[1] * plane.v[1];
  shadow->v[2][1] = 0.f - light.v[1] * plane.v[2];
  shadow->v[3][1] = 0.f - light.v[1] * plane.v[3];

  shadow->v[0][2] = 0.f - light.v[2] * plane.v[0];
  shadow->v[1][2] = 0.f - light.v[2] * plane.v[1];
  shadow->v[2][2] = dot - light.v[2] * plane.v[2];
  shadow->v[3][2] = 0.f - light.v[2] * plane.v[3];

  shadow->v[0][3] = 0.f - light.v[3] * plane.v[0];
  shadow->v[1][3] = 0.f - light.v[3] * plane.v[1];
  shadow->v[2][3] = 0.f - light.v[3] * plane.v[2];
  shadow->v[3][3] = dot - light.v[3] * plane.v[3];
}

void render_body(float alpha = 1.0f, GLenum mode = GL_RENDER, bool is_line = false, const Material& mat = mat_gold) {
  Context& ctx = g_Context;
  int body_sz = (int)ctx.bodies.size();
  for(int i = 0; i < body_sz; i++) {
    glm::vec3 pos = ctx.bodies[i]->pose.p;
    glm::vec3 scl = ctx.bodies[i]->size;
    glm::quat q(ctx.bodies[i]->pose.q);
    BoundingSphere& sphere = ctx.bodies[i]->sphere;
    if (mode == GL_SELECT) {
      glLoadName(i);
    }
    set_material(mat, alpha);
    glPushMatrix();
      glTranslatef(pos.x, pos.y, pos.z);
      glMultMatrixf(glm::value_ptr(glm::mat4_cast(q)));
      glScalef(scl.x, scl.y, scl.z);
      glutSolidCube(1.0f);
    glPopMatrix();
    if ((i == ctx.grab_idx) && (is_line)) { // add wire
      glEnable(GL_LIGHTING);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_POLYGON_OFFSET_FILL);
      glPolygonOffset(1.0f, 1.0f);
      set_material(mat_white, 1.0f);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glMultMatrixf(glm::value_ptr(glm::mat4_cast(q)));
        glScalef(scl.x, scl.y, scl.z);
        glutSolidCube(1.0f);
      glPopMatrix();
      glDisable(GL_POLYGON_OFFSET_FILL);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
  }
}

void render_joint() {
  auto& ctx = g_Context;
  for(auto& joint : ctx.joints) {
    glm::vec3 start = joint->global_pose0.p;
    glm::vec3 end   = joint->global_pose1.p;
    GLfloat yellow[]   = { 0.8f, 0.8f, 0.0f, 1.0f };
    render_pipe(start, end, 0.01f, 8, yellow);
  }
}

void render_rubber_band() {
  Context& ctx = g_Context;
  if (ctx.grab_joint) {
    glm::vec3 start = ctx.grab_joint->body0->pose.p;
    glm::vec3 end   = ctx.grab_joint->global_pose1.p;
    GLfloat white[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
    render_pipe(start, end, 0.005f, 8, white);
  }
}

void display_body(bool is_disp_barrier, Material& mat = mat_gold) {
  Context& ctx = g_Context;
  render_floor(1.0f, 1.0f, 24, 32); // actual floor

  glDisable(GL_DEPTH_TEST);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  glEnable(GL_STENCIL_TEST);
  glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
  glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
  render_floor(1.0f, 1.0f, 24, 32);       // floor pixels just get their stencil set to 1. 
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_DEPTH_TEST);

  glStencilFunc(GL_EQUAL, 1, 0xffffffff); // draw if ==1
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

  glDisable(GL_DEPTH_TEST);
  glCullFace(GL_FRONT);
  glPushMatrix();
    glScalef(1.0f, -1.0f, 1.0f); // for reflection on plane(y=0.0f)
    glLightfv(GL_LIGHT0, GL_POSITION, &g_Context.light.v[0]);
    render_body(0.25f, GL_RENDER); // reflection
  glPopMatrix();
  glLightfv(GL_LIGHT0, GL_POSITION, &g_Context.light.v[0]);
  glCullFace(GL_BACK);

  calc_shadow_matrix(&g_Context.floor_shadow, g_Context.floor, g_Context.light);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glDisable(GL_LIGHTING);        // force the 50% black
  glColor4f(0.0, 0.0, 0.0, 0.5);
  glPushMatrix();
    glMultMatrixf((GLfloat*)g_Context.floor_shadow.v);
    render_body(1.0f, GL_RENDER); // projected shadow
  glPopMatrix();
  glEnable(GL_LIGHTING);
  glDisable(GL_POLYGON_OFFSET_FILL);
  glDisable(GL_STENCIL_TEST);

  glEnable(GL_DEPTH_TEST);
  render_body(1.0f, GL_RENDER, true, mat);   // actual draw

  if (ctx.collision.show_bounding) { // bounding sphere
    for(auto& sphere : ctx.spheres) {
      glEnable(GL_LIGHTING);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_POLYGON_OFFSET_FILL);
      glPolygonOffset(1.0f, 1.0f);
      set_material(mat_white, 1.0f);
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glm::vec3 p((float)sphere->position->x, (float)sphere->position->y, (float)sphere->position->z);
      glPushMatrix();
        glTranslatef(p.x, p.y, p.z);
        glutSolidSphere(sphere->radius, 10, 10);
      glPopMatrix();
      glDisable(GL_POLYGON_OFFSET_FILL);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
  }

  if (is_disp_barrier) { // test for collision
    set_material(mat_chrome);
    render_sphere(barrier_pos, barrier_radius, 32);
  }

  if (ctx.params.show_joint) {
    glDisable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_joint();
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  if (1) {
    glDisable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_rubber_band();
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  if (ctx.collision.show_contact) {
    for(auto& pair : ctx.contacts) {
      glm::vec3 pos(pair.contact.position);
      set_material(mat_rubbr_black);
      render_sphere(pos, 0.01f, 32);
      GLfloat white[] = { 0.8f, 0.8f, 0.8f, 1.0f };
      glm::vec3 start = pos;
      glm::vec3 end   = start + glm::vec3(pair.contact.normal) * /*(float)pair.contact.distance*/5.0f;
      render_arrow(start, end,   0.005f, 8, 0.03f, white);
    }
    for(auto& pair : ctx.self_contacts) {
      glm::vec3 pos(pair.contact.position);
      set_material(mat_rubbr_black);
      render_sphere(pos, 0.01f, 32);
      GLfloat white[] = { 0.8f, 0.8f, 0.8f, 1.0f };
      glm::vec3 start = pos;
      glm::vec3 end   = start + glm::vec3(pair.contact.normal) * /*(float)pair.contact.distance*/5.0f;
      render_arrow(start, end,   0.005f, 8, 0.03f, white);
    }
    ctx.contacts.clear();
    ctx.contacts.shrink_to_fit();
    ctx.self_contacts.clear();
    ctx.self_contacts.shrink_to_fit();
  }
}

class SceneChain : public Scene {
public:
  SceneChain() {
    auto& ctx = g_Context;
    ctx.params.has_swing_limit = false;
    ctx.params.has_twist_limit = false;
    ctx.params.min_swing_angle = -2.0f;
    ctx.params.max_swing_angle = +2.0f;
    ctx.params.min_twist_angle = -2.0f;
    ctx.params.max_twist_angle = +2.0f;
    create_objects();
  }
  ~SceneChain() {
    destroy_objects();
  }
#if USE_DEBUG
  void calc_energy(Context& ctx) {
    Float k = 0;
    Float u = 0;
    for(int i = 0; i < g_Context.params.num_objs; i++) {
      k += ctx.bodies[i]->get_kinetic_energy(ctx.params.substeps);
      u += ctx.bodies[i]->get_potential_energy();
    }
//    printf("K=%8.20lf\n", k);
//    printf("U=%8.20lf\n", u);
    ctx.debug_info.energies.emplace_back(k, u);
  }
#endif
  virtual void Update(Context& ctx, Float dt) {
    Float   sdt       = dt / (Float)g_Context.params.substeps;
    int     bodies_sz = (int)g_Context.bodies.size();
    int     joints_sz = (int)g_Context.joints.size();
    for(int i = 0; i < g_Context.params.substeps; i++) {
      for(int j = 0; j < bodies_sz; j++) {
        g_Context.bodies[j]->integrate(sdt, gravity);
      }
      for(int j = 0; j < joints_sz; j++) {
        g_Context.joints[j]->solve_pos(sdt);
      }
      for(int j = 0; j < bodies_sz; j++) {
        g_Context.bodies[j]->update(sdt);
      }
      for(int j = 0; j < joints_sz; j++) {
        g_Context.joints[j]->solve_vel(sdt);
      }
    }
#if USE_DEBUG
    calc_energy(g_Context);
#endif
  }
  virtual void Render(float alpha = 1.0f) {
    display_body(false);
  }
};

class SceneTwistedRope : public Scene {
public:
  SceneTwistedRope() {
    Context& ctx = g_Context;
    ctx.params.has_swing_limit = true;
    ctx.params.has_twist_limit = true;
    ctx.params.min_swing_angle = 0.0f;
    ctx.params.max_swing_angle = 0.0f;
    ctx.params.min_twist_angle = 0.0f;
    ctx.params.max_twist_angle = 0.0f;
    create_objects2();
//    read_objects();

  }
  ~SceneTwistedRope() {
    destroy_objects();
  }
  void collect_collision_pairs() {
    auto& ctx = g_Context;
    ctx.contacts.clear();
    ctx.contacts.shrink_to_fit();
    ctx.self_contacts.clear();
    ctx.self_contacts.shrink_to_fit();
#if !(USE_OBB_COL)
    for(auto& sphere : ctx.spheres) {
      Float distance = glm::length(*(sphere->position) - barrier_pos);
      if (distance < barrier_radius) { // sphere vs sphere
        Vec3 n = glm::normalize(*(sphere->position) - barrier_pos);
        Float penetration = barrier_radius - distance;
        Contact contact;
        contact.position = *(sphere->position) - sphere->radius * n;
        contact.normal = n;
        contact.distance = penetration;
        BodyPairContact pair;
        pair.body0 = sphere->body;
        pair.body1 = nullptr;
        pair.contact = contact;
        ctx.contacts.push_back(pair);
      }
    }
    if (ctx.collision.use_self_contact) {
      for(auto& sphere0 : ctx.spheres) {// self collision
        for(auto& sphere1 : ctx.spheres) {
          if (sphere0 == sphere1){
            continue;
          }
          Float distance = glm::length(*(sphere0->position) - *(sphere1->position));
          Float radius2  = sphere0->radius + sphere1->radius - 0.004f;
          if (distance < radius2) {
            Vec3 n = glm::normalize(*(sphere0->position) - *(sphere1->position)); // normal( 1 -> 0 )
            Float penetration = radius2 - distance;
            Contact contact;
            contact.position = *(sphere0->position) + sphere1->radius * n;
            contact.normal = n;
            contact.distance = penetration * 0.5f;
            BodyPairContact pair;
            pair.body0 = sphere0->body;
            pair.body1 = sphere0->body;
            pair.contact = contact;
            ctx.self_contacts.push_back(pair);
          }
        }
      }
    }
#else
    for(auto& body : ctx.bodies) {
      OBB obb;
      obb.position    = body->pose.p;
      obb.size        = body->size * (Float)0.5; // half-size
      obb.orientation = glm::mat3_cast(body->pose.q);
      Sphere sphere;
      sphere.position = barrier_pos;
      sphere.radius   = barrier_radius;
      Vec3 closest_p;
      if (obb_sphere_intersection(&closest_p, obb, sphere)) {
        Vec3 n = glm::normalize(closest_p - barrier_pos);
        Contact contact; // for eq.26
        contact.position = closest_p;
        contact.normal   = n;
        contact.distance = glm::length(closest_p - barrier_pos);
        BodyPairContact pair;
        pair.body0   = body;
        pair.body1   = nullptr;
        pair.contact = contact;
        ctx.contacts.push_back(pair);
      }
    }
#endif
  }
  void solve_collision_pairs(Float dt, Float k = (Float)2.0) {
    auto& ctx = g_Context;
    for(auto& pair : ctx.contacts) {
#if USE_OBB_COL
      Vec3 r = pair.contact.position - pair.body0->pose.p; // eq.(26)
      pair.body0->pose.p      += apply_quaternion(r, pair.body0->pose.q) * dt;
      pair.body0->prev_pose.p += apply_quaternion(r, pair.body0->prev_pose.q) * dt * k; // todo use xpbd code
#else
      pair.body0->prev_pose = pair.body0->pose;
      pair.body0->pose.p += pair.contact.normal * pair.contact.distance * dt * k; // todo use xpbd code
#endif
    }
    for(auto& pair : ctx.self_contacts) {
#if USE_OBB_COL

#else
      pair.body0->prev_pose = pair.body0->pose;
      pair.body0->pose.p += pair.contact.normal * pair.contact.distance * dt * k; // todo use xpbd code
      pair.body1->prev_pose = pair.body1->pose;
      pair.body1->pose.p -= pair.contact.normal * pair.contact.distance * dt * k; // todo use xpbd code
#endif
    }
  }
  virtual void Update(Context& ctx, Float dt) {
    Float   sdt       = dt / (Float)ctx.params.substeps;
    int     bodies_sz = (int)ctx.bodies.size();
    int     joints_sz = (int)ctx.joints.size();
    if (ctx.collision.use_ground) {
      collect_collision_pairs(); // no substeps
    }
    for(int i = 0; i < ctx.params.substeps; i++) {
      for(int j = 0; j < bodies_sz; j++) {
        ctx.bodies[j]->integrate(sdt, gravity);
      }
      for(int j = 0; j < joints_sz; j++) {
        ctx.joints[j]->solve_pos(sdt);
      }
      if (ctx.collision.use_ground) {
        solve_collision_pairs(sdt, ctx.collision.safety_multiplier); // TODO apply XPBD
      }
      for(int j = 0; j < bodies_sz; j++) {
        ctx.bodies[j]->update(sdt);
      }
      for(int j = 0; j < joints_sz; j++) {
        ctx.joints[j]->solve_vel(sdt);
      }
    }
  }
  virtual void Render(float alpha = 1.0f) {
    auto& ctx = g_Context;
    display_body(ctx.collision.use_ground, mat_bronze);
//    ctx.contacts.clear();
//    ctx.contacts.shrink_to_fit();
  }
};

class SceneKnot : public Scene {
private:
  void set_default_param(Joint* joint, Context& ctx) {
    joint->compliance  = ctx.params.compliance;
    joint->rot_damping = ctx.params.rot_damping;
    joint->pos_damping = ctx.params.pos_damping;
    joint->has_swing_limit = ctx.params.has_swing_limit;       // swing
    joint->min_swing_angle = ctx.params.min_swing_angle * PI_;
    joint->max_swing_angle = ctx.params.max_swing_angle * PI_;
    joint->has_twist_limit = ctx.params.has_twist_limit;       // twist
    joint->min_twist_angle = ctx.params.min_twist_angle * PI_;
    joint->max_twist_angle = ctx.params.max_twist_angle * PI_;
  }
  void create_knot_objects() {
    auto& ctx = g_Context;
    static const Vec3 object_size(0.02f, 0.04f, 0.02f);
    int subdiv_num = ctx.params.num_objs;
    subdiv_num = (subdiv_num < 50) ? 50 : subdiv_num;
    int subdiv_num_half = subdiv_num / 2;
    Vec3  z_axis((Float)0.0, (Float)0.0, (Float)1.0);
    for(int i = 0; i < subdiv_num; i++) {
      Pose pose;
      pose.p.x = (Float)(i - subdiv_num_half) * (Float)0.02;      // [-1.0, +1.0]
      //pose.p.y = (pose.p.x * pose.p.x) * (Float)6.0 + (Float)3.0; // y = ax^2
      pose.p.y = 4.0f;
      pose.p.z = (Float)0.0;
      Body* body = new Body(pose);
      //Float density = ((i < 5) || (i > subdiv_num - 5)) ? (Float)10000000000000.0 : (Float)0.0000001;
      body->set_box(object_size/*, density*/);
#if 0
      Vec3  y_axis(0.0f, 1.0f, 0.0f);
      Float angle = (subdiv_num < subdiv_num_half) ? (Float)45.0 : (Float)-45.0;
      Quat q = glm::angleAxis(glm::degrees(angle), y_axis);
      Vec3 euler = glm::eulerAngles(q);
      body->omega = euler; // initial angular velocity
#endif
      ctx.bodies.push_back(body);
    }
    Body* last_body = nullptr;
    Pose joint_pose0;
    Pose joint_pose1;
    for(int i = 0; i < subdiv_num; i++) {
      Body* body      = ctx.bodies[i];
      Vec3  vec_half  = (i == 0) ? Vec3((Float)0.0, (Float)0.0, (Float)0.0) : (body->pose.p - last_body->pose.p) * (Float)0.5;
      joint_pose0.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
      joint_pose1.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
      joint_pose0.p = -vec_half;
      joint_pose1.p = +vec_half;
      if (!last_body) { // first
        joint_pose1 = joint_pose0;
        joint_pose1.p += body->pose.p;
      }
      Joint* joint = new Joint(/*Joint::Type::Fixed*/Joint::Type::Spherical, body, last_body, joint_pose0, joint_pose1);
      set_default_param(joint, ctx);
      ctx.joints.push_back(joint);
      last_body = body;
    }
    joint_pose0.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
    joint_pose1.q = glm::angleAxis((Float)0.5 * (Float)PI_, z_axis);
    joint_pose0.p = Vec3((Float)0.0, (Float)0.0, (Float)0.0);
    joint_pose1.p = Vec3((Float)0.0, (Float)0.0, (Float)0.0);
    joint_pose0.p = last_body->pose.p;
    Joint* joint = new Joint(Joint::Type::Spherical, nullptr, last_body, joint_pose0, joint_pose1);
    set_default_param(joint, ctx);
    ctx.joints.push_back(joint);
  }
public:
  SceneKnot() {
    Context& ctx = g_Context;
    ctx.params.substeps        = 40;
    ctx.params.compliance      = 0.0f;
    ctx.params.has_swing_limit = true;
    ctx.params.min_swing_angle = -0.0f;
    ctx.params.max_swing_angle = +0.0f;
    ctx.params.has_twist_limit = true;
    ctx.params.min_twist_angle = -0.0f;
    ctx.params.max_twist_angle = +0.0f;
    create_knot_objects();
  }
  ~SceneKnot() {
    destroy_objects();
  }
  virtual void Update(Context& ctx, Float dt) {
    Float sdt       = dt / (Float)ctx.params.substeps;
    int   bodies_sz = (int)ctx.bodies.size();
    int   joints_sz = (int)ctx.joints.size();
#if 0
    int subdiv_num = ctx.params.num_objs;
    subdiv_num = (subdiv_num < 50) ? 50 : subdiv_num;
    for(int i = 0; i < subdiv_num; i++) {
      if ((i < 5) || (i > (subdiv_num - 5))) {
        auto& joint = ctx.joints[i];
        joint->has_twist_limit = true; // edge off
        joint->min_twist_angle = -2.0f * PI_;
        joint->max_twist_angle = +2.0f * PI_;
      }
    }
#endif
#if 1
    Float radian = ((Float)(ctx.frame % 660) / (Float)660.0) * (Float)2.0 * PI_;
    auto& joint = ctx.joints[0]; // left most
    joint->local_pose1.p.x = (Float)-1.0 + (Float)sin(radian) * (Float)1.5;
    joint->local_pose1.p.y = 4.0f;
    joint->local_pose1.p.z = 0.0f;
    Float angle = ((Float)(ctx.frame % 600) / (Float)600.0) * (Float)360.0;
    Vec3  y_axis((Float)0.0, (Float)1.0, (Float)0.0);
    joint->local_pose1.q = glm::angleAxis(angle, y_axis);
#endif
    if (ctx.collision.use_ground) {
//      collect_collision_pairs(); // no substeps
    }
    for(int i = 0; i < ctx.params.substeps; i++) {
      for(int j = 0; j < bodies_sz; j++) {
        ctx.bodies[j]->integrate(sdt, gravity);
      }
      for(int j = 0; j < joints_sz; j++) {
        ctx.joints[j]->solve_pos(sdt);
      }
      if (ctx.collision.use_ground) {
//        solve_collision_pairs(sdt, ctx.collision.safety_multiplier); // TODO apply XPBD
      }
      for(int j = 0; j < bodies_sz; j++) {
        ctx.bodies[j]->update(sdt);
      }
      for(int j = 0; j < joints_sz; j++) {
        ctx.joints[j]->solve_vel(sdt);
      }
    }
  }
  virtual void Render(float alpha = 1.0f) {
    auto& ctx = g_Context;
    display_body(false, mat_rubbr_red);
  }
};

void initialize(int argc, char* argv[]) {
#if defined(WIN32)
  _CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif
  Context& ctx = g_Context;
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glClearAccum(0.0f, 0.0f, 0.0f, 0.0f); 
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glEnable(GL_DEPTH_TEST);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  //glEnable(GL_LIGHT1);

  GLfloat ambient[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lmodel_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
  GLfloat local_view[] = { 0.0 };

  glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, specular);

  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);

//  GLfloat light0pos[] = { 0.0, 5.0, 10.0, 1.0 };
  GLfloat light0pos[] = { 15.0, 15.0, 15.0, 1.0 };
//  static const GLfloat light1pos[] = { 5.0, 3.0, 0.0, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
  ctx.light.v[0] = light0pos[0];
  ctx.light.v[1] = light0pos[1];
  ctx.light.v[2] = light0pos[2];
  ctx.light.v[3] = light0pos[3];

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  //glEnable(GL_POLYGON_SMOOTH);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_AUTO_NORMAL);
  glEnable(GL_NORMALIZE);

  init_imgui();
  atexit(finalize);

  glm::vec3 v0(-1.0f, 0.0f,  0.0f);
  glm::vec3 v1(+1.0f, 0.0f,  0.0f);
  glm::vec3 v2(+1.0f, 0.0f, -1.0f);
  find_plane(&ctx.floor, v0, v1, v2);

  ctx.raycaster = new Raycaster();
}

void restart() {
  if (g_Context.scene) {
    delete g_Context.scene;
    g_Context.scene = nullptr;
  }
}

void time_step(GLfloat time);

void display_imgui() {
  auto& ctx = g_Context;
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();

  {
    ImGui::SetNextWindowPos(ImVec2(  10,  10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(270, 130), ImGuiCond_FirstUseEver);
    ImGui::Begin("Debug");
    ImGui::Checkbox("Show Depth",   &ctx.debug_info.show_depth);
    ImGui::SliderFloat("DoF",       &ctx.debug_info.dof,     0.0f,  0.2f);
    ImGui::SliderFloat("focus",     &ctx.debug_info.focus, - 5.0f,  3.5f);
    ImGui::End();
    ImGui::Begin("Params");
    if (ImGui::Button("Restart")) {
      restart();
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
      ctx.paused = true;
      time_step((float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f);
    }
    ImGui::SameLine();
    if (ImGui::Button("Run")) {
      ctx.paused = false;
    }
    if (ImGui::SliderInt("Num Objs", &ctx.params.num_objs, 2, 100)) {
      restart();
    }
    if (ImGui::SliderFloat("Compliance", &ctx.params.compliance, 0.0f, 0.5f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->compliance = ctx.params.compliance;
        }
      }
    }
    ImGui::SliderInt("Substeps", &ctx.params.substeps, 30, 400);
    if (ImGui::SliderFloat("Rot Damp", &ctx.params.rot_damping, 0.0f, 1000000.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->rot_damping = ctx.params.rot_damping;
        }
      }
    }
    if (ImGui::SliderFloat("Pos Damp", &ctx.params.pos_damping, 0.0f, 1000000.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->pos_damping = ctx.params.pos_damping;
        }
      }
    }
    if (ImGui::Combo("Joint", &ctx.params.joint_type, "Spherical\0Hinge\0Fixed\0\0")) {
      restart();
    }
    if (ImGui::Checkbox("Swing Limit", &ctx.params.has_swing_limit)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->has_swing_limit = ctx.params.has_swing_limit;
        }
      }
    }
    if (ImGui::SliderFloat("SLimit Comp", &ctx.params.swing_limit_compliance, 0.0f, 0.5f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->swing_limit_compliance = ctx.params.swing_limit_compliance;
        }
      }
    }
    if (ImGui::SliderFloat("Min SAngle", &ctx.params.min_swing_angle, -2.0f, 0.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->min_swing_angle = ctx.params.min_swing_angle * PI_;
        }
      }
    }
    if (ImGui::SliderFloat("Max SAngle", &ctx.params.max_swing_angle,  0.0f, +2.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->max_swing_angle = ctx.params.max_swing_angle * PI_;
        }
      }
    }
    if (ImGui::Checkbox("Twist Limit", &ctx.params.has_twist_limit)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->has_twist_limit = ctx.params.has_twist_limit;
        }
      }
    }
    if (ImGui::SliderFloat("TLimit Comp", &ctx.params.twist_limit_compliance, 0.0f, 0.5f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->twist_limit_compliance = ctx.params.twist_limit_compliance;
        }
      }
    }
    if (ImGui::SliderFloat("Min TAngle", &ctx.params.min_twist_angle, -2.0f, 0.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->min_twist_angle = ctx.params.min_twist_angle * PI_;
        }
      }
    }
    if (ImGui::SliderFloat("Max TAngle", &ctx.params.max_twist_angle,  0.0f, +2.0f)) {
      for(auto& joint : ctx.joints) {
        if (joint != ctx.grab_joint) {
          joint->max_twist_angle = ctx.params.max_twist_angle * PI_;
        }
      }
    }
    ImGui::Checkbox("Show Joint", &ctx.params.show_joint);
    ImGui::SameLine();
    ImGui::Checkbox("Show Force", &ctx.params.show_force);
    ImGui::End();
    ImGui::Begin("Camera");
    if (ImGui::Button("Reset")) {
      ctx.camera.reset();
    }
    ImGui::SliderFloat("Pos X", &ctx.camera.pos.x, cam_clamp_pos_min_x, cam_clamp_pos_max_x);
    ImGui::SliderFloat("Pos Y", &ctx.camera.pos.y, cam_clamp_pos_min_y, cam_clamp_pos_max_y);
    ImGui::SliderFloat("Pos Z", &ctx.camera.pos.z, cam_clamp_pos_min_z, cam_clamp_pos_max_z);
    ImGui::SliderFloat("Tgt X", &ctx.camera.tgt.x, cam_clamp_tgt_min_x, cam_clamp_tgt_max_x);
    ImGui::SliderFloat("Tgt Y", &ctx.camera.tgt.y, cam_clamp_tgt_min_y, cam_clamp_tgt_max_y);
    ImGui::End();

    ImGui::Begin("Collision");
    ImGui::Checkbox("Use Ground",   &ctx.collision.use_ground);
    ImGui::Checkbox("Use Self Contact", &ctx.collision.use_self_contact);
    ImGui::Checkbox("Show Contact", &ctx.collision.show_contact);
    ImGui::Checkbox("Show Bounding", &ctx.collision.show_bounding);
    ImGui::SliderFloat("Safety K", &ctx.collision.safety_multiplier, 1.0f, 40.0f);
    ImGui::End();
#if 0
    std::vector<Energy>& energies = ctx.debug_info.energies;
    int sz = (int)energies.size();
    if (sz > 0) {
      ImGui::Begin("Energy");
      std::vector<float> K, U, E;
      for (const auto& e : energies) {
        K.push_back((float)e.k);
        U.push_back((float)e.u);
        E.push_back((float)(e.k + e.u));
      }
      ImGui::PlotLines("K / Time", &K[0], sz);
      ImGui::PlotLines("U / Time", &U[0], sz);
      ImGui::PlotLines("E / Time", &E[0], sz);
      ImGui::End();
    }
#endif
  }

  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

void display_axis() {
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glm::vec3 left( -5.0f, 0.1f, 0.0f);
  glm::vec3 right(+5.0f, 0.1f, 0.0f);
  glm::vec3 bottm( 0.0f, -3.0f, 0.0f);
  glm::vec3 top(   0.0f, +3.0f, 0.0f);
  glm::vec3 back(  0.0f, 0.1f, -3.0f);
  glm::vec3 front( 0.0f, 0.1f, +3.0f);
  GLfloat red[]   = { 0.8f, 0.0f, 0.0f, 1.0f };
  GLfloat green[] = { 0.0f, 0.8f, 0.0f, 1.0f };
  GLfloat blue[]  = { 0.0f, 0.0f, 0.8f, 1.0f };
  render_arrow(left,  right, 0.01f, 8, 0.3f, red);
  render_arrow(bottm, top,   0.01f, 8, 0.3f, green);
  render_arrow(back,  front, 0.01f, 8, 0.3f, blue);
}

void display_string() {
  auto& ctx = g_Context;
  if (!ctx.params.show_force) {
    return;
  }
  glColor3d(1.0f, 1.0f, 1.0f);
  char debug[128];
  for(auto& joint : ctx.joints) {
    glm::f64vec3 win;
    glm::f64vec3 obj(joint->global_pose1.p);
    gluProject(obj.x, obj.y, obj.z, g_Context.modelview_mtx, g_Context.proj_mtx,  g_Context.vp, &win.x, &win.y, &win.z);
    float realy = (float)g_Context.vp[3] - (float)win.y - 1.0f;
    sprintf(debug, "  f=%1.0fN", std::floor(-joint->force));
    std::string time_text(debug);
    render_string(time_text, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT), (float)win.x, realy);
  }
}

void display_depth() {
  if (g_Context.debug_info.show_depth == false) { return; }
  GLint view[4];
  GLubyte *buffer;
  glGetIntegerv(GL_VIEWPORT, view);
  buffer = (GLubyte *)malloc(size_t(view[2]) * size_t(view[3]));
  glReadPixels(view[0], view[1], view[2], view[3], GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, buffer);
  glDisable(GL_DEPTH_TEST);
    glDrawPixels(view[2], view[3], GL_LUMINANCE, GL_UNSIGNED_BYTE, buffer);
  glEnable(GL_DEPTH_TEST);
  free(buffer);
}

void display_actor(float alpha = 1.0f) {
  Material mat[] = {
    mat_emerald, mat_jade, mat_obsidian, mat_pearl, mat_ruby, mat_turquoise, mat_brass, mat_bronze
  };
  for(int i = 0; i < 8; i++) {
    set_material(mat[i], alpha);
    glPushMatrix();
      glTranslatef(-6.0f + 1.0f * (float)i, 0.4f, -3.0f + 2.0f * (float)i);
      float ang = (float)(g_Context.frame % 120) * 3.0f;
      glRotatef(ang, 0.0f, 1.0f, 0.0f);
      glutSolidTeapot(0.5f);
    glPopMatrix();
  }
}

glm::vec3 calc_closest_box_vtx(int idx, const glm::vec3& begin, const glm::vec3& end) {
  glm::vec3 pos = g_Context.bodies[idx]->pose.p;
  glm::vec3 scl = g_Context.bodies[idx]->size;
  glm::quat q(g_Context.bodies[idx]->pose.q);
  float     length = FLT_MAX;
  glm::vec3 ret = pos; // center pos
  for(int i = 0; i < 8; i++) {
    glm::vec4 trans = glm::mat4_cast(q) * glm::scale(scl) * box_vtx[i];
    trans.xyz() += pos;
    glm::vec3 v(trans);
    glm::vec3 p = glm::closestPointOnLine(v, begin, end);
    float l = glm::length(v - p);
    if (l < length) {
      length = l;
      ret    = p; // update closest point
    }
  }
  return ret;
}

void display(void){
  glClear(GL_ACCUM_BUFFER_BIT);
  int   num_accum = 8;
  struct jitter_point{ GLfloat x, y; };
  jitter_point j8[] = {
    {-0.334818f,  0.435331f},
    { 0.286438f, -0.393495f},
    { 0.459462f,  0.141540f},
    {-0.414498f, -0.192829f},
    {-0.183790f,  0.082102f},
    {-0.079263f, -0.317383f},
    { 0.102254f,  0.299133f},
    { 0.164216f, -0.054399f}
  };
  GLint viewport[4];
  glGetIntegerv (GL_VIEWPORT, viewport);
  for(int i = 0 ; i < num_accum; i++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Context& ctx = g_Context;
    Vec3 pos = ctx.camera.pos;
    Float eye_jitter = (pos.z - ctx.debug_info.focus) / pos.z;
    eye_jitter = (eye_jitter < 0.1f) ? 0.1f : eye_jitter;
    pos.x += ctx.debug_info.dof * j8[i].x * eye_jitter;
    pos.y += ctx.debug_info.dof * j8[i].y * eye_jitter;
    Vec3 tgt = ctx.camera.tgt;
    Vec3 vec = tgt - pos;
    tgt.y = pos.y + vec.y * ((pos.z - ctx.debug_info.focus) / pos.z);
    tgt.z = ctx.debug_info.focus;
    gluLookAt(pos.x, pos.y, pos.z, tgt.x, tgt.y, tgt.z, 0.0, 1.0, 0.0); // pos, tgt, up
#if USE_TEST_SCENE
    display_axis();

    render_floor(1.0f, 1.0f, 16, 24);

    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_STENCIL_TEST);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
    render_floor(1.0f, 1.0f, 16, 24);       // floor pixels just get their stencil set to 1. 
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glEnable(GL_DEPTH_TEST);

    glStencilFunc(GL_EQUAL, 1, 0xffffffff); // draw if ==1
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

    glDisable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);
    glPushMatrix();
      glScalef(1.0f, -1.0f, 1.0f); // for reflection on plane(y=0.0f)
      glLightfv(GL_LIGHT0, GL_POSITION, &g_Context.light.v[0]);
      display_actor(0.1f);
    glPopMatrix();
    glCullFace(GL_FRONT);
    glLightfv(GL_LIGHT0, GL_POSITION, &g_Context.light.v[0]);

    glEnable(GL_POLYGON_OFFSET_FILL);
      calc_shadow_matrix(&g_Context.floor_shadow, g_Context.floor, g_Context.light);
      glDisable(GL_LIGHTING);        // force the 50% black
      glColor4f(0.0, 0.0, 0.0, 0.5f);
      glPushMatrix();
        glMultMatrixf((GLfloat*)g_Context.floor_shadow.v);
        glCullFace(GL_FRONT);
        display_actor();
        glCullFace(GL_BACK);
      glPopMatrix();
      glEnable(GL_LIGHTING);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_STENCIL_TEST);

    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_FRONT);
    display_actor();               // actual draw
    glCullFace(GL_BACK);
#else
    if (ctx.scene) {
      ctx.scene->Render();
    }
#endif
    glAccum(GL_ACCUM, 1.0f / num_accum);
  }
  glAccum(GL_RETURN, 1.0f);

  glGetDoublev(GL_MODELVIEW_MATRIX,  g_Context.modelview_mtx); // store current matrix
  glGetDoublev(GL_PROJECTION_MATRIX, g_Context.proj_mtx);

  display_string();
  display_depth();
  display_imgui();

  glutSwapBuffers();
  glutPostRedisplay();
}

void reshape_imgui(int width, int height) {
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize.x = (float)width;
  io.DisplaySize.y = (float)height;
}

void reshape(int width, int height){
//  static const GLfloat light0pos[] = { 0.0, 5.0, 10.0, 1.0 };
//  static const GLfloat light1pos[] = { 5.0, 3.0, 0.0, 1.0 };
  glShadeModel(GL_SMOOTH);

  reshape_imgui(width, height);
  glViewport(0, 0, width, height);
  g_Context.window_w = width;
  g_Context.window_h = height;
  glGetIntegerv(GL_VIEWPORT, g_Context.vp);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(70.0, (double)width / (double)height, 0.01f, 100.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

//  gluLookAt(0.0, 1.6, 15.0f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // pos, tgt, up
//  glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
//  glLightfv(GL_LIGHT1, GL_POSITION, light1pos);
}

void keyboard(unsigned char key, int x , int y){
  switch(key) {
  case 's': write_image(GL_RGBA); /*write_objects();*/ break;
  case 'd': write_image(GL_DEPTH_COMPONENT); break;
  case 'r': restart(); break;
  case 27: exit(0); break; // esc
  }
}

void special(int key, int x, int y) {
  auto& ctx = g_Context;
  int previous = ctx.scene_num;
#if USE_TW_SCENE
  switch(key) {
  case GLUT_KEY_LEFT:  ctx.scene_num--; break;
  case GLUT_KEY_RIGHT: ctx.scene_num++; break;
  }
#endif
  ctx.scene_num = (ctx.scene_num <  0)           ? Scene::eChain_Of_100_Bodies : ctx.scene_num;
  ctx.scene_num = (ctx.scene_num == Scene::eNum) ? Scene::eNum - 1             : ctx.scene_num;
  if (previous != ctx.scene_num) {
    if (ctx.scene){
      delete ctx.scene;
      ctx.scene = nullptr;
    }
  }
}

void time_step(GLfloat time) {
  auto& ctx  = g_Context;
  GLfloat dt = (GLfloat)fixed_dt;//time - ctx.time;
  ctx.scene->Update(ctx, dt);
  //ctx.time = time;
#if USE_CAPTURE
  keyboard('s', 0, 0); // screenshot
#endif
  while(1) {
    if (((float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f - time) > fixed_dt) {
      break; // keep 60fps
    }
  }
  ctx.frame++;
}

void idle(void){
  GLfloat time = (float)glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
  auto&   ctx  = g_Context;
  if (ctx.scene == nullptr) {
    switch(g_Context.scene_num) {
    case Scene::eChain_Of_100_Bodies: ctx.scene = new SceneChain();       break;
    case Scene::eTwistedRope:         ctx.scene = new SceneTwistedRope(); break;
    case Scene::eKnot:                ctx.scene = new SceneKnot();        break;
    }
  }
  if (ctx.paused == false) {
    time_step(time);
  }
}

GLint do_select(GLint x, GLint y){
  GLuint select_buf[100];
  glSelectBuffer(100, select_buf);
  glRenderMode(GL_SELECT);
  glInitNames();
  glPushName(~0);
  glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPickMatrix(x, g_Context.window_h - y, 4, 4, g_Context.vp);
    gluPerspective(70.0, (double)g_Context.window_w / (double)g_Context.window_h, 0.01f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT);
    render_body(1.0f, GL_SELECT); // select
  glPopMatrix();
  GLint hits = glRenderMode(GL_RENDER);
  if (hits <= 0) {
    return -1;
  }
  return select_buf[(hits - 1) * 4 + 3];
}

void start_grab(int x, int y) {
  Context& ctx = g_Context;
  ctx.grab_idx = do_select((GLint)x, (GLint)y);
  reshape(ctx.window_w, ctx.window_h);
  if (ctx.grab_idx >= 0) {
    ctx.raycaster->set_from_camera(x, y, true, &ctx.grab_depth);
    glm::vec3 pos = ctx.bodies[ctx.grab_idx]->pose.p; // translate
    glm::quat q(ctx.bodies[ctx.grab_idx]->pose.q);    // rotate
    glm::mat4 mtx = glm::translate(glm::mat4(1.0f), glm::vec3(pos)) * glm::mat4_cast(q);
    Vec3 hit;
    ctx.grab_distance = ctx.raycaster->intersect_objects(&hit, mtx);
    if (ctx.grab_distance > 0.0f) {
      ctx.grab_point = hit;
      Body* body = ctx.bodies[ctx.grab_idx];
      Pose pose1;
      pose1.p = hit;
      body->pose.inv_transform(&hit);
      Pose pose0;
      pose0.p = hit;
      ctx.grab_joint = new Joint(Joint::Type::Spherical, body, nullptr, pose0, pose1);
      ctx.grab_joint->compliance = 10.0f;
      ctx.grab_joint->update_global_poses();
      ctx.joints.push_back(ctx.grab_joint);
    }
  }
}

void move_grabbed(int x, int y) {
  Context& ctx = g_Context;
  if (ctx.grab_joint == nullptr)  { return; }
  ctx.raycaster->set_from_camera(x, y, false, &ctx.grab_depth);
  ctx.grab_joint->local_pose1.p = ctx.raycaster->ray.origin + ctx.raycaster->ray.direction * ctx.grab_distance;
}

void end_grab() {
  Context& ctx = g_Context;
  if (ctx.grab_idx >= 0) {
    if (ctx.grab_joint) {
      ctx.joints.pop_back();
      delete ctx.grab_joint;
      ctx.grab_joint = nullptr;
      ctx.grab_idx = -1;
    }
  }
}

void mouse_left_sub(int state, int x, int y) {
  Context& ctx = g_Context;
  switch(state) {
  case GLUT_DOWN: start_grab(x, y); break;
  case GLUT_UP:   end_grab();       break;
  }
}

void mouse_middle_sub(int state, int x, int y) {
  Context& ctx = g_Context;
  ctx.camera.is_zoom = (state == GLUT_DOWN) ? true : false;
  if (ctx.camera.is_zoom) {
    ctx.camera.zoom_start.x = x;
    ctx.camera.zoom_start.y = y;
  }
}

void mouse_right_sub(int state, int x, int y) {
  Context& ctx = g_Context;
  ctx.camera.is_move = (state == GLUT_DOWN) ? true : false;
  if (ctx.camera.is_move) {
    ctx.camera.move_start.x = x;
    ctx.camera.move_start.y = y;
  }
}

void mouse(int button, int state, int x, int y ) {
  Context& ctx = g_Context;
  switch(button) { // only handle button here
  case GLUT_LEFT_BUTTON:   mouse_left_sub  (state, x, y); break;
  case GLUT_MIDDLE_BUTTON: mouse_middle_sub(state, x, y); break;
  case GLUT_RIGHT_BUTTON:  mouse_right_sub (state, x, y); break;
  }
}

void motion_cam_sub(int x, int y) {
  Context& ctx = g_Context;
  if (ctx.camera.is_zoom) {
    ctx.camera.handle_zoom((float)(y - ctx.camera.zoom_start.y) * 0.1f);
    ctx.camera.zoom_start.x = x;
    ctx.camera.zoom_start.y = y;
  }
  if (ctx.camera.is_move) {
    ctx.camera.handle_motion(x - ctx.camera.move_start.x, y - ctx.camera.move_start.y);
    ctx.camera.clamp();
    ctx.camera.move_start.x = x;
    ctx.camera.move_start.y = y;
  }
}

void motion(int x, int y) {
  Context& ctx = g_Context;
  if (ctx.grab_joint) {
    move_grabbed(x, y);
  }
  motion_cam_sub(x, y);
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
#ifdef __FREEGLUT_EXT_H__
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
  //glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_ACCUM | GLUT_STENCIL);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_SINGLE | GLUT_ACCUM | GLUT_STENCIL);

  glutInitWindowSize(640, 480);
  glutCreateWindow("Detailed Rigid Body Simulation with Extended Position Based Dynamics");

  initialize(argc, argv);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);

  //glutMouseFunc(mouse);     // ImGui_ImplGLUT_MouseFunc
  //glutMotionFunc(motion);   // ImGui_ImplGLUT_MotionFunc
  //glutKeyboardFunc(keyboard); // ImGui_ImplGLUT_KeyboardFunc

  glutMainLoop();
  return 0;
}
