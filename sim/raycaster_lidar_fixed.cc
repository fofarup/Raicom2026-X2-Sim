// Project-owned minimal MuJoCo lidar sensor for the Raicom simulator.
// It intentionally implements only the pos_w point-cloud mode used by this
// project, and restores publisher metadata in reset().
#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Lidar {
  int sensor_id = -1;
  int camera_id = -1;
  int camera_body_id = -1;
  int horizontal = 0;
  int vertical = 0;
  double fov_h = 0.0;
  double fov_v = 0.0;
  double min_distance = 0.08;
  double max_distance = 8.0;
  int update_period = 1;
  int step = 0;
  std::array<int, 3> object_body_ids{{-1, -1, -1}};
  std::array<int, 4> claw_geom_ids{{-1, -1, -1, -1}};
};

std::vector<double> Parse(const char* text) {
  std::vector<double> values;
  std::stringstream stream(text ? text : "");
  double value;
  while (stream >> value) values.push_back(value);
  return values;
}

int FindSensor(const mjModel* model, int instance) {
  for (int id = 0; id < model->nsensor; ++id) {
    if (model->sensor_type[id] == mjSENS_PLUGIN &&
        model->sensor_plugin[id] == instance) {
      return id;
    }
  }
  return -1;
}

int RootBody(const mjModel* model, int body) {
  while (body > 0 && model->body_parentid[body] > 0) {
    body = model->body_parentid[body];
  }
  return body;
}

// mj_ray can exclude one body, but the X2 consists of many articulated child
// bodies. Step past hits belonging to the same free-body tree as the lidar so
// the published cloud cannot map its own arms and legs as an obstacle ring.
mjtNum RaySkippingRobot(const mjModel* model, const mjData* data,
                        const mjtNum origin[3], const mjtNum direction[3],
                        int robot_body, mjtNum max_distance, int* geom_id) {
  const int robot_root = RootBody(model, robot_body);
  mjtNum ray_origin[3] = {origin[0], origin[1], origin[2]};
  mjtNum travelled = 0;
  for (int attempt = 0; attempt < 32 && travelled < max_distance; ++attempt) {
    int hit_geom = -1;
    const mjtNum hit = mj_ray(model, data, ray_origin, direction, nullptr, 1,
                              robot_body, &hit_geom);
    if (hit < 0 || hit_geom < 0) {
      *geom_id = -1;
      return -1;
    }
    const int hit_body = model->geom_bodyid[hit_geom];
    if (RootBody(model, hit_body) != robot_root) {
      *geom_id = hit_geom;
      return travelled + hit;
    }
    const mjtNum advance = hit + 1e-3;
    travelled += advance;
    for (int axis = 0; axis < 3; ++axis) {
      ray_origin[axis] += advance * direction[axis];
    }
  }
  *geom_id = -1;
  return -1;
}

void WriteMetadata(mjtNum* state, const Lidar* lidar) {
  const int rays = lidar->horizontal * lidar->vertical;
  state[0] = lidar->horizontal;
  state[1] = lidar->vertical;
  state[2] = 0;           // pos_w field offset (publisher's xyz source)
  state[3] = rays * 3;    // pos_w field length
  state[4] = rays * 3;    // distance field offset
  state[5] = rays;
}

int SensorSize(const mjModel* model, int instance, int) {
  const auto size = Parse(mj_getPluginConfig(model, instance, "size"));
  if (size.size() != 2 || size[0] <= 0 || size[1] <= 0) {
    mju_error("fixed lidar: size must contain two positive integers");
  }
  return static_cast<int>(size[0]) * static_cast<int>(size[1]) * 4;
}

int Init(const mjModel* model, mjData* data, int instance) {
  auto* lidar = new Lidar;
  const auto size = Parse(mj_getPluginConfig(model, instance, "size"));
  const auto fov_h = Parse(mj_getPluginConfig(model, instance, "fov_h"));
  const auto fov_v = Parse(mj_getPluginConfig(model, instance, "fov_v"));
  const auto distance = Parse(mj_getPluginConfig(model, instance, "dis_range"));
  const auto period = Parse(mj_getPluginConfig(model, instance, "n_step_update"));
  lidar->horizontal = static_cast<int>(size.at(0));
  lidar->vertical = static_cast<int>(size.at(1));
  lidar->fov_h = fov_h.at(0) * mjPI / 180.0;
  lidar->fov_v = fov_v.at(0) * mjPI / 180.0;
  if (distance.size() == 2) {
    lidar->min_distance = distance[0];
    lidar->max_distance = distance[1];
  }
  if (!period.empty()) lidar->update_period = std::max(1, static_cast<int>(period[0]));
  lidar->sensor_id = FindSensor(model, instance);
  if (lidar->sensor_id < 0) {
    delete lidar;
    return -1;
  }
  lidar->camera_id = model->sensor_objid[lidar->sensor_id];
  lidar->camera_body_id = model->cam_bodyid[lidar->camera_id];
  lidar->object_body_ids = {
      mj_name2id(model, mjOBJ_BODY, "medicine_box"),
      mj_name2id(model, mjOBJ_BODY, "mugmug"),
      mj_name2id(model, mjOBJ_BODY, "bread")};
  lidar->claw_geom_ids = {
      mj_name2id(model, mjOBJ_GEOM, "left_claw_finger"),
      mj_name2id(model, mjOBJ_GEOM, "left_claw_mirror_finger"),
      mj_name2id(model, mjOBJ_GEOM, "right_claw_finger"),
      mj_name2id(model, mjOBJ_GEOM, "right_claw_mirror_finger")};
  data->plugin_data[instance] = reinterpret_cast<uintptr_t>(lidar);
  WriteMetadata(data->plugin_state + model->plugin_stateadr[instance], lidar);
  return 0;
}

void Reset(const mjModel*, mjtNum* state, void* plugin_data, int) {
  auto* lidar = reinterpret_cast<Lidar*>(plugin_data);
  lidar->step = 0;
  WriteMetadata(state, lidar);
}

void Destroy(mjData* data, int instance) {
  delete reinterpret_cast<Lidar*>(data->plugin_data[instance]);
  data->plugin_data[instance] = 0;
}

void Compute(const mjModel* model, mjData* data, int instance, int) {
  auto* lidar = reinterpret_cast<Lidar*>(data->plugin_data[instance]);
  if (++lidar->step < lidar->update_period) return;
  lidar->step = 0;
  WriteMetadata(data->plugin_state + model->plugin_stateadr[instance], lidar);

  const int rays = lidar->horizontal * lidar->vertical;
  mjtNum* positions = data->sensordata + model->sensor_adr[lidar->sensor_id];
  mjtNum* distances = positions + rays * 3;
  const mjtNum* origin = data->cam_xpos + 3 * lidar->camera_id;
  const mjtNum* rotation = data->cam_xmat + 9 * lidar->camera_id;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  int ray = 0;
  for (int v = 0; v < lidar->vertical; ++v) {
    const double vf = lidar->vertical == 1 ? 0.0 :
        (0.5 - static_cast<double>(v) / (lidar->vertical - 1)) * lidar->fov_v;
    for (int h = 0; h < lidar->horizontal; ++h, ++ray) {
      const double hf = lidar->horizontal == 1 ? 0.0 :
          (0.5 - static_cast<double>(h) / (lidar->horizontal - 1)) * lidar->fov_h;
      const mjtNum local[3] = {
          -std::sin(hf) * std::cos(vf), std::sin(vf),
          -std::cos(hf) * std::cos(vf)};
      mjtNum direction[3];
      mju_mulMatVec3(direction, rotation, local);
      int geom_id = -1;
      const mjtNum distance = RaySkippingRobot(
          model, data, origin, direction, lidar->camera_body_id,
          lidar->max_distance, &geom_id);
      mjtNum* point = positions + ray * 3;
      if (distance >= lidar->min_distance && distance <= lidar->max_distance) {
        distances[ray] = distance;
        point[0] = origin[0] + distance * direction[0];
        point[1] = origin[1] + distance * direction[1];
        point[2] = origin[2] + distance * direction[2];
      } else {
        distances[ray] = lidar->max_distance;
        point[0] = point[1] = point[2] = nan;
      }
    }
  }
  // Reserve two samples for the sensor's true MuJoCo world pose.  This
  // replaces the bundled odometry publisher when it emits invalid all-zero
  // packets after GUI Reset.  They are skipped by obstacle consumers.
  if (rays >= 7) {
    positions[0] = origin[0];
    positions[1] = origin[1];
    positions[2] = origin[2];
    const mjtNum local_forward[3] = {0, 0, -1};
    mjtNum world_forward[3];
    mju_mulMatVec3(world_forward, rotation, local_forward);
    positions[3] = origin[0] + world_forward[0];
    positions[4] = origin[1] + world_forward[1];
    positions[5] = origin[2] + world_forward[2];
    distances[0] = distances[1] = 0;
    // Samples 2..4 expose physical free-body positions for acceptance checks:
    // medicine box, mug and bread.  They are sensor observations from the
    // current mjData state, not commanded or assumed poses.
    for (int object = 0; object < 3; ++object) {
      mjtNum* point = positions + (object + 2) * 3;
      const int body = lidar->object_body_ids[object];
      if (body >= 0) {
        const mjtNum* body_position = data->xpos + body * 3;
        point[0] = body_position[0];
        point[1] = body_position[1];
        point[2] = body_position[2];
      } else {
        point[0] = point[1] = point[2] = nan;
      }
      distances[object + 2] = 0;
    }
    // Samples 5 and 6 are the midpoint of each physical finger pair.
    for (int side = 0; side < 2; ++side) {
      mjtNum* point = positions + (side + 5) * 3;
      const int first = lidar->claw_geom_ids[side * 2];
      const int second = lidar->claw_geom_ids[side * 2 + 1];
      if (first >= 0 && second >= 0) {
        const mjtNum* a = data->geom_xpos + first * 3;
        const mjtNum* b = data->geom_xpos + second * 3;
        for (int axis = 0; axis < 3; ++axis) point[axis] = (a[axis] + b[axis]) / 2;
      } else {
        point[0] = point[1] = point[2] = nan;
      }
      distances[side + 5] = 0;
    }
  }
}

constexpr const char* kAttributes[] = {
    "fov_h", "fov_v", "size", "dis_range", "n_step_update",
    "sensor_data_types"};

}  // namespace

mjPLUGIN_LIB_INIT {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.sensor.ray_caster_lidar";
  plugin.nattribute = sizeof(kAttributes) / sizeof(kAttributes[0]);
  plugin.attributes = kAttributes;
  plugin.capabilityflags = mjPLUGIN_SENSOR;
  plugin.needstage = mjSTAGE_POS;
  plugin.nstate = +[](const mjModel*, int) { return 6; };
  plugin.nsensordata = SensorSize;
  plugin.init = Init;
  plugin.destroy = Destroy;
  plugin.reset = Reset;
  plugin.compute = Compute;
  mjp_registerPlugin(&plugin);
}
