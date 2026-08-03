#include <dlfcn.h>
#include <array>
#include <iostream>
#include <cmath>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: validate MODEL_XML PLUGIN_SO\n";
    return 2;
  }
  void* plugin = dlopen(argv[2], RTLD_NOW | RTLD_GLOBAL);
  if (!plugin) {
    std::cerr << "plugin load failed: " << dlerror() << '\n';
    return 2;
  }
  char error[2048] = {};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::cerr << "model load failed: " << error << '\n';
    return 2;
  }
  mjData* data = mj_makeData(model);
  for (int i = 0; i < 20; ++i) mj_step(model, data);
  int sensor = mj_name2id(model, mjOBJ_SENSOR, "raycaster_lidar");
  if (sensor < 0) {
    std::cerr << "raycaster_lidar missing\n";
    return 2;
  }
  int plugin_id = model->sensor_plugin[sensor];
  int state = model->plugin_stateadr[plugin_id];
  int horizontal = static_cast<int>(data->plugin_state[state]);
  int vertical = static_cast<int>(data->plugin_state[state + 1]);
  std::cout << "model_ok nq=" << model->nq << " nv=" << model->nv << '\n';
  std::cout << "lidar_rays=" << horizontal << 'x' << vertical
            << " sensor_dim=" << model->sensor_dim[sensor] << '\n';
  int finite = 0;
  mjtNum minimum = 1e9, maximum = -1e9;
  const int address = model->sensor_adr[sensor];
  for (int i = 0; i < model->sensor_dim[sensor]; ++i) {
    mjtNum value = data->sensordata[address + i];
    if (std::isfinite(value)) {
      ++finite;
      minimum = std::min(minimum, value);
      maximum = std::max(maximum, value);
    }
  }
  std::cout << "finite_values=" << finite << " bounds=" << minimum << ".."
            << maximum << '\n';
  int camera = mj_name2id(model, mjOBJ_CAMERA, "RayCasterLidar");
  const mjtNum* camera_position = data->cam_xpos + camera * 3;
  const mjtNum* camera_rotation = data->cam_xmat + camera * 9;
  std::cout << "camera=" << camera_position[0] << ',' << camera_position[1]
            << ',' << camera_position[2] << " xmat=";
  for (int i = 0; i < 9; ++i) std::cout << camera_rotation[i] << (i == 8 ? '\n' : ',');
  for (const auto& direction : {std::array<mjtNum, 3>{1, 0, 0},
                                std::array<mjtNum, 3>{-1, 0, 0},
                                std::array<mjtNum, 3>{0, 1, 0},
                                std::array<mjtNum, 3>{0, -1, 0}}) {
    int geom = -1;
    mjtNum distance = mj_ray(model, data, camera_position, direction.data(),
                             nullptr, 1, -1, &geom);
    std::cout << "ray=" << direction[0] << ',' << direction[1] << ','
              << direction[2] << " distance=" << distance << " geom=" << geom << '\n';
  }
  for (int id = 0; id < model->ngeom; ++id) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, id);
    const char* body_name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[id]);
    if ((name && std::string(name).find("table") != std::string::npos) ||
        (body_name && std::string(body_name).find("table") != std::string::npos)) {
      const mjtNum* position = data->geom_xpos + id * 3;
      std::cout << "table_geom=" << id << " pos=" << position[0] << ','
                << position[1] << ',' << position[2]
                << " rbound=" << model->geom_rbound[id] << '\n';
      if (model->geom_type[id] == mjGEOM_MESH) {
        int mesh = model->geom_dataid[id];
        std::array<mjtNum, 3> lower{1e9, 1e9, 1e9};
        std::array<mjtNum, 3> upper{-1e9, -1e9, -1e9};
        for (int j = 0; j < model->mesh_vertnum[mesh]; ++j) {
          const float* local = model->mesh_vert + 3 * (model->mesh_vertadr[mesh] + j);
          mjtNum local_num[3] = {local[0], local[1], local[2]};
          mjtNum world[3];
          mju_mulMatVec3(world, data->geom_xmat + id * 9, local_num);
          for (int axis = 0; axis < 3; ++axis) {
            world[axis] += position[axis];
            lower[axis] = std::min(lower[axis], world[axis]);
            upper[axis] = std::max(upper[axis], world[axis]);
          }
        }
        std::cout << "table_bounds=" << lower[0] << ',' << lower[1] << ',' << lower[2]
                  << ".." << upper[0] << ',' << upper[1] << ',' << upper[2] << '\n';
      }
    }
  }
  for (int id = 0; id < model->nbody; ++id) {
    const char* name = mj_id2name(model, mjOBJ_BODY, id);
    if (name && (std::string(name).find("mug") != std::string::npos ||
                 std::string(name).find("medicine") != std::string::npos ||
                 std::string(name).find("bread") != std::string::npos ||
                 std::string(name).find("claw") != std::string::npos ||
                 std::string(name).find("wrist") != std::string::npos ||
                 std::string(name).find("hand") != std::string::npos)) {
      const mjtNum* p = data->xpos + id * 3;
      std::cout << "object_body=" << id << " name=" << name << " pos="
                << p[0] << ',' << p[1] << ',' << p[2] << '\n';
    }
  }
  bool ok = horizontal > 0 && vertical > 0;
  mj_deleteData(data);
  mj_deleteModel(model);
  dlclose(plugin);
  return ok ? 0 : 1;
}
