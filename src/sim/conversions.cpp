#include <core/json.h>
#include <core/maze.h>
#include <sim/conversions.h>

namespace ssim {

RobotDescription Convert(std::ifstream &fs) {
  nlohmann::json json;
  json << fs;

  RobotDescription robot_description;
  auto footprint = robot_description.footprint;
  for (auto json_pt : json["footprint"]) {
    footprint.emplace_back(json_pt["x"], json_pt["y"]);
  }

  robot_description.motor.u_static = json["motor"]["u_static"];
  robot_description.motor.u_kinetic = json["motor"]["u_kinetic"];
  robot_description.motor.J = json["motor"]["J"];
  robot_description.motor.b = json["motor"]["b"];
  robot_description.motor.K = json["motor"]["K"];
  robot_description.motor.R = json["motor"]["R"];
  robot_description.motor.L = json["motor"]["L"];

  robot_description.left_wheel.pose = {json["left_wheel"]["pose"]["x"], json["left_wheel"]["pose"]["y"]};
  robot_description.left_wheel.radius = json["left_wheel"]["radius"];
  robot_description.left_wheel.thickness = json["left_wheel"]["thickness"];

  robot_description.right_wheel.pose = {json["right_wheel"]["pose"]["x"], json["right_wheel"]["pose"]["y"]};
  robot_description.right_wheel.radius = json["right_wheel"]["radius"];
  robot_description.right_wheel.thickness = json["right_wheel"]["thickness"];

  auto &sensor_description = robot_description.sensors;
  sensor_description.front.p.x = json["range_sensors"]["front"]["x"];
  sensor_description.front.p.y = json["range_sensors"]["front"]["y"];
  sensor_description.front.p.theta = json["range_sensors"]["front"]["theta"];
  sensor_description.front.a = json["range_sensors"]["front"]["a"];
  sensor_description.front.b = json["range_sensors"]["front"]["b"];
  sensor_description.front.c = json["range_sensors"]["front"]["c"];

  sensor_description.back_right.p.x = json["range_sensors"]["back_right"]["x"];
  sensor_description.back_right.p.y = json["range_sensors"]["back_right"]["y"];
  sensor_description.back_right.p.theta = json["range_sensors"]["back_right"]["theta"];
  sensor_description.back_right.a = json["range_sensors"]["back_right"]["a"];
  sensor_description.back_right.b = json["range_sensors"]["back_right"]["b"];
  sensor_description.back_right.c = json["range_sensors"]["back_right"]["c"];

  sensor_description.back_left.p.x = json["range_sensors"]["back_left"]["x"];
  sensor_description.back_left.p.y = json["range_sensors"]["back_left"]["y"];
  sensor_description.back_left.p.theta = json["range_sensors"]["back_left"]["theta"];
  sensor_description.back_left.a = json["range_sensors"]["back_left"]["a"];
  sensor_description.back_left.b = json["range_sensors"]["back_left"]["b"];
  sensor_description.back_left.c = json["range_sensors"]["back_left"]["c"];

  sensor_description.gerald_right.p.x = json["range_sensors"]["gerald_right"]["x"];
  sensor_description.gerald_right.p.y = json["range_sensors"]["gerald_right"]["y"];
  sensor_description.gerald_right.p.theta = json["range_sensors"]["gerald_right"]["theta"];
  sensor_description.gerald_right.a = json["range_sensors"]["gerald_right"]["a"];
  sensor_description.gerald_right.b = json["range_sensors"]["gerald_right"]["b"];
  sensor_description.gerald_right.c = json["range_sensors"]["gerald_right"]["c"];

  sensor_description.gerald_left.p.x = json["range_sensors"]["gerald_left"]["x"];
  sensor_description.gerald_left.p.y = json["range_sensors"]["gerald_left"]["y"];
  sensor_description.gerald_left.p.theta = json["range_sensors"]["gerald_left"]["theta"];
  sensor_description.gerald_left.a = json["range_sensors"]["gerald_left"]["a"];
  sensor_description.gerald_left.b = json["range_sensors"]["gerald_left"]["b"];
  sensor_description.gerald_left.c = json["range_sensors"]["gerald_left"]["c"];

  sensor_description.front_right.p.x = json["range_sensors"]["front_right"]["x"];
  sensor_description.front_right.p.y = json["range_sensors"]["front_right"]["y"];
  sensor_description.front_right.p.theta = json["range_sensors"]["front_right"]["theta"];
  sensor_description.front_right.a = json["range_sensors"]["front_right"]["a"];
  sensor_description.front_right.b = json["range_sensors"]["front_right"]["b"];
  sensor_description.front_right.c = json["range_sensors"]["front_right"]["c"];

  sensor_description.front_left.p.x = json["range_sensors"]["front_left"]["x"];
  sensor_description.front_left.p.y = json["range_sensors"]["front_left"]["y"];
  sensor_description.front_left.p.theta = json["range_sensors"]["front_left"]["theta"];
  sensor_description.front_left.a = json["range_sensors"]["front_left"]["a"];
  sensor_description.front_left.b = json["range_sensors"]["front_left"]["b"];
  sensor_description.front_left.c = json["range_sensors"]["front_left"]["c"];

  return robot_description;
}

std::array<Line2d, 16> Convert(Node const &node) {
  // TODO optimize for each specific configuration of walls so that we don't have to check all 16 lines every time
  double r = node.row();
  double c = node.row();
  return {
      // each line is made of two (col/row points) like {pt 1 col, pt 1 row, pt 2 col, pt 2 row}
      // north wall lines
      Line2d{c, r, c, r + WALL_THICKNESS_CU},
      {c, r, c + 1, r},
      {c + 1, r, c, r + WALL_THICKNESS_CU},
      {c + 1, r, c + 1, r + WALL_THICKNESS_CU},
      // east wall lines
      {c + 1 - WALL_THICKNESS_CU, r, c + 1 - WALL_THICKNESS_CU, r + 1},
      {c + 1 - WALL_THICKNESS_CU, r, c + 1, r},
      {c + 1, r, c + 1, r + 1},
      {c + 1 - WALL_THICKNESS_CU, r + 1, c + 1, r + 1},
      // south wall lines
      {c, r + 1 - WALL_THICKNESS_CU, c, r + 1},
      {c, r + 1 - WALL_THICKNESS_CU, c + 1, r + 1 - WALL_THICKNESS_CU},
      {c + 1, r + 1 - WALL_THICKNESS_CU, c + 1, r + 1},
      {c, r + 1, c + 1, r + 1},
      // west wall lines
      {c, r, c, r + 1},
      {c, r, c + WALL_THICKNESS_CU, r},
      {c + WALL_THICKNESS_CU, r, c + WALL_THICKNESS_CU, r + 1},
      {c, r + 1, c + WALL_THICKNESS_CU, r + 1}
  };
}

} // namespace msgs
