#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";
using namespace std::chrono_literals;
int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用
  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;
  std::optional<auto_aim::Target> target;  // Kalman 目标
  while (!exiter.exit()) {
    // 读取一帧
    if (!camera.try_read_for(img, t, 100ms)) {
      continue;
    }
    ++frame_count;

    // 1. YOLO 识别装甲板
    auto armors = yolo.detect(img, frame_count);

    // 2. 可视化
    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
      tools::draw_text(
        img, auto_aim::COLORS[armor.color] + "_" + auto_aim::ARMOR_NAMES[armor.name],
        armor.center, cv::Scalar(0, 0, 255), 1.0, 2);
    }

    // 3. 选优先级最高的装甲板
    auto_aim::Armor * target_armor = nullptr;
    for (auto & armor : armors) {
      if (armor.duplicated) {
        continue;
      }
      if (target_armor == nullptr || armor.priority < target_armor->priority) {
        target_armor = &armor;
      }
    }

    if (target_armor == nullptr) {
      target.reset();  // 丢失目标，重置滤波器
      continue;
    }

    // 4. PnP 解算
    solver.set_R_gimbal2world(gimbal.q(t));
    solver.solve(*target_armor);

    // 5. Kalman：首次初始化，后续先预测再更新（换目标则重建）
    if (!target) {
      target.emplace(*target_armor, t);
    } else {
      target->predict(t);
      if (target->name == target_armor->name) {
        target->update(*target_armor);
      } else {
        target.reset();
        target.emplace(*target_armor, t);
      }
    }

    // 6. 用滤波后的状态（世界系）转云台系，控制云台
    Eigen::VectorXd x = target->ekf_x();
    Eigen::Vector3d xyz_world(x[0], x[2], x[4]);  // 预测的旋转中心
    Eigen::Vector3d xyz_gimbal = solver.R_gimbal2world().transpose() * xyz_world;
    const double yaw = std::atan2(xyz_gimbal.y(), xyz_gimbal.x());
    const double pitch =
      std::atan2(-xyz_gimbal.z(), std::hypot(xyz_gimbal.x(), xyz_gimbal.y()));
    gimbal.send(true, false, yaw, pitch);

    // 7. plotter 上报（便于观察滤波轨迹）
    nlohmann::json j;
    j["task"] = 2;
    j["id"] = frame_count;
    j["armor"] = {
      {"name", auto_aim::ARMOR_NAMES[target->name]},
      {"xyz", {x[0], x[2], x[4]}},
      {"yaw", yaw},
      {"pitch", pitch},
    };
    plotter.plot(j);
  }
  return 0;
}
