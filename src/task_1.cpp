#include <chrono>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
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
  while (!exiter.exit()) {
    // 读取一帧
    if (!camera.try_read_for(img, t, 100ms)) {
      continue;
    }
    ++frame_count;

    // 1. YOLO 识别装甲板
    auto armors = yolo.detect(img, frame_count);

    // 2. 可视化：画出所有识别到的装甲板
    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
      tools::draw_text(
        img, auto_aim::COLORS[armor.color] + "_" + auto_aim::ARMOR_NAMES[armor.name],
        armor.center, cv::Scalar(0, 0, 255), 1.0, 2);
    }

    // 3. 选择优先级最高的装甲板作为目标（priority 越小优先级越高）
    auto_aim::Armor * target = nullptr;
    for (auto & armor : armors) {
      if (armor.duplicated) {
        continue;
      }
      if (target == nullptr || armor.priority < target->priority) {
        target = &armor;
      }
    }

    if (target != nullptr) {
      // 4. 用当前时刻云台姿态更新旋转，PnP 解算目标三维坐标与角度
      solver.set_R_gimbal2world(gimbal.q(t));
      solver.solve(*target);

      // 5. 控制云台指向目标（单位：rad）
      const double yaw = target->yaw_raw;
      const double pitch = target->ypr_in_gimbal.y();
      gimbal.send(true, false, yaw, pitch);

      // 6. 用plotter发送解算结果（便于调试观察）
      nlohmann::json j;
      j["task"] = 1;
      j["id"] = frame_count;
      j["armor"] = {
        {"name", auto_aim::ARMOR_NAMES[target->name]},
        {"color", auto_aim::COLORS[target->color]},
        {"xyz_in_gimbal",
         {target->xyz_in_gimbal.x(), target->xyz_in_gimbal.y(), target->xyz_in_gimbal.z()}},
        {"yaw", yaw},
        {"pitch", pitch},
      };
      plotter.plot(j);
    } else {
      // 没有目标：云台不控制
      gimbal.send(false, false, 0.0f, 0.0f);
    }
  }
  return 0;
}
