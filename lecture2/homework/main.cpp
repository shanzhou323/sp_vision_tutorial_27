#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include <string>

int main()
{
    Camera cam;
    auto_aim::YOLO detector("./configs/yolo.yaml");

    cv::Mat frame;
    int frame_count = 0;

    while (true) {
        if (!cam.read(frame) || frame.empty()) continue;

        auto armors = detector.detect(frame, frame_count++);

        for (const auto& armor : armors) {
            // 绿色闭合矩形（4个关键点首尾相连）
            tools::draw_points(frame, armor.points, cv::Scalar(0, 255, 0), 2);

            // 标签文字：bluefour / redone 这种
            std::string label =
                auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];
            if (!armor.points.empty()) {
                tools::draw_text(frame, label,
                                 cv::Point(armor.points[0].x, armor.points[0].y - 10),
                                 cv::Scalar(0, 0, 255), 1.0, 2);
            }
        }

        cv::imshow("img", frame);
        if (cv::waitKey(1) == 'q') break;
    }
    return 0;
}
