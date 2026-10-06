#include <Eigen/Dense>   // 必须最先，防止与 OpenCV 头冲突
#include "io/camera.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include <string>

int main()
{
    Camera camera;
    auto_charge::AprilTagDetector detector("./configs/apriltag.yaml");

    cv::Mat img;
    while (true) {
        if (!camera.read(img) || img.empty()) continue;

        auto tags = detector.detect(img);

        for (const auto & tag : tags) {
            tools::draw_points(img, tag.corners, cv::Scalar(0, 255, 0), 2);
            cv::circle(img, tag.center, 5, cv::Scalar(0, 0, 255), -1);

            std::string label = "id=" + std::to_string(tag.id);
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
            cv::Point text_org(tag.corners[0].x, tag.corners[0].y - baseline - 20);
            cv::putText(img, label, text_org,
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') break;
    }
    return 0;
}
