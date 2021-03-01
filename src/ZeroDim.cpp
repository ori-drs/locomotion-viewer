//
// Created by Romeo Orsolino on 06/04/2020.
//

#include <locomotion_viewer/ZeroDim.h>

namespace locomotion_viewer {
    ZeroDim::ZeroDim(std::string base_frame,
                   std::string marker_topic,
                   ros::NodeHandle nh) : RvizVisualTools(base_frame, marker_topic, nh) {}

    ZeroDim::~ZeroDim() {}

    bool ZeroDim::publishEigenSphere(Eigen::Vector3d & point,
                                     rviz_visual_tools::colors color,
                                     rviz_visual_tools::scales scale,
                                     const std::string & ns)
    {
        geometry_msgs::Point temp;
        temp.x = point.x();
        temp.y = point.y();
        temp.z = point.z();

        return publishSphere(temp, color, scale, ns);
    }

    bool ZeroDim::publishEigenSpheres(const Eigen::VectorXd & eigen_path_x,
                                      const Eigen::VectorXd & eigen_path_y,
                                      const Eigen::VectorXd & eigen_path_z,
                                      rviz_visual_tools::colors color,
                                      rviz_visual_tools::scales scale,
                                      const std::string & ns)
    {
    std::vector<geometry_msgs::Point> trajectory;
        int points_num = eigen_path_x.rows();
        trajectory.reserve(points_num);
        geometry_msgs::Point temp;
        for (int i = 0; i < points_num; ++i)
        {
            temp.x = eigen_path_x(i);
            temp.y = eigen_path_y(i);
            temp.z = eigen_path_z(i);

            trajectory.push_back(temp);
        }
        return publishSpheres(trajectory, color, scale, ns);
    }
}