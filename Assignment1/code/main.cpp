#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotate;
    rotation_angle = rotation_angle * MY_PI / 180;
    rotate << cos(rotation_angle), -sin(rotation_angle), 0, 0, 
              sin(rotation_angle), cos(rotation_angle), 0, 0,
              0, 0, 1, 0, 0, 0, 0, 1;
    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f persp2ortho;
    persp2ortho << zNear, 0, 0, 0,
                   0, zNear, 0, 0,
                   0, 0, zNear + zFar, -zNear * zFar,
                   0, 0, 1, 0;
    Eigen::Matrix4f ortho;
    eye_fov = eye_fov * MY_PI / 180;
    // Find explanation in assignment 2
    float yTop = -abs(zNear) * tan(eye_fov / 2);
    float xRight = yTop * aspect_ratio;
    Vector3f translate(0, 0, (zNear + zFar) / 2);
    ortho << 1 / xRight, 0, 0, 0,
             0, 1 / yTop, 0, 0,
             0, 0, 2 / (zNear - zFar), 0,
             0, 0, 0, 1;
    ortho = ortho * get_view_matrix(translate);
    projection = ortho * persp2ortho;
    
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    angle = angle * MY_PI / 180;
    Eigen::Matrix3f crossProd;
    crossProd << 0, -axis[2], axis[1],
                 axis[2], 0, -axis[0],
                 -axis[1], axis[0], 0;
    rotation = cos(angle) * rotation + (1 - cos(angle)) * axis * axis.transpose() + sin(angle) * crossProd;
    Eigen::Matrix4f rot;
    rot << rotation(0, 0), rotation(0, 1), rotation(0, 2), 0,
           rotation(1, 0), rotation(1, 1), rotation(1, 2), 0,
           rotation(2, 0), rotation(2, 1), rotation(2, 2), 0,
           0, 0, 0, 1;
    return rot;
}

int main(int argc, const char** argv)
{
    // if(1) {
    //     Vector3f axis(sqrt(3) / 3, sqrt(3) / 3, sqrt(3) / 3);
    //     float angle = 30;
    //     std::cout << get_rotation(axis, angle) << std::endl;
    //     return 0;
    // }
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
