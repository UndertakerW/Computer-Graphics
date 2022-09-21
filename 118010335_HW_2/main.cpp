#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
// add some other header files you need
#include "xml.h"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;
    // std::clog << "view" << std::endl << view << std::endl;  // check data

    return view;
}


Eigen::Matrix4f get_model_matrix(float rotation_angle, Eigen::Vector3f T, Eigen::Vector3f S, Eigen::Vector3f P0, Eigen::Vector3f P1)
{

    //Step 1: Build the Translation Matrix M_trans:
    Eigen::Matrix4f M_trans;
    M_trans << 1, 0, 0, T[0],
        0, 1, 0, T[1],
        0, 0, 1, T[2],
        0, 0, 0, 1;

    //Step 2: Build the Scale Matrix M_scale:
    Eigen::Matrix4f M_scale;
    M_scale << S[0], 0, 0, 0,
        0, S[1], 0, 0,
        0, 0, S[2], 0,
        0, 0, 0, 1;

    //Step 3: Implement Rodrigues' Rotation Formular, rotation by angle theta around axix u, then get the model matrix
	// The axis u is determined by two points, u = P1-P0: Eigen::Vector3f P0 ,Eigen::Vector3f P1  
    // Create the model matrix for rotating the triangle around a given axis. // Hint: normalize axis first
    Eigen::Vector3f u = P1 - P0;
    u.normalize();
    
    // v_rot = v*cos(theta) + k.cross(v)*sin(theta) + k*k.dot(v)*(1-cos(theta));
    float radian = rotation_angle / 360 * 2 * MY_PI;
    Eigen::Matrix4f M_rot;
    M_rot << std::cos(radian) + u[0] * u[0] * (1 - std::cos(radian)),
        u[0] * u[1] * (1 - std::cos(radian)) - u[2] * std::sin(radian),
        u[0] * u[2] * (1 - std::cos(radian)) + u[1] * std::sin(radian),
        0,
        u[1] * u[0] * (1 - std::cos(radian)) + u[2] * std::sin(radian),
        std::cos(radian) + u[1] * u[1] * (1 - std::cos(radian)),
        u[1] * u[2] * (1 - std::cos(radian)) - u[0] * std::sin(radian),
        0,
        u[2] * u[0] * (1 - std::cos(radian)) - u[1] * std::sin(radian),
        u[2] * u[1] * (1 - std::cos(radian)) + u[0] * std::sin(radian),
        std::cos(radian) + u[2] * u[2] * (1 - std::cos(radian)),
        0,
        0, 0, 0, 1;

    Eigen::Matrix4f model = M_scale * M_rot * M_trans;

	//Step 4: Use Eigen's "AngleAxisf" to verify your Rotation
	//Eigen::AngleAxisf rotation_vector(radian, Vector3f(axis[0], axis[1], axis[2]));  
	//Eigen::Matrix3f rotation_matrix;
	//rotation_m = rotation_vector.toRotationMatrix();

    Eigen::AngleAxisf rotation_vector(radian, Vector3f(u[0], u[1], u[2]));
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Matrix3f M_rot_3d;
    M_rot_3d << std::cos(radian) + u[0] * u[0] * (1 - std::cos(radian)),
        u[0] * u[1] * (1 - std::cos(radian)) - u[2] * std::sin(radian),
        u[0] * u[2] * (1 - std::cos(radian)) + u[1] * std::sin(radian),
        u[1] * u[0] * (1 - std::cos(radian)) + u[2] * std::sin(radian),
        std::cos(radian) + u[1] * u[1] * (1 - std::cos(radian)),
        u[1] * u[2] * (1 - std::cos(radian)) - u[0] * std::sin(radian),
        u[2] * u[0] * (1 - std::cos(radian)) - u[1] * std::sin(radian),
        u[2] * u[1] * (1 - std::cos(radian)) + u[0] * std::sin(radian),
        std::cos(radian) + u[2] * u[2] * (1 - std::cos(radian));

    // check
    std::clog << "get_model_matrix Step 4: Rotation checked, result:"
        << M_rot_3d.isApprox(rotation_matrix) << std::endl;

	return model;
}



Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Implement this function

   

    float angle = eye_fov * MY_PI / 180.0; // half angle
    float height = zNear * tan(angle) * 2;
    float width = height * aspect_ratio;

    auto t = -zNear * tan(angle / 2);
    auto r = t * aspect_ratio;
    auto l = -r;
    auto b = -t;




    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // frustum -> cubic
    Eigen::Matrix4f M_frus_2_cub(4, 4);
    M_frus_2_cub << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // orthographic projection

    // translate to the origin
    Eigen::Matrix4f M_ortho_trans(4, 4);
    M_ortho_trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    // scale to the normal cube
    Eigen::Matrix4f M_ortho_scale(4, 4);
    M_ortho_scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    // final orthographic projection transformation matrix
    Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_trans;

    // squash all transformations

    // final perspective projection transformation matrix
    Eigen::Matrix4f projection;
    projection = M_ortho * M_frus_2_cub;

    // check
    std::clog << "projection" << std::endl << projection << std::endl;

    return projection;
}

int main(int argc, const char** argv)
{
    // Default values: to be replaced by the xml file

    // define your eye position "eye_pos" to a proper position
    Eigen::Vector3f eye_pos = { 0, 0, 8 };


    // define a triangle named by "pos" and "ind"

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    Eigen::Vector3f T = { 1, 1, 0.5 };
    Eigen::Vector3f S = { 0.5, 2, 3 };
    Eigen::Vector3f P0 = { 1, 2, 3 };
    Eigen::Vector3f P1 = { 20, 5, 3 };

    // add parameters for get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
    float eye_fov = 45;
    float aspect_ratio = 1;
    float zNear = 0.1;
    float zFar = 50;

    float angle = -45;

    std::string xml_path = "parameters.xml";
    bool xml_result = ReadParaXml(xml_path, eye_pos, angle, T, S, P0, P1,
        pos, eye_fov, aspect_ratio, zNear, zFar);

    std::clog << "Xml file parsed, result: " << xml_result << std::endl;

    bool command_line = false;
    std::string filename = "result.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(1024, 1024);

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, T, S, P0, P1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::clog << "angle: " << angle << std::endl;
    

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
