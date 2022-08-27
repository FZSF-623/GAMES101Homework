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

    rotation_angle = rotation_angle/180.0 *MY_PI;

    model << cos(rotation_angle), -sin(rotation_angle), 0.0, 0.0,
             sin(rotation_angle), cos(rotation_angle), 0.0, 0.0,
             0, 0, 1, 0,
             0, 0, 0, 1;
             
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

    eye_fov = eye_fov/180.0 * MY_PI;

    float t,b,l,r;
    t = tan(eye_fov/2.0) *zNear;
    r = aspect_ratio * t;
    b = -t, l= -r;

    zNear = -zNear, zFar = -zFar;

    Eigen::Matrix4f pro_to_ortho, ortho, scale, tranf;

    pro_to_ortho << zNear, 0, 0, 0,
                    0, zNear, 0, 0,
                    0, 0, zNear+zFar, -zFar*zNear,
                    0, 0, 1, 0;
    
    scale << 2.0/(r-l), 0, 0, 0,
             0, 2.0/(t-b), 0, 0,
             0, 0, 2.0/(zNear-zFar), 0,
             0, 0, 0, 1;

    tranf << 1, 0, 0, -(l+r)/2.0,
             0, 1, 0, -(t+b)/2.0,
             0, 0, 1, -(zFar+zNear)/2.0,
             0, 0, 0, 1;

    ortho = scale * tranf;

    projection = ortho *pro_to_ortho;

    return projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis,float angle)
{
    angle = angle/180.0 * MY_PI;
    Eigen::Matrix4f R, N, I = Eigen::Matrix4f::Identity();
    Eigen::Vector4f axi;
    Eigen::RowVector4f taxi;

    axi << axis.x(), axis.y(), axis.z(), 0;
    taxi << axis.x(), axis.y(), axis.z(), 0;

    N << 0, -axis.z(), axis.y(), 0,
         axis.z(), 0, -axis.x(), 0,
         -axis.y(), axis.x(), 0, 0,
         0, 0, 0, 1;

    R = cos(angle) * I + (1 - cos(angle)) * axi * taxi + sin(angle) * N;
 
    R(3,3) = 1;

    return R;

}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    Eigen::Vector3f axis = Eigen::Vector3f::Identity();

    if (argc >= 6) {
        command_line = true;
        float a, b, c;
        a = std::stof(argv[2]);
        b = std::stof(argv[3]);
        c = std::stof(argv[4]);
        axis << a, b, c;
        angle = std::stof(argv[5]); // -r by default
        if (argc == 7) {
            filename = std::string(argv[6]);
        }
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

        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    float a, b, c;
    std::cout << "Please input axis: ";
    std::cin >> a >> b >> c;

    axis << a, b, c;

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(axis,angle));
        //r.set_model(get_model_matrix(angle));
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
