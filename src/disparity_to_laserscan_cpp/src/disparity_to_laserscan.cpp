#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <limits>
#include <algorithm>

class DisparityToLaser : public rclcpp::Node
{
public:
    DisparityToLaser() : Node("disparity_to_laserscan"), focal_length_(0.0), baseline_(0.06)
    {
        // Suscripción a CameraInfo
        camera_info_left_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/left/camera_info", 10,
            std::bind(&DisparityToLaser::leftCameraInfoCallback, this, std::placeholders::_1));

        camera_info_right_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/right/camera_info", 10,
            std::bind(&DisparityToLaser::rightCameraInfoCallback, this, std::placeholders::_1));

        // Suscripción a DisparityImage
        disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
            "/disparity", 10,
            std::bind(&DisparityToLaser::disparityCallback, this, std::placeholders::_1));

        // Publicación del LaserScan
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // Parámetros
        this->declare_parameter<int>("scan_height", 1);
        this->declare_parameter<double>("range_min", 0.1);
        this->declare_parameter<double>("range_max", 5.0);

        this->declare_parameter("camera_fov_horizontal", 1.0472);  // ~60° en radianes
        this->declare_parameter("num_laser_points", 720);          // Cantidad de puntos del "lidar"
    }

private:

    void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        left_camera_info_ = msg;
        tryCalculateBaseline();
    }

    void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        right_camera_info_ = msg;
        tryCalculateBaseline();
    }

    void tryCalculateBaseline() {
        if (left_camera_info_ && right_camera_info_) {
            focal_length_ = left_camera_info_->k[0];

            // Calculo el baseline usando la matriz P de la derecha
            // P = [p00, p01, p02, p03, ...] fila mayor a menor, ROS lo serializa como vector de 12 elementos
            baseline_ = 0.06; // - right_camera_info_->p[3] / focal_length_; TODO: arreglar esto, da mal el calculo

            // RCLCPP_INFO(this->get_logger(), "Baseline calculada: %.4f m, fx: %.2f", baseline_, focal_length_);
            
            // Ya no necesito volver a calcular
            left_camera_info_.reset();
            right_camera_info_.reset();
        }
    }

    void disparityCallback(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
        if (focal_length_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Focal length no disponible, saltando disparidad");
            return;
        }
        
        auto scan = sensor_msgs::msg::LaserScan();
        scan.header = msg->header;
        scan.header.stamp = this->now(); 
        scan.header.frame_id = "lidar_link";
        
        int width = msg->image.width;
        int height = msg->image.height;
        int scan_height;
        double range_min, range_max;
        double camera_fov_horizontal;  // FOV horizontal de la cámara en radianes
        int num_laser_points = 360;    // Cantidad fija de puntos (ajustable)
        
        this->get_parameter("scan_height", scan_height);
        this->get_parameter("range_min", range_min);
        this->get_parameter("range_max", range_max);
        this->get_parameter("camera_fov_horizontal", camera_fov_horizontal); // e.g., 1.0472 rad = 60°
        
        // Configurar LaserScan de 360 grados
        scan.angle_min = -M_PI;  // -180 grados
        scan.angle_max = M_PI;   // +180 grados
        scan.angle_increment = (2.0 * M_PI) / num_laser_points;
        scan.range_min = range_min;
        scan.range_max = range_max;
        scan.ranges.resize(num_laser_points, std::numeric_limits<float>::infinity());
        
        // Calcular qué índices del LaserScan corresponden al FOV de la cámara
        // Asumiendo que la cámara apunta hacia adelante (ángulo 0)
        double camera_angle_center = 0.0;  // Ajustar si la cámara está rotada
        double camera_angle_min = camera_angle_center - (camera_fov_horizontal / 2.0);
        double camera_angle_max = camera_angle_center + (camera_fov_horizontal / 2.0);
        
        // Normalizar ángulos al rango [-PI, PI]
        auto normalize_angle = [](double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        };
        
        camera_angle_min = normalize_angle(camera_angle_min);
        camera_angle_max = normalize_angle(camera_angle_max);
        
        // Procesar datos de disparidad
        int row_start = height / 2;
        const float* disp_data = reinterpret_cast<const float*>(&msg->image.data[0]);
        
        // Mapear cada píxel de la imagen al LaserScan
        for (int img_col = 0; img_col < width; ++img_col) {
            // Calcular el ángulo correspondiente a este píxel en la imagen
            double pixel_angle_in_fov = ((double)img_col / width - 0.5) * camera_fov_horizontal;
            double pixel_angle_global = normalize_angle(camera_angle_center + pixel_angle_in_fov);
            
            // Encontrar el índice correspondiente en el LaserScan
            int laser_index = static_cast<int>(
                (pixel_angle_global - scan.angle_min) / scan.angle_increment
            );
            
            // Asegurar que el índice esté dentro del rango
            if (laser_index >= 0 && laser_index < num_laser_points) {
                float disp = disp_data[row_start * width + img_col];
                
                if (disp > 0.0f) {
                    float depth = (focal_length_ * baseline_) / disp;
                    scan.ranges[laser_index] = std::min(std::max(depth, (float)range_min), (float)range_max);
                }
                // Si disp <= 0, ya está inicializado como inf
            }
        }
        
        scan_pub_->publish(scan);
    }


    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_right_sub_;
    rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    sensor_msgs::msg::CameraInfo::SharedPtr left_camera_info_;
    sensor_msgs::msg::CameraInfo::SharedPtr right_camera_info_;

    float focal_length_;
    float baseline_; // definir según tu cámara en metros
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityToLaser>());
    rclcpp::shutdown();
    return 0;
}
