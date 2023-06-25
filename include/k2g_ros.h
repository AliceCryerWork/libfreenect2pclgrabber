#include <chrono>
#include "k2g.h"
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/image_encodings.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>


using namespace std::chrono_literals;


struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, K2G & k2g): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), k2g_(k2g){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;
  K2G & k2g_;
};


class K2GRos: public rclcpp::Node
{

public:


	K2GRos(Processor freenect_processor = CPU): Node("kinect2"), k2g_(freenect_processor),
			cloud_(new pcl::PointCloud<pcl::PointXYZRGB>(512, 424)),
			size_color_(1920, 1080),
		  	size_depth_(512, 424)
	{
		
		point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kinect2/hd/points", 1);
		color_pub_ = create_publisher<sensor_msgs::msg::Image>("/kinect2/hd/image_color", 1);
		color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/kinect2/hd/camera_info", 1);
		depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/kinect2/sd/image_depth", 1);
		depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/kinect2/sd/camera_info", 1);

		libfreenect2::Freenect2Device::IrCameraParams ir = k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams rgb = k2g_.getRgbParameters();

		createCameraInfoColor(rgb);
		createCameraInfoDepth(ir);

  		k2g_.storeParameters();

		RCLCPP_INFO_STREAM(get_logger(), "Device serial: " << k2g_.getSerialNumber());
		RCLCPP_INFO_STREAM(get_logger(), "Device firmware: " << k2g_.getFirmwareVersion());

		timer_ = create_wall_timer(10ms, std::bind(&K2GRos::timer_callback, this));
    }



	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishDepth(){
		cv::Mat tmp_depth;
		
		k2g_.getDepth(tmp_depth);
		auto depth_image = mat_to_image(tmp_depth);
		depth_image->encoding = sensor_msgs::image_encodings::MONO8;
		depth_pub_-> publish(*depth_image);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishColor(){
		cv::Mat tmp_color;

		k2g_.getColor(tmp_color);
		auto color_image = mat_to_image(tmp_color);
		color_image->encoding = sensor_msgs::image_encodings::BGRA8;
		color_pub_-> publish(*color_image);
	}

	// Depth and color are aligned and registered 
	void publishDepthColor(const bool full_hd = true, const bool remove_points = false){
		
		cv::Mat tmp_depth, tmp_color;
		k2g_.get(tmp_depth, tmp_color, full_hd, remove_points);
		
		auto depth_image = mat_to_image(tmp_depth);
		depth_image->encoding = sensor_msgs::image_encodings::MONO8;
		depth_pub_-> publish(*depth_image);

		auto color_image =mat_to_image(tmp_color);
		color_image->encoding = sensor_msgs::image_encodings::BGRA8;
		color_pub_-> publish(*color_image);

	}

	// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
	void publishAll(const bool full_hd = true, const bool remove_points = true){

		cv::Mat tmp_depth, tmp_color;

		k2g_.get(tmp_color, tmp_depth, cloud_, full_hd, remove_points);

		tmp_depth.convertTo(tmp_depth, CV_8UC1, -256.0 / 65535.0, 256.0);

		auto depth_image = mat_to_image(tmp_depth);

		depth_image->encoding = sensor_msgs::image_encodings::MONO8;
		depth_pub_-> publish(*depth_image);

		auto color_image=mat_to_image(tmp_color);
		color_image->encoding = sensor_msgs::image_encodings::BGRA8;
		color_pub_-> publish(*color_image);

		sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    	pcl::toROSMsg(*cloud_, *pc2_msg_);
		pc2_msg_->header.frame_id = "kinect2";

		pc2_msg_->header.stamp = now();
		point_cloud_pub_-> publish(*pc2_msg_);
	}

	~K2GRos()
	{
	// Stop and close the device
	k2g_.shutDown();
	}

	void shutDown(){
		k2g_.shutDown();
	}

	
	void publishCameraInfoColor(){
		color_info_pub_-> publish(camera_info_color_);
	}

	void publishCameraInfoDepth(){
		depth_info_pub_-> publish(camera_info_depth_);
	}

	void mirror(){
		k2g_.mirror();
	}

   
private:

	std::shared_ptr<sensor_msgs::msg::Image> mat_to_image(cv::Mat mat)
	{
	auto image = std::make_shared<sensor_msgs::msg::Image>();

	image->height = mat.rows;
	image->width = mat.cols;
	image->step = mat.step;

	image->encoding = cv::typeToString(mat.type());

	image->data.resize(image->step * image->height);
	
	image->header.stamp = now();
	image->header.frame_id = "kinect2";
	std::copy(mat.datastart, mat.dataend, image->data.begin());

	return image;
	}


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		return k2g_.getCloud();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		return k2g_.updateCloud(cloud);
	}


	void createCameraInfoColor(libfreenect2::Freenect2Device::ColorCameraParams color_params)
	{
	  cv::Mat proj_matrix_color = cv::Mat::zeros(3, 4, CV_64F);
	  cv::Mat camera_matrix_color = cv::Mat::eye(3, 3, CV_64F);
	  cv::Mat distortion_matrix_color = cv::Mat::zeros(1, 5, CV_64F);

	  camera_matrix_color.at<double>(0, 0) = color_params.fx;
	  camera_matrix_color.at<double>(1, 1) = color_params.fy;
	  camera_matrix_color.at<double>(0, 2) = color_params.cx;
	  camera_matrix_color.at<double>(1, 2) = color_params.cy;
	  camera_matrix_color.at<double>(2, 2) = 1;
	  camera_matrix_color.copyTo(proj_matrix_color(cv::Rect(0, 0, 3, 3)));
	  
	  createCameraInfo(size_color_, camera_matrix_color, distortion_matrix_color, cv::Mat::eye(3, 3, CV_64F), 
	  																   proj_matrix_color, camera_info_color_, true);
	}

	void createCameraInfoDepth(libfreenect2::Freenect2Device::IrCameraParams ir_params)
	{
		cv::Mat proj_matrix_depth = cv::Mat::zeros(3, 4, CV_64F);
		cv::Mat camera_matrix_depth = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distortion_matrix_depth = cv::Mat::zeros(1, 5, CV_64F);

		camera_matrix_depth.at<double>(0, 0) = ir_params.fx;
		camera_matrix_depth.at<double>(1, 1) = ir_params.fy;
		camera_matrix_depth.at<double>(0, 2) = ir_params.cx;
		camera_matrix_depth.at<double>(1, 2) = ir_params.cy;
		camera_matrix_depth.at<double>(2, 2) = 1;
		camera_matrix_depth.copyTo(proj_matrix_depth(cv::Rect(0, 0, 3, 3)));
		
		createCameraInfo(size_depth_, camera_matrix_depth, distortion_matrix_depth, cv::Mat::eye(3, 3, CV_64F), 
																		   proj_matrix_depth, camera_info_depth_, false);
	}

	void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, 
						  const cv::Mat &projection, sensor_msgs::msg::CameraInfo &cameraInfo, const bool color ) const
	{

		if (color)
		{
			cameraInfo.header.frame_id = "kinect2_rgb_optical_frame";	
		}
		else
		{
			cameraInfo.header.frame_id = "kinect2_ir_optical_frame";	
		}
		cameraInfo.height = size.height;
		cameraInfo.width = size.width;

		const double *itR = rotation.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itR)
		{
			cameraInfo.r[i] = *itR;
		}

		const double *itP = projection.ptr<double>(0, 0);
		for(size_t i = 0; i < 12; ++i, ++itP)
		{
			cameraInfo.p[i] = *itP;
		}

		cameraInfo.distortion_model = "plumb_bob";
		cameraInfo.d.resize(distortion.cols);
		const double *itD = distortion.ptr<double>(0, 0);
		
		for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
		{
			cameraInfo.d[i] = *itD;
		}
	}

	void timer_callback(){
		publishAll();
	}




	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    sensor_msgs::msg::PointCloud2 point_cloud_2_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
    cv::Mat color_, depth_;
    cv::Size size_color_, size_depth_;
	rclcpp::TimerBase::SharedPtr timer_;
    K2G k2g_;
    libfreenect2::SyncMultiFrameListener * listener_;
    sensor_msgs::msg::CameraInfo  camera_info_color_, camera_info_depth_;

};
