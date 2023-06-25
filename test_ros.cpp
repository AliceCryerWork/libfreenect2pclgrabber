/*
Copyright 2016, Giacomo Dabisias & Michele Mambrini"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo  Dabisias, PhD Student & Michele Mambrini
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g_ros.h"
#include <rclcpp/rclcpp.hpp>
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>



int main(int argc, char * argv[])
{
	std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
	Processor freenectprocessor = OPENGL;

	if(argc > 1){
		freenectprocessor = static_cast<Processor>(atoi(argv[1]));
	}

	rclcpp::init(argc, argv);
	//K2GRos K2G_ros;
		  // Try to start the Kinect V2
  try {
    auto kinect2_node = std::make_shared<K2GRos>(freenectprocessor);
    rclcpp::spin(kinect2_node);

  } catch (const std::exception & e) {
    std::cout << "Failed to initialize kinect2_node: " << e.what() << std::endl;
  }


	//K2G_ros.shutDown();


  rclcpp::shutdown();

  return 0;

}

