
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int64.h"
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>


/* Autor: 	David Reiser
 * Date:	28.01.16
 * Description:
 * reading a sonar sensor value of std_msgs::Int64 in cm, taking the input sonar frame as position,
 * and assemble the values as a 3D pointcloud correlating to the fixed frame.
 * Optional a voxel grid can be applied to always get the highest point of the points
 * (was used as plant detection algorithm for the 3D frame
 * apply a voxel grid  * with the size x, y,z

 *  * */

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class ASSEMBLER
{
public:
	boost::mutex            lock1_;
    boost::mutex            lock2_;
    boost::mutex            lock3_;
    
    tf::TransformListener *tf_listener; 
    //variables for the voxel grid	
	int neigbour_nr;
	double radius, distance,frequency;
	double x,y,z;
	std::string fixed_frame, sonar_frame;
	bool apply_voxel;

	ros::Publisher cloud_out_pub;
	//complete cloud
	pcl::PointCloud<pcl::PointXYZI> assembled_cloud;
	
	ASSEMBLER()
	{
		//applied voxel grid values
		x=0.1;
		y=0.1;
		z=0.1;
		tf_listener=new tf::TransformListener();
		apply_voxel=false;
	}
	
	~ ASSEMBLER()
	{
	}
	
	void readSonar(const std_msgs::Int64::ConstPtr& msg)
	{
		ROS_INFO("got point");
		
		//create a pcl point cloud first...and transform
		 pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		 pcl::PointCloud<pcl::PointXYZI> transform_dummy;
		  sensor_msgs::PointCloud2 transformed_cloud; 
		 //create point of the actual sonar position in xyz directly set point
		 pcl::PointXYZI sonar_point;
		 sonar_point.x=0;
		 sonar_point.y=0;
		 sonar_point.z=msg->data/100.0;
		 //set a intensity... first just use the z value...
		 sonar_point.intensity=msg->data;
		 //set the point to a point cloud
		 transform_dummy.push_back(sonar_point);
		 transform_dummy.height=1;
		 transform_dummy.width=transform_dummy.points.size();
		 pcl::toROSMsg(transform_dummy,transformed_cloud);
		 transformed_cloud.header.frame_id=sonar_frame;
		 transformed_cloud.header.stamp=ros::Time(0);
		 //transform point in the right frame
		 try{
		 tf_listener->waitForTransform(fixed_frame,sonar_frame,ros::Time(0),ros::Duration(5.0));
		 pcl_ros::transformPointCloud(fixed_frame,transformed_cloud,transformed_cloud,*tf_listener);
			}
			catch(...)
			{
				ROS_INFO("transform error");
			}
		 pcl::fromROSMsg(transformed_cloud,*input);
		 assembled_cloud=assembled_cloud+*input;
		 pcl::copyPointCloud(assembled_cloud,*input);
		 //apply voxel filter to the dataset
		 if(apply_voxel)
		 {
			input=apply_voxel_filter(input,x,y,z);
		 }
		 //copy pointcloud to assembled cloud
		 pcl::copyPointCloud(*input,assembled_cloud);
		 sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		 pcl::toROSMsg(assembled_cloud,cloud);
		 //set the new header and size tranforms
		 cloud.header.frame_id=fixed_frame;
		 cloud.header.stamp=ros::Time::now();
		 cloud.width=assembled_cloud.points.size();
		 cloud_out_pub.publish(cloud);
		 ROS_INFO("read_data cloud1 with %d points",assembled_cloud.points.size());
		
	}
			
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI>::Ptr apply_voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr input,float x, float y,float z)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
		 
		 pcl::VoxelGrid<pcl::PointXYZI> vox_grid;

		if(input->points.size()>0)
		{
		vox_grid.setLeafSize (x,y,z);
		// ... and downsampling the point cloud
		//std::cout << "PointCloud before filtering: " << input->width * input->height << " data points." <<endl;
		vox_grid.setInputCloud (input);
		vox_grid.filter (*output);
		//std::cout << "PointCloud after filtering: " << output->width * output->height << " data points." << endl;
		}
								
		return output;
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 ASSEMBLER o;
std::string sonar_in_str,cloud_out_str;

	  n.param<std::string>("sonar_in", sonar_in_str, "/sonar"); 
	  n.param<std::string>("sonar_cloud", cloud_out_str, "/sonar_cloud");
	  n.param<std::string>("fixed_frame", o.fixed_frame, "/odom");
	  n.param<std::string>("sonar_frame", o.sonar_frame, "/sonar_frame");
	    
	   n.param<double>("x", o.x, 5);
	   n.param<double>("y", o.y, 5);
	   n.param<double>("z", o.z, 10);
	   n.param<double>("frequency", o.frequency, 10);
	   n.param<bool>("apply_voxel",o.apply_voxel,false);
	  	  
	  ros::Subscriber sub_sonar = n.subscribe(sonar_in_str,1, &ASSEMBLER::readSonar, &o);
	  
      //create publishers
      o.cloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out_str.c_str(), 1);
	
	ros::spin();
  

}
