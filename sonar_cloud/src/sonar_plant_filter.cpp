#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>


/* reading the input cloud of a scanned sonar cloud of the phoenix frame,
 * and reduce the information to the center points of the plants.
 * for that first a noise filter get applied afterwards a
 * ransac line fitting gets used to remove the ground plane.
 * the last step is to create the center point out of the resulting plant positions
 * inputs neigbour_nr and radius specify the noise removal 
 * distance the cluster distance for the plants
 * 
 * by David Reiser 18.3.16
 *  * */

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


class PCL_CLASS
{
public:
	
	int neigbour_nr;
	double radius, distance;
	
// public variables
	ros::Publisher cloud_out_pub;
	
	PCL_CLASS()
	{
		
	}
	
	~ PCL_CLASS()
	{
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
		//now there is some processing neccessary...convert msg to pcl point cloud		
		pcl::fromROSMsg(*msg, *input);
		//apply ransac plane removal:
		input=remove_ground_plane(input);
		//now remove outliers:
		input= apply_outlier_filter(input, neigbour_nr,radius);
		//cluter clouds with the given distance,and get out the center of every cluster
		input=cluster_clouds(input,distance);
			
		publish_data(input,msg->header.frame_id);
		
	}
	//-----------------------------------------------------------------------
	void publish_data(pcl::PointCloud<pcl::PointXYZ>::Ptr input,std::string frame_id)
	{
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(*input,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=frame_id;
		cloud.header.stamp=ros::Time::now();
		cloud.width=input->points.size();
		cloud_out_pub.publish(cloud);
				
	}
	
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double dist)
	{
		
		 pcl::PointCloud<pcl::PointXYZ>::Ptr center_points (new pcl::PointCloud<pcl::PointXYZ>);
		 // Creating the KdTree object for the search method of the extraction
		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (input);

		  std::vector<pcl::PointIndices> cluster_indices;
		  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		  ec.setClusterTolerance (dist); // in cm
		  ec.setMinClusterSize (10);
		  ec.setMaxClusterSize (25000);
		  ec.setSearchMethod (tree);
		  ec.setInputCloud (input);
		  ec.extract (cluster_indices);
	
		  int j = 0;
		  //now move trough all clusters of the point cloud...
		  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		  {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (input->points[*pit]); 
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			//compute centroid of point cloud
			pcl::PointXYZ center;
			Eigen::Vector4f centroid; 
			pcl::compute3DCentroid(*cloud_cluster,centroid); 
			center.x=centroid[0];
			center.y=centroid[1];
			center.z=centroid[2];
			center_points->push_back(center);
			
			j++;
		  }
		
		center_points->width=center_points->points.size();
		
		//return input;
		return center_points;
		
		
	}
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr remove_ground_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		//see also 	pcl  cluster extraction tutorial
		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		// Create the segmentation object for the planar model and set all the parameters
		  pcl::SACSegmentation<pcl::PointXYZ> seg;
		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		  pcl::PCDWriter writer;
		  seg.setOptimizeCoefficients (true);
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (100);
		  seg.setDistanceThreshold (0.05);

		  int i=0, nr_points = (int) cloud->points.size ();
		  while (cloud->points.size () > 0.3 * nr_points)
		  {
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud (cloud);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
			  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			  break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (cloud);
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);
			std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud = *cloud_f;
		}
		
		
	
		return cloud;
	}
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr apply_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int nr, double radius)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(radius);
		outrem.setMinNeighborsInRadius (nr);
		// apply filter
		outrem.filter (*cloud_filtered);
	
			std::cerr << "Cloud before filtering: " << cloud->points.size()<< "and after " <<cloud->points.size()<< std::endl;			
		return cloud_filtered;
		
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 PCL_CLASS o;
std::string front_str,cloud_out;

	  n.param<std::string>("cloud_in", front_str, "/error_cloud_front"); 
	  n.param<std::string>("cloud_out", cloud_out, "/error_cloud_front"); 
	  
	   n.param<double>("radius", o.radius, 5);
	   n.param<int>("neigbour_nr", o.neigbour_nr, 5);
	   n.param<double>("distance", o.distance, 10);
	  	  
	  ros::Subscriber sub_laser_front = n.subscribe(front_str,1, &PCL_CLASS::readFrontCloud, &o);
	
	  
      //create publishers
      o.cloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 1);
      
	
	ros::spin();
  

}
