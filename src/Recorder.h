#pragma warning( disable : 4996 )

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl\io\pcd_io.h>

//#define RECORD false

class SimpleOpenNIViewer{
public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	void run ();

	pcl::visualization::CloudViewer viewer;
	bool record;
	int count;
};

 
