#include "imgprocesscallback.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl\ModelCoefficients.h>
#include <pcl\point_types.h>

#include <pcl\filters\extract_indices.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\features\normal_3d.h>
#include <pcl\kdtree\kdtree.h>
#include <pcl\segmentation\extract_clusters.h>
#include <stdint.h>
#include "HandModel.h"
#include <osg\Array>
#include <osg\Geode>
#include <osg\PositionAttitudeTransform>
#include <osg/ShapeDrawable>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertMatToPCL(cv::Mat depthimage){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	vector<cv::Mat> xyzplanes;
	cv::split(depthimage,xyzplanes);
	float* xdata = (float*)xyzplanes[0].data;
	float* ydata = (float*)xyzplanes[1].data;
	float* zdata = (float*)xyzplanes[2].data;

	for(int i=0;i<depthimage.rows;i++){
		for(int j=0;j<depthimage.cols;j++){
			pcl::PointXYZRGBA point;
			point.x = *xdata;
			point.y = *ydata;
			point.z = *zdata;
			point.rgba = 0;
			cloud->push_back(point);
			xdata++;ydata++;zdata++;
		}
	}
	xdata = NULL; ydata = NULL; zdata = NULL;
	return cloud;
}


imgprocesscallback::imgprocesscallback(cv::Mat* frame,cv::Mat* depthframe,osg::Group* root):
_frame(frame),_prevframe(*frame),_depthframe(depthframe),_bgframe(*depthframe),_root(root)
{
	HandModel hm;
	pcl::ModelCoefficients::Ptr mc =  hm.removeplane(convertMatToPCL(*depthframe));

	osg::Vec3 a(0,1,0);
	osg::Vec3 b(mc->values[0],mc->values[1],mc->values[2]);
	osg::Quat q;

	osg::Vec3 v = a^b;
	double s = v.length();
	double c = a*b;
	float data1[16] ={0,-v.z(),v.y(),0,v.z(),0,-v.x(),0,-v.y(),v.x(),0,0,0,0,0,1};
	cv::Mat vx(4,4,CV_32FC1,data1);
	cv::Mat rot = cv::Mat::eye(4,4,CV_32FC1) + vx + vx*vx*((1-c)/(s*s));
	osg::Matrixd m((float*)rot.data);
		
	q = m.getRotate()*osg::Quat(osg::DegreesToRadians(90.0f),osg::Vec3(1,0,0));

	//osg::Box* unitCube = new osg::Box(osg::Vec3(0,0,0), 10.0f,0.01f,10.0f);
	//osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube); 
	//osg::Geode* basicShapesGeode = new osg::Geode();
	//basicShapesGeode->addDrawable(unitCubeDrawable);
	//osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform;
	//pat->addChild(basicShapesGeode);
	//pat->setAttitude(q);
	//root->addChild(pat);
}


imgprocesscallback::~imgprocesscallback(void)
{
}

void imgprocesscallback::operator()(osg::Node* node, osg::NodeVisitor* nv){
	
	//cv::imshow("abcd",cv::abs(*_depthframe - _bgframe));
	//cv::waitKey(20);
	traverse(node,nv);
}
