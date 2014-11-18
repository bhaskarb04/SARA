#pragma once
#ifndef _POINT_CLOUD_VIEWER_
#define _POINT_CLOUD_VIEWER_
#pragma warning( disable : 4996 )

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
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

#include <osg\Array>
#include <osg\Geode>
#include <osg\PositionAttitudeTransform>
#include <osgViewer\Viewer>
#include <osgDB\ReadFile>
#include <osgGA\TrackballManipulator>

class pointCloudViewer : public osgViewer::Viewer{
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<osg::PositionAttitudeTransform> clouds;
	osg::ref_ptr<osg::PositionAttitudeTransform> planes;
public:
	pointCloudViewer();
	~pointCloudViewer();
	void removeClouds();
	void addPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
};

#endif