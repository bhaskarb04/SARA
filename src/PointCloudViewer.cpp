#include "PointCloudViewer.h"


pointCloudViewer::pointCloudViewer(){

	clouds = new osg::PositionAttitudeTransform;
	planes = new osg::PositionAttitudeTransform;
	//planes->addChild(osgDB::readNodeFile("cessna.osg"));
	root = new osg::Group;

	root->addChild(clouds);
	root->addChild(planes);

	this->setSceneData(root);
	this->setUpViewInWindow(10,10,800,600);
	this->realize();
	this->setCameraManipulator(new osgGA::TrackballManipulator);
	
}
pointCloudViewer::~pointCloudViewer(){
	root->removeChildren(0,root->getNumChildren());
}
void pointCloudViewer::removeClouds(){
	clouds->removeChildren(0,clouds->getNumChildren());
}
void pointCloudViewer::addPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcldata){
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
  
	osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
	
	for (int i=0; i<pcldata->points.size(); i++) {
	vertices->push_back (osg::Vec3 (pcldata->points[i].x, pcldata->points[i].y, pcldata->points[i].z));
	uint32_t rgb_val_;
	memcpy(&rgb_val_, &(pcldata->points[i].rgb), sizeof(uint32_t));
  
	uint32_t red,green,blue;
	blue=rgb_val_ & 0x000000ff;
	rgb_val_ = rgb_val_ >> 8;
	green=rgb_val_ & 0x000000ff;
	rgb_val_ = rgb_val_ >> 8;
	red=rgb_val_ & 0x000000ff;
  
	colors->push_back (osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f));
	}
  
	geometry->setVertexArray (vertices.get());
	geometry->setColorArray (colors.get());
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
  
	geode->addDrawable (geometry.get());
	osg::StateSet* state = geometry->getOrCreateStateSet();
	state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
	clouds->addChild(geode);
}