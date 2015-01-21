#include "PointCloudViewer.h"
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/LineWidth>

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
	//this->setHome(osg::Vec3d(-0.54,-0.38,-1.53),osg::Vec3d(-0.21,-0.15,-0.62),osg::Vec3d(-0.1,-0.9,0.27));
	
}
pointCloudViewer::~pointCloudViewer(){
	root->removeChildren(0,root->getNumChildren());
}
void pointCloudViewer::removeClouds(){
	clouds->removeChildren(0,clouds->getNumChildren());
}
void pointCloudViewer::removeShapes(){
	planes->removeChildren(0,planes->getNumChildren());
}

osg::ref_ptr<osg::Geode> pointCloudViewer::convertPointCloudToGeode(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcldata, osg::Vec4& coloravg){
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
  
	osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
	coloravg = osg::Vec4(0,0,0,0);
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
	coloravg = coloravg + colors->at(i);
	}
	coloravg = coloravg / pcldata->points.size();
	geometry->setVertexArray (vertices.get());
	geometry->setColorArray (colors.get());
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
  
	geode->addDrawable (geometry.get());
	osg::StateSet* state = geometry->getOrCreateStateSet();
	state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
	return geode;
}

osg::ref_ptr<osg::Geode> pointCloudViewer::convertPointCloudToGeode(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcldata){
	osg::Vec4 dummy;
	return (convertPointCloudToGeode(pcldata,dummy));
}
void pointCloudViewer::addPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcldata){
	
	clouds->addChild(convertPointCloudToGeode(pcldata));
	//this->setHome(clouds->getBound().center()-osg::Vec3d(1,1,1),clouds->getBound().center(),osg::Vec3(0,1,0));
	//osg::BoundingBox mybbox = convertPointCloudToGeode(pcldata)->getBoundingBox();
	//
	//osg::Box* unitCube = new osg::Box(mybbox.center(),mybbox.xMax() - mybbox.xMin(),
	//												  mybbox.yMax() - mybbox.yMin(),
	//												  mybbox.zMax() - mybbox.zMin());

	//// Declare an instance of the shape drawable class and initialize 
	//// it with the unitCube shape we created above.
	//// This class is derived from 'drawable' so instances of this
	//// class can be added to Geode instances.
	//osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);

	//// Declare a instance of the geode class: 
	//osg::Geode* basicShapesGeode = new osg::Geode();

	//// Add the unit cube drawable to the geode:
	//basicShapesGeode->addDrawable(unitCubeDrawable);

	//basicShapesGeode->getOrCreateStateSet()->
	//				setAttribute( new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE) );
	//osg::StateSet* stateset = basicShapesGeode->getOrCreateStateSet();
 //   osg::LineWidth* linewidth = new osg::LineWidth();
 //   linewidth->setWidth(2.0f);
 //   stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
 //   stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

	//// Add the goede to the scene:
	//clouds->addChild(basicShapesGeode);

	//osg::Vec3d eye,center,up;
	//this->getCameraManipulator()->getInverseMatrix().getLookAt(eye,center,up);
}

void pointCloudViewer::addShape(osg::Node* node){
	planes->addChild(node);
}

void pointCloudViewer::setHome(osg::Vec3d eye ,osg::Vec3d center ,osg::Vec3d up){
	this->getCameraManipulator()->setHomePosition(eye,center,up);
	this->home();
}

osg::BoundingBox pointCloudViewer::getBoundingBoxFromCloud(){
	osg::BoundingBox box;
	if(clouds->getNumChildren())
		return clouds->getChild(0)->asGeode()->getBoundingBox();
	return box;
}