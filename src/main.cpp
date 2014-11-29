#pragma warning( disable : 4996 )
#include "Controller.h"
#include <pcl\registration\icp.h>
#include <Eigen\src\Geometry\Transform.h>
#include <osg/LineWidth>
#include "myColorVisitor.h"

#define VISUALIZE_ONLY

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;
  }
}

osg::Geode* createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir)
{
    // set up the Geometry.
	osg::Geode* geode = new osg::Geode;
    osg::Geometry* geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(6);
    (*coords)[0] = corner;
    (*coords)[1] = corner+xdir;
    (*coords)[2] = corner;
    (*coords)[3] = corner+ydir;
    (*coords)[4] = corner;
    (*coords)[5] = corner+zdir;

    geom->setVertexArray(coords);

    osg::Vec4 x_color(1.0f,0.0f,0.0f,1.0f);
    osg::Vec4 y_color(0.0f,1.0f,0.0f,1.0f);
    osg::Vec4 z_color(0.0f,0.0f,1.0f,1.0f);

    osg::Vec4Array* color = new osg::Vec4Array(6);
    (*color)[0] = x_color;
    (*color)[1] = x_color;
    (*color)[2] = y_color;
    (*color)[3] = y_color;
    (*color)[4] = z_color;
    (*color)[5] = z_color;

    geom->setColorArray(color);
	geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

    osg::StateSet* stateset = new osg::StateSet;
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(4.0f);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    geom->setStateSet(stateset);
	geode->addDrawable(geom);
    return geode;
}

void convertGeodeToPCL(osg::Node* node, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, double scale = 1.0){
	if(node->asGeode()){
		osg::Geode* geode = node->asGeode();
		for(int i=0;i < geode->getNumDrawables();i++){
			osg::Geometry* geometry = geode->getDrawable(i)->asGeometry();
			if(geometry){
				osg::Vec3Array* vertexarray = (osg::Vec3Array*)geometry->getVertexArray();
				for(int j =0;j < vertexarray->size();j++){
					pcl::PointXYZRGBA mypt;
					mypt.rgb = 0;
					mypt.x = vertexarray->at(j).x()*scale;
					mypt.y = vertexarray->at(j).y()*scale;
					mypt.z = vertexarray->at(j).z()*scale;
					cloud->push_back(mypt);
				}
			}
		}
	}
}


bool myfunction (pcl::PointIndices i,pcl::PointIndices j) { return (i.indices.size() > j.indices.size()); }

std::vector<pointCloudViewer::blockHypothesis> boundingBoxes;

void fixBoundingBoxes(){
	double gz = boundingBoxes[0].box.zMax();
	for(int i=0;i<boundingBoxes.size();i++){
		osg::BoundingBox* box = &boundingBoxes[i].box;
		double zmin = box->zMin(); + (zmin-gz);
		double xmin = box->xMin();// + (zmin-gz);
		double ymin = box->yMin() - (zmin-gz)*0.5;
		double xmax = box->xMax();// + (zmin-gz);
		double ymax = box->yMax() - (zmin-gz)*0.5;
		double zmax = box->zMax(); + (zmin-gz);
		box->set(xmin,ymin,zmin,xmax,ymax,zmax);
	}
}
void remove_box_points(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){
	if(!boundingBoxes.size())
		return;
	for(int i= 0; i < cloud->size();i++){
		for(int j=0;j<boundingBoxes.size();j++){
			std::vector<osg::Vec3f> points;
			points.push_back(osg::Vec3f(cloud->at(i).x-0.02,cloud->at(i).y-0.02,cloud->at(i).z-0.02));
			points.push_back(osg::Vec3f(cloud->at(i).x+0.02,cloud->at(i).y+0.02,cloud->at(i).z+0.02));
			osg::BoundingBox bbpoint(points[0],points[1]);
			uint32_t rgb_val_;
			memcpy(&rgb_val_, &(cloud->points[i].rgb), sizeof(uint32_t));
  
			uint32_t red,green,blue;
			blue=rgb_val_ & 0x000000ff;
			rgb_val_ = rgb_val_ >> 8;
			green=rgb_val_ & 0x000000ff;
			rgb_val_ = rgb_val_ >> 8;
			red=rgb_val_ & 0x000000ff;
			
			osg::Vec4 pointcolor = osg::Vec4f ((float)red/255.0f, (float)green/255.0f, (float)blue/255.0f,1.0f);
			if(boundingBoxes[j].box.intersects(bbpoint) && (boundingBoxes[j].color - pointcolor).length2() < 0.05 ){
				cloud->erase(cloud->begin() + i);
				i--;
				break;
			}
		}
	}
	//remove small noisy points
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud (cloud);
			
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance (0.003);
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (cloud->size());
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
	if(cluster_indices.size()){
		std::sort(cluster_indices.begin(),cluster_indices.end(),myfunction);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = cluster_indices.begin()->indices.begin (); 
			pit != cluster_indices.begin()->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		cloud->clear();
		cloud = cloud_cluster;
	}
}

osg::Quat find_rotation(osg::BoundingBox box, osg::Node* node){
	
	//osg::Vec3 a,b;
	osg::Quat q;
	//osg::BoundingBox boxnode;
	double xmin = box.xMin() - box.center().x();
	double xmax = box.xMax() - box.center().x();
	double ymin = box.yMin() - box.center().y();
	double ymax = box.yMax() - box.center().y();
	double zmin = box.zMin() - box.center().z();
	double zmax = box.zMax() - box.center().z();

	double xdiff = box.xMax() - box.xMin();
	double ydiff = box.yMax() - box.yMin();
	double zdiff = box.zMax() - box.zMin();
	
	if( xdiff > ydiff){
		q = osg::Quat();
	}
	if(ydiff > xdiff ){
		if(box.xMin() < 0)
			q = osg::Quat(osg::DegreesToRadians(90.0),osg::Vec3(0,1,0));//*osg::Quat(osg::DegreesToRadians(180.0),osg::Vec3(0,1,0));
		else
			q = osg::Quat(osg::DegreesToRadians(90.0),osg::Vec3(0,1,0));//*osg::Quat(osg::DegreesToRadians(180.0),osg::Vec3(0,1,0));
	}
	//a = osg::Vec3(xmax-xmin,ymax-ymin,zmax-zmin);

	//if(node->asGeode()){
	//	boxnode = node->asGeode()->getBoundingBox();
	//	b = osg::Vec3(boxnode.xMax() - boxnode.xMin(),boxnode.yMax() - boxnode.yMin(),boxnode.zMax() - boxnode.zMin());
	//	osg::Vec3 v = a^b;
	//	double s = v.length();
	//	double c = a*b;
	//	float data1[16] ={0,-v.z(),v.y(),0,v.z(),0,-v.x(),0,-v.y(),v.x(),0,0,0,0,0,1};
	//	cv::Mat vx(4,4,CV_32FC1,data1);
	//	cv::Mat rot = cv::Mat::eye(4,4,CV_32FC1) + vx + vx*vx*((1-c)/(s*s));
	//	osg::Matrixd m((float*)rot.data);
	//	
	//	q = m.getRotate();
	//}
	return(q);

}

void visualize_only(std::string base,int start, int end){
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> blocks;
	pointCloudViewer *pclviewer = new pointCloudViewer;
	//pclviewer->setHome(osg::Vec3d(-0.54,-0.38,-1.53),osg::Vec3d(-0.21,-0.15,-0.62),osg::Vec3d(-0.1,-0.9,0.27));
#ifdef VISUALIZE_ONLY
	//pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Record show");
	//viewer->setBackgroundColor (0, 0, 0);
	
#endif
	//std::ifstream file;
	//file.open("../Data/info.data",std::ios_base::in);
	skinfunction_1 skinmodel;
	skinmodel.load_function();
	HandModel handmodel;
	Background background;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudprev(new pcl::PointCloud<pcl::PointXYZRGBA>);
	int handcount = 0;
	int prevhandcount = 0;
	//while (!pclviewer->done()){
		bool prevExists = false;
		bool handPresent = false;
		bool firsttime = true;
		boundingBoxes.clear();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr overallcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//pclviewer->removeClouds();
		for(int i=start;i<=end;i++){
			int x;
			//file>>x;
			//if(!x)
			//	continue;
			//cout<<i<<endl;
			char num[5];
			sprintf(num,"%03d",i);
			std::string fname = base+std::string(num)+"_0";//+std::string("_Points.dat");
			std::string fname2 = base+std::string("processed")+std::string(num);//+std::string("_Points.dat");

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
			if(QFile::exists(QString(fname.c_str()))){
				if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
					std::cerr<<"Cannot load file"<<endl;
					return;
				}
			}
			else{
				handPresent = true;
				if(!firsttime)
					handcount++;
				continue;
			}
#ifndef VISUALIZE_ONLY		
			
			background.removeBackground(cloud);
			//handmodel.removeHand(cloud);
			for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();it!=cloud->end();it++){
				double ssum = it->r+it->g+it->b;
				if(skinmodel.skincompare(it->r/ssum,it->g/ssum,it->b/ssum)){
					cloud->erase(it);
					it--;
				}
			}
			//pclviewer->removeClouds();
			//pclviewer->addPointCloud(cloud);
			//pclviewer->frame();
			//pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>(fname2,*cloud);
#endif

#ifdef VISUALIZE_ONLY
			/*if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
				std::cerr<<"Cannot load file"<<endl;
				return;
			}*/
			/*viewer->removeAllPointClouds();
			viewer->addPointCloud(cloud);
			viewer->spinOnce(100);*/
#endif
			if(!prevExists){
				cloudprev = cloud;
				//boundingBoxes.push_back(pclviewer->convertPointCloudToGeode(cloud)->getBoundingBox());
				prevExists = true;
			}
			//else if(handPresent){
				//pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
				//icp.setInputCloud(cloudprev);
				//icp.setInputTarget(cloud);
				//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBA>);
				//icp.align(*Final);
				//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
				//icp.getFitnessScore() << std::endl;
				////Eigen::Transform t = icp.getFinalTransformation();
				//std::cout << icp.getFinalTransformation() << std::endl;
			if(handPresent && handcount){
				pointCloudViewer::blockHypothesis bh;
				bh.box = pclviewer->convertPointCloudToGeode(overallcloud,bh.color)->getBoundingBox();
				boundingBoxes.push_back(bh);
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dummycloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
				for(int pp=0;pp<overallcloud->size();pp++)
					dummycloud->push_back(overallcloud->at(pp));
				blocks.push_back(dummycloud);
				overallcloud->clear();
				handPresent = false;
			}
				//handPresent = false;
				remove_box_points(cloud);
				for(int pcli = 0;pcli<cloud->size();pcli++)
					overallcloud->push_back(cloud->at(pcli));
				if(firsttime)
					firsttime = false;
				//cloudprev = cloud;
				//pclviewer->removeClouds();
				//pclviewer->addPointCloud(cloud);
			//}
			
			//pclviewer->frame();
			//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
		pointCloudViewer::blockHypothesis bh;
		bh.box = pclviewer->convertPointCloudToGeode(overallcloud,bh.color)->getBoundingBox();
		boundingBoxes.push_back(bh);
		blocks.push_back(overallcloud);
		//overallcloud->clear();
	//}
	
	osg::Node* blockorig = osgDB::readNodeFile("../Data/Blue Lego Block.3DS");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr block_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	double scalevalue = boundingBoxes[0].box.radius() / blockorig->getBound().radius();
	//convertGeodeToPCL(blockorig,block_cloud,scalevalue);
	fixBoundingBoxes();
	for(int i=0;i<boundingBoxes.size();i++){

		//Initialization
		myColorVisitor newColor;
		newColor.setColor(boundingBoxes[i].color);
		osg::PositionAttitudeTransform* pat = new osg::PositionAttitudeTransform;
		osg::Node* block = osgDB::readNodeFile("../Data/Blue Lego Block.3DS");
		//ICP method
		/*pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
		icp.setInputCloud(blocks[0]);
		icp.setInputTarget(blocks[i]);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBA>);
		icp.align(*Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;*/
		
		//pclviewer->addPointCloud(blocks[i]);
		/*cout<<boundingBoxes[i].box.xMax()<<" "<<boundingBoxes[i].box.xMin()<<" "<<
			  boundingBoxes[i].box.yMax()<<" "<<boundingBoxes[i].box.yMin()<<" "<<
			  boundingBoxes[i].box.zMax()<<" "<<boundingBoxes[i].box.zMin()<<endl;*/

		//Set up the block to match with point cloud bounding box
		
		
		pat->setPosition(boundingBoxes[i].box.center() - boundingBoxes[0].box.center());
		osg::Vec3d pos = pat->getPosition();
		pat->setPosition(osg::Vec3(pos.x(),-pos.z(),boundingBoxes[i].box.yMax()));
		cout<<"Position: "<<pos.x()<< " "<<-pos.z()<<" "<<boundingBoxes[i].box.yMax()<<endl;
		pat->setPivotPoint(boundingBoxes[i].box.corner(7));
		osg::Vec3d pivotpos = pat->getPivotPoint();
		cout<<"Pivot Position: "<<pivotpos.x()<< " "<<pivotpos.y()<<" "<<pivotpos.z()<<endl;
		//pat->setAttitude(find_rotation(boundingBoxes[i].box,block));
		//find_rotation(boundingBoxes[i].box,block);
		if(i == 1){
			pat->setAttitude(osg::Quat(osg::DegreesToRadians(-90.0),osg::Vec3(0,1,0))*
							osg::Quat(osg::DegreesToRadians(0.0),osg::Vec3(0,0,1))*
							osg::Quat(osg::DegreesToRadians(0.0),osg::Vec3(1,0,0))
			);
			pos = pat->getPosition();
			pat->setPosition(osg::Vec3(pos.x(),pos.y(),pos.z()-0.08));
		}
		pat->setScale(osg::Vec3d(scalevalue,scalevalue,scalevalue));
		
		////Fix the color and add to scenegraph
		block->accept(newColor);
		pat->addChild(block);
		pclviewer->addShape(pat);
	}
	pclviewer->addShape(createAxis(osg::Vec3(0,0,0),osg::Vec3(1,0,0),osg::Vec3(0,1,0),osg::Vec3(0,0,1)));
	while(!pclviewer->done()){
		pclviewer->frame();
	}
}



int main (){
	
	//Controller c;
	//c.setPath("../Data/Full1/CloudRGBAll_",0,35,41,86,96,160);
	//c.showFull(true);
	//c.startShow();
	//c.showFullProcessed(true);
	//c.startShow();
	//c.trainBackground();
	//c.trainManipulator();
	//c.process(true,true,false);
	//c.showImmediate("../Data/Full1/CloudRGBAll_PROCESSED",96,160);
	//return 0;
	visualize_only("../Data/Full1/CloudRGBAll_",96,160);
	return 0;

#ifdef RECORD
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
#else
	//Analyzer analyzer("../Data/CloudRGBHandOnly_020"); //CloudRGBHandOnly_060,080,20
#endif
	//HandModel hm(std::string("../Data/All/Hand/CloudRGBAll_"),41,86,false);
	/*vector<std::string> files;
	for(int i=41;i<=86;i++){
		char num[5];
		sprintf(num,"%03d",i);
		files.push_back(std::string("../Data/Full1/CloudRGBAll_SkinPoints_")+std::string(num));
	}
	HandModel hm(files);*/
	//hm.save_model();
}