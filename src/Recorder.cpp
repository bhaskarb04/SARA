#include "Recorder.h"


void SimpleOpenNIViewer::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
		
	if (!viewer.wasStopped()){
		//}
		// if(cv::waitKey(50) == 27)
		// record = !record;
		if(record){
			char num[5];
			sprintf(num,"%03d",count++);
			std::string str = "CloudRGBAll_"+std::string(num);
			cout<<"Saving "<<str<<endl;
				pcl::io::savePCDFileBinary(str, *cloud );
		}

		viewer.showCloud (cloud);
	}
}

void SimpleOpenNIViewer::run(){

	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

	interface->registerCallback (f);

	interface->start ();
#ifdef RECORD
	record = RECORD;
#else
	record = false;
#endif
	count =0;
	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep (boost::posix_time::seconds (1));
	}

	interface->stop ();
}