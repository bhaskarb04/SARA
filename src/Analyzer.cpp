#include "Analyzer.h"
//void histogram();

Analyzer::Analyzer(std::string filename){
	pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Record show");
	viewer->setBackgroundColor (0, 0, 0);
	//viewer->addCoordinateSystem (1.0);
	int count = 0;
//	while(!viewer->wasStopped()){
		//char num[5];
		//sprintf(num,"%03d",count++);
		//std::string str = "Cloud_"+std::string(num);
		
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZ>);
	   
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename,*cloud) == -1)
			return;
	   
		//pcl::copyPointCloud(*pcloud,*cloud);
		/*for(int i=0;i<cloud->size();i++){
			uint8_t r = 255;
			uint8_t g = 0;
			uint8_t b = 0;
			int32_t rgb = (r << 16) | (g << 8) | b; 
			cloud->at(i).rgb = *(float *)(&rgb);
		}*/
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);
		/*if(coefficients->values.size()){
			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
												<< coefficients->values[1] << " "
												<< coefficients->values[2] << " " 
												<< coefficients->values[3] << std::endl;
		}*/

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane1(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane2(new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		/*for(int i=inliers->indices.size()-1;i>=0;i--){
			cloud->at(inliers->indices[i]).r = 0;
			cloud->at(inliers->indices[i]).g = 255;
			cloud->at(inliers->indices[i]).b = 0;
			plane1->push_back(cloud->at(inliers->indices[i]));
			cloud->erase(cloud->begin()+inliers->indices[i]);
		}*/

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		// Extract the inliers
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud);

		viewer->addPlane(*coefficients,"plane1");
		seg.segment (*inliers, *coefficients);
		
		extract.setIndices (inliers);
		extract.filter (*cloud);
		
		if(coefficients->values.size()){
			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
												<< coefficients->values[1] << " "
												<< coefficients->values[2] << " " 
												<< coefficients->values[3] << std::endl;
		}
		/*for(int i=inliers->indices.size()-1;i>=0;i--){
			cloud->at(inliers->indices[i]).r = 0;
			cloud->at(inliers->indices[i]).g = 255;
			cloud->at(inliers->indices[i]).b = 0;
			plane2->push_back(cloud->at(inliers->indices[i]));
			cloud->erase(cloud->begin()+inliers->indices[i]);
		}*/

		for(int i=0;i<cloud->size();i++){
			double dist = abs(coefficients->values[0]*cloud->at(i).x +
								coefficients->values[1]*cloud->at(i).y + 
								coefficients->values[2]*cloud->at(i).z + 
								coefficients->values[3])/
								sqrt(coefficients->values[0]*coefficients->values[0] + 
								coefficients->values[1]*coefficients->values[1] +
								coefficients->values[2]*coefficients->values[2]);
			if(dist < 0.1){
				cloud->erase(cloud->begin()+i);
				i--;
			}

		}
		
		viewer->addPlane(*coefficients,"plane2");

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbfloor1(plane1);
		//viewer->addPointCloud<pcl::PointXYZRGBA> (plane1, rgbfloor1, "floor1");
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgbfloor2(plane2);
		//viewer->addPointCloud<pcl::PointXYZRGBA> (plane2, rgbfloor2, "floor2");

		//pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		//tree->setInputCloud (cloud);

		//std::vector<pcl::PointIndices> cluster_indices;
		//pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		//ec.setClusterTolerance (0.02); // 2cm
		//ec.setMinClusterSize (100);
		//ec.setMaxClusterSize (25000);
		//ec.setSearchMethod (tree);
		//ec.setInputCloud (cloud);
		//ec.extract (cluster_indices);

		

		viewer->spin();
		boost::this_thread::sleep( boost::posix_time::microseconds(100));
//	}
   
	return;
}

//void histogram(string filename,string windowname)
//{
//  Mat src, dst;
//
//  /// Load image
//  src = imread( filename, 1 );
//
//  if( !src.data )
//    { return ; }
//
//  /// Separate the image in 3 places ( B, G and R )
//  vector<Mat> bgr_planes;
//  split( src, bgr_planes );
//
//  /// Establish the number of bins
//  int histSize = 256;
//
//  /// Set the ranges ( for B,G,R) )
//  float range[] = { 0, 256 } ;
//  const float* histRange = { range };
//
//  bool uniform = true; bool accumulate = false;
//
//  Mat b_hist, g_hist, r_hist;
//
//  /// Compute the histograms:
//  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
//  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
//  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
//
//  // Draw the histograms for B, G and R
//  int hist_w = 512; int hist_h = 400;
//  int bin_w = cvRound( (double) hist_w/histSize );
//
//  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
//
//  /// Normalize the result to [ 0, histImage.rows ]
//  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//
//  /// Draw for each channel
//  for( int i = 1; i < histSize; i++ )
//  {
//      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
//                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
//                       Scalar( 255, 0, 0), 2, 8, 0  );
//      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
//                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
//                       Scalar( 0, 255, 0), 2, 8, 0  );
//      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
//                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
//                       Scalar( 0, 0, 255), 2, 8, 0  );
//  }
//
//  /// Display
//  namedWindow(windowname, CV_WINDOW_AUTOSIZE );
//  imshow(windowname, histImage );
//
//  waitKey(10);
//}