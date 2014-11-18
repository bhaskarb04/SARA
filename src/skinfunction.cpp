#include "skinfunction.h"

void skinfunction_1::skintrain(vector<cv::Point3f> skinvalues){
	int step = skinvalues.size()/10; //step for validation
	rmean=0;
	double best=0;
	_mean = cv::Point3d(0,0,0);
	for(int i=0;i<10;i++){
		float minval=130;
		double msum=0;
		for(int j=0;j<skinvalues.size();j++){
			if(j > (step*i) && j < step*(i+1)+1){ //means i am within the validation block
				j= step*(i+1)+1;
				continue;
			}
			_mean.x=_mean.x+skinvalues[j].x;
			_mean.y=_mean.y+skinvalues[j].y;
			//minval=minval>skinvalues[j].x && minval > 150?skinvalues[j].x:minval;
			//msum+=skinvalues[j].x;
			
		}
		//msum/=(skinvalues.size()-step);
		//msum*=0.9;
		_mean.x/=(skinvalues.size()-step);
		_mean.y/=(skinvalues.size()-step);


		_stddev=cv::Point3d(0,0,0);
		for(int j=0;j<skinvalues.size();j++){
			if(j > (step*i) && j < step*(i+1)+1){ //means i am within the validation block
				j= step*(i+1)+1;
				continue;
			}
			_stddev.x+=(_mean.x - skinvalues[j].x)*(_mean.x - skinvalues[j].x);
			_stddev.y+=(_mean.y - skinvalues[j].y)*(_mean.y - skinvalues[j].y);
			//minval=minval>skinvalues[j].x && minval > 150?skinvalues[j].x:minval;
			//msum+=skinvalues[j].x;
			
		}
		_stddev.x=sqrt(_stddev.x/(skinvalues.size()-step));
		_stddev.y=sqrt(_stddev.y/(skinvalues.size()-step));

		float count=0;
		
		for(int j=step*i;j<step*(i+1);j++){
			float r = skinvalues[j].x;
			float g = skinvalues[j].y;
			float b = skinvalues[j].z;

			/*float Y  = 16 + 65.481*r + 128.553*g + 24.966*b;
			float Cb = 128 + -37.797*r + -74.203*g + 112*b;
			float Cr = 128 + 112*r + -93.786*g + -18.214*b;
			if((r > 140 &&
				g > 75 && g < 193 && 
				b > 43 && b < 161 &&
				abs(r-g) > 28 && abs(r-g) < 130 &&
				abs(r-b) > 45 && abs(r-b) < 187 &&
				r > g && r > b) || 
				(Y> 80 && Cr >= 135 && Cr <= 180 && Cb >=85 && Cb <= 135))	
				count++;*/
			double timesstddev = 3;
			if( r > _mean.x - timesstddev*_stddev.x && r < _mean.x + timesstddev*_stddev.x && 
				g > _mean.y - timesstddev*_stddev.y && g < _mean.y + timesstddev*_stddev.y)
				count++;
		}
		cout<<count/step<<" "<<msum<<endl;
		if(count/step > best){
			rmean = msum;
			best = count/step;
		}
	}
	//cout<<"Best: "<<best<<" "<<rmean<<endl;
	mskin=best-(1-best);
	mnonskin=0;
	mboth=1-mskin;
	trained=true;
}

bool skinfunction_1::skincompare(float r, float g, float b){
	double timesstddev = 3;
	if( r > _mean.x - timesstddev*_stddev.x && r < _mean.x + timesstddev*_stddev.x && 
				g > _mean.y - timesstddev*_stddev.y && g < _mean.y + timesstddev*_stddev.y)
				return true;
	return false;
	//cv::Mat m(1,1,CV_8UC3);
	//m.at<cv::Vec3b>(0,0)=cv::Vec3b(r,g,b);
	//cv::cvtColor(m,m,CV_RGB2YCrCb);
	float Y  = 16 + 65.481*r + 128.553*g + 24.966*b;
	float Cb = 128 + -37.797*r + -74.203*g + 112*b;
	float Cr = 128 + 112*r + -93.786*g + -18.214*b;
	if(r > 140 &&
				g > 75 && g < 193 && 
				b > 43 && b < 161 &&
				abs(r-g) > 28 && abs(r-g) < 130 &&
				abs(r-b) > 45 && abs(r-b) < 187 &&
				r > g && r > b)
				return true;
	//cv::Vec3b c = m.at<cv::Vec3b>(0,0);
	if( Y> 80 && Cr >= 135 && Cr <= 180 &&
		Cb >=85 && Cb <= 135)
			return true;

	return false;
}

void skinfunction_1::save_function(){
	std::ofstream file;
	file.open("skinfunction1.dat",std::ios_base::out);
	file<<_mean.x<<endl;
	file<<_mean.y<<endl;
	file<<_stddev.x<<endl;
	file<<_stddev.y<<endl;
	file.close();
}

void skinfunction_1::load_function(){
	std::ifstream file;
	file.open("skinfunction1.dat",std::ios_base::in);
	file>>_mean.x;
	file>>_mean.y;
	file>>_stddev.x;
	file>>_stddev.y;
	file.close();
}

void skinfunction_2::skintrain(vector<cv::Point3f> skinvalues){
	int step = skinvalues.size()/10; //step for validation
	double best=0;
	
	for(int i=0;i<10;i++){
		vector<float> temp1,temp2,tempout1,tempout2;
		for(int j=0;j<skinvalues.size();j++){
			if(j > (step*i) && j < step*(i+1)+1){ //means i am within the validation block
				tempout1.push_back(skinvalues[j].x);
				tempout2.push_back(skinvalues[j].y);
			}
			else{
				temp1.push_back(skinvalues[j].x);
				temp2.push_back(skinvalues[j].y);
			}

		}
		
		cv::Mat m1(temp1.size(),1,CV_32FC1,temp1.data());
		cv::EM em1(1, cv::EM::COV_MAT_SPHERICAL, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 300, 0.1));
		m1 = m1.reshape(1, 0);
		em1.train( m1 );

		cv::Mat m2(temp2.size(),1,CV_32FC1,temp2.data());
		cv::EM em2(1, cv::EM::COV_MAT_SPHERICAL, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 300, 0.1));
		m2 = m2.reshape(1, 0);
		em2.train( m2 );
	
		double smin1=DBL_MAX,smax1=DBL_MIN;
		double smin2=DBL_MAX,smax2=DBL_MIN;
		double savg1=0,savg2=0;
		for(int sid=0;sid<temp1.size();sid++){
			cv::Mat sample1(1,1,CV_32FC1),sample2(1,1,CV_32FC1);
			sample1.at<float>(0) = temp1[sid];
			sample2.at<float>(0) = temp2[sid];
			cv::Vec2d pvalue1 = em1.predict(sample1);
			cv::Vec2d pvalue2 = em2.predict(sample2);
			
			if(smin1 > pow(10,pvalue1.val[0]))
				smin1 = pow(10,pvalue1.val[0]);
			if(smax1 < pow(10,pvalue1.val[0]))
				smax1 = pow(10,pvalue1.val[0]);

			if(smin2 > pow(10,pvalue2.val[0]))
				smin2 = pow(10,pvalue2.val[0]);
			if(smax2 < pow(10,pvalue2.val[0]))
				smax2 = pow(10,pvalue2.val[0]);

			savg1+=pow(10,pvalue1.val[0]);
			savg2+=pow(10,pvalue2.val[0]);
		}

		cout << smin1 <<" "<< smax1 << endl;
		cout << smin2 <<" "<< smax2 << endl;
		cout << savg1/temp1.size() <<" "<< savg2/temp1.size() << endl;
		//cout<<ssum/tempout1.size()<<endl;

		//cv::Mat m2(temp2.size(),1,CV_32FC1,temp2.data());
		//em2.train(m2);
		//cv::Vec2d pvalue2 = em1.predict(cv::Mat(tempout2.size(),1,CV_64FC1,tempout2.data()));

	//	float count=0;
	//	for(int j=step*i;j<step*(i+1);j++){
	//		if(skinvalues[j].x > msum)
	//			count++;
	//	}
	//	//cout<<count/step<<" "<<msum<<endl;
	//	if(count/step > best){
	//		rmean = msum;
	//		best = count/step;
	//	}
	}
	////cout<<"Best: "<<best<<" "<<rmean<<endl;
	//mskin=best-(1-best);
	//mnonskin=0;
	//mboth=1-mskin;
	//trained=true;
}

bool skinfunction_2::skincompare(float r, float g, float b){
	return false;
}

void skinfunction_2::save_function(){
	std::ofstream file;
	file.open("skinfunction2.dat",std::ios_base::out);
	
	file.close();
}

void skinfunction_2::load_function(){
	std::ifstream file;
	file.open("skinfunction2.dat",std::ios_base::in);
	
	file.close();
}


