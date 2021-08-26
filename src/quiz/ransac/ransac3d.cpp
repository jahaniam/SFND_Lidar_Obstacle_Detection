/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"



pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

double Distance(const pcl::PointXYZ &point, double A, double B, double C, double D){
	double distance = fabs(A * point.x + B * point.y + C*point.z + D) / sqrt(pow(A,2) + pow(B,2)+ pow(C,2));
	return distance;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (int i=0;i<maxIterations;++i){
		std::unordered_set<int> inliers;

		int max_inliers = 0; 


		while (inliers.size()<3){
			inliers.insert(rand()%cloud->size());
		}
		auto itr = inliers.begin();
		double y1 = cloud->points[*itr].y;
		double x1 = cloud->points[*itr].x;
		double z1 = cloud->points[*itr].z;
		++itr;
		double x2 = cloud->points[*itr].x;
		double y2 = cloud->points[*itr].y;
		double z2 = cloud->points[*itr].z;
		++itr;
		double x3 = cloud->points[*itr].x;
		double y3 = cloud->points[*itr].y;
		double z3 = cloud->points[*itr].z;

		double A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		double B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		double C = (x2-z1)*(y3-y1)-(y2-y1)*(x3-x1);
		double D = -(A*x1 + B*y1 + C*z1);

		for (int index=0;index<cloud->points.size();++index){
			if (inliers.count(index)>0){continue;}

			auto new_point = cloud->points[index];
			double distance = Distance(new_point, A, B, C, D);
			if (distance<=distanceTol){
				inliers.insert(index);
			}
			
		}
		if (inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		} 
	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.2);
	std::cout<<"size"<<inliers.size()<<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
