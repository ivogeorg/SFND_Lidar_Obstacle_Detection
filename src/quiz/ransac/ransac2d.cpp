/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <cmath>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// DONE:: Fill in this function

	// For max iterations 
	for (int i = 0; i < maxIterations; i ++) {  // or while (maxIterations --) {}

		// Randomly sample subset and fit line
		// 1. PointCloud is indexed
		// 2. pick two random indices and keep them
		int i1 = rand() % cloud->points.size();  // a random integer index in [0, size)
		int i2 = rand() % cloud->points.size();

		// sanity check
		// cout << "i1 = " << i1 << ", i2 = " << i2 << endl;
		
		// 3. fit the line
		float x1 = cloud->points[i1].x;
		float y1 = cloud->points[i1].y;
		float x2 = cloud->points[i2].x;
		float y2 = cloud->points[i2].y;
		float A = y1 - y2;
		float B = x2 - x1;
		float C = x1 * y2 - x2 * y1;

		// 4. iterate through the the points (including the 2 points)
		// 5. fill in an unordered_set with inlier indices
		std::unordered_set<int> candidateSet;
		for (int j = 0; j < cloud->points.size(); j ++) {
			float px = cloud->points[j].x;
			float py = cloud->points[j].y;

			// Measure distance between every point and fitted line
			float d = fabs(A * px + B * py + C) / sqrt(A * A + B * B);

			// If distance is no farther than the threshold, insert into inlier set
			if (d <= distanceTol) candidateSet.insert(j);
		}

		// 6. If the last set has more inliers than the currently held, swap them
		if (candidateSet.size() > inliersResult.size()) inliersResult.swap(candidateSet);

		// sanity check
		// cout << inliersResult.size() << " inliers" << endl; 
	}

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// DONE:: Fill in this function

	// For max iterations 
	for (int i = 0; i < maxIterations; i ++) {  // or while (maxIterations --) {}

		// Randomly sample subset and fit line
		// 1. PointCloud is indexed
		// 2. Pick 3 random indices
		int i1 = rand() % cloud->points.size();  // a random integer index in [0, size)
		int i2 = rand() % cloud->points.size();
		int i3 = rand() % cloud->points.size();

		// sanity check
		// cout << "i1 = " << i1 << ", i2 = " << i2 << endl;
		
		// 3. Fit the plane through these 3 points (may be degenerate but that will be filtered)
		float x1 = cloud->points[i1].x;
		float y1 = cloud->points[i1].y;
		float z1 = cloud->points[i1].z;

		float x2 = cloud->points[i2].x;
		float y2 = cloud->points[i2].y;
		float z2 = cloud->points[i2].z;

		float x3 = cloud->points[i3].x;
		float y3 = cloud->points[i3].y;
		float z3 = cloud->points[i3].z;

		float A = (y1 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
		float B = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1);
		float C = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
		float D = - (A * x1 + B * y1 + C * z1);

		// 4. iterate through the the points (including the 3 points)
		// 5. fill in an unordered_set with inlier indices
		std::unordered_set<int> candidateSet;
		for (int j = 0; j < cloud->points.size(); j ++) {
			float px = cloud->points[j].x;
			float py = cloud->points[j].y;
			float pz = cloud->points[j].z;

			// Measure distance between every point and fitted line
			float d = fabs(A * px + B * py + C * pz + D) / sqrt(A * A + B * B + C * C);

			// If distance is no farther than the threshold, insert into inlier set
			if (d <= distanceTol) candidateSet.insert(j);
		}

		// 6. If the last set has more inliers than the currently held, swap them
		if (candidateSet.size() > inliersResult.size()) inliersResult.swap(candidateSet);

		// sanity check
		// cout << inliersResult.size() << " inliers" << endl; 
	}

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// DONE:: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.25);

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
