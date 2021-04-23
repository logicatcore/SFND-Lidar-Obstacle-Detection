#ifndef RANSAC_H
#define RANSAC_H

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
// #include "../../processPointClouds.cpp"
#include<math.h>

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
	int idx1{0}, idx2{0}, maxIdx{cloud->points.size()};
	float A{0}, B{0}, C{0}, denom{0};
	
	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line
		idx1 = std::rand() % maxIdx;
		do {
			idx2 = std::rand() % maxIdx;
		}while(idx2 == idx1);
		
		A = cloud->points[idx1].y - cloud->points[idx2].y;
		B = cloud->points[idx2].x - cloud->points[idx1].x ;
		C = cloud->points[idx1].x * cloud->points[idx2].y - cloud->points[idx2].x * cloud->points[idx1].y;
		denom = sqrt(pow(A, 2) + pow(B, 2));
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int idx = 0; idx < cloud->points.size(); idx++){
				if (abs(A * cloud->points[idx].x + B * cloud->points[idx].y + C)/denom < distanceTol) {
					inliers.insert(idx);
				}
		}
		if (inliersResult.size() < inliers.size()){
				inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> RansacPlaneIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int idx1{0}, idx2{0}, idx3{0}, maxIdx{cloud->points.size()};
	float A{0}, B{0}, C{0}, D{0}, denom{0};
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	// For max iterations 
	while (maxIterations--) {
		std::unordered_set<int> inliers;
		// Randomly sample subset and fit line
		idx1 = std::rand() % maxIdx;
		do {
			idx2 = std::rand() % maxIdx;
		}while(idx2 == idx1);
		do {
			idx3 = std::rand() % maxIdx;
		}while(idx3 == idx1 | idx3 == idx2);
		
		x1 = cloud->points[idx1].x;
		y1 = cloud->points[idx1].y;
		z1 = cloud->points[idx1].z;

		x2 = cloud->points[idx2].x;
		y2 = cloud->points[idx2].y;
		z2 = cloud->points[idx2].z;

		x3 = cloud->points[idx3].x;
		y3 = cloud->points[idx3].y;
		z3 = cloud->points[idx3].z;

		A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); 
		B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		D = - (A * x1 + B * y1 + C * z1);

		denom = sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int idx = 0; idx < cloud->points.size(); idx++){
				if (abs(A * cloud->points[idx].x + B * cloud->points[idx].y + C * cloud->points[idx].z + D)/denom < distanceTol) {
					inliers.insert(idx);
				}
		}
		if (inliersResult.size() < inliers.size()){
				inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
	
  srand(time(NULL));
	int idx1{0}, idx2{0}, idx3{0}, maxIdx{cloud->points.size()};
	float A{0}, B{0}, C{0}, D{0}, denom{0};
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	// For max iterations 
	while (maxIterations--) {
    typename pcl::PointCloud<PointT>::Ptr tmpPlane(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr tmpObstacles(new pcl::PointCloud<PointT>);

		// Randomly sample subset and fit line
		idx1 = std::rand() % maxIdx;
		do {
			idx2 = std::rand() % maxIdx;
		}while(idx2 == idx1);
		do {
			idx3 = std::rand() % maxIdx;
		}while(idx3 == idx1 | idx3 == idx2);
		
		x1 = cloud->points[idx1].x;
		y1 = cloud->points[idx1].y;
		z1 = cloud->points[idx1].z;

		x2 = cloud->points[idx2].x;
		y2 = cloud->points[idx2].y;
		z2 = cloud->points[idx2].z;

		x3 = cloud->points[idx3].x;
		y3 = cloud->points[idx3].y;
		z3 = cloud->points[idx3].z;

		A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); 
		B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		D = - (A * x1 + B * y1 + C * z1);

		denom = sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (int idx = 0; idx < cloud->points.size(); idx++){
				if (abs(A * cloud->points[idx].x + B * cloud->points[idx].y + C * cloud->points[idx].z + D)/denom < distanceTol) {
					tmpPlane->push_back(cloud->points[idx]);
				}
        else{
          tmpObstacles->push_back(cloud->points[idx]);
        }
		}
		if (plane->size() < tmpPlane->size()){
				plane = tmpPlane;
        obstacles = tmpObstacles;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> results(obstacles, plane);
  return results;
}

#endif