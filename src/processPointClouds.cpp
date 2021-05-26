// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // DONE: Create two new point clouds, one cloud with obstacles and other with segmented plane

    // 1. Create two PCD clouds, roadCloud and obstCloud
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT>()), obstCloud (new pcl::PointCloud<PointT>());
    
    // 2. Create extractor object
    pcl::ExtractIndices<PointT> extract;

    // 3. Extract the road (inliers)
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);  // false might be the default
    extract.filter (*roadCloud);

    // 3.5. Alternative implementation
    // for (int index: inliers->indices) roadCloud->points.push_back(cloud->points[index]);

    // 4. Extract the obstacles (rest of the cloud)
    extract.setNegative (true);  // the points with indices NOT in inliers
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, roadCloud);
    return segResult;
}


// Naive implementation of RANSAC for plane segmentation
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
	}

	// Return indicies of inliers from fitted line with most inliers	
	return inliersResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // 1. extract inlier set
    std::unordered_set<int> inliersSet = RansacPlane(cloud, maxIterations, distanceThreshold);

 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // 2. populate argument pcl::PointIndices::Ptr for cloud separation
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
 
 
    inliers->indices.insert(inliers->indices.end(), inliersSet.begin(), inliersSet.end());
   // Separate the road (points with indices in inliers) from the rest of the cloud (where obstacles will remain)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}


// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
// 	// pcl::PointIndices::Ptr inliers;

//     // DONE:: Find planar inliers for the cloud for the road surface.

//     // 1. Create segmentation object
//     pcl::SACSegmentation<PointT> seg;

//     // 2. Configure segmentation object (model, method, iterations, threshold, etc.)
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setMaxIterations (maxIterations);
//     seg.setDistanceThreshold (distanceThreshold);

//     // 3. Set input PCD cloud
//     seg.setInputCloud (cloud);

//     // 4. Create model coefficients and point indeces objects
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());  // for plane model
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

//     // 5. Perform segmentation
//     seg.segment (*inliers, *coefficients);

//     if (inliers->indices.size () == 0)
//     {
//       std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

//     // Separate the road (points with indices in inliers) from the rest of the cloud (where obstacles will remain)
//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

//     return segResult;
// }


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);  // in m
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)  // or "for (pcl::PointIndices getIndices: clusterIndices) {}""
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices)  // or "for (int index: getIndices.indices)""
            cloud_cluster->push_back ((*cloud)[idx]); // or cloud_cluster->push_back(cloud->points[index])
            // NOTE:: Apparently, (*cloud)[index] and cloud->points[index] are equivalent!
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

        // Add to clusters
        // TODO:: clusters is a pointer: shouldn't this be clusters->push_back()?
        // Return type: A vector of pointers to point clouds, so no deref of cloud_cluster!
        clusters.push_back(cloud_cluster);

        // Write out
        // std::stringstream ss;
        // ss << "cloud_cluster_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        
        j ++;  // or without it in case of "for (pcl::PointIndices getIndices: clusterIndices) {}""
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    // NOTE:: move constructor used for the caller
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
