/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <functional>
#include <limits>

#include <yarp/sig/PointCloud.h>

#include <pcl/segmentation/extract_clusters.h>

namespace segmentation {

/******************************************************************************/
class Segmentation {
public:
    /**************************************************************************/
    static int RANSAC(std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_scene,
                         std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_table,
                         std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_object) {

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_scene_pcl(new pcl::PointCloud<pcl::PointXYZRGBA>);
        yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(*pc_scene, *pc_scene_pcl);

        std::vector<int> inliers;
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (pc_scene_pcl));

        // get the inliers
        pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model_plane);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);

        // copy all inliers of the model computed to the table point cloud
        for (size_t i = 0; i < pc_scene->size(); i++) {
            const auto& p = (*pc_scene)(i);
            if (std::find(inliers.begin(), inliers.end(), i) != inliers.end()) {
                pc_table->push_back(p);
            } else {
                pc_object->push_back(p);
            }
        }
        yInfo()<<"Segmented object with"<<pc_object->size()<<"points";
        return inliers.size();
    }

    /**************************************************************************/
    static std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> extractClusters(
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_object_pcl) {

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(pc_object_pcl);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(0.05); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pc_object_pcl);
        ec.extract(cluster_indices);

        yInfo() << "Found" << cluster_indices.size() << "point cloud clusters";
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters;
        for (auto it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                cloud_cluster->push_back((*pc_object_pcl)[*pit]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
            yInfo() << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points.";
        }

        return clusters;
    }
};

}

#endif
