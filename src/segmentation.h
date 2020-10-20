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
    static double RANSAC(std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_scene,
                         std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_table,
                         std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_object,
                         const int num_points = 100) {

        // generate random indexes
        std::random_device rnd_device;
        std::mt19937 mersenne_engine(rnd_device());
        std::uniform_int_distribution<int> dist(0, pc_scene->size() - 1);
        auto gen = std::bind(dist, mersenne_engine);
        std::vector<int> remap(num_points);
        std::generate(std::begin(remap), std::end(remap), gen);

        // implement RANSAC
        const auto threshold_1 = .01F; // [cm]
        for (size_t i = 0; i < remap.size(); i++) {
            auto& pi = (*pc_scene)(remap[i]);
            auto h = 0.F;
            size_t n = 0;
            for (size_t j = 0; j < remap.size(); j++) {
                const auto& pj = (*pc_scene)(remap[j]);
                if (std::fabs(pj.z - pi.z) < threshold_1) {
                    h += pj.z;
                    n++;
                }
            }
            h /= n;

            if (n > (remap.size() >> 1)) {
                pc_table->clear();
                pc_object->clear();
                const auto threshold_2 = h + threshold_1;
                for (size_t i = 0; i < pc_scene->size(); i++) {
                    const auto& p = (*pc_scene)(i);
                    if (p.z < threshold_2) {
                        pc_table->push_back(p);
                    } else {
                        pc_object->push_back(p);
                    }
                }
                return h;
            }
        }

        return std::numeric_limits<double>::quiet_NaN();
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
