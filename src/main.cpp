/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include <random>
#include <fstream>
#include <algorithm>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/math/Math.h>
#include <yarp/pcl/Pcl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>

#include <gazebo/common/Plugin.hh>

#include "rpc_IDL.h"
#include "viewer.h"
#include "segmentation.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace viewer;
using namespace segmentation;

/******************************************************************************/
class FitModule : public RFModule, public rpc_IDL {
    ResourceFinder rf;
    PolyDriver gaze;

    RpcServer rpcPort;
    BufferedPort<ImageOf<PixelRgb>> rgbPort;
    BufferedPort<ImageOf<PixelFloat>> depthPort;

    ImageOf<PixelFloat>* depthImage{nullptr};

    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_scene{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_table{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_object{nullptr};
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_model_pcl;

    Matrix Teye;
    double table_height{numeric_limits<double>::quiet_NaN()};

    unique_ptr<Viewer> viewer;
    bool view_fit{false};
    double scale{1.0};
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    string model_mesh{""};
    double score{std::numeric_limits<double>::infinity()};

    /**************************************************************************/
    bool attach(RpcServer& source) override
    {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override
    {
        this->rf = rf;
        const string name = "model-alignment";

        Property gaze_options;
        gaze_options.put("device", "gazecontrollerclient");
        gaze_options.put("local", "/"+name+"/gaze");
        gaze_options.put("remote", "/iKinGazeCtrl");
        if (!gaze.open(gaze_options)) {
            yError() << "Unable to open gaze driver!";
            return false;
        }

        rgbPort.open("/"+name+"/rgb:i");
        depthPort.open("/"+name+"/depth:i");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        pc_model_pcl = make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

        viewer = make_unique<Viewer>(10, 370, 350, 350);
        viewer->start();

        return true;
    }

    /**************************************************************************/
    bool home() override
    {
        // home gazing
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->lookAtAbsAnglesSync({0., -50., 10.});
        igaze->waitMotionDone();
        return true;
    }

    /**************************************************************************/
    bool align() override
    {
        if (!segment())
        {
            return false;
        }
        yInfo() << "Object segmented";
        return fit();
    }

    /**************************************************************************/
    bool segment() override
    {
        // get depth data
        // FILL IN CODE...

        // get image data
        auto* rgbImage = rgbPort.read();

        if ((rgbImage == nullptr) || (depthImage == nullptr)) {
            yError() << "Unable to receive image data!";
            return false;
        }

        if ((rgbImage->width() != depthImage->width()) ||
                (rgbImage->height() != depthImage->height()) ) {
            yError() << "Received image data with wrong size!";
            return false;
        }

        const auto w = rgbImage->width();
        const auto h = rgbImage->height();

        // get camera extrinsics
        IGazeControl* igaze;
        gaze.view(igaze);
        Vector cam_x, cam_o;
        igaze->getLeftEyePose(cam_x, cam_o);
        Teye = axis2dcm(cam_o);
        Teye.setSubcol(cam_x, 0, 3);

        // get camera intrinsics
        Bottle info;
        igaze->getInfo(info);
        const auto fov_h = info.find("camera_intrinsics_left").asList()->get(0).asDouble();
        const auto view_angle = 2. * std::atan((w / 2.) / fov_h) * (180. / M_PI);

        // aggregate image data in the point cloud of the whole scene
        pc_scene = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        Vector x{0., 0., 0., 1.};
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                const auto rgb = (*rgbImage)(u, v);
                const auto depth = (*depthImage)(u, v);
                if (depth > 0.F) {
                    x[0] = depth * (u - .5 * (w - 1)) / fov_h;
                    x[1] = depth * (v - .5 * (h - 1)) / fov_h;
                    x[2] = depth;
                    x = Teye * x;

                    pc_scene->push_back(DataXYZRGBA());
                    auto& p = (*pc_scene)(pc_scene->size() - 1);
                    p.x = (float)x[0];
                    p.y = (float)x[1];
                    p.z = (float)x[2];
                    p.r = 0;
                    p.g = 255;
                    p.b = 0;
                    p.a = 255;
                }
            }
        }

        // segment out the table and the object
        pc_table = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        pc_object = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        table_height = Segmentation::RANSAC(pc_scene, pc_table, pc_object);
        if (isnan(table_height)) {
            yError() << "Segmentation failed!";
            return false;
        }

        // update viewer
        Vector cam_foc;
        igaze->get3DPointOnPlane(0, {w/2., h/2.}, {0., 0., 1., -table_height}, cam_foc);
        viewer->addTable({cam_foc[0], cam_foc[1], cam_foc[2]}, {0., 0., 1.});
        viewer->addObject(pc_object);
        viewer->addCamera({cam_x[0], cam_x[1], cam_x[2]}, {cam_foc[0], cam_foc[1], cam_foc[2]},
        {0., 0., 1.}, view_angle);

        if (pc_object->size() > 0) {
            return true;
        } else {
            yError() << "Unable to segment any object!";
            return false;
        }
    }

    /**************************************************************************/
    bool is_model_valid() override
    {
        return load_model("mustard_bottle");
    }

    /**************************************************************************/
    bool load_model(const string &model_name) override
    {
        yInfo() << "Looking for" << model_name;
        string file_name = "models/" + model_name;
        model_mesh = rf.findFile(file_name + "/meshes/mesh.stl");
        if (model_mesh.empty()) {
            yError() << "Unable to find" << file_name + "/meshes/mesh.stl";
            return false;
        }
        string model_file = rf.findFile(file_name + "/model.sdf");
        if (model_file.empty()) {
            yError() << "Unable to find" << file_name + "/model.sdf";
            return false;
        }

        // use loadPolygonFileSTL to load the mesh
        // and convert it to point cloud
        // FILL IN CODE...
        pcl::PolygonMesh mesh;

        return false;
    }

    /**************************************************************************/
    bool fit() override
    {
        if (!pc_object || pc_object->size() < 0) {
            yError() << "No object to fit";
            return false;
        }

        // Copy yarp object to pcl object
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_object_pcl(new pcl::PointCloud<pcl::PointXYZRGBA>);
        yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(*pc_object, *pc_object_pcl);

        // Rescale model by the correct size
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_model_pcl_scaled(new pcl::PointCloud<pcl::PointXYZRGBA>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform (0,0) = transform (0,0) * scale;
        transform (1,1) = transform (1,1) * scale;
        transform (2,2) = transform (2,2) * scale;
        pcl::transformPointCloud (*pc_model_pcl, *pc_model_pcl_scaled, transform);
        yInfo() << "Rescaling model by " << scale << scale << scale;

        // Run icp
        Eigen::Matrix4d T(Eigen::Matrix4d::Identity());
        icp.setInputSource(pc_object_pcl);
        icp.setInputTarget(pc_model_pcl_scaled);

        // set parameters for icp
        // FILL IN CODE...
        // Set the max correspondence distance

        // Set the maximum number of iterations (criterion 1)
        // This should be large if initial alignment is poor

        // Set the transformation epsilon (criterion 2)

        // Set the euclidean distance difference epsilon (criterion 3)

        // Use icp to do alignment
        // FILL IN CODE...
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGBA>);

        if (icp.hasConverged())
        {
            // Transform the input dataset using the final transformation
            T = icp.getFinalTransformation().cast<double>();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA> ());
            pcl::transformPointCloud(*pc_model_pcl_scaled, *transformed_cloud, T.inverse().cast<float>());
            score = computeScore(transformed_cloud, pc_object_pcl);
            yInfo() << "ICP has converged with score" << score;
            std::vector<double> color{1.0, 0.0, 0.0};
            viewer->addModel(model_mesh, T.inverse(), color, scale);
            return true;
        }
        else
        {
            yError() << "ICP did not converge";
            return false;
        }
    }

    /**************************************************************************/
    double get_score() override
    {
        return score;
    }

    /**************************************************************************/
    std::vector<double> get_parameters() override
    {
        if (icp.getInputSource())
        {
            std::vector<double> params;
            params.push_back(icp.getMaxCorrespondenceDistance());
            params.push_back(icp.getMaximumIterations());
            params.push_back(icp.getTransformationEpsilon());
            params.push_back(icp.getEuclideanFitnessEpsilon());
            return params;
        }
        else
        {
            std::vector<double> r(4,-1.0);
            return r;
        }
    }

    /**************************************************************************/
    double estimateScale(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudA,
                         const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloudB)
    {
        pcl::PCA<pcl::PointXYZRGBA> pca;
        pca.setInputCloud(cloudA);
        Eigen::Vector3f ev_A = pca.getEigenValues();

        pca.setInputCloud(cloudB);
        Eigen::Vector3f ev_B = pca.getEigenValues();

        double s = sqrt(ev_B[0])/sqrt(ev_A[0]);
        return s;
    }

    /**************************************************************************/
    double computeScore(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud,
                        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud)
    {
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> tree;
        tree.setInputCloud(input_cloud);

        double fitness_score = 0.0;
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        // For each point in the source dataset
        int nr = 0;
        for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
            // Find the nearest neighbor
            int nn = tree.nearestKSearch(transformed_cloud->points[i], 1, nn_indices, nn_dists);
            fitness_score += nn_dists[0];
            nr++;
        }
        if (nr > 0)
            return (fitness_score / nr);
        else
            return (std::numeric_limits<double>::infinity());
    }

    /**************************************************************************/
    bool clean_viewer() override
    {
        viewer->clean();
        return true;
    }

    /**************************************************************************/
    double getPeriod() override
    {
        return 1.0;
    }

    /**************************************************************************/
    bool updateModule() override
    {
        return true;
    }

    /**************************************************************************/
    bool interruptModule() override
    {
        viewer->stop();

        // interrupt blocking read
        depthPort.interrupt();
        rgbPort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override
    {
        // restore default contexts
        IGazeControl* igaze;
        gaze.view(igaze);
        igaze->stopControl();
        igaze->restoreContext(0);

        rpcPort.close();
        depthPort.close();
        rgbPort.close();
        gaze.close();
        return true;
    }
};

/******************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "Unable to find YARP server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    FitModule module;
    return module.runModule(rf);
}

