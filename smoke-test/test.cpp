/*
 * Copyright (C) 2020 iCub Tech Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>
#include <cmath>
#include <fstream>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>
#include <yarp/pcl/Pcl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;

/**********************************************************************/
class TestAssignmentModelAlignment : public yarp::robottestingframework::TestCase
{
    RpcClient visionPort;
    BufferedPort<Bottle> objectPort;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_object_pcl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc_transformed_pcl;

public:
    /******************************************************************/
    TestAssignmentModelAlignment() :
        yarp::robottestingframework::TestCase("TestAssignmentModelAlignment") {}

    /******************************************************************/
    virtual ~TestAssignmentModelAlignment() {}

    /******************************************************************/
    bool setup(Property& property) override
    {
        float rpcTmo=(float)property.check("rpc-timeout",Value(120.0)).asDouble();

        visionPort.open("/"+getName()+"/vision:rpc");
        visionPort.asPort().setTimeout(rpcTmo);
        if (!Network::connect(visionPort.getName(), "/model-alignment/rpc")) {
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /model-alignment/rpc");
        }

        objectPort.open("/"+getName()+"/object:o");
        if (!Network::connect(objectPort.getName(), "/assignment_model-alignment-mustard_bottle/mover:i")) {
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /assignment_model-alignment-mustard_bottle/mover:i");
        }

        pc_object_pcl = make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
        pc_transformed_pcl = make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

        return true;
    }

    /******************************************************************/
    void tearDown() override
    {
        visionPort.close();
        objectPort.close();
    }

    /******************************************************************/
    void run() override
    {
        unsigned int score{0};
        {
            Bottle cmd, rep;
            cmd.addString("home");
            if (!visionPort.write(cmd, rep))
            {
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
            }
        }
        Time::delay(2.0);
         
        {
            Bottle cmd, rep;
            cmd.addString("is_model_valid");
            if (visionPort.write(cmd, rep))
            {
                if (rep.get(0).asVocab() == Vocab::encode("ok"))
                {
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Loaded model correctly"));
                    score+=5;
                }
                else
                {
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("The model was not loaded properly");
                }
            }
            else
            {
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
            }
        }

        const double low{0.0001};
        const double high{0.1};
        const double min_distance{0.3};
        unsigned int min_iterations{100};
        const double max_transf_epsilon{0.00000001};
        const double max_fitness_epsilon{0.0001};
        double response_perc{0.0};
        for (int attempt = 1; attempt <= 5; attempt++) {
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Starting attempt #%d...", attempt));
            double response_score;
            {
                Bottle cmd, rep;
                cmd.addString("randomize");
                if (visionPort.write(cmd, rep))
                {
                    if (rep.get(0).asVocab() == Vocab::encode("ok"))
                    {
                        Time::delay(1.0);
                        cmd.clear();
                        rep.clear();
                        cmd.addString("align");
                        if (visionPort.write(cmd, rep))
                        {
                            if (rep.get(0).asVocab() == Vocab::encode("ok"))
                            {
                                cmd.clear();
                                rep.clear();
                                cmd.addString("get_point_clouds");
                                if (visionPort.write(cmd, rep))
                                {
                                    Bottle *b = rep.get(0).asList();
                                    Bottle *source = b->get(0).asList();
                                    Bottle *transformed = b->get(1).asList();
                                    yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> pc_object, pc_transformed;
                                    pc_object.fromBottle(*source);
                                    pc_transformed.fromBottle(*transformed);
                                    yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(pc_object, *pc_object_pcl);
                                    yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(pc_transformed, *pc_transformed_pcl);
                                    response_score = computeScore(pc_transformed_pcl, pc_object_pcl);
                                    double result = fabs(response_score - 0.00033);
                                    if (result <= low)
                                    {
                                        response_perc++;
                                        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done aligning"));
                                    }
                                    if (result > low && result <= high)
                                    {
                                        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Almost there with alignment..."));
                                    }
                                    if (result > high)
                                    {
                                        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Epic failure while aligning"));
                                    }
                                }
                                else
                                {
                                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
                                }
                            }
                        }
                        else
                        {
                            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
                        }
                    }

                    Time::delay(2.0);
                }
                else
                {
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
                }
            }
        }

        response_perc/=5.0;
        response_perc*=100.0;
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Percentage of attempt with success: %f", response_perc));
        if (response_perc <= 33.0)
        {
            score+=0;
        }
        if (response_perc > 33.0 && response_perc <=66.0)
        {
            score+=10;
        }
        if (response_perc > 66.0)
        {
            score+=20;
        }

        {
            std::vector<double> response_params;
            response_params.resize(4);
            Bottle cmd, rep;
            cmd.addString("get_parameters");
            if (visionPort.write(cmd, rep))
            {
                Bottle *reply = rep.get(0).asList();
                response_params[0] = reply->get(0).asDouble();
                response_params[1] = reply->get(1).asDouble();
                response_params[2] = reply->get(2).asDouble();
                response_params[3] = reply->get(3).asDouble();
            }
            else
            {
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /model-alignment/rpc");
            }

            if ((response_params[0] - min_distance) >= 0.0)
            {
                score+=2;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting maximum correspondence distance"));
            }
            else
            {
                score+=0;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Try to play with maximum correspondence distance"));
            }

            if ((response_params[1] - min_iterations) >= 0)
            {
                score+=2;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting maximum number of iterations"));
            }
            else
            {
                score+=0;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Try to play with maximum number of iterations"));
            }

            if ((max_transf_epsilon - response_params[2]) >= 0.0)
            {
                score+=2;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting the transformation epsilon"));
            }
            else
            {
                score+=0;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Try to play with transformation epsilon"));
            }

            if ((max_fitness_epsilon - response_params[3]) >= 0.0)
            {
                score+=2;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting the euclidean distance difference epsilon"));
            }
            else
            {
                score+=0;
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Try to play with euclidean distance difference epsilon"));
            }
        }

        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score > 15, Asserter::format("Total score = %d", score));
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
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentModelAlignment)
