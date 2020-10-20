/*
 * Copyright (C) 2020 iCub Tech Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>
#include <cmath>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;

/**********************************************************************/
class TestAssignmentAlignment : public yarp::robottestingframework::TestCase
{
    RpcClient visionPort;

public:
    /******************************************************************/
    TestAssignmentAlignment() :
        yarp::robottestingframework::TestCase("TestAssignmentAlignment") {}

    /******************************************************************/
    virtual ~TestAssignmentAlignment() {}

    /******************************************************************/
    bool setup(Property& property) override
    {
        float rpcTmo=(float)property.check("rpc-timeout",Value(120.0)).asDouble();

        visionPort.open("/"+getName()+"/vision:rpc");
        visionPort.asPort().setTimeout(rpcTmo);
        if (!Network::connect(visionPort.getName(), "/alignment/rpc")) {
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /alignment/rpc");
        }

        return true;
    }

    /******************************************************************/
    void tearDown() override
    {
        visionPort.close();
    }

    /******************************************************************/
    void run() override
    {
        unsigned int score{0};
        const double optimal{0.000336};
        const double high{0.0001};
        const double low{0.1};
        const double max_distance{1.0};
        unsigned int max_iterations{2000};
        const double transf_epsilon{1e-8};
        const double fitness_epsilon{1e-9};
        for (int attempt = 1; attempt <= 1; attempt++) {
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Starting attempt #%d...", attempt));
            double response_score;
            bool response_model;
            std::vector<double> response_params;
            response_params.resize(4);
            
            {
                Bottle cmd, rep;
                cmd.addString("home");
                if (!visionPort.write(cmd, rep))
                {
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /alignment/rpc");
                }
            }

            {
                Bottle cmd, rep;
                cmd.addString("is_model_valid");
                if (visionPort.write(cmd, rep))
                {
                    if (rep.get(0).asVocab() == Vocab::encode("ok"))
                    {
                        response_model = true;
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
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /alignment/rpc");
                }
            }

            {
                Bottle cmd, rep;
                cmd.addString("align");
                if (visionPort.write(cmd, rep))
                {
                    if (rep.get(0).asVocab() == Vocab::encode("ok"))
                    {
                        cmd.clear();
                        rep.clear();
                        cmd.addString("get_score");
                        if (visionPort.write(cmd, rep))
                        {
                            response_score = rep.get(0).asDouble();
                            double result = fabs(response_score - optimal);
                            if (result <= high)
                            {
                                score+=20;
                                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done aligning"));
                            }
                            if (result > high && result <= low)
                            {
                                score+=10;
                                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Almost there with alignment..."));
                            }
                            if (result > low)
                            {
                                score+=0;
                                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Epic failure while aligning"));
                            }
                        }
                        else
                        {
                            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /alignment/rpc");

                        }
                    }
                }
                else
                {
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /alignment/rpc");
                }
            }

            {
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
                    ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to /alignment/rpc");
                }

                if (fabs(response_params[0] - max_distance) < 1.0)
                {
                    score+=2;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting maximum correspondence distance"));
                }
                else
                {
                    score+=0;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format(""));
                }

                if (fabs(response_params[1] - max_iterations) < 1000)
                {
                    score+=2;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting maximum number of iterations"));
                }
                else
                {
                    score+=0;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format(""));
                }

                if (fabs(response_params[2] - transf_epsilon) < 1e-8)
                {
                    score+=2;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting the transformation epsilon"));
                }
                else
                {
                    score+=0;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format(""));
                }

                if (fabs(response_params[3] - fitness_epsilon) < 1e-9)
                {
                    score+=2;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Well done setting the euclidean distance difference epsilon"));
                }
                else
                {
                    score+=0;
                    ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format(""));
                }
            }
        }

        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score > 15, Asserter::format("Total score = %d", score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentAlignment)