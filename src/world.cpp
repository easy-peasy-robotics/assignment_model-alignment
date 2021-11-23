/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include <boost/bind.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

namespace gazebo {

/******************************************************************************/
class ModelMover : public gazebo::ModelPlugin
{
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr renderer_connection;
    yarp::os::BufferedPort<yarp::os::Bottle> port;

    /**************************************************************************/
    void onWorldFrame() {
        if (auto* b = port.read(false)) {
            if (b->size() >= 6) {
                const auto x = b->get(0).asFloat64();
                const auto y = b->get(1).asFloat64();
                const auto z = b->get(2).asFloat64();
                const auto r = b->get(3).asFloat64();
                const auto p = b->get(4).asFloat64();
                const auto yw = b->get(5).asFloat64();
                ignition::math::Pose3d pose(x, y, z, r, p, yw);
                model->SetWorldPose(pose);
            }
        }
    }

public:
    /**************************************************************************/
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) {
        this->model = model;
        port.open("/" + model->GetName() + "/mover:i");
        auto bind = boost::bind(&ModelMover::onWorldFrame, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
    }

    /**************************************************************************/
    virtual ~ModelMover() {
        if (!port.isClosed()) {
            port.close();
        }
    }
};

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::ModelMover)
