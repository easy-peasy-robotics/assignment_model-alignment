# Copyright: (C) 2020 iCub Fondazione Istituto Italiano di Tecnologia (IIT) 
# All Rights Reserved.
#
# rpc.thrift

/**
* rpc_IDL
*
* IDL Interface
*/

struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

service rpc_IDL
{
   /**
   * Go home with gaze.
   * @return true/false on success/failure.
   */
   bool home();

   /**
   * Segment out the object point cloud.
   * @return true/false on success/failure.
   */
   bool segment();

   /**
   * Fit the object point cloud with the provided model.
   * @return true/false on success/failure.
   */
   bool fit();

   /**
   * Clean viewer.
   * @return true/false on success/failure.
   */
   bool clean_viewer();

   /**
   * Load the mesh.
   * @param model_name name of the model to load.
   * @return true/false on success/failure.
   **/
   bool load_model(1:string model_name)

   /**
   * Check if model is valid.
   * @return true/false on valid/not valid.
   **/
   bool is_model_valid()

   /**
   * Align model and point cloud.
   * @return true/false on success/failure.
   **/
   bool align()

   /**
   * Get icp parameters.
   * @return icp parameters.
   **/
   list<double> get_parameters()   
   
   /**
   * Get the source and the transformed point cloud.
   * @return Bottle containing source and transformed point cloud.
   **/
   Bottle get_point_clouds()

}
