/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include <geometry_msgs/PoseArray.h>
#include <mbf_abstract_nav/MoveBaseFlexConfig.h>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/Path.h>

#include "mbf_mesh_nav/mesh_navigation_server.h"

namespace mbf_mesh_nav
{
MeshNavigationServer::MeshNavigationServer(const TFPtr& tf_listener_ptr)
  : AbstractNavigationServer(tf_listener_ptr)
  , recovery_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshRecovery")
  , controller_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshController")
  , planner_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshPlanner")
  , mesh_ptr_(new mesh_map::MeshMap(*tf_listener_ptr_))
  , setup_reconfigure_(false)
{
  // advertise services and current goal topic
  check_pose_cost_srv_ =
      private_nh_.advertiseService("check_pose_cost", &MeshNavigationServer::callServiceCheckPoseCost, this);
  check_path_cost_srv_ =
      private_nh_.advertiseService("check_path_cost", &MeshNavigationServer::callServiceCheckPathCost, this);
  clear_mesh_srv_ = private_nh_.advertiseService("clear_mesh", &MeshNavigationServer::callServiceClearMesh, this);

  // dynamic reconfigure server for mbf_mesh_nav configuration; also include
  // abstract server parameters
  dsrv_mesh_ = boost::make_shared<dynamic_reconfigure::Server<mbf_mesh_nav::MoveBaseFlexConfig>>(private_nh_);
  dsrv_mesh_->setCallback(boost::bind(&MeshNavigationServer::reconfigure, this, _1, _2));

  ROS_INFO_STREAM("Reading map file...");
  mesh_ptr_->readMap();

  // initialize all plugins
  initializeServerComponents();

  // start all action servers
  startActionServers();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr MeshNavigationServer::newPlannerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_mesh_nav::MeshPlannerExecution>(
      plugin_name, boost::static_pointer_cast<mbf_mesh_core::MeshPlanner>(plugin_ptr), mesh_ptr_, last_config_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr MeshNavigationServer::newControllerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractController::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_mesh_nav::MeshControllerExecution>(
      plugin_name, boost::static_pointer_cast<mbf_mesh_core::MeshController>(plugin_ptr), vel_pub_, goal_pub_,
      tf_listener_ptr_, mesh_ptr_, last_config_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr MeshNavigationServer::newRecoveryExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr)
{
  return boost::make_shared<mbf_mesh_nav::MeshRecoveryExecution>(
      plugin_name, boost::static_pointer_cast<mbf_mesh_core::MeshRecovery>(plugin_ptr), tf_listener_ptr_,
      boost::ref(mesh_ptr_), last_config_);
}

mbf_abstract_core::AbstractPlanner::Ptr MeshNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));
    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_DEBUG_STREAM("mbf_mesh_core-based planner plugin " << planner_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_FATAL_STREAM("Failed to load the " << planner_type << " planner, are you sure it's properly registered"
                                           << " and that the containing library is built? " << ex_mbf_core.what());
  }

  return planner_ptr;
}

bool MeshNavigationServer::initializePlannerPlugin(const std::string& name,
                                                   const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  mbf_mesh_core::MeshPlanner::Ptr mesh_planner_ptr =
      boost::static_pointer_cast<mbf_mesh_core::MeshPlanner>(planner_ptr);
  ROS_DEBUG_STREAM("Initialize planner \"" << name << "\".");

  if (!mesh_ptr_)
  {
    ROS_FATAL_STREAM("The mesh pointer has not been initialized!");
    return false;
  }
  return mesh_planner_ptr->initialize(name, mesh_ptr_);
}

mbf_abstract_core::AbstractController::Ptr
MeshNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  try
  {
    controller_ptr = controller_plugin_loader_.createInstance(controller_type);
    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_DEBUG_STREAM("mbf_mesh_core-based controller plugin " << controller_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_FATAL_STREAM("Failed to load the " << controller_type << " controller, are you sure it's properly registered"
                                           << " and that the containing library is built? " << ex_mbf_core.what());
  }
  return controller_ptr;
}

bool MeshNavigationServer::initializeControllerPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  ROS_DEBUG_STREAM("Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!mesh_ptr_)
  {
    ROS_FATAL_STREAM("The mesh pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshController::Ptr mesh_controller_ptr =
      boost::static_pointer_cast<mbf_mesh_core::MeshController>(controller_ptr);
  mesh_controller_ptr->initialize(name, tf_listener_ptr_, mesh_ptr_);
  ROS_DEBUG_STREAM("Controller plugin \"" << name << "\" initialized.");
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr MeshNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        recovery_plugin_loader_.createInstance(recovery_type));
    std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
    ROS_DEBUG_STREAM("mbf_mesh_core-based recovery behavior plugin " << recovery_name << " loaded.");
  }
  catch (pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_FATAL_STREAM("Failed to load the " << recovery_type
                                           << " recovery behavior, are you sure it's properly registered"
                                           << " and that the containing library is built? " << ex_mbf_core.what());
  }

  return recovery_ptr;
}

bool MeshNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                    const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  ROS_DEBUG_STREAM("Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    ROS_FATAL_STREAM("The tf listener pointer has not been initialized!");
    return false;
  }

  if (!mesh_ptr_)
  {
    ROS_FATAL_STREAM("The mesh map pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshRecovery::Ptr behavior = boost::static_pointer_cast<mbf_mesh_core::MeshRecovery>(behavior_ptr);
  behavior->initialize(name, tf_listener_ptr_, mesh_ptr_);
  ROS_DEBUG_STREAM("Recovery behavior plugin \"" << name << "\" initialized.");
  return true;
}

void MeshNavigationServer::stop()
{
  AbstractNavigationServer::stop();
  // TODO
  // ROS_INFO_STREAM_NAMED("mbf_mesh_nav", "Stopping mesh map for shutdown");
  // mesh_ptr_->stop();
}

MeshNavigationServer::~MeshNavigationServer()
{
}

void MeshNavigationServer::reconfigure(mbf_mesh_nav::MoveBaseFlexConfig& config, uint32_t level)
{
  // Make sure we have the original configuration the first time we're called,
  // so we can restore it if needed
  if (!setup_reconfigure_)
  {
    default_config_ = config;
    setup_reconfigure_ = true;
  }

  if (config.restore_defaults)
  {
    config = default_config_;
    // if someone sets restore defaults on the parameter server, prevent looping
    config.restore_defaults = false;
  }

  // fill the abstract configuration common to all MBF-based navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.planner_frequency = config.planner_frequency;
  abstract_config.planner_patience = config.planner_patience;
  abstract_config.planner_max_retries = config.planner_max_retries;
  abstract_config.controller_frequency = config.controller_frequency;
  abstract_config.controller_patience = config.controller_patience;
  abstract_config.controller_max_retries = config.controller_max_retries;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  abstract_config.oscillation_timeout = config.oscillation_timeout;
  abstract_config.oscillation_distance = config.oscillation_distance;
  abstract_config.restore_defaults = config.restore_defaults;
  mbf_abstract_nav::AbstractNavigationServer::reconfigure(abstract_config, level);

  last_config_ = config;
}

bool MeshNavigationServer::callServiceCheckPoseCost(mbf_msgs::CheckPose::Request& request,
                                                    mbf_msgs::CheckPose::Response& response)
{
  // TODO implement
  return false;
}

bool MeshNavigationServer::callServiceCheckPathCost(mbf_msgs::CheckPath::Request& request,
                                                    mbf_msgs::CheckPath::Response& response)
{
  // TODO implement
  return false;
}

bool MeshNavigationServer::callServiceClearMesh(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  mesh_ptr_->resetLayers();
  return true;
}

} /* namespace mbf_mesh_nav */
