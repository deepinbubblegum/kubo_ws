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

#include <lvr2/geometry/Handles.hpp>
#include <lvr2/util/Meap.hpp>

#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>

#include "cvp_mesh_planner/cvp_mesh_planner.h"
//#define DEBUG
//#define USE_UPDATE_WITH_S
//#define USE_UPDATE_FMM

PLUGINLIB_EXPORT_CLASS(cvp_mesh_planner::CVPMeshPlanner, mbf_mesh_core::MeshPlanner);

namespace cvp_mesh_planner
{
CVPMeshPlanner::CVPMeshPlanner()
{
}

CVPMeshPlanner::~CVPMeshPlanner()
{
}

uint32_t CVPMeshPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                    std::string& message)
{
  const auto& mesh = mesh_map->mesh();
  std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>> path;

  // mesh_map->combineVertexCosts(); // TODO should be outside the planner

  ROS_INFO("start wave front propagation.");

  mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);
  mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);

  uint32_t outcome = waveFrontPropagation(goal_vec, start_vec, path);

  path.reverse();

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = mesh_map->mapFrame();

  cost = 0;
  float dir_length;
  if (!path.empty())
  {
    mesh_map::Vector vec = path.front().first;
    lvr2::FaceHandle fH = path.front().second;
    path.pop_front();

    const auto& face_normals = mesh_map->faceNormals();
    for (auto& next : path)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.pose = mesh_map::calculatePoseFromPosition(vec, next.first, face_normals[fH], dir_length);
      cost += dir_length;
      vec = next.first;
      fH = next.second;
      plan.push_back(pose);
    }

    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose = mesh_map::calculatePoseFromPosition(vec, goal_vec, face_normals[fH], dir_length);
    cost += dir_length;
    plan.push_back(pose);
  }

  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub.publish(path_msg);
  mesh_map->publishVertexCosts(potential, "Potential");
  ROS_INFO_STREAM("Path length: " << cost << "m");

  if (publish_vector_field)
  {
    mesh_map->publishVectorField("vector_field", vector_map, publish_face_vectors);
  }

  return outcome;
}

bool CVPMeshPlanner::cancel()
{
  cancel_planning = true;
  return true;
}

bool CVPMeshPlanner::initialize(const std::string& plugin_name,
                                  const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
{
  mesh_map = mesh_map_ptr;
  name = plugin_name;
  map_frame = mesh_map->mapFrame();
  private_nh = ros::NodeHandle("~/" + name);

  private_nh.param("publish_vector_field", publish_vector_field, false);
  private_nh.param("publish_face_vectors", publish_face_vectors, false);
  private_nh.param("goal_dist_offset", goal_dist_offset, 0.3f);

  path_pub = private_nh.advertise<nav_msgs::Path>("path", 1, true);
  const auto& mesh = mesh_map->mesh();
  direction = lvr2::DenseVertexMap<float>(mesh.nextVertexIndex(), 0);
  // TODO check all map dependencies! (loaded layers etc...)

  reconfigure_server_ptr = boost::shared_ptr<dynamic_reconfigure::Server<cvp_mesh_planner::CVPMeshPlannerConfig>>(
      new dynamic_reconfigure::Server<cvp_mesh_planner::CVPMeshPlannerConfig>(private_nh));

  config_callback = boost::bind(&CVPMeshPlanner::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

  return true;
}

void CVPMeshPlanner::reconfigureCallback(cvp_mesh_planner::CVPMeshPlannerConfig& cfg, uint32_t level)
{
  ROS_INFO_STREAM("New height diff layer config through dynamic reconfigure.");
  if (first_config)
  {
    config = cfg;
    first_config = false;
    return;
  }
  config = cfg;
}

void CVPMeshPlanner::computeVectorMap()
{
  const auto& mesh = mesh_map->mesh();
  const auto& face_normals = mesh_map->faceNormals();
  const auto& vertex_normals = mesh_map->vertexNormals();

  for (auto v3 : mesh.vertices())
  {
    // if(vertex_costs[v3] > config.cost_limit || !predecessors.containsKey(v3))
    // continue;

    const lvr2::VertexHandle& v1 = predecessors[v3];

    // if predecessor is pointing to it self, continue with the next vertex.
    if (v1 == v3)
      continue;

    // get the cut face
    const auto& optFh = cutting_faces.get(v3);
    // if no cut face, continue with the next vertex
    if (!optFh)
      continue;

    const lvr2::FaceHandle& fH = optFh.get();

    const auto& vec3 = mesh.getVertexPosition(v3);
    const auto& vec1 = mesh.getVertexPosition(v1);

    // compute the direction vector and rotate it by theta, which is stored in
    // the direction vertex map
    const auto dirVec = (vec1 - vec3).rotated(vertex_normals[v3], direction[v3]);
    // store the normalized rotated vector in the vector map
    vector_map.insert(v3, dirVec.normalized());
  }
  mesh_map->setVectorMap(vector_map);
}

uint32_t CVPMeshPlanner::waveFrontPropagation(const mesh_map::Vector& start, const mesh_map::Vector& goal,
                                                std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path)
{
  return waveFrontPropagation(start, goal, mesh_map->edgeDistances(), mesh_map->vertexCosts(), path, potential,
                              predecessors);
}

inline bool CVPMeshPlanner::waveFrontUpdateWithS(lvr2::DenseVertexMap<float>& distances,
                                                   const lvr2::DenseEdgeMap<float>& edge_weights,
                                                   const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2,
                                                   const lvr2::VertexHandle& v3)
{
  const auto& mesh = mesh_map->mesh();

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double u1_sq = u1 * u1;
  const double u2_sq = u2 * u2;

  const double A = sqrt(std::max<double>((-u1 + u2 + c) * (u1 - u2 + c) * (u1 + u2 - c) * (u1 + u2 + c), 0));
  const double B = sqrt(std::max<double>((-a + b + c) * (a - b + c) * (a + b - c) * (a + b + c), 0));
  const double sx = (c_sq + u1_sq - u2_sq) / (2 * c);
  const double sy = -A / (2 * c);
  const double p = (-a_sq + b_sq + c_sq) / (2 * c);
  const double hc = B / (2 * c);
  const double dy = hc - sy;
  // const double dy = (A + B) / (2 * c);
  // const double dx = (u2_sq - u1_sq + b_sq - a_sq) / (2*c);
  const double dx = p - sx;

  const double u3tmp_sq = dx * dx + dy * dy;
  double u3tmp = sqrt(u3tmp_sq);

  if (u3tmp < u3)
  {
    if (distances[v1] < distances[v2])
    {
      const double S = sy * p - sx * hc;
      const double t1cos = (u3tmp_sq + b_sq - u1_sq) / (2 * u3tmp * b);
      if (S <= 0 && std::fabs(t1cos) <= 1)
      {
        const double theta = acos(t1cos);
        predecessors[v3] = v1;
        direction[v3] = static_cast<float>(theta);
        distances[v3] = static_cast<float>(u3tmp);
        const lvr2::FaceHandle fh = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fh);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fh, theta, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        return true;
      }
      else
      {
        u3tmp = u1 + b;
        if (u3tmp < u3)
        {
          predecessors[v3] = v1;
          direction[v3] = 0;
          distances[v3] = u3tmp;
          const lvr2::FaceHandle fh = mesh.getFaceBetween(v1, v2, v3).unwrap();
          cutting_faces.insert(v3, fh);
#ifdef DEBUG
          mesh_map->publishDebugVector(v3, v1, fh, 0, mesh_map::color(0.9, 0.9, 0.2),
                                       "dir_vec" + std::to_string(v3.idx()));
#endif
          return true;
        }
        return false;
      }
    }
    else
    {
      const double S = sx * hc - hc * c + sy * c - sy * p;
      const double t2cos = (a_sq + u3tmp_sq - u2_sq) / (2 * a * u3tmp);
      if (S <= 0 && std::fabs(t2cos) <= 1)
      {
        const lvr2::FaceHandle fh = mesh.getFaceBetween(v1, v2, v3).unwrap();
        const double theta = -acos(t2cos);
        direction[v3] = static_cast<float>(theta);
        distances[v3] = static_cast<float>(u3tmp);
        predecessors[v3] = v2;
        cutting_faces.insert(v3, fh);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fh, theta, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        return true;
      }
      else
      {
        u3tmp = u2 + a;
        if (u3tmp < u3)
        {
          direction[v3] = 0;
          distances[v3] = u3tmp;
          predecessors[v3] = v2;
          const lvr2::FaceHandle fh = mesh.getFaceBetween(v1, v2, v3).unwrap();
          cutting_faces.insert(v3, fh);
#ifdef DEBUG
          mesh_map->publishDebugVector(v3, v2, fh, 0, mesh_map::color(0.9, 0.9, 0.2),
                                       "dir_vec" + std::to_string(v3.idx()));
#endif
          return true;
        }
        return false;
      }
    }
  }
  return false;
}

inline bool CVPMeshPlanner::waveFrontUpdate(lvr2::DenseVertexMap<float>& distances,
                                              const lvr2::DenseEdgeMap<float>& edge_weights,
                                              const lvr2::VertexHandle& v1, const lvr2::VertexHandle& v2,
                                              const lvr2::VertexHandle& v3)
{
  const auto& mesh = mesh_map->mesh();

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double u1_sq = u1 * u1;
  const double u2_sq = u2 * u2;

  const double sx = (c_sq + u1_sq - u2_sq) / (2 * c);
  const double sy = -sqrt(std::max(u1_sq - sx*sx, 0.0));

  const double p = (b_sq + c_sq -a_sq) / (2 * c);
  const double hc = sqrt(std::max(b_sq - p*p, 0.0));

  const double dy = hc - sy;
  const double dx = p - sx;

  const double u3tmp_sq = dx * dx + dy * dy;
  double u3tmp = sqrt(u3tmp_sq);

  if (!std::isfinite(u3tmp))
  {
    ROS_ERROR_STREAM("u3 tmp is not finite!");
  }
  if (u3tmp < u3)
  {
    const double t0a = (a_sq + b_sq - c_sq) / (2 * a * b);
    const double t1a = (u3tmp_sq + b_sq - u1_sq) / (2 * u3tmp * b);
    const double t2a = (a_sq + u3tmp_sq - u2_sq) / (2 * a * u3tmp);

    // corner case: side b + u1 ~= u3
    if (std::fabs(t1a) > 1)
    {
      u3tmp = u1 + b;
      if (u3tmp < u3)
      {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v1;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        distances[v3] = static_cast<float>(u3tmp);
        direction[v3] = 0;
        return true;
      }
      return false;
    }
    // corner case: side a + u2 ~= u3
    else if (std::fabs(t2a) > 1)
    {
      u3tmp = u2 + a;
      if (u3tmp < u3)
      {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v2;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        distances[v3] = static_cast<float>(u3tmp);
        direction[v3] = 0;
        return true;
      }
      return false;
    }

    const double theta0 = acos(t0a);
    const double theta1 = acos(t1a);
    const double theta2 = acos(t2a);

#ifdef DEBUG
    if (!std::isfinite(theta0 + theta1 + theta2))
    {
      ROS_ERROR_STREAM("------------------");
      if (std::isnan(theta0))
        ROS_ERROR_STREAM("Theta0 is NaN!");
      if (std::isnan(theta1))
        ROS_ERROR_STREAM("Theta1 is NaN!");
      if (std::isnan(theta2))
        ROS_ERROR_STREAM("Theta2 is NaN!");
      if (std::isinf(theta2))
        ROS_ERROR_STREAM("Theta2 is inf!");
      if (std::isinf(theta2))
        ROS_ERROR_STREAM("Theta2 is inf!");
      if (std::isinf(theta2))
        ROS_ERROR_STREAM("Theta2 is inf!");
      if (std::isnan(t1a))
        ROS_ERROR_STREAM("t1a is NaN!");
      if (std::isnan(t2a))
        ROS_ERROR_STREAM("t2a is NaN!");
      if (std::fabs(t2a) > 1)
      {
        ROS_ERROR_STREAM("|t2a| is > 1: " << t2a);
        ROS_INFO_STREAM("a: " << a << ", u3: " << u3tmp << ", u2: " << u2 << ", a+u2: " << a + u2);
      }
      if (std::fabs(t1a) > 1)
      {
        ROS_ERROR_STREAM("|t1a| is > 1: " << t1a);
        ROS_INFO_STREAM("b: " << b << ", u3: " << u3tmp << ", u1: " << u1 << ", b+u1: " << b + u1);
      }
    }
#endif

    if (theta1 < theta0 && theta2 < theta0)
    {
      auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
      cutting_faces.insert(v3, fH);
      distances[v3] = static_cast<float>(u3tmp);
      if (theta1 < theta2)
      {
        predecessors[v3] = v1;
        direction[v3] = theta1;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, theta1, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
      }
      else
      {
        predecessors[v3] = v2;
        direction[v3] = -theta2;
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, -theta2, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
      }
      return true;
    }
    else if (theta1 < theta2)
    {
      u3tmp = u1 + b;
      if (u3tmp < u3)
      {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v1;
        distances[v3] = static_cast<float>(u3tmp);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v1, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        direction[v3] = 0;
        return true;
      }
      return false;
    }
    else
    {
      u3tmp = u2 + a;
      if (u3tmp < u3)
      {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v2;
        distances[v3] = static_cast<float>(u3tmp);
#ifdef DEBUG
        mesh_map->publishDebugVector(v3, v2, fH, 0, mesh_map::color(0.9, 0.9, 0.2),
                                     "dir_vec" + std::to_string(v3.idx()));
#endif
        direction[v3] = 0;
        return true;
      }
      return false;
    }
  }
  return false;
}


inline bool CVPMeshPlanner::waveFrontUpdateFMM(
    lvr2::DenseVertexMap<float> &distances,
    const lvr2::DenseEdgeMap<float> &edge_weights,
    const lvr2::VertexHandle &v1tmp,
    const lvr2::VertexHandle &v2tmp,
    const lvr2::VertexHandle &v3)
{
  const auto& mesh = mesh_map->mesh();

  bool v1_smaller = distances[v1tmp] < distances[v2tmp];
  const lvr2::VertexHandle v1 = v1_smaller ? v1tmp : v2tmp;
  const lvr2::VertexHandle v2 = v1_smaller ? v2tmp : v1tmp;

  const double u1 = distances[v1];
  const double u2 = distances[v2];
  const double u3 = distances[v3];

  const lvr2::OptionalEdgeHandle e12h = mesh.getEdgeBetween(v1, v2);
  const double c = edge_weights[e12h.unwrap()];
  const double c_sq = c * c;

  const lvr2::OptionalEdgeHandle e13h = mesh.getEdgeBetween(v1, v3);
  const double b = edge_weights[e13h.unwrap()];
  const double b_sq = b * b;

  const lvr2::OptionalEdgeHandle e23h = mesh.getEdgeBetween(v2, v3);
  const double a = edge_weights[e23h.unwrap()];
  const double a_sq = a * a;

  const double delta_u = u2 - u1;
  const double cos_theta = (a_sq + b_sq - c_sq) / (2 * a * b);

  const double k0 = a_sq + b_sq - 2 * a * b * cos_theta;
  const double k1 = 2 * b * delta_u * (a * cos_theta - b);
  const double k2 = b_sq * (delta_u * delta_u - a_sq * (1 - cos_theta * cos_theta));

  double t;
  const double r = k1 * k1 - 4 * k0 * k2;
  if(r < 0)
  {
    t = -k1 / (2 * k0);
  }
  else
  {
    t = (-k1 + sqrt(r)) / (2 * k0);
  }

  const double e = b * (t - delta_u) / t;

  if(delta_u < t && (e < a / cos_theta) && (e > a * cos_theta))
  {
    const double u3_tmp = u1 + t;
    if(u3_tmp < u3)
    {
      auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
      cutting_faces.insert(v3, fH);
      predecessors[v3] = v1;
      distances[v3] = static_cast<float>(u3_tmp);
      const double theta = acos(cos_theta);
      const double phi = asin((e * sin(theta)) / sqrt(a_sq * e * e - 2 * a * cos_theta));
      direction[v3] = theta + phi - M_PI_2;
      return true;
    }
  }
  else {
    const double u1t = u1 + b;
    const double u2t = u2 + a;

    if (u1t < u2t) {
      if (u1t < u3) {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v1;
        distances[v3] = static_cast<float>(u1t);
        direction[v3] = 0;
        return true;
      }
    } else {
      if (u2t < u3) {
        auto fH = mesh.getFaceBetween(v1, v2, v3).unwrap();
        cutting_faces.insert(v3, fH);
        predecessors[v3] = v2;
        distances[v3] = static_cast<float>(u2t);
        direction[v3] = 0;
        return true;
      }
    }
  }

return false;
}

uint32_t CVPMeshPlanner::waveFrontPropagation(const mesh_map::Vector& original_start,
                                                const mesh_map::Vector& original_goal,
                                                const lvr2::DenseEdgeMap<float>& edge_weights,
                                                const lvr2::DenseVertexMap<float>& costs,
                                                std::list<std::pair<mesh_map::Vector, lvr2::FaceHandle>>& path,
                                                lvr2::DenseVertexMap<float>& distances,
                                                lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors)
{
  ROS_DEBUG_STREAM("Init wave front propagation.");

  const auto& mesh = mesh_map->mesh();
  const auto& vertex_costs = mesh_map->vertexCosts();
  auto& invalid = mesh_map->invalid;

  mesh_map->publishDebugPoint(original_start, mesh_map::color(0, 1, 0), "start_point");
  mesh_map->publishDebugPoint(original_goal, mesh_map::color(0, 0, 1), "goal_point");

  mesh_map::Vector start = original_start;
  mesh_map::Vector goal = original_goal;

  // Find the containing faces of start and goal
  const auto& start_opt = mesh_map->getContainingFace(start, 0.4);
  const auto& goal_opt = mesh_map->getContainingFace(goal, 0.4);

  ros::WallTime t_initialization_start = ros::WallTime::now();

  // reset cancel planning
  cancel_planning = false;

  if (!start_opt)
    return mbf_msgs::GetPathResult::INVALID_START;
  if (!goal_opt)
    return mbf_msgs::GetPathResult::INVALID_GOAL;

  const auto& start_face = start_opt.unwrap();
  const auto& goal_face = goal_opt.unwrap();

  mesh_map->publishDebugFace(start_face, mesh_map::color(0, 0, 1), "start_face");
  mesh_map->publishDebugFace(goal_face, mesh_map::color(0, 1, 0), "goal_face");

  path.clear();
  distances.clear();
  predecessors.clear();

  if (goal_face == start_face)
  {
    return mbf_msgs::GetPathResult::SUCCESS;
  }

  lvr2::DenseVertexMap<bool> fixed(mesh.nextVertexIndex(), false);

  // clear vector field map
  vector_map.clear();

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for (auto const& vH : mesh.vertices())
  {
    distances.insert(vH, std::numeric_limits<float>::infinity());
    predecessors.insert(vH, vH);
  }

  lvr2::Meap<lvr2::VertexHandle, float> pq;
  // Set start distance to zero
  // add start vertex to priority queue
  for (auto vH : mesh.getVerticesOfFace(start_face))
  {
    const mesh_map::Vector diff = start - mesh.getVertexPosition(vH);
    const float dist = diff.length();
    distances[vH] = dist;
    vector_map.insert(vH, diff);
    cutting_faces.insert(vH, start_face);
    fixed[vH] = true;
    pq.insert(vH, dist);
  }

  std::array<lvr2::VertexHandle, 3> goal_vertices = mesh.getVerticesOfFace(goal_face);
  ROS_DEBUG_STREAM("The goal is at (" << goal.x << ", " << goal.y << ", " << goal.z << ") at the face ("
                                     << goal_vertices[0] << ", " << goal_vertices[1] << ", " << goal_vertices[2]
                                     << ")");
  mesh_map->publishDebugPoint(mesh.getVertexPosition(goal_vertices[0]), mesh_map::color(0, 0, 1), "goal_face_v1");
  mesh_map->publishDebugPoint(mesh.getVertexPosition(goal_vertices[1]), mesh_map::color(0, 0, 1), "goal_face_v2");
  mesh_map->publishDebugPoint(mesh.getVertexPosition(goal_vertices[2]), mesh_map::color(0, 0, 1), "goal_face_v3");

  float goal_dist = std::numeric_limits<float>::infinity();

  ROS_DEBUG_STREAM("Start wavefront propagation...");

  size_t fixed_cnt = 0;
  size_t fixed_set_cnt = 0;
  ros::WallTime t_wavefront_start = ros::WallTime::now();
  double initialization_duration = (t_wavefront_start - t_initialization_start).toNSec() * 1e-6;

  while (!pq.isEmpty() && !cancel_planning)
  {
    lvr2::VertexHandle current_vh = pq.popMin().key();

    fixed[current_vh] = true;
    fixed_set_cnt++;

    if (distances[current_vh] > goal_dist)
      continue;

    if (vertex_costs[current_vh] > config.cost_limit)
      continue;

    if (invalid[current_vh])
      continue;

    if (current_vh == goal_vertices[0] || current_vh == goal_vertices[1] || current_vh == goal_vertices[2])
    {
      if (goal_dist == std::numeric_limits<float>::infinity() && fixed[goal_vertices[0]] && fixed[goal_vertices[1]] &&
          fixed[goal_vertices[2]])
      {
        ROS_DEBUG_STREAM("Wave front reached the goal!");
        goal_dist = distances[current_vh] + goal_dist_offset;
      }
    }

    try
    {
      std::vector<lvr2::FaceHandle> faces;
      mesh.getFacesOfVertex(current_vh, faces);

      for (auto fh : faces)
      {
        const auto vertices = mesh.getVerticesOfFace(fh);
        const lvr2::VertexHandle& a = vertices[0];
        const lvr2::VertexHandle& b = vertices[1];
        const lvr2::VertexHandle& c = vertices[2];

        if (invalid[a] || invalid[b] || invalid[c])
          continue;

        // We are looking for a face where exactly
        // one vertex is not in the fixed set
        if (fixed[a] && fixed[b] && fixed[c])
        {
          // All distance values of the face are already fixed
#ifdef DEBUG
          mesh_map->publishDebugFace(fh, mesh_map::color(1, 0, 0), "fmm_fixed_" + std::to_string(fixed_cnt++));
#endif
          continue;
        }
        else if (fixed[a] && fixed[b] && !fixed[c])
        {
          // c is free
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, a, b, c))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, a, b, c))
#else
          if (waveFrontUpdate(distances, edge_weights, a, b, c))
#endif
          {
            pq.insert(c, distances[c]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else if (fixed[a] && !fixed[b] && fixed[c])
        {
          // b is free
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, c, a, b))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, c, a, b))
#else
          if (waveFrontUpdate(distances, edge_weights, c, a, b))
#endif
          {
            pq.insert(b, distances[b]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else if (!fixed[a] && fixed[b] && fixed[c])
        {
          // a if free
#ifdef USE_UPDATE_WITH_S
          if (waveFrontUpdateWithS(distances, edge_weights, b, c, a))
#elif defined USE_UPDATE_FMM
          if (waveFrontUpdateFMM(distances, edge_weights, b, c, a))
#else
          if (waveFrontUpdate(distances, edge_weights, b, c, a))
#endif
          {
            pq.insert(a, distances[a]);
#ifdef DEBUG
            mesh_map->publishDebugFace(fh, mesh_map::color(0, 1, 1), "fmm_update");
            sleep(2);
#endif
          }
        }
        else
        {
          // two free vertices -> skip that face
          continue;
        }
      }
    }
    catch (lvr2::PanicException exception)
    {
      invalid.insert(current_vh, true);
      ROS_ERROR_STREAM("Found invalid vertex!");
      continue;
    }
    catch (lvr2::VertexLoopException exception)
    {
      invalid.insert(current_vh, true);
      ROS_ERROR_STREAM("Found invalid vertex!");
      continue;
    }
  }

  if (cancel_planning)
  {
    ROS_WARN_STREAM("Wave front propagation has been canceled!");
    return mbf_msgs::GetPathResult::CANCELED;
  }
  ros::WallTime t_wavefront_end = ros::WallTime::now();
  double wavefront_propagation_duration = (t_wavefront_end - t_wavefront_start).toNSec() * 1e-6;
  ROS_DEBUG_STREAM("Finished wave front propagation.");
  ROS_DEBUG_STREAM("Computing the vector map...");
  computeVectorMap();

  ros::WallTime t_vector_field_end = ros::WallTime::now();
  double vector_field_duration = (t_vector_field_end - t_wavefront_end).toNSec() * 1e-6;

  bool path_exists = false;
  for (auto goal_vertex : goal_vertices)
  {
    if (goal_vertex != predecessors[goal_vertex])
    {
      path_exists = true;
      break;
    }
  }

  if (!path_exists)
  {
    ROS_WARN("Predecessor of the goal is not set! No path found!");
    return mbf_msgs::GetPathResult::NO_PATH_FOUND;
  }

  ROS_DEBUG_STREAM("Start vector field back tracking!");

  lvr2::FaceHandle current_face = goal_face;
  mesh_map::Vector current_pos = goal;
  path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(current_pos, current_face));

  // move from the goal position towards the start position
  while (current_pos.distance2(start) > config.step_width && !cancel_planning)
  {
    // move current pos ahead on the surface following the vector field,
    // updates the current face if necessary
    try
    {
      if (mesh_map->meshAhead(current_pos, current_face, config.step_width))
      {
        path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(current_pos, current_face));
      }
      else
      {
        ROS_WARN_STREAM("Could not find a valid path, while back-tracking from the goal");
        return mbf_msgs::GetPathResult::NO_PATH_FOUND;
      }
    }
    catch (lvr2::PanicException exception)
    {
      ROS_ERROR_STREAM("Could not find a valid path, while back-tracking from the goal: HalfEdgeMesh panicked!");
      return mbf_msgs::GetPathResult::NO_PATH_FOUND;
    }
  }
  path.push_front(std::pair<mesh_map::Vector, lvr2::FaceHandle>(start, start_face));

  ros::WallTime t_path_backtracking = ros::WallTime::now();
  double path_backtracking_duration = (t_path_backtracking - t_vector_field_end).toNSec() * 1e-6;

  ROS_INFO_STREAM("Processed " << fixed_set_cnt << " vertices in the fixed set.");
  ROS_INFO_STREAM("Initialization duration (ms): " << initialization_duration);
  ROS_INFO_STREAM("Execution time wavefront propagation (ms): "<< wavefront_propagation_duration);
  ROS_INFO_STREAM("Vector field post computation (ms): " << vector_field_duration);
  ROS_INFO_STREAM("Path backtracking duration (ms): " << path_backtracking_duration);

  if (cancel_planning)
  {
    ROS_WARN_STREAM("Wave front propagation has been canceled!");
    return mbf_msgs::GetPathResult::CANCELED;
  }

  ROS_INFO_STREAM("Successfully finished vector field back tracking!");
  return mbf_msgs::GetPathResult::SUCCESS;
}

} /* namespace cvp_mesh_planner */

