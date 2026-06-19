/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/** Ported to Gazebo Harmonic (gz-sim 8) / ROS 2 Jazzy.                */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_SFM_PLUGIN_PEDESTRIANSFMPLUGIN_HH_
#define GAZEBO_SFM_PLUGIN_PEDESTRIANSFMPLUGIN_HH_

// C++
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Gazebo (gz-sim 8 / Harmonic)
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace gazebo_sfm_plugin
{
/// \brief A gz-sim System that drives a Gazebo actor using the
/// Social Force Model (lightsfm). Replaces the Gazebo Classic ModelPlugin.
class PedestrianSFMPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  /// \brief Constructor
public:
  PedestrianSFMPlugin();

  /// \brief Destructor
public:
  ~PedestrianSFMPlugin() override = default;

  // Documentation inherited (ISystemConfigure).
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited (ISystemPreUpdate).
public:
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Helper to detect the closest obstacle (fills sfmActor.obstacles1).
private:
  void HandleObstacles(gz::sim::EntityComponentManager &_ecm);

  /// \brief Helper to detect nearby pedestrians (other actors).
private:
  void HandlePedestrians(gz::sim::EntityComponentManager &_ecm);

  //-------------------------------------------------

  /// \brief this actor as a SFM agent
private:
  sfm::Agent sfmActor;

  /// \brief names of the other models in my walking group.
private:
  std::vector<std::string> groupNames;

  /// \brief vector of pedestrians detected.
private:
  std::vector<sfm::Agent> otherActors;

  /// \brief Maximum distance to detect nearby pedestrians.
private:
  double peopleDistance = 5.0;

  /// \brief Entity of the actor this system is attached to.
private:
  gz::sim::Entity actorEntity{gz::sim::kNullEntity};

  /// \brief Entity of the world.
private:
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};

  /// \brief Copy of the plugin SDF element (for deferred parsing).
private:
  std::shared_ptr<const sdf::Element> sdf;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update (sim time).
private:
  std::chrono::steady_clock::duration lastUpdate{0};

  /// \brief Accumulated animation/script time for the skeleton animation.
private:
  std::chrono::steady_clock::duration animationTime{0};

  /// \brief Last world pose written for the actor (for velocity estimation).
private:
  gz::math::Pose3d lastActorPose;

  /// \brief Time delta of the current update step (seconds).
private:
  double currentDt = 0.0;

  /// \brief Last known planar position of each other pedestrian, used to
  /// estimate their velocity (actors are kinematic and expose no velocity).
private:
  std::unordered_map<gz::sim::Entity, gz::math::Vector3d> prevPedPos;

  /// \brief Whether the actor pose/animation has been initialized.
private:
  bool initialized = false;

  /// \brief List of model names to ignore for obstacle avoidance.
private:
  std::vector<std::string> ignoreModels;

  /// \brief Animation name of this actor.
private:
  std::string animationName;
};
}  // namespace gazebo_sfm_plugin

#endif  // GAZEBO_SFM_PLUGIN_PEDESTRIANSFMPLUGIN_HH_
