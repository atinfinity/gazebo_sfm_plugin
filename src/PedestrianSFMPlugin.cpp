/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.cpp                                            */
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

#include <gazebo_sfm_plugin/PedestrianSFMPlugin.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/AxisAlignedBox.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>

using namespace gazebo_sfm_plugin;

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedestrianSFMPlugin::PedestrianSFMPlugin() = default;

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->actorEntity = _entity;
  this->sdf = _sdf;
  this->worldEntity = gz::sim::worldEntity(_ecm);

  // Verify the plugin is attached to an actor.
  if (!_ecm.EntityHasComponentType(_entity,
        gz::sim::components::Actor::typeId))
  {
    gzerr << "[PedestrianSFMPlugin] Plugin must be attached to an <actor>; "
          << "entity [" << _entity << "] is not an actor. Disabling.\n";
    this->actorEntity = gz::sim::kNullEntity;
    return;
  }

  // A mutable clone is required because nested traversal (GetElement) is not
  // available on a const sdf::Element.
  sdf::ElementPtr sdfClone = _sdf->Clone();

  // SFM agent identity.
  this->sfmActor.id = static_cast<int>(this->actorEntity);

  // Initialize the SFM agent state from the actor's initial world pose.
  gz::math::Pose3d pose = gz::sim::worldPose(this->actorEntity, _ecm);
  gz::math::Vector3d rpy = pose.Rot().Euler();
  this->sfmActor.position.set(pose.Pos().X(), pose.Pos().Y());
  this->sfmActor.yaw = utils::Angle::fromRadian(rpy.Z());
  // Actors are kinematic (no physics), so velocity starts at zero and is later
  // estimated from successive poses.
  this->sfmActor.velocity.set(0.0, 0.0);
  this->sfmActor.linearVelocity = 0.0;
  this->sfmActor.angularVelocity = 0.0;
  this->lastActorPose = pose;

  // Maximum (desired) velocity of the pedestrian.
  this->sfmActor.desiredVelocity = sdfClone->Get<double>("velocity", 0.8).first;

  // Optional radius (kept faithful to the SFM default when absent).
  this->sfmActor.radius =
      sdfClone->Get<double>("radius", this->sfmActor.radius).first;

  // Social Force Model weights (preserve lightsfm defaults when not given).
  this->sfmActor.params.forceFactorDesired =
      sdfClone->Get<double>("goal_weight",
                            this->sfmActor.params.forceFactorDesired).first;
  this->sfmActor.params.forceFactorObstacle =
      sdfClone->Get<double>("obstacle_weight",
                            this->sfmActor.params.forceFactorObstacle).first;
  this->sfmActor.params.forceFactorSocial =
      sdfClone->Get<double>("social_weight",
                            this->sfmActor.params.forceFactorSocial).first;
  this->sfmActor.params.forceFactorGroupGaze =
      sdfClone->Get<double>("group_gaze_weight",
                            this->sfmActor.params.forceFactorGroupGaze).first;
  this->sfmActor.params.forceFactorGroupCoherence =
      sdfClone->Get<double>("group_coh_weight",
                            this->sfmActor.params.forceFactorGroupCoherence).first;
  this->sfmActor.params.forceFactorGroupRepulsion =
      sdfClone->Get<double>("group_rep_weight",
                            this->sfmActor.params.forceFactorGroupRepulsion).first;

  // Animation parameters.
  this->animationFactor = sdfClone->Get<double>("animation_factor", 4.5).first;
  this->animationName =
      sdfClone->Get<std::string>("animation_name", WALKING_ANIMATION).first;
  this->peopleDistance = sdfClone->Get<double>("people_distance", 5.0).first;

  // Pedestrians in this actor's walking group.
  if (sdfClone->HasElement("group"))
  {
    this->sfmActor.groupId = this->sfmActor.id;
    sdf::ElementPtr modelElem =
        sdfClone->GetElement("group")->GetElement("model");
    while (modelElem)
    {
      this->groupNames.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  else
  {
    this->sfmActor.groupId = -1;
  }

  // Other obstacles to ignore (by model name).
  if (sdfClone->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
        sdfClone->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  // Always ignore our own model.
  auto *nameComp = _ecm.Component<gz::sim::components::Name>(this->actorEntity);
  if (nameComp)
    this->ignoreModels.push_back(nameComp->Data());

  // Goals / trajectory waypoints.
  if (sdfClone->HasElement("trajectory"))
  {
    sdf::ElementPtr trajElem = sdfClone->GetElement("trajectory");
    if (trajElem->HasElement("cyclic"))
      this->sfmActor.cyclicGoals = trajElem->Get<bool>("cyclic", false).first;

    sdf::ElementPtr wpElem = trajElem->GetElement("waypoint");
    while (wpElem)
    {
      gz::math::Vector3d g = wpElem->Get<gz::math::Vector3d>();
      sfm::Goal goal;
      goal.center.set(g.X(), g.Y());
      goal.radius = 0.3;
      this->sfmActor.goals.push_back(goal);
      wpElem = wpElem->GetNextElement("waypoint");
    }
  }

  // --- Set up the components needed to drive the actor programmatically. ---
  namespace components = gz::sim::components;

  // Choose which skeleton animation to play.
  auto *animNameComp =
      _ecm.Component<components::AnimationName>(this->actorEntity);
  if (nullptr == animNameComp)
    _ecm.CreateComponent(this->actorEntity,
                         components::AnimationName(this->animationName));
  else
    *animNameComp = components::AnimationName(this->animationName);
  _ecm.SetChanged(this->actorEntity, components::AnimationName::typeId,
                  gz::sim::ComponentState::OneTimeChange);

  // Animation time is advanced by this plugin to coordinate the walk cycle.
  if (nullptr == _ecm.Component<components::AnimationTime>(this->actorEntity))
    _ecm.CreateComponent(this->actorEntity, components::AnimationTime());

  // The rendered world pose is composed as Pose * TrajectoryPose. We zero the
  // base Pose so TrajectoryPose alone defines the actor's full world pose,
  // matching the Gazebo Classic SetWorldPose() semantics.
  auto *poseComp = _ecm.Component<components::Pose>(this->actorEntity);
  if (nullptr == poseComp)
    _ecm.CreateComponent(this->actorEntity,
                         components::Pose(gz::math::Pose3d::Zero));
  else
    *poseComp = components::Pose(gz::math::Pose3d::Zero);

  // Initial trajectory pose. The DAE skins are authored Z-up (upright) and face
  // their heading at yaw=0, so neither the roll nor the yaw offset that Gazebo
  // Classic required are needed in gz-sim.
  gz::math::Pose3d initTraj(
      pose.Pos().X(), pose.Pos().Y(), 1.20,
      0.0, 0.0, this->sfmActor.yaw.toRadian());
  if (nullptr == _ecm.Component<components::TrajectoryPose>(this->actorEntity))
    _ecm.CreateComponent(this->actorEntity,
                         components::TrajectoryPose(initTraj));
  else
    *_ecm.Component<components::TrajectoryPose>(this->actorEntity) =
        components::TrajectoryPose(initTraj);
  _ecm.SetChanged(this->actorEntity, components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);
  this->lastActorPose = initTraj;

  gzmsg << "[PedestrianSFMPlugin] configured actor entity [" << _entity
        << "] with " << this->sfmActor.goals.size() << " goal(s).\n";
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (this->actorEntity == gz::sim::kNullEntity || _info.paused)
    return;

  namespace components = gz::sim::components;

  // Time delta (seconds).
  std::chrono::duration<double> dtDur = _info.simTime - this->lastUpdate;
  double dt = dtDur.count();
  this->lastUpdate = _info.simTime;
  if (dt <= 0.0)
    return;
  this->currentDt = dt;

  auto *trajPoseComp =
      _ecm.Component<components::TrajectoryPose>(this->actorEntity);
  if (nullptr == trajPoseComp)
    return;
  gz::math::Pose3d actorPose = trajPoseComp->Data();

  // Update the SFM agent's perception of the world.
  this->HandleObstacles(_ecm);
  this->HandlePedestrians(_ecm);

  // Compute social forces and integrate the agent's motion.
  sfm::SFM.computeForces(this->sfmActor, this->otherActors);
  sfm::SFM.updatePosition(this->sfmActor, dt);

  // Desired heading: the SFM yaw, with rotate-in-place smoothing as in the
  // original plugin. (gz-sim needs no model-orientation offset.)
  utils::Angle h = this->sfmActor.yaw;
  double yaw = h.toRadian();
  gz::math::Vector3d rpy = actorPose.Rot().Euler();
  utils::Angle current = utils::Angle::fromRadian(rpy.Z());
  double diff = (h - current).toRadian();
  if (std::fabs(diff) > GZ_DTOR(10))
  {
    current = current + utils::Angle::fromRadian(diff * 0.005);
    yaw = current.toRadian();
  }

  gz::math::Pose3d newPose;
  newPose.Pos().X(this->sfmActor.position.getX());
  newPose.Pos().Y(this->sfmActor.position.getY());
  newPose.Pos().Z(1.20);
  newPose.Rot() = gz::math::Quaterniond(0.0, 0.0, yaw);

  // Distance traveled coordinates translational motion with the walk cycle.
  double distanceTraveled = (newPose.Pos() - actorPose.Pos()).Length();

  // Write the new actor root pose.
  *trajPoseComp = components::TrajectoryPose(newPose);
  _ecm.SetChanged(this->actorEntity, components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);
  this->lastActorPose = newPose;

  // Advance the skeleton animation proportionally to distance traveled.
  auto *animTimeComp =
      _ecm.Component<components::AnimationTime>(this->actorEntity);
  if (nullptr != animTimeComp)
  {
    auto animTime = animTimeComp->Data() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                distanceTraveled * this->animationFactor));
    *animTimeComp = components::AnimationTime(animTime);
    _ecm.SetChanged(this->actorEntity, components::AnimationTime::typeId,
                    gz::sim::ComponentState::OneTimeChange);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandleObstacles(
    gz::sim::EntityComponentManager &_ecm)
{
  namespace components = gz::sim::components;

  double minDist = 10000.0;
  gz::math::Vector3d closestObs;
  this->sfmActor.obstacles1.clear();

  const gz::math::Vector3d actorPos(this->sfmActor.position.getX(),
                                    this->sfmActor.position.getY(), 1.2138);

  _ecm.Each<components::Model, components::Name>(
      [&](const gz::sim::Entity &_ent,
          const components::Model *,
          const components::Name *_name) -> bool
      {
        // Other pedestrians are handled separately, not as static obstacles.
        if (_ecm.EntityHasComponentType(_ent, components::Actor::typeId))
          return true;
        // Explicitly ignored models (and our own model).
        if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                      _name->Data()) != this->ignoreModels.end())
          return true;

        // The world-frame AABB is populated by the physics system, but only
        // for models that carry the component. Request it lazily; the value
        // becomes available on a subsequent update.
        auto *bboxComp = _ecm.Component<components::AxisAlignedBox>(_ent);
        if (nullptr == bboxComp)
        {
          _ecm.CreateComponent(_ent, components::AxisAlignedBox());
          return true;
        }

        gz::math::AxisAlignedBox box = bboxComp->Data();
        if (box == gz::math::AxisAlignedBox())  // not populated yet
          return true;

        gz::math::Vector3d modelPos = gz::sim::worldPose(_ent, _ecm).Pos();
        std::tuple<bool, double, gz::math::Vector3d> intersect =
            box.Intersect(modelPos, actorPos, 0.05, 8.0);

        if (std::get<0>(intersect))
        {
          gz::math::Vector3d offset = std::get<2>(intersect) - actorPos;
          double modelDist = offset.Length();
          if (modelDist < minDist)
          {
            minDist = modelDist;
            closestObs = std::get<2>(intersect);
          }
        }
        return true;
      });

  if (minDist <= 10.0)
  {
    utils::Vector2d ob(closestObs.X(), closestObs.Y());
    this->sfmActor.obstacles1.push_back(ob);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandlePedestrians(
    gz::sim::EntityComponentManager &_ecm)
{
  namespace components = gz::sim::components;

  this->otherActors.clear();

  const gz::math::Vector3d selfPos(this->sfmActor.position.getX(),
                                   this->sfmActor.position.getY(), 0.0);

  // Every actor in the world is a potential pedestrian.
  _ecm.Each<components::Actor, components::Name>(
      [&](const gz::sim::Entity &_ent,
          const components::Actor *,
          const components::Name *_name) -> bool
      {
        if (_ent == this->actorEntity)
          return true;

        // Other actors are driven kinematically via their TrajectoryPose;
        // their base Pose is zeroed, so read the trajectory pose for the
        // real world pose (falling back to the Pose for scripted actors).
        gz::math::Pose3d otherPose;
        auto *tp = _ecm.Component<components::TrajectoryPose>(_ent);
        if (nullptr != tp)
          otherPose = tp->Data();
        else
          otherPose = gz::sim::worldPose(_ent, _ecm);

        const gz::math::Vector3d otherPos(otherPose.Pos().X(),
                                          otherPose.Pos().Y(), 0.0);

        // Estimate velocity from the previous planar position.
        gz::math::Vector3d vel(0.0, 0.0, 0.0);
        auto prevIt = this->prevPedPos.find(_ent);
        if (prevIt != this->prevPedPos.end() && this->currentDt > 0.0)
          vel = (otherPos - prevIt->second) / this->currentDt;
        this->prevPedPos[_ent] = otherPos;

        if ((otherPos - selfPos).Length() >= this->peopleDistance)
          return true;

        sfm::Agent ped;
        ped.id = static_cast<int>(_ent);
        ped.position.set(otherPos.X(), otherPos.Y());
        gz::math::Vector3d rpy = otherPose.Rot().Euler();
        ped.yaw = utils::Angle::fromRadian(rpy.Z());
        ped.radius = this->sfmActor.radius;
        ped.velocity.set(vel.X(), vel.Y());
        ped.linearVelocity = vel.Length();
        ped.angularVelocity = 0.0;

        // Group membership.
        if (this->sfmActor.groupId != -1)
        {
          auto it = std::find(this->groupNames.begin(),
                              this->groupNames.end(), _name->Data());
          ped.groupId = (it != this->groupNames.end())
                            ? this->sfmActor.groupId
                            : -1;
        }

        this->otherActors.push_back(ped);
        return true;
      });
}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(PedestrianSFMPlugin,
              gz::sim::System,
              PedestrianSFMPlugin::ISystemConfigure,
              PedestrianSFMPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(PedestrianSFMPlugin,
                    "gazebo_sfm_plugin::PedestrianSFMPlugin")
