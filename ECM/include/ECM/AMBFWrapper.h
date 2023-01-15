#ifndef _AMBFWRAPPER_
#define _AMBFWRAPPER_

#include "PCH/pch.h"

enum class RigidBodyID
{
  BASELINK, TARGETFK, TARGETIK
};

// Make it as a singleton or nested class of AutomatingECM
class AMBFWrapper
{
public:
  AMBFWrapper();
  const std::vector<std::string> ObjectNames() const { return AMBFClient_->getRigidBodyNames(); }
  const bool IsActive(RigidBodyID rigidBodyId) const;
  
  const Vector3f P_N_W(RigidBodyID rigidBodyId);
  void P_N_W(RigidBodyID rigidBodyId, const Vector3f P_n_w) const;
  
  const Vector3f R_N_W_rpy(RigidBodyID rigidBodyId);
  void R_N_W_rpy(RigidBodyID rigidBodyId, const Vector3f P_n_w_rpy) const;

  void JointPose(RigidBodyID rigidBodyId, std::vector<float> desiredQs) const;
  ~AMBFWrapper();

private:
  RigidBody* RigidBodyToHandler(RigidBodyID rigidBodyId) const;

private:
  Client* AMBFClient_ {nullptr};
  RigidBody* baselinkHandler_ {nullptr};
  RigidBody* targetFKHandler_ {nullptr};
  RigidBody* targetIKHandler_ {nullptr};
  std::vector<std::string>controlableJoints_;
};

#endif