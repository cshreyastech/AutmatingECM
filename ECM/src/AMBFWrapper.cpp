#include "ECM/AMBFWrapper.h"

AMBFWrapper::AMBFWrapper()
{
  AMBFClient_ = new Client("ECM");
  AMBFClient_->connect();
  usleep(20000);


  std::vector<std::string> objectNames = AMBFClient_->getRigidBodyNames();
  for(std::string objectName : objectNames)
    std::cout << objectName << ", ";
  std::cout << std::endl;

  baselinkHandler_ = AMBFClient_->getRigidBody("ecm/baselink", true);
  targetFKHandler_ = AMBFClient_->getRigidBody("ecm/target_fk", true);
  targetIKHandler_ = AMBFClient_->getRigidBody("ecm/target_ik", true);
  usleep(1000000);

  baselinkHandler_->set_joint_pos<int>(0, 0);
  controlableJoints_ = baselinkHandler_->get_joint_names();
}

RigidBody* AMBFWrapper::RigidBodyToHandler(RigidBodyID rigidBodyId) const
{
  switch (rigidBodyId)
  {
  case RigidBodyID::BASELINK:
    return baselinkHandler_;
    break;
  case RigidBodyID::TARGETFK :
    return targetFKHandler_;
    break;
  case RigidBodyID::TARGETIK :
    return targetIKHandler_;
    break;

  default:
    return nullptr;
    break;
  }
}

const bool AMBFWrapper::IsActive(RigidBodyID rigidBodyId) const
{
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  return (handler != nullptr);
}

const Vector3f AMBFWrapper::P_N_W(RigidBodyID rigidBodyId)
{
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  // Implement exception handling class
  if (handler == nullptr) throw("Handler - null pointer exception\n");
  tf::Vector3 P_n_w_tf = handler->get_pos();

  
  return Vector3f(P_n_w_tf[0], P_n_w_tf[1], P_n_w_tf[2]);
}

void AMBFWrapper::P_N_W(RigidBodyID rigidBodyId, const Vector3f P_n_w) const
{
  // Check for null pointer
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  if (handler == nullptr) throw("Handler - null pointer exception\n");
  
  handler->set_pos(P_n_w(0), P_n_w(1), P_n_w(2));
}

const Vector3f AMBFWrapper::R_N_W_rpy(RigidBodyID rigidBodyId)
{
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  // Implement exception handling class
  if (handler == nullptr) throw("Handler - null pointer exception\n");

  tf::Vector3 R_n_w_tf = handler->get_rpy();
  return Vector3f(R_n_w_tf[0], R_n_w_tf[1], R_n_w_tf[2]);
}

void AMBFWrapper::R_N_W_rpy(RigidBodyID rigidBodyId, const Vector3f P_n_w_rpy) const
{
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  if (handler == nullptr) throw("Handler - null pointer exception\n");

  handler->set_rpy(P_n_w_rpy(0), P_n_w_rpy(1), P_n_w_rpy(2));
}

void AMBFWrapper::JointPose(RigidBodyID rigidBodyId, std::vector<float> desiredQs) const
{
  RigidBody* handler = RigidBodyToHandler(rigidBodyId);
  if (handler == nullptr) throw("Handler - null pointer exception\n");

  if(rigidBodyId == RigidBodyID::BASELINK && desiredQs.size() != controlableJoints_.size())
    throw("Invalid Number of joints commaned\n");

  for(int i = 0; i < desiredQs.size(); i++)
    handler->set_joint_pos<std::string>(controlableJoints_.at(i), desiredQs.at(i));
}

AMBFWrapper::~AMBFWrapper()
{
  std::cout << "~AMBFWrapper()\n";
  
  delete baselinkHandler_;
  delete targetFKHandler_;
  delete targetIKHandler_;

  AMBFClient_->~Client();
}