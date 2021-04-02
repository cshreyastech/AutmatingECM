#ifndef ECM_H
#define ECM_H

#include<vector>
#include<cmath>
#include<iostream>
#include <unordered_map>
#include <memory>
#include <any>
#include <array>
#include <algorithm>
#include <type_traits>

#include "DH.h"
#include "Utilities.h"

//#include "ambf_client/ambf_client.h"
#include "ecm_manupulation/AMBFClient.h"

//------------------------------------------------------------------------------

typedef Client* ClientPtr;

//------------------------------------------------------------------------------

namespace ECMSpecs
{
  enum Bodies
  {
    baselink = 0,
    yawlink = 1,
    pitchbacklink = 2,
    maininsertionlink = 3,
    toollink = 4
  };

  enum BodyParams
  {
    mass = 0,
    inertia = 1
  };


  enum Joints
  {
    baselink_yawlink = 0,
    yawlink_pitchbacklink = 1,
    pitchendlink_maininsertionlink = 2,
    maininsertionlink_toollink = 3
  };

  enum JointParams
  {
    dh = 0,
    joints_limit = 1
  };

  static const Bodies allBodies[] = { baselink, yawlink, pitchbacklink, maininsertionlink, toollink };
  static const BodyParams allBodyParams[] = { mass, inertia };

  static const Joints allJoints[] = { baselink_yawlink, yawlink_pitchbacklink, pitchendlink_maininsertionlink, maininsertionlink_toollink };
  static const JointParams allJointParams[] = { dh, joints_limit};
}


struct EnumClassHash
{
    template <typename T>
    std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

class ECM
{
public:
    ECM();

    void cleanup();
    ~ECM(void);

protected:
    std::vector<DH *> DH_Vector;
    const float L_rcc = 0.3822;
    const float L_scopelen = 0.385495;

    const std::string ecm_bodies_enum_to_str(ECMSpecs::Bodies enumVal);
    const std::string ecm_body_params_enum_to_str(ECMSpecs::BodyParams enumVal);
    const std::string ecm_joints_enum_to_str(ECMSpecs::Joints enumVal);
    const std::string ecm_joint_params_enum_to_str(ECMSpecs::JointParams enumVal);

    template <typename TT>
    const std::string enum_to_str(TT t);
    std::string getECMBodies(const ECMSpecs::Bodies e);
    std::string getECMBodyParams(const ECMSpecs::BodyParams e);
    const std::string getECMJoints(const ECMSpecs::Joints e);
    const std::string getECMJointParams(const ECMSpecs::JointParams e);

    template <typename Key>
    using HashType = typename std::conditional<std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;

    template <typename Key, typename T>
    using enum_unordered_map = std::unordered_map<Key, T, HashType<Key>>;

    //                  body name                      //property
    enum_unordered_map<ECMSpecs::Bodies, enum_unordered_map<ECMSpecs::BodyParams, std::any>> bodies_param_map;
    enum_unordered_map<ECMSpecs::Joints, enum_unordered_map<ECMSpecs::JointParams, std::any>> joints_param_map;

    enum_unordered_map<ECMSpecs::Joints, enum_unordered_map<ECMSpecs::JointParams, std::any>>::iterator itr;
    enum_unordered_map<ECMSpecs::JointParams, std::any>::iterator ptr;

    ClientPtr clientPtr;
    rigidBodyPtr getBaseLinkHandler();
    void initlizeECMParams();


};

#endif // ECM_H
