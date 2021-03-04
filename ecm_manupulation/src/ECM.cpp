#include "ecm_manupulation/ECM.h"

ECM::ECM() {
    this->initlizeECMParams();
}

const std::string ECM::ecm_bodies_enum_to_str(ECMSpecs::Bodies enumVal)
{
    if (enumVal == ECMSpecs::Bodies::baselink) return "baselink";
    else if (enumVal == ECMSpecs::Bodies::yawlink) return "yawlink";
    else if (enumVal == ECMSpecs::Bodies::pitchbacklink) return "pitchbacklink";
    else if (enumVal == ECMSpecs::Bodies::maininsertionlink) return "maininsertionlink";
    else if (enumVal == ECMSpecs::Bodies::toollink) return "toollink";
}

const std::string ECM::ecm_body_params_enum_to_str(ECMSpecs::BodyParams enumVal)
{
    if (enumVal == ECMSpecs::BodyParams::mass) return "mass";
    else if (enumVal == ECMSpecs::BodyParams::inertia) return "inertia";
}

const std::string ECM::ecm_joints_enum_to_str(ECMSpecs::Joints enumVal)
{
    if (enumVal == ECMSpecs::Joints::baselink_yawlink) return "baselink_yawlink";
    else if (enumVal == ECMSpecs::Joints::yawlink_pitchbacklink) return "yawlink_pitchbacklink";
    else if (enumVal == ECMSpecs::Joints::pitchendlink_maininsertionlink) return "pitchendlink_maininsertionlink";
    else if (enumVal == ECMSpecs::Joints::maininsertionlink_toollink) return "maininsertionlink_toollink";
}

const std::string ECM::ecm_joint_params_enum_to_str(ECMSpecs::JointParams enumVal)
{
    if (enumVal == ECMSpecs::JointParams::dh) return "dh";
    else if (enumVal == ECMSpecs::JointParams::joints_limit) return "joints_limit";
}

void ECM::initlizeECMParams() {

    std::vector<std::vector<std::vector<float>>> joint_params = {
        //     alpha,   a,      theta,    d,            offset,     joint_type
        {
            { M_PI_2,   0.0,    0.0,    0.0,            M_PI_2,    0 },
            { -M_PI_2,  0.0,    0.0,    0.0,           -M_PI_2,    0 },
            { M_PI_2,   0.0,    0.0,    0.0,            -L_rcc,    1 },
            { 0.0,      0.0,    0.0,    L_scopelen,        0.0,    0 }
        },

        {
            { (float) (-91.96 * (M_PI / 180.0)) , (float) (91.96 * (M_PI / 180.0))  },
            { (float) (-60.00 * (M_PI / 180.0)) , (float) (60.00 * (M_PI / 180.0))  },
            {                                0.0, 0.24                              },
            { (float) (-175.00 * (M_PI / 180.0)), (float) (175.00 * (M_PI / 180.0)) }
        }
    };

    for ( const ECMSpecs::Joints joint : ECMSpecs::allJoints ) {
        joints_param_map.insert(make_pair(joint, enum_unordered_map<ECMSpecs::JointParams, std::any>()));

        for ( const ECMSpecs::JointParams joint_param : ECMSpecs::allJointParams ) {
            joints_param_map[joint].insert(make_pair(joint_param, joint_params[joint_param][joint]));
        }
    }

}

std::string ECM::getECMBodies(const ECMSpecs::Bodies e) {
    return ecm_bodies_enum_to_str(e);
}

std::string ECM::getECMBodyParams(const ECMSpecs::BodyParams e) {
    return ecm_body_params_enum_to_str(e);
}

const std::string ECM::getECMJoints(const ECMSpecs::Joints e) {
    return this->ecm_joints_enum_to_str(e);
}

const std::string ECM::getECMJointParams(const ECMSpecs::JointParams e) {
    return this->ecm_joint_params_enum_to_str(e);
}

template <typename TT>
const std::string ECM::enum_to_str(TT t)
{
    if (std::is_same<TT, ECMSpecs::Bodies>::value) return "Bodies";
    else if (std::is_same<TT, ECMSpecs::BodyParams>::value) return "BodyParams";
    else if (std::is_same<TT, ECMSpecs::Joints>::value) return this->getECMJoints(static_cast<ECMSpecs::Joints>(t));
    else if (std::is_same<TT, ECMSpecs::JointParams>::value) return this->getECMJointParams(static_cast<ECMSpecs::JointParams>(t));
}

void ECM::cleanup() {
    for(DH *dh : DH_Vector)
        dh->~DH();
}

ECM::~ECM(void){
    cleanup();
}
