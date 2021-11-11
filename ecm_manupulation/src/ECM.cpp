#include "ecm_manupulation/ECM.h"

ECM::ECM() {
    clientPtr = AMBFClient::getInstance();
    clientPtr->connect();
    usleep(20000);

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

    return "";
}

const std::string ECM::ecm_joints_enum_to_str(ECMSpecs::Joints enumVal)
{
    if (enumVal == ECMSpecs::Joints::baselink_yawlink) return "baselink_yawlink";
    else if (enumVal == ECMSpecs::Joints::yawlink_pitchbacklink) return "yawlink_pitchbacklink";
    else if (enumVal == ECMSpecs::Joints::pitchendlink_maininsertionlink) return "pitchendlink_maininsertionlink";
    else if (enumVal == ECMSpecs::Joints::maininsertionlink_toollink) return "maininsertionlink_toollink";

    return "";
}

const std::string ECM::ecm_joint_params_enum_to_str(ECMSpecs::JointParams enumVal)
{
    if (enumVal == ECMSpecs::JointParams::dh) return "dh";
    else if (enumVal == ECMSpecs::JointParams::joints_limit) return "joints_limit";

    return "";
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
    std::vector<std::vector<std::vector<float>>> body_params;
    std::vector<float> body_mass = {};
    std::vector<Eigen::Vector3f> body_inertia = {};

//    rigidBodyPtr yawlink_handler = clientPtr->getRigidBody("ecm/yawlink", true);
//    usleep(1000000);
//    yawlink_handler->set_joint_pos(0, 0.0);
//    std::vector<std::string> children = yawlink_handler->get_children_names();


//    baselink_handler->set_joint_pos<std::string>("baselink-yawlink", 0.0);


    rigidBodyPtr baselink_handler = this->getBaseLinkHandler();

    std::vector<std::string> children = baselink_handler->get_children_names();
//    rigidBodyPtr child_handler = clientPtr->getRigidBody("yawlink", true);
//    usleep(1000000);
//    std::cout << "mass: " << child_handler->get_mass() << std::endl;

    for(const string child : children) {
        std::cout << "child: " << child << std::endl;
//        float mass = 0.0;
        rigidBodyPtr child_handler = clientPtr->getRigidBody(child, true);

        std::cout << "mass: " << child_handler->get_mass() << std::endl;

        std::cout << "inertia: " << child_handler->get_inertia() << std::endl;

////        bodies_param_map.insert(make_pair(child, enum_unordered_map<ECMSpecs::BodyParams, std::any>()));

////        for ( const ECMSpecs::BodyParams body_param : ECMSpecs::allBodyParams ) {

//////            ECMSpecs::JointParams joints_param = ptr->first;
////            if (body_param == ECMSpecs::BodyParams::mass) {
////                std::vector<float> param = std::any_cast<float>(joints_param_map[joint][joints_param]);


////                for(float val : param)
////                    std::cout << val << ", ";

////            } else if (joints_param == ECMSpecs::JointParams::joints_limit) {
////                std::vector<float> param = std::any_cast<std::vector<float>>(joints_param_map[joint][joints_param]);
////                for(float val : param)
////                    std::cout << val << ", ";
////            }
////            bodies_param_map[body].insert(make_pair(body_param, body_params[body_param][body]));
////        }
        child_handler = nullptr;
    }

//    for( const ECMSpecs::Bodies body : ECMSpecs::allBodies ) {
//        std::cout << "body: " << enum_to_str(body) << std::endl;
//        body_mass.emplace
//    }

//    for( const ECMSpecs::Bodies body : ECMSpecs::allBodies ) {
//        bodies_param_map.insert(make_pair(body, enum_unordered_map<ECMSpecs::BodyParams, std::any>()));

//    }

    for ( const ECMSpecs::Joints joint : ECMSpecs::allJoints ) {
        joints_param_map.insert(make_pair(joint, enum_unordered_map<ECMSpecs::JointParams, std::any>()));

        for ( const ECMSpecs::JointParams joint_param : ECMSpecs::allJointParams ) {
            joints_param_map[joint].insert(make_pair(joint_param, joint_params[joint_param][joint]));
        }
    }


}

rigidBodyPtr ECM::getBaseLinkHandler() {
    rigidBodyPtr baselink_handler = clientPtr->getRigidBody("ecm/baselink", true);
    usleep(1000000);
    baselink_handler->set_joint_pos<std::string>("baselink-yawlink", 0.0);

    return baselink_handler;
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
    if (std::is_same<TT, ECMSpecs::Bodies>::value) return this->getECMBodies(static_cast<ECMSpecs::Bodies>(t));
    else if (std::is_same<TT, ECMSpecs::BodyParams>::value) return getECMBodyParams(static_cast<ECMSpecs::BodyParams>(t));
    else if (std::is_same<TT, ECMSpecs::Joints>::value) return this->getECMJoints(static_cast<ECMSpecs::Joints>(t));
    else if (std::is_same<TT, ECMSpecs::JointParams>::value) return this->getECMJointParams(static_cast<ECMSpecs::JointParams>(t));
}

void ECM::cleanup() {
    for(DH *dh : DH_Vector)
        dh->~DH();
}

ECM::~ECM(void){
    cleanup();
    AMBFClient::cleanUp(clientPtr);
}
