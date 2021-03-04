#include "ecm_manupulation/Kinematics.h"


Kinematics::Kinematics() : ECM() {

}


void Kinematics::testParams() {
    for (itr = joints_param_map.begin(); itr != joints_param_map.end(); itr++) {
        const ECMSpecs::Joints joint = itr->first;
        std::cout << "joint: " << joint << std::endl;
//        std::cout << "joint: " << joint << enum_to_str<ECMSpecs::Joints>(joint) << std::endl; This doest work

        for (ptr = itr->second.begin(); ptr != itr->second.end(); ptr++) {
            ECMSpecs::JointParams joints_param = ptr->first;
//            std::cout << "joints_param: " << joints_param << enum_to_str<ECMSpecs::JointParams>(joints_param) << std::endl;

            if (joints_param == ECMSpecs::JointParams::dh) {
                std::vector<float> param = std::any_cast<std::vector<float>>(joints_param_map[joint][joints_param]);


                for(float val : param)
                    std::cout << val << ", ";

            } else if (joints_param == ECMSpecs::JointParams::joints_limit) {
                std::vector<float> param = std::any_cast<std::vector<float>>(joints_param_map[joint][joints_param]);
//                std::cout << "joints_param: " << joints_param << enum_to_str<ECMSpecs::JointParams>(joints_param) << std::endl;

                for(float val : param)
                    std::cout << val << ", ";
            }
            std::cout << std::endl;
        }
        std::cout << "----------------------------------" << std::endl;
    }
}

Matrix4f Kinematics::computeFK(std::vector<float> joint_pos) {
    Utilities utilities;

    DH_Vector.clear();

    int joint_pos_n = joint_pos.size();

    for(int i = 0; i < joint_pos_n; i++) {
        DH_PARAMETERS(i, 2) = joint_pos[i];

        std::vector<float> dh = std::any_cast<std::vector<float>>(joints_param_map[static_cast<ECMSpecs::Joints>(i)][ECMSpecs::JointParams::dh]);
        dh[2] = joint_pos[i];
        joints_param_map[static_cast<ECMSpecs::Joints>(i)][ECMSpecs::JointParams::dh] = dh;
    }

    if(joint_pos_n > 2) {
        DH_PARAMETERS(2, 2) = 0.0;
        DH_PARAMETERS(2, 3) = joint_pos[2];

        std::vector<float> dh = std::any_cast<std::vector<float>>(joints_param_map[static_cast<ECMSpecs::Joints>(2)][ECMSpecs::JointParams::dh]);
        dh[2] = 0;
        dh[3] = joint_pos[2];
        joints_param_map[static_cast<ECMSpecs::Joints>(2)][ECMSpecs::JointParams::dh] = dh;
    }


    for ( const ECMSpecs::Joints joint : ECMSpecs::allJoints ) {
        std::vector<float> dh = std::any_cast<std::vector<float>>(joints_param_map[joint][ECMSpecs::JointParams::dh]);
        DH_Vector.push_back(
                    new DH(dh[0], dh[1], dh[2], dh[3], dh[4], utilities.joint_type_enum_to_str((JointType)dh[5])));
        dh.clear();
    }


    T_1_0_ = DH_Vector[0]->get_trans();
    Matrix4f T_2_1 = DH_Vector[1]->get_trans();
    Matrix4f T_3_2 = DH_Vector[2]->get_trans();
    Matrix4f T_4_3 = DH_Vector[3]->get_trans();

    T_2_0_ = T_1_0_ * T_2_1;
    T_3_0_ = T_2_0_ * T_3_2;
    T_4_0_ = T_3_0_ * T_4_3;

    if(joint_pos_n == 1) return T_1_0_;
    if(joint_pos_n == 2) return T_2_0_;
    if(joint_pos_n == 3) return T_3_0_;
    if(joint_pos_n == 4) return T_4_0_;
}


std::vector<float> Kinematics::computeIK(Matrix4f T_4_0) {
    Utilities utilities;

    Matrix4f T_PinchJoint_4;
    T_PinchJoint_4 << Matrix4f::Identity();


    float j1 = std::atan2(T_4_0(0, 3), -1.0 * T_4_0(2, 3));

    float xz_diag = std::sqrt(std::pow(T_4_0(0, 3), 2) + std::pow(T_4_0(2, 3), 2));
    float j2 = -1 * std::atan2(T_4_0(1, 3), xz_diag);

    float j3 = (T_4_0.block<3, 1>(0, 3)).norm() + (L_rcc - L_scopelen);

    Matrix4f T_4_0_FK = this->computeFK(std::vector<float>{j1, j2, j3});

    Matrix3f R_IK_in_FK = (T_4_0_FK.block<3, 3>(0, 0)).inverse() * (T_4_0.block<3, 3>(0, 0));

    float j4 = utilities.rpy_from_rotation(R_IK_in_FK)[2];
    return std::vector<float>{j1, j2, j3, j4};
}

void Kinematics::testIK(const std::vector<float> desired_q) {
//    We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
//    in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)

    Matrix4f T_4_0 = this->computeFK(desired_q);
    std::cout << "T_4_0 " << std::endl << T_4_0 << std::endl;

    std::vector<float> computed_q = this->computeIK(T_4_0);


    std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << std::endl;
    std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << std::endl;
    std::cout << "diff   : "
              << std::roundf(desired_q[0] - computed_q[0]) << ", "
              << std::roundf(desired_q[1] - computed_q[1]) << ", "
              << std::roundf(desired_q[2] - computed_q[2]) << ", "
              << std::roundf(desired_q[3] - computed_q[3])
              << std::endl;

    Eigen::MatrixXf jacobian = this->getJacobian(desired_q);

    std::cout << "jacobian " << std::endl << jacobian << std::endl;


}


Eigen::MatrixXf Kinematics::getJacobian(const std::vector<float> desired_q) {
    Eigen::MatrixXf jacobian(6, 4);

    //Get Angular Velocity from Transformation matrix
    jacobian.block<3,1>(0, 0) = T_1_0_.block<3,1>(0, 2);
    jacobian.block<3,1>(0, 1) = T_2_0_.block<3,1>(0, 2);
    jacobian.block<3,1>(0, 2) = Eigen::Vector3f::Zero();
    jacobian.block<3,1>(0, 3) = T_4_0_.block<3,1>(0, 2);


    //Get Linear Velocity from Transformation matrix
    float ct1 = cos(desired_q[0]);
    float st1 = sin(desired_q[0]);

    float ct2 = cos(desired_q[1]);
    float st2 = sin(desired_q[1]);

    jacobian(3, 0) = ct1 * ct2 * (L_rcc - desired_q[2]) + ct1 * ct2 * (L_scopelen + desired_q[2]);
    jacobian(4, 0) = 0.0;
    jacobian(5, 0) = st1 * ct2 * (L_rcc - desired_q[2]) + st1 * ct2 * (L_scopelen + desired_q[2]);

    jacobian(3, 1) = -st1 * st2 * (L_rcc - desired_q[2]) - st1 * st2 * (L_scopelen + desired_q[2]);
    jacobian(4, 1) = ct2 * (L_rcc - desired_q[2]) + ct2 * (L_scopelen + desired_q[2]);
    jacobian(5, 1) = - ct1 * st2 * (L_rcc - desired_q[2]) + ct1 * st2 * (L_scopelen + desired_q[2]);

    jacobian(3, 2) = 0;
    jacobian(4, 2) = 0;
    jacobian(5, 2) = - 2 * ct1 * ct2;

    jacobian.block<3,1>(3, 3) = Eigen::Vector3f::Zero();


    return jacobian;
}


//void ECM::testIK() {
//    Client client;
//    client.connect();
//    usleep(20000);


//    vector<string> object_names = client.getRigidBodyNames();

//    std::cout << "object_names" <<std::endl;
//    for(std::string object_name : object_names)
//        std::cout << object_name << ", ";
//    std::cout << std::endl;

//    rigidBodyPtr b = client.getRigidBody("ecm/baselink", true);
//    rigidBodyPtr target_fk_handler = client.getRigidBody("ecm/target_fk", true);
//    rigidBodyPtr target_ik_handler = client.getRigidBody("ecm/target_ik", true);
//    usleep(1000000);

//    Utilities utilities;


//    Vector3f P_0_w;
//    P_0_w[0]= b->get_pos()[0];
//    P_0_w[1]= b->get_pos()[1];
//    P_0_w[2]= b->get_pos()[2];

//    Eigen::Matrix3f R_0_w = utilities.rotation_from_euler(b->get_rpy()[0], b->get_rpy()[1], b->get_rpy()[2]);

//    Eigen::Matrix4f T_0_w = utilities.get_frame(R_0_w, P_0_w);

//    int n_poses = 5;
//    for(int i = 0; i < n_poses; i++) {
//        std::vector<float> desired_q;

//        for(std::vector<float> joint_limit : ECM_JOINT_LIMITS_) {
//            float low = joint_limit[0];
//            float high = joint_limit[1];
//            desired_q.emplace_back(utilities.get_random_between_range(low, high));
//        }

//        Matrix4f T_4_0 = this->computeFK(desired_q);


//        if(target_ik_handler) {
//            Eigen::Matrix4f T_4_w = T_0_w * T_4_0;
//            target_ik_handler->set_pos(T_4_w(0, 3), T_4_w(1, 3), T_4_w(2, 3));

//            Eigen::Vector3f r_4_w_rpy = utilities.rpy_from_rotation(T_4_w.block<3,3>(0,0));
//            target_ik_handler->set_rpy(r_4_w_rpy[0], r_4_w_rpy[1], r_4_w_rpy[2]);
//        }

//        std::vector<float> computed_q = this->computeIK(T_4_0);


//        if(target_fk_handler) {
//            Eigen::Matrix4f T_4_0_fk = this->computeFK(computed_q);

//            Eigen::Matrix4f T_4_w_fk = T_0_w * T_4_0_fk;
//            target_fk_handler->set_pos(T_4_w_fk(0, 3), T_4_w_fk(1, 3), T_4_w_fk(2, 3));

//            Eigen::Vector3f r_4_w_rpy_fk = utilities.rpy_from_rotation(T_4_w_fk.block<3,3>(0,0));
//            target_fk_handler->set_rpy(r_4_w_rpy_fk[0], r_4_w_rpy_fk[1], r_4_w_rpy_fk[2]);
//        }

//        b->set_joint_pos<std::string>(              "baselink-yawlink", computed_q[0]);
//        b->set_joint_pos<std::string>(         "yawlink-pitchbacklink", computed_q[1]);
//        b->set_joint_pos<std::string>("pitchendlink-maininsertionlink", computed_q[2]);
//        b->set_joint_pos<std::string>(    "maininsertionlink-toollink", computed_q[3]);

//        std::cout << "desired: " <<  desired_q[0] << ", " <<  desired_q[1] << ", " <<  desired_q[2] << ", " <<  desired_q[3] << std::endl;
//        std::cout << "cal    : " << computed_q[0] << ", " << computed_q[1] << ", " << computed_q[2] << ", " << computed_q[3] << std::endl;
//        std::cout << "diff   : "
//                  << std::roundf(desired_q[0] - computed_q[0]) << ", "
//                  << std::roundf(desired_q[1] - computed_q[1]) << ", "
//                  << std::roundf(desired_q[2] - computed_q[2]) << ", "
//                  << std::roundf(desired_q[3] - computed_q[3]) << ", "
//                  << std::endl;

//        usleep(1000000);
//    }
//}

void Kinematics::cleanup() {
}

Kinematics::~Kinematics(void){
    cleanup();
}
