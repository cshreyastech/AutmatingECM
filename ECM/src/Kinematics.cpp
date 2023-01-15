#include "ECM/Kinematics.h"

Kinematics::Kinematics(AMBFWrapper* ambfWrapper) : ambfWrapper_(ambfWrapper)
{

}

const std::string joint_type_enum_to_str(JointType enumVal)
{
  if (enumVal == JointType::ROTATIONAL) return "ROTATIONAL";
  else if (enumVal == JointType::PRISMATIC) return "PRISMATIC";
  return "UnsupportJoint";
}

Matrix4f Kinematics::ComputeFK(std::vector<float> joint_pos) {
  DH_Vector_.clear();

  int joint_pos_n = joint_pos.size();
  for(int i = 0; i < joint_pos_n; i++)
    dh_params_[i][2] = joint_pos[i];

  if(joint_pos_n > 2) {
    dh_params_[2][2] = 0.0;
    dh_params_[2][3] = joint_pos[2];
  }

  int row_n = sizeof(dh_params_) / sizeof(dh_params_[0]);
  for(int row = 0; row < row_n; row++) {
    DH_Vector_.push_back(new DH(dh_params_[row][0], dh_params_[row][1], dh_params_[row][2], dh_params_[row][3], dh_params_[row][4], joint_type_enum_to_str((JointType)dh_params_[row][5])));
  }

  Matrix4f T_1_0 = DH_Vector_[0]->get_trans();
  Matrix4f T_2_1 = DH_Vector_[1]->get_trans();
  Matrix4f T_3_2 = DH_Vector_[2]->get_trans();
  Matrix4f T_4_3 = DH_Vector_[3]->get_trans();

  Matrix4f T_2_0 = T_1_0 * T_2_1;
  Matrix4f T_3_0 = T_2_0 * T_3_2;
  Matrix4f T_4_0 = T_3_0 * T_4_3;

  if(joint_pos_n == 1) return T_1_0;
  if(joint_pos_n == 2) return T_2_0;
  if(joint_pos_n == 3) return T_3_0;
  if(joint_pos_n == 4) return T_4_0;

  return Matrix4f::Zero();
}


std::vector<float> Kinematics::ComputeIK(Matrix4f T_4_0) {
  Utilities utilities;

  Matrix4f T_PinchJoint_4;
  T_PinchJoint_4 << Matrix4f::Identity();


  float j1 = std::atan2(T_4_0(0, 3), -1.0 * T_4_0(2, 3));

  float xz_diag = std::sqrt(std::pow(T_4_0(0, 3), 2) + std::pow(T_4_0(2, 3), 2));
  float j2 = -1 * std::atan2(T_4_0(1, 3), xz_diag);

  float j3 = (T_4_0.block<3, 1>(0, 3)).norm() + (L_rcc_ - L_scopelen_);

  Matrix4f T_4_0_FK = this->ComputeFK(std::vector<float>{j1, j2, j3});

  Matrix3f R_IK_in_FK = (T_4_0_FK.block<3, 3>(0, 0)).inverse() * (T_4_0.block<3, 3>(0, 0));

  float j4 = utilities.RPYfromRotation(R_IK_in_FK)[2];
  return std::vector<float>{j1, j2, j3, j4};
}

void Kinematics::TestIK() 
{

  std::vector<std::string> objectNames = ambfWrapper_->ObjectNames();

  // std::cout << "object_names" <<std::endl;
  // for(std::string objectName : objectNames)
  //   std::cout << objectName << ", ";
  // std::cout << std::endl;

  Vector3f P_0_w = ambfWrapper_->P_N_W(RigidBodyID::BASELINK);
  Vector3f R_0_w_rpy = ambfWrapper_->R_N_W_rpy(RigidBodyID::BASELINK);

  Utilities utilities;
  Matrix3f R_0_w = utilities.RotationFromEuler(R_0_w_rpy);

  Eigen::Matrix4f T_0_w = utilities.GetFrame(R_0_w, P_0_w);

  int n_poses = 20;
  for(int i = 0; i < n_poses; i++) 
  {
    std::vector<float> desiredQs;

    for(std::vector<float> jointLimit : ECM_JOINT_LIMITS_) {
      float low = jointLimit[0];
      float high = jointLimit[1];
      desiredQs.emplace_back(utilities.GetRandomBetweenRange(low, high));
    }

    Matrix4f T_4_0 = this->ComputeFK(desiredQs);

    if(ambfWrapper_->IsActive(RigidBodyID::TARGETIK)) 
    {
      Matrix4f T_4_w = T_0_w * T_4_0;
      
      ambfWrapper_->P_N_W(RigidBodyID::TARGETIK, T_4_w.block<3,1>(0,3));

      Vector3f R_4_w_rpy = utilities.RPYfromRotation(T_4_w.block<3,3>(0,0));
      ambfWrapper_->R_N_W_rpy(RigidBodyID::TARGETIK, R_4_w_rpy.block<3,1>(0,3));
    }

    std::vector<float> computedQs = this->ComputeIK(T_4_0);

    // std::cout << "desired: " <<  desiredQ[0] << ", " <<  desiredQ[1] << ", " <<  
    //                              desiredQ[2] << ", " <<  desiredQ[3] << std::endl;

    // std::cout << "cal    : " << computedQ[0] << ", " << computedQ[1] << ", " << 
    //                             computedQ[2] << ", " << computedQ[3] << std::endl;
    // std::cout << "diff   : "
    //           << std::roundf(desiredQ[0] - computedQ[0]) << ", "
    //           << std::roundf(desiredQ[1] - computedQ[1]) << ", "
    //           << std::roundf(desiredQ[2] - computedQ[2]) << ", "
    //           << std::roundf(desiredQ[3] - computedQ[3]) << ", "
    //           << std::endl;

    if(ambfWrapper_->IsActive(RigidBodyID::TARGETFK))
    {
      Matrix4f T_4_0_fk = ComputeFK(computedQs);

      Eigen::Matrix4f T_4_w_fk = T_0_w * T_4_0_fk;
      ambfWrapper_->P_N_W(RigidBodyID::TARGETFK, T_4_w_fk.block<3,1>(0,3));
      
      Vector3f R_4_w_rpy_fk = utilities.RPYfromRotation(T_4_w_fk.block<3,3>(0,0));
      ambfWrapper_->R_N_W_rpy(RigidBodyID::TARGETFK, R_4_w_rpy_fk.block<3,1>(0,3));
    }

    ambfWrapper_->JointPose(RigidBodyID::BASELINK, computedQs);
    usleep(250000);
  }
}

Kinematics::~Kinematics()
{
  std::cout << "~Kinematics()\n";
  // delete [] dh_params_;
}