#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
#include <unordered_map>

//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

struct KinematicsFixture {
  KinematicsFixture () {
    ClearLogOutput();
        clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
        clientPtr->connect();


        baseHandler = clientPtr->getRigidBody(base_name, true);
        usleep(1000000);

        //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
        //server side during first execution
        baseHandler->set_joint_pos(base_name, 0.0f); 

        tf::Vector3 P_0_w_tf = baseHandler->get_pos();
        Eigen::Vector3d P_0_w;
        P_0_w[0]= P_0_w_tf[0];
        P_0_w[1]= P_0_w_tf[1];
        P_0_w[2]= P_0_w_tf[2];
        
        tf::Vector3 R_0_w_tf = baseHandler->get_rpy();

        Eigen::Matrix3d R_0_w = eigenUtilities.rotation_from_euler<Eigen::Matrix3d>(R_0_w_tf[0], R_0_w_tf[1], R_0_w_tf[2]);

        T_0_w = eigenUtilities.get_frame<Eigen::Matrix3d, Eigen::Vector3d, Eigen::Matrix4d>(R_0_w, P_0_w);


    rbdlModel = new Model;

    base = Body(1., RigidBodyDynamics::Math::Vector3d (0.001, 0., 0.06), 
           RigidBodyDynamics::Math::Vector3d (0., 0., 0.));
    ROOT_base = Joint( SpatialVector (0., 0., 0., 0., 0., 0.));

    ROOT_baseST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    ROOT_baseST.r = RigidBodyDynamics::Math::Vector3dZero;
    base_id = rbdlModel->AddBody(0, ROOT_baseST, JointTypeFixed, base, "base");


    link1 = Body (1., RigidBodyDynamics::Math::Vector3d (0., -0.017, 0.134), 
          RigidBodyDynamics::Math::Vector3d (0.0452, 0.0446, 0.0041));
    base_link1 = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    base_link1ST.E = RigidBodyDynamics::Math::Matrix3dIdentity;
    base_link1ST.r = RigidBodyDynamics::Math::Vector3d(0., 0.0, 0.103);
    link1_id = rbdlModel->AddBody(base_id, base_link1ST, JointTypeRevoluteZ, link1, "link1");


    link2 = Body (1., RigidBodyDynamics::Math::Vector3d (0., -0.074, 0.009), 
                  RigidBodyDynamics::Math::Vector3d (0.0227, 0.0037, 0.0224));
    link1_link2 = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    link1_link2ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link1_link2ST.E(0, 0) = 1.0;
    link1_link2ST.E(1, 2) = -1.0;
    link1_link2ST.E(2, 1) = 1.0;
    link1_link2ST.r = RigidBodyDynamics::Math::Vector3d(0., 0.013, 0.209);

    link2_id = rbdlModel->AddBody(link1_id, link1_link2ST, JointTypeRevoluteZ, link2, "link2");


    link3 = Body (1., RigidBodyDynamics::Math::Vector3d (0., 0.004, -0.794), 
                  RigidBodyDynamics::Math::Vector3d (0.0417, 0.0418, 0.0038));
    link2_link3 = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    link2_link3ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link2_link3ST.E(0, 0) = 1.0;
    link2_link3ST.E(1, 2) = 1.0;
    link2_link3ST.E(2, 1) = -1.0;
    link2_link3ST.r = RigidBodyDynamics::Math::Vector3d(0., -0.194, -0.009);
    link3_id = rbdlModel->AddBody(link2_id, link2_link3ST, JointTypeRevoluteZ, link3, "link3");


    link4 = Body (1., RigidBodyDynamics::Math::Vector3d (-0.001, 0.081, 0.008), 
                  RigidBodyDynamics::Math::Vector3d (0.0249, 0.0036, 0.0247));
    link3_link4 = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    link3_link4ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link3_link4ST.E(0, 0) = 1.0;
    link3_link4ST.E(1, 2) = 1.0;
    link3_link4ST.E(2, 1) = -1.0;
    link3_link4ST.r = RigidBodyDynamics::Math::Vector3d(0., -0.013, 0.202);
    link4_id = rbdlModel->AddBody(link3_id, link3_link4ST, JointTypeRevoluteZ, link4, "link4");

    link5 = Body (1., RigidBodyDynamics::Math::Vector3d (-0.002, -0.001, -0.39), 
                  RigidBodyDynamics::Math::Vector3d (0.0363, 0.035, 0.0045));
    link4_link5 = Joint ( RigidBodyDynamics::Math::SpatialVector (0., 0., 1., 0., 0., 0.));

    link4_link5ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link4_link5ST.E(0, 0) = 1.0;
    link4_link5ST.E(1, 2) = -1.0;
    link4_link5ST.E(2, 1) = 1.0;
    link4_link5ST.r = RigidBodyDynamics::Math::Vector3d(-0.002, 0.202, -0.008);
    link5_id = rbdlModel->AddBody(link4_id, link4_link5ST, JointTypeRevoluteZ, link5, "link5");

    link6 = Body (1., RigidBodyDynamics::Math::Vector3d (0., 0.007, 0.068), 
              RigidBodyDynamics::Math::Vector3d (0.0114, 0.0116, 0.0037));
    link5_link6 = Joint ( RigidBodyDynamics::Math::SpatialVector (0., 0., 1., 0., 0., 0.));

    link5_link6ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link5_link6ST.E(0, 0) = 1.0;
    link5_link6ST.E(1, 2) = -1.0;
    link5_link6ST.E(2, 1) = 1.0;
    link5_link6ST.r = RigidBodyDynamics::Math::Vector3d(0.002, -0.052, 0.204);
    link6_id = rbdlModel->AddBody(link5_id, link5_link6ST, JointTypeRevoluteZ, link6, "link6");

    link7 = Body (1., RigidBodyDynamics::Math::Vector3d (0.006, 0., 0.015), 
              RigidBodyDynamics::Math::Vector3d (0.0012, 0.0013, 0.001));
    link6_link7 = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

    link6_link7ST.E = RigidBodyDynamics::Math::Matrix3dZero;
    link6_link7ST.E(0, 0) = 1.0;
    link6_link7ST.E(1, 2) = 1.0;
    link6_link7ST.E(2, 1) = -1.0;
    link6_link7ST.r = RigidBodyDynamics::Math::Vector3d(-0.003, -0.05, 0.053);
    link6_id = rbdlModel->AddBody(link6_id, link6_link7ST, JointTypeRevoluteZ, link7, "link7");

    Q = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
    Tau = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

    KUKA_JOINT_LIMITS[ "link1" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link2" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link3" ] = { -2.094f, 2.094f };
    KUKA_JOINT_LIMITS[ "link4" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link5" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link6" ] = { -2.094f, 2.094f }; 
    KUKA_JOINT_LIMITS[ "link7" ] = { -3.054f, 3.054f }; 

    ClearLogOutput();
  }

  ~KinematicsFixture () {
    delete rbdlModel;
    clientPtr->cleanUp();
  }

  Model *rbdlModel = nullptr;
  AMBFClientPtr clientPtr = nullptr;
  rigidBodyPtr baseHandler = nullptr;
  std::string base_name = "base";
  Eigen::Matrix4d T_0_w;
  // std::vector<std::vector<float>> KUKA_JOINT_LIMITS;
  std::unordered_map<std::string, std::vector<float>> KUKA_JOINT_LIMITS;
  std::unordered_map<std::string, std::vector<float>>::iterator KUKA_JOINT_LIMITS_itr;

  EigenUtilities eigenUtilities;

  unsigned int base_id, link1_id, link2_id, link3_id, link4_id, link5_id, link6_id, link7_id;
  Body base, link1, link2, link3, link4, link5, link6, link7;
  Joint ROOT_base, base_link1, link1_link2, link2_link3, link3_link4, link4_link5, link5_link6, link6_link7;
  SpatialTransform ROOT_baseST, base_link1ST, link1_link2ST, link2_link3ST, link3_link4ST, 
                   link4_link5ST, link5_link6ST, link6_link7ST;

  VectorNd Q;
  VectorNd QDot;
  VectorNd QDDot;
  VectorNd Tau;
};


TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestKUKAPositionNeutral", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

    std::vector<string> joints = baseHandler->get_joint_names();
    std::vector<std::string> baseChildren = baseHandler->get_children_names();

    CHECK (baseChildren.size() == Q.size());

    for(int i = 0; i < 20; i++)
    {
        for(std::string joint : joints)
        {
            baseHandler->set_joint_pos<std::string>(joint, 0.0f);
        }

        usleep(250000);
    }

    Q.setZero();

    for(std::string body : baseChildren)
    {
        unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());

        if(rbdlBodyId < rbdlModel->mBodies.size())
        {
            rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
            
            // n - respective body frame
            const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

            const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                                    RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
            P_n_w_rbd_ambf.setZero();
            
            P_n_w_rbd_ambf(0) = P_n_w_tf[0];
            P_n_w_rbd_ambf(1) = P_n_w_tf[1];
            P_n_w_rbd_ambf(2) = P_n_w_tf[2];

            RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
            P_n_w_rbd_rbdl_4d.setOnes();
            P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
            P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
            P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

            P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
            P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
               
            CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
        }
    }
}

TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestKUKAPIbyFourPositionNeutral", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

    std::vector<string> joints = baseHandler->get_joint_names();
    std::vector<std::string> baseChildren = baseHandler->get_children_names();

    CHECK (baseChildren.size() == Q.size());

    for(int i = 0; i < 20; i++)
    {
        for(std::string joint : joints)
        {
            baseHandler->set_joint_pos<std::string>(joint, M_PI / 4.0f);
        }

        usleep(250000);
    }

    Q.setZero();
    Q[0] = M_PI / 4.0;
    Q[1] = M_PI / 4.0;
    Q[2] = M_PI / 4.0;
    Q[3] = M_PI / 4.0;
    Q[4] = M_PI / 4.0;
    Q[5] = M_PI / 4.0;
    Q[6] = M_PI / 4.0;

    for(std::string body : baseChildren)
    {
        unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());

        if(rbdlBodyId < rbdlModel->mBodies.size())
        {
            rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
            
            // n - respective body frame
            const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

            const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                                    RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
            P_n_w_rbd_ambf.setZero();
            
            P_n_w_rbd_ambf(0) = P_n_w_tf[0];
            P_n_w_rbd_ambf(1) = P_n_w_tf[1];
            P_n_w_rbd_ambf(2) = P_n_w_tf[2];

            RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
            P_n_w_rbd_rbdl_4d.setOnes();
            P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
            P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
            P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

            P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
            P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
            
            CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
        }
    }
}


TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestKUKARandomPosition", "") 
{
  // We call ForwardDynamics() as it updates the spatial transformation
  // matrices
  ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

  std::vector<string> joints = baseHandler->get_joint_names();


  std::vector<std::string> baseChildren = baseHandler->get_children_names();

  // std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
  // std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
  // for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
  // {
  //   std::cout << mBodyNameMapItr->first << ", " << mBodyNameMapItr->second << std::endl;
  // }

  // This will not be true for ECM
  const int nJoints = baseChildren.size();
  CHECK (nJoints == Q.size());

  float q_desired[nJoints];
  std::unordered_map<std::string, tf::Vector3> P_n_w_rbd_ambf_map;

  for(int i = 0; i < 5; i++)
  {
    P_n_w_rbd_ambf_map.clear();

    for( std::string body : baseChildren)
    {
      std::vector<float> joint_limit = KUKA_JOINT_LIMITS[body];
      float low = joint_limit.at(0);
      float high = joint_limit.at(1);

      unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());
      // body_id starts with 1 in RBDL Model. 
      // Storing corresponding Joint angles with 0 order index.
      q_desired[--rbdlBodyId] = eigenUtilities.get_random_between_range(low, high);
    }

    

    for(int j = 0; j < 10; j++)
    {
      for(std::string body : baseChildren)
      {
        unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());
        unsigned int rbdlBodyIdZeroIndex = rbdlBodyId - 1;
        
        baseHandler->set_joint_pos<std::string>(joints.at(rbdlBodyIdZeroIndex), 
                      q_desired[rbdlBodyIdZeroIndex]);

        rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
        usleep(250000);
        P_n_w_rbd_ambf_map[body] = rigidBodyHandler->get_pos();
      }
    }
    // Set joint angles for both ambf and RBDL model
    Q.setZero();
    for(std::string body : baseChildren)
    {
      unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());
      unsigned int rbdlBodyIdZeroIndex = rbdlBodyId - 1;
      Q[rbdlBodyIdZeroIndex] = q_desired[rbdlBodyIdZeroIndex];

      std::cout << "body: " << body << ", q_desired: " << Q[rbdlBodyIdZeroIndex] << std::endl;
    }

    // Compare AMBF VS RBDL FK
    for(std::string body : baseChildren)
    {
      unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());
      if(rbdlBodyId < rbdlModel->mBodies.size())
      {
        const tf::Vector3 P_n_w_tf = P_n_w_rbd_ambf_map[body];
        const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                                RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

        RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
        P_n_w_rbd_ambf.setZero();
        
        P_n_w_rbd_ambf(0) = P_n_w_tf[0];
        P_n_w_rbd_ambf(1) = P_n_w_tf[1];
        P_n_w_rbd_ambf(2) = P_n_w_tf[2];

        RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
        P_n_w_rbd_rbdl_4d.setOnes();
        P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
        P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
        P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

        P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

        RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
        P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
        
        CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
      }
    }
    std::cout << "------- end of test: " << i << " ---------" << std::endl;
  }
}

