#include "rbdl_model_tests/Kuka.h"


TEST_CASE_METHOD(Kuka, __FILE__"_TestPositionNeutral", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

    vector<string> joints = baseHandler->get_joint_names();
    std::vector<std::string> baseChildren = baseHandler->get_children_names();

    CHECK (baseChildren.size() == Q.size());

    std::map< std::string, unsigned int > mBodyNameMap = rbdlModel->mBodyNameMap;
    std::map<std::string, unsigned int>::iterator mBodyNameMapItr;
    for(mBodyNameMapItr = mBodyNameMap.begin(); mBodyNameMapItr != mBodyNameMap.end(); mBodyNameMapItr++)
    {
      std::cout << mBodyNameMapItr->first << ", " << mBodyNameMapItr->second << std::endl;
    }


    // for(int i = 0; i < 20; i++)
    // {
    //     for(std::string joint : joints)
    //     {
    //         baseHandler->set_joint_pos<std::string>(joint, 0.0f);
    //     }

    //     usleep(250000);
    // }

    // Q.setZero();

    // for(std::string body : baseChildren)
    // {
    //     unsigned int rbdlBodyId = rbdlModel->GetBodyId(body.c_str());

    //     if(rbdlBodyId < rbdlModel->mBodies.size())
    //     {
    //         rigidBodyPtr rigidBodyHandler = clientPtr->getRigidBody(body, true);
            
    //         // n - respective body frame
    //         const tf::Vector3 P_n_w_tf = rigidBodyHandler->get_pos();

    //         const RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
    //                                                                 RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

    //         RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
    //         P_n_w_rbd_ambf.setZero();
            
    //         P_n_w_rbd_ambf(0) = P_n_w_tf[0];
    //         P_n_w_rbd_ambf(1) = P_n_w_tf[1];
    //         P_n_w_rbd_ambf(2) = P_n_w_tf[2];

    //         RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
    //         P_n_w_rbd_rbdl_4d.setOnes();
    //         P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
    //         P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
    //         P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);

    //         P_n_w_rbd_rbdl_4d = T_0_w * (P_n_w_rbd_rbdl_4d);

    //         RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
    //         P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
               
    //         CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
    //     }
    // }
}

/*
TEST_CASE_METHOD(Kuka, __FILE__"_TestPositionBaseRotated90Deg", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);

    vector<string> joints = baseHandler->get_joint_names();
    std::vector<std::string> baseChildren = baseHandler->get_children_names();

    CHECK (baseChildren.size() == Q.size());
    std::string baseJoint = joints.at(0);

    for(int i = 0; i < 20; i++)
    {
        baseHandler->set_joint_pos<std::string>(baseJoint, 0.5f);
        for(int i = 1; i < joints.size(); i++)
        {
            std::string joint = joints.at(i);
            baseHandler->set_joint_pos<std::string>(joint, 0.0f);
        }

        usleep(250000);
    }

    Q[0] = 0.5 * M_PI;

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
*/