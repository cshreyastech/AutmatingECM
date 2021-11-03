#include "rbdl_model_tests/RBDLTestPrep.h"
#include "rbdl_model_tests/EigenUtilities.h"
//#include "rbdl_model_tests/Human36Fixture.h"


//const double TEST_PREC = 1.0e-12;
const double TEST_LAX = 1.0e-7;

struct KinematicsFixture {
    KinematicsFixture () {
        ClearLogOutput();

        BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
        base = rbdlModelPtr->getBaseRigidBody();


        clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
        clientPtr->connect();

        baseHandler = clientPtr->getRigidBody(base, true);
        usleep(1000000);

        EigenUtilities eigenUtilities;

        //base is rigid body name, not a joint. This is a hacky way to enable ros topics in the 
        //server side during first execution
        baseHandler->set_joint_pos(base, 0.0f); 
        tf::Vector3 P_0_w_tf = baseHandler->get_pos();
        Eigen::Vector3f P_0_w;
        P_0_w[0]= P_0_w_tf[0];
        P_0_w[1]= P_0_w_tf[1];
        P_0_w[2]= P_0_w_tf[2];
        
        tf::Vector3 R_0_w_tf = baseHandler->get_rpy();
        Eigen::Matrix3f R_0_w = eigenUtilities.rotation_from_euler(R_0_w_tf[0], R_0_w_tf[1], R_0_w_tf[2]);
        T_0_w = eigenUtilities.get_frame(R_0_w, P_0_w);

        rbdlModel = rbdlModelPtr->getRBDLModel();
        Q = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        QDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        QDDot = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);
        Tau = VectorNd::Constant ((size_t) rbdlModel->dof_count, 0.);

        ClearLogOutput();
    }

    ~KinematicsFixture () {
        delete rbdlModel;
        clientPtr->cleanUp();
    }

    Model *rbdlModel = nullptr;
    AMBFClientPtr clientPtr = nullptr;
    rigidBodyPtr baseHandler = nullptr;
    std::string base;
    Eigen::Matrix4f T_0_w;

    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
};

TEST_CASE_METHOD(KinematicsFixture, __FILE__"_TestPositionNeutral", "") 
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    ForwardDynamics(*rbdlModel, Q, QDot, Tau, QDDot);


    vector<string> joints = baseHandler->get_joint_names();

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

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_ambf;
            P_n_w_rbd_ambf.setZero();
            
            P_n_w_rbd_ambf(0) = P_n_w_tf[0];
            P_n_w_rbd_ambf(1) = P_n_w_tf[1];
            P_n_w_rbd_ambf(2) = P_n_w_tf[2];


            RigidBodyDynamics::Math::Vector3d P_n_0_rbd_rbdl = CalcBodyToBaseCoordinates(*rbdlModel, Q, rbdlBodyId, 
                                                                    RigidBodyDynamics::Math::Vector3d(0., 0., 0.),true);

            RigidBodyDynamics::Math::Vector4d P_n_w_rbd_rbdl_4d;
            P_n_w_rbd_rbdl_4d.setOnes();
            P_n_w_rbd_rbdl_4d(0) = P_n_0_rbd_rbdl(0);
            P_n_w_rbd_rbdl_4d(1) = P_n_0_rbd_rbdl(1);
            P_n_w_rbd_rbdl_4d(2) = P_n_0_rbd_rbdl(2);
            P_n_w_rbd_rbdl_4d(3) = 1.0f;

            RigidBodyDynamics::Math::MatrixNd T_0_w_rbd = RigidBodyDynamics::Math::MatrixNd(4, 4);
            RigidBodyDynamics::Math::MatrixNd T_n_w_rbd = RigidBodyDynamics::Math::MatrixNd(4, 4);


            T_0_w_rbd(0, 0) = T_0_w(0, 0);
            T_0_w_rbd(0, 1) = T_0_w(0, 1);
            T_0_w_rbd(0, 2) = T_0_w(0, 2);
            T_0_w_rbd(0, 3) = T_0_w(0, 3);

            T_0_w_rbd(1, 0) = T_0_w(1, 0);
            T_0_w_rbd(1, 1) = T_0_w(1, 1);
            T_0_w_rbd(1, 2) = T_0_w(1, 2);
            T_0_w_rbd(1, 3) = T_0_w(1, 3);

            T_0_w_rbd(2, 0) = T_0_w(2, 0);
            T_0_w_rbd(2, 1) = T_0_w(2, 1);
            T_0_w_rbd(2, 2) = T_0_w(2, 2);
            T_0_w_rbd(2, 3) = T_0_w(2, 3);

            T_0_w_rbd(3, 0) = T_0_w(3, 0);
            T_0_w_rbd(3, 1) = T_0_w(3, 1);
            T_0_w_rbd(3, 2) = T_0_w(3, 2);
            T_0_w_rbd(3, 3) = T_0_w(3, 3);

            P_n_w_rbd_rbdl_4d = T_0_w_rbd * (P_n_w_rbd_rbdl_4d);

            RigidBodyDynamics::Math::Vector3d P_n_w_rbd_rbdl;
            P_n_w_rbd_rbdl = P_n_w_rbd_rbdl_4d.block<3,1>(0,0);
               
            CHECK_THAT(P_n_w_rbd_ambf, AllCloseMatrix(P_n_w_rbd_rbdl, TEST_PREC, TEST_PREC));
        }
    }
}