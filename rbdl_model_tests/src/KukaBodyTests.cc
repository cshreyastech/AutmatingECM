
#include "rbdl_model_tests/RBDLTestPrep.h"

TEST_CASE (__FILE__"_TestKukaBodyMass", "" )
{
    ClientPtr clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
    clientPtr->connect();

    BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
    std::string baseRigidBody = rbdlModelPtr->getBaseRigidBody();
    struct RigidBodyDynamics::Model rbdlModel = rbdlModelPtr->getRBDLModel();

    vector<string> rigidBodyNamesAmbf = clientPtr->getRigidBodyNames();

    for(std::string bodyAMBF : rigidBodyNamesAmbf)
    {
        unsigned int rbdlBodyId = rbdlModel.GetBodyId(bodyAMBF.c_str());

        if(rbdlBodyId && rbdlBodyId <rbdlModel.mBodies.size())
        {
            rigidBodyPtr rigidBodyHandlerAmbf = clientPtr->getRigidBody(bodyAMBF, true);
            usleep(1000000);
            double bodyMassAMBF = rigidBodyHandlerAmbf->get_mass();
            double bodyMassRBDL = rbdlModel.mBodies.at(rbdlBodyId).mMass;

            CHECK (bodyMassAMBF == bodyMassRBDL);
        }

    }
    rbdlModelPtr->~BuildRBDLModel();
    clientPtr->cleanUp();
}



// TEST ( TestBodyInertiaToBeEqual ) {
//     ClientPtr clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
//     clientPtr->connect();

//     BuildRBDLModelPtr rbdlModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
//     std::string baseRigidBody = rbdlModelPtr->getBaseRigidBody();
//     struct RigidBodyDynamics::Model rbdlModel = rbdlModelPtr->getRBDLModel();

//     vector<string> rigidBodyNamesAmbf = clientPtr->getRigidBodyNames();
//     std::cout << "rigidBodyNamesAmbf: " << std::endl;

//     for(std::string bodyAMBF : rigidBodyNamesAmbf)
//     {
//         unsigned int rbdlBodyId = rbdlModel.GetBodyId(bodyAMBF.c_str());

//         if(rbdlBodyId && rbdlBodyId <rbdlModel.mBodies.size())
//         {
//             rigidBodyPtr rigidBodyHandlerAmbf = clientPtr->getRigidBody(bodyAMBF, true);
//             usleep(1000000);
//             double bodyMassAMBF = rigidBodyHandlerAmbf->get_mass();
//             double bodyMassRBDL = rbdlModel.mBodies.at(rbdlBodyId).mMass;

//             // std::cout << bodyAMBF << ", " << rbdlBodyId << ", " << rigidBodyHandlerAmbf->get_mass() << ", " 
//             //           << rbdlModel.mBodies.at(rbdlBodyId).mMass << std::endl;

//             CHECK_EQUAL(bodyMassAMBF, bodyMassRBDL);
//         }

//     }

//     // std::cout << "ID for body: " << bodyName << ", is " << rbdlModelPtr->getBodyId(bodyName) << std::endl;
//     // boost::optional<rbdlBody> bodyBoostRbdlBody = rbdlModelPtr->getRBDLBody(bodyName);
//     // rbdlBody bodyRbdlBody =bodyBoostRbdlBody.get();

//     // const Math::Vector3d com_yaml(0.0, -0.0169, 0.1342);

//     // const Math::Matrix3d inertia_yaml (
//     // 0.0453,     0.0,    0.0,
//     // 0.0,        0.0446, 0.0,
//     // 0.0,        0.0,    0.0041
//     // );

    
//     // CHECK_CLOSE(1.0, bodyRbdlBody.mMass, TEST_PREC);
//     // CHECK_ARRAY_CLOSE(inertia_yaml.data(), bodyRbdlBody.mInertia.data(), 9, TEST_PREC);
//     // CHECK_ARRAY_CLOSE(com_yaml.data(), bodyRbdlBody.mCenterOfMass.data(), 3, TEST_PREC);
//     // CHECK_ARRAY_CLOSE(inertia_yaml.data(), bodyRbdlBody.mInertia.data(), 9, TEST_PREC);
// //    bodyParamPtr rootBodyParamPtr = buildRBDLModelPtr->getBodyParamPtr(bodyName);
// //    double mass = rootBodyParamPtr->Mass();

// //    std::cout << "mass: " << mass << std::endl;

// //    std::unordered_map<std::string, jointParamPtr> jointChildrenMap = buildRBDLModelPtr->getJointChildren(bodyName);
// //    std::unordered_map<std::string, jointParamPtr>::iterator itr;

// //    if(!jointChildrenMap.empty()) {
// //        for (itr = jointChildrenMap.begin(); itr != jointChildrenMap.end(); itr++) {
// //            std::string jointName = itr->first;

// //            std::cout << "jointName: " << jointName << std::endl;
// //        }
// //    }


//     //    tf::Quaternion rot = baselink_handler->get_rot();
//     //    std::cout << rot[0] << ", " << rot[1] << ", " << rot[2] << ", " << rot[3] << std::endl;
// //    Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

// //    Matrix3d inertia_C (
// //      1.4, 0., 0.,
// //      0., 2., 0.,
// //      0., 0., 3.);

// //    SpatialMatrix reference_inertia (
// //      4.843, -1.98, -2.145, 0, -1.43, 1.32,
// //      -1.98, 6.334, -1.716, 1.43, 0, -1.65,
// //      -2.145, -1.716, 7.059, -1.32, 1.65, 0,
// //      0, 1.43, -1.32, 1.1, 0, 0,
// //      -1.43, 0, 1.65, 0, 1.1, 0,
// //      1.32, -1.65, 0, 0, 0, 1.1
// //      );

//     //	cout << LogOutput.str() << endl;

// //    SpatialRigidBodyInertia body_rbi = SpatialRigidBodyInertia::createFromMassComInertiaC (body.mMass, body.mCenterOfMass, body.mInertia);

// //    CHECK_ARRAY_CLOSE (reference_inertia.data(), body_rbi.toMatrix().data(), 36, TEST_PREC);
// //    CHECK_ARRAY_CLOSE (inertia_C.data(), body.mInertia.data(), 9, TEST_PREC);

//     //  buildRBDLModelPtr_->cleanUp();
//     //  rbdlTestsPrep->~RbdlTestsPrep();

//     rbdlModelPtr->~BuildRBDLModel();
//     clientPtr->cleanUp();
// }


// TEST ( TestRBBLBodyToAMBF ) {
//    const string bodyName = "base";

//    BuildRBDLModelPtr buildRBDLModelPtr = RBDLTestPrep::getInstance()->getRBDLModelInstance();
//    boost::optional<rbdlBody> bodyBoostRbdlBody = buildRBDLModelPtr->getRBDLBody(bodyName);
//    rbdlBody bodyRbdlBody = bodyBoostRbdlBody.get();


// //    ClientPtr clientPtr = RBDLTestPrep::getInstance()->getAMBFClientInstance();
// //    clientPtr->connect();
// // //    usleep(20000);


// //    rigidBodyPtr body_handler = clientPtr->getRigidBody(bodyName, true);
// //    usleep(250000);

// //    std::cout << "get_num_joints(): " << body_handler->get_num_joints() << std::endl;

// //    float body_mass = body_handler->get_mass();

//    tf::Vector3 body_inertia(0.0, 0.0, 0.0);
// //    body_inertia = body_handler->get_inertia();

// //    CHECK_CLOSE(body_mass, bodyRbdlBody.mMass, TEST_PREC);

// }
