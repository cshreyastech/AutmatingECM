using RosMessageTypes.EcmMoveit;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class ECMSourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    
    // Variables required for ROS communication
    public string topicName = "SourceDestination_input";

    public GameObject ecm;
    public GameObject target;
    public GameObject targetPlacement;
    
    private int numRobotJoints = 4;
    private readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);
    
    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    
    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        
        //jointArticulationBodies = new ArticulationBody[numRobotJoints];
        //string baselink_yawlink = "baselink/baselink_yawlink";
        //jointArticulationBodies[0] = ecm.transform.Find(baselink_yawlink).GetComponent<ArticulationBody>();

        
        //string yawlink_pitchbacklink = baselink_yawlink + "/yawlink/yawlink_pitchbacklink";
        //jointArticulationBodies[1] = ecm.transform.Find(yawlink_pitchbacklink).GetComponent<ArticulationBody>();
        
        //string pitchendlink_maininsertionlink = yawlink_pitchbacklink + "pitchbacklink/pitchbacklink_pitchbottomlink/pitchbottomlink/pitchbottomlink_pitchendlink" +
        //                                                                "/pitchendlink/pitchendlink_maininsertionlink";

        //jointArticulationBodies[2] = ecm.transform.Find(pitchendlink_maininsertionlink).GetComponent<ArticulationBody>();
        /*
        string maininsertionlink_toollink = pitchendlink_maininsertionlink + "/maininsertionlink/maininsertionlink_toollink";
        jointArticulationBodies[3] = niryoOne.transform.Find(maininsertionlink_toollink).GetComponent<ArticulationBody>();
        */
    }

    public void Publish()
    {
        ECMMoveitJoints sourceDestinationMessage = new ECMMoveitJoints();
        
        sourceDestinationMessage.baselink_yawlink = -1.0;
        sourceDestinationMessage.yawlink_pitchbacklink = -1.0;
        sourceDestinationMessage.pitchendlink_maininsertionlink = -1.0;
        sourceDestinationMessage.maininsertionlink_toollink = -1.0;

        sourceDestinationMessage.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };

        sourceDestinationMessage.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = targetPlacement.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>()
        };
        
        //sourceDestinationMessage.baselink_yawlink = jointArticulationBodies[0].xDrive.target;
        //sourceDestinationMessage.yawlink_pitchbacklink = jointArticulationBodies[1].xDrive.target;
        //sourceDestinationMessage.pitchendlink_maininsertionlink = jointArticulationBodies[2].xDrive.target;
        //sourceDestinationMessage.maininsertionlink_toollink = jointArticulationBodies[3].xDrive.target;
        /*
        // Pick Pose
        sourceDestinationMessage.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = targetPlacement.transform.position.To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };
        */
        // Finally send the message to server_endpoint.py running in ROS
        ros.Send(topicName, sourceDestinationMessage);
    }
}
