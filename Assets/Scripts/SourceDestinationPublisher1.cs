using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.UrRobotiq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher1 : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
    { "base_link/base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };


    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur3e_joints";

    [SerializeField]
    GameObject m_UR3e;
    [SerializeField]
    GameObject m_Place;  // The Place GameObject representing the pick pose

    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<UR3eMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_UR3e.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new UR3eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
        }

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Place.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Place.transform.eulerAngles.y, 0).To<FLU>()
        };

        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
