using RosMessageTypes.UrRobotiq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class GripperPublisher : MonoBehaviour
{
    [SerializeField]
    private string m_TopicName = "/ur3_gripper"; // ROS topic for gripper

    // ROS Connector
    private ROSConnection m_Ros;

    [SerializeField]
    private GameObject m_JointLinkObject; // The GameObject containing the joint whose position you want to use

    private UrdfJointRevolute m_JointArticulationBody;

    void Start()
    {
        // Validate and initialize ROS connection
        m_Ros = ROSConnection.GetOrCreateInstance();
        if (m_Ros == null)
        {
            Debug.LogError("ROSConnection instance not found!");
            return;
        }

        m_Ros.RegisterPublisher<UR3eGripperMsg>(m_TopicName);  // Registering to publish the gripper value

        // Validate and initialize UrdfJointRevolute
        if (m_JointLinkObject != null)
        {
            m_JointArticulationBody = m_JointLinkObject.GetComponent<UrdfJointRevolute>();
            if (m_JointArticulationBody == null)
            {
                Debug.LogError("No UrdfJointRevolute component found on m_JointLinkObject!");
            }
        }
        else
        {
            Debug.LogWarning("m_JointLinkObject is not assigned!");
        }
    }

    public void Publish()
    {
        if (m_JointArticulationBody == null)
        {
            Debug.LogWarning("UrdfJointRevolute component is missing. Cannot publish gripper value.");
            return;
        }

        // Extract the joint position using the UrdfJointRevolute component
        float jointPosition = m_JointArticulationBody.GetPosition(); // Use this for revolute joint position

        // Map the joint position to the gripper value (ensure it is in a valid range)
        float gripperValue = Mathf.Clamp(jointPosition, 0f, 1f); // Adjust range if necessary

        // Create and populate the gripper message
        var gripperMessage = new UR3eGripperMsg
        {
            gripper = gripperValue // Setting the gripper value
        };

        // Publish the gripper value
        m_Ros.Publish(m_TopicName, gripperMessage);
        Debug.Log($"Gripper value {gripperValue} published to {m_TopicName}");
    }
}
