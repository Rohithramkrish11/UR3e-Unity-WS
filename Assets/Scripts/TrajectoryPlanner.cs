using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.UrRobotiq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f; // Time to wait between joint updates

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "ur3e_moveit"; // ROS service name for trajectory planning

    [SerializeField]
    GameObject m_UR3e; // Robot GameObject
    [SerializeField]
    GameObject m_Target;   // Target GameObject for pick pose

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f; // Offset for the pick pose

    // Robot Joints
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<UrMoverServiceRequest, UrMoverServiceResponse>(m_RosServiceName);

        // Initialize joint articulation bodies
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            var jointObject = m_UR3e.transform.Find(linkName);

            if (jointObject == null)
            {
                Debug.LogError($"Joint object not found: {linkName}");
                continue;
            }

            var jointComponent = jointObject.GetComponent<ArticulationBody>();
            if (jointComponent == null)
            {
                Debug.LogError($"ArticulationBody component not found on: {linkName}");
                continue;
            }

            m_JointArticulationBodies[i] = jointComponent;
        }
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3eMoveitJointsMsg</returns>
    UR3eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new UR3eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
            Debug.Log($"Joint {i + 1} position: {joints.joints[i]} (radians)");
        }

        return joints;
    }

    /// <summary>
    ///     Send the pick_pose to the ROS service and execute the returned trajectory.
    /// </summary>
    public void PublishJoints()
    {
        var request = new UrMoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Calculate the target position relative to the robot's base frame
        Vector3 targetPositionRelative = m_Target.transform.position - m_UR3e.transform.position;

        // Apply offset and convert to ROS FLU coordinate system
        request.pick_pose = new PoseMsg
        {
            position = (targetPositionRelative + m_PickPoseOffset).To<FLU>(), // Convert to ROS FLU
            orientation = Quaternion.Euler(90, 0, 0).To<FLU>() // Gripper faces downward in ROS
        };

        // Log the request to check what data is being sent
        Debug.Log("Sending request with the following data:");
        Debug.Log($"Pick Pose Position (ROS FLU): {request.pick_pose.position}");
        Debug.Log($"Pick Pose Orientation (ROS FLU): {request.pick_pose.orientation}");
        Debug.Log("Joint Angles (radians): " + string.Join(", ", request.joints_input.joints));

        // Send the request to the ROS service
        m_Ros.SendServiceMessage<UrMoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    /// <summary>
    ///     Handle the response from the ROS service.
    /// </summary>
    void TrajectoryResponse(UrMoverServiceResponse response)
    {
        // Check if the response contains valid trajectories
        if (response != null)
        {
            if (response.trajectories.Length > 0)
            {
                Debug.Log("Trajectory returned successfully.");
                Debug.Log($"Number of trajectories: {response.trajectories.Length}");
                StartCoroutine(ExecuteTrajectories(response));
            }
            else
            {
                Debug.LogError("No trajectories returned. Check ROS service logs for details.");
            }
        }
        else
        {
            Debug.LogError("Failed to receive a valid response from UrMoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectory.
    /// </summary>
    IEnumerator ExecuteTrajectories(UrMoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            Debug.Log("Executing trajectory...");

            // For every robot pose in the trajectory
            foreach (var t in response.trajectories[0].joint_trajectory.points)
            {
                var jointPositions = t.positions;
                var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                Debug.Log("Setting joint positions (degrees): " + string.Join(", ", result));

                // Set the joint values for every joint
                for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                }

                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(k_JointAssignmentWait);
            }

            Debug.Log("Trajectory execution complete.");
        }
        else
        {
            Debug.LogError("No valid trajectories in response.");
        }
    }
}