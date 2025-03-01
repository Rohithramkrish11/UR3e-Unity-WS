//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UrRobotiq
{
    [Serializable]
    public class UR3eTrajectoryMsg : Message
    {
        public const string k_RosMessageName = "ur_robotiq/UR3eTrajectory";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg[] trajectory;

        public UR3eTrajectoryMsg()
        {
            this.trajectory = new Moveit.RobotTrajectoryMsg[0];
        }

        public UR3eTrajectoryMsg(Moveit.RobotTrajectoryMsg[] trajectory)
        {
            this.trajectory = trajectory;
        }

        public static UR3eTrajectoryMsg Deserialize(MessageDeserializer deserializer) => new UR3eTrajectoryMsg(deserializer);

        private UR3eTrajectoryMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.trajectory, Moveit.RobotTrajectoryMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.trajectory);
            serializer.Write(this.trajectory);
        }

        public override string ToString()
        {
            return "UR3eTrajectoryMsg: " +
            "\ntrajectory: " + System.String.Join(", ", trajectory.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
