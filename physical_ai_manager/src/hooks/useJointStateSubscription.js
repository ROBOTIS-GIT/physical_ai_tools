import { useEffect, useRef, useCallback } from 'react';
import { useSelector } from 'react-redux';
import ROSLIB from 'roslib';

const JOINT_STATE_TOPICS = [
  { name: '/robot/arm_left_follower/joint_states', type: 'sensor_msgs/msg/JointState' },
  { name: '/robot/arm_right_follower/joint_states', type: 'sensor_msgs/msg/JointState' },
  { name: '/robot/head_follower/joint_states', type: 'sensor_msgs/msg/JointState' },
  { name: '/robot/lift_follower/joint_states', type: 'sensor_msgs/msg/JointState' },
];

const ACTION_CHUNK_TOPIC = {
  name: '/inference/trajectory_preview',
  type: 'trajectory_msgs/msg/JointTrajectory',
};

const THROTTLE_MS = 33;

export default function useJointStateSubscription(setJointValues, setActionChunk, enabled = true) {
  const rosHost = useSelector((state) => state.ros.rosHost);
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);
  const subscribersRef = useRef([]);
  const lastJointUpdateRef = useRef(0);

  const handleJointState = useCallback(
    (msg) => {
      const now = Date.now();
      if (now - lastJointUpdateRef.current < THROTTLE_MS) return;
      lastJointUpdateRef.current = now;

      if (msg.name && msg.position) {
        setJointValues({ name: msg.name, position: msg.position });
      }
    },
    [setJointValues]
  );

  const handleActionChunk = useCallback(
    (msg) => {
      if (!msg.joint_names || !msg.points || msg.points.length === 0) return;

      if (setActionChunk) {
        setActionChunk({
          names: msg.joint_names,
          points: msg.points.map((p) => p.positions),
        });
      }
    },
    [setActionChunk]
  );

  useEffect(() => {
    if (!enabled || !rosHost) return;

    const ros = new ROSLIB.Ros({ url: rosbridgeUrl || `ws://${rosHost}:9090` });
    const subs = [];

    ros.on('connection', () => {
      JOINT_STATE_TOPICS.forEach((topic) => {
        const sub = new ROSLIB.Topic({
          ros,
          name: topic.name,
          messageType: topic.type,
          throttle_rate: THROTTLE_MS,
        });
        sub.subscribe(handleJointState);
        subs.push(sub);
      });

      const chunkSub = new ROSLIB.Topic({
        ros,
        name: ACTION_CHUNK_TOPIC.name,
        messageType: ACTION_CHUNK_TOPIC.type,
      });
      chunkSub.subscribe(handleActionChunk);
      subs.push(chunkSub);

      subscribersRef.current = subs;
    });

    ros.on('error', (err) => {
      console.error('Joint state ROS connection error:', err);
    });

    return () => {
      subs.forEach((sub) => {
        try {
          sub.unsubscribe();
        } catch (_e) { /* ignore */ }
      });
      subscribersRef.current = [];
      try {
        ros.close();
      } catch (_e) { /* ignore */ }
    };
  }, [enabled, rosHost, rosbridgeUrl, handleJointState, handleActionChunk]);
}
