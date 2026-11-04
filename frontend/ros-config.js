(function attachRosDashboardConfig(global) {
  const TOPICS = {
    proximityRange: { name: "/proximity/range", type: "sensor_msgs/Range" },
    thermalTemperature: { name: "/thermal/temperature", type: "sensor_msgs/Temperature" },
    hardwareOdometry: { name: "/hardware/odometry", type: "nav_msgs/Odometry" },
    hardwareImu: { name: "/hardware/imu", type: "sensor_msgs/Imu" },
    lidarScan: { name: "/scan", type: "sensor_msgs/LaserScan" },
    diagnostics: { name: "/diagnostics", type: "diagnostic_msgs/DiagnosticArray" },
    cameraCompressed: { name: "/camera/image_compressed", type: "sensor_msgs/CompressedImage" },
    cameraVideo2Compressed: { name: "/camera/video2/image_compressed", type: "sensor_msgs/CompressedImage" },
    cameraWebcamCompressed: { name: "/camera/webcam/image_compressed", type: "sensor_msgs/CompressedImage" },
    cameraVideo2Healthy: { name: "/camera/video2/healthy", type: "std_msgs/Bool" },
    cameraWebcamHealthy: { name: "/camera/webcam/healthy", type: "std_msgs/Bool" },
    cameraPrimarySource: { name: "/camera/primary_source", type: "std_msgs/String" },
    emergencyStatus: { name: "/emergency/status", type: "std_msgs/Bool" },
    flexbeStatus: { name: "/flexbe/status", type: "std_msgs/String" },
    jointStates: { name: "/joint_states", type: "sensor_msgs/JointState" },
    cmdVel: { name: "/cmd_vel", type: "geometry_msgs/Twist" },
  };

  const SERVICES = {
    emergencyEstop: { name: "/emergency/estop", type: "std_srvs/srv/Trigger" },
    flexbeExecuteBehavior: { name: "/flexbe/execute_behavior", type: "example_interfaces/srv/AddTwoInts" },
  };

  const BEHAVIORS = {
    1: "move_forward",
    2: "move_backward",
    3: "rotate_left",
    4: "rotate_right",
    5: "rotate_180",
  };

  const TELEOP = {
    linearMps: 0.5,
    angularRadps: 1.0,
    boostMultiplier: 1.5,
    streamIntervalMs: 100,
    flipperStepRad: 0.15,
  };

  const DEVELOPMENT_HOST = "localhost";
  const host = (global.location && global.location.hostname) || DEVELOPMENT_HOST;

  global.RosDashboardConfig = {
    rosbridgeUrl: `ws://${host}:9090`,
    rosbridgeUrls: [`ws://${host}:9090`, "ws://127.0.0.1:9090"],
    usbCameraStreamPath: "/api/v1/cameras/usb/stream.mjpeg",
    cameraDevices: {
      video2: "/dev/video2",
      webcam: "0",
      webcamFallback: "/dev/video0",
    },
    reconnectBaseMs: 1000,
    reconnectMaxMs: 12000,
    staleThresholdMs: 3000,
    maxImageBytesBase64: 4 * 1024 * 1024,
    topics: TOPICS,
    services: SERVICES,
    behaviors: BEHAVIORS,
    teleop: TELEOP,
  };
})(window);
