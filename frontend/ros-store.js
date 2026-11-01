(function attachRosStore(global) {
  function nowMs() {
    return Date.now();
  }

  function quaternionToHeadingRad(q) {
    const siny = 2 * (q.w * q.z + q.x * q.y);
    const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny, cosy);
  }

  function radiansToDegrees(rad) {
    return (rad * 180) / Math.PI;
  }

  function parseDiagValue(status, keyMatch) {
    if (!status || !Array.isArray(status.values)) {
      return "n/a";
    }
    const found = status.values.find((v) =>
      String(v.key || "")
        .toLowerCase()
        .includes(keyMatch)
    );
    return found ? String(found.value || "n/a") : "n/a";
  }

  function parseDiagnosticArray(msg) {
    const entries = Array.isArray(msg.status) ? msg.status : [];
    const battery = entries.find((s) => String(s.name || "").toLowerCase().includes("battery"));
    const cpu = entries.find((s) => String(s.name || "").toLowerCase().includes("cpu"));
    const memory = entries.find((s) => String(s.name || "").toLowerCase().includes("memory"));

    return {
      battery: {
        name: battery?.name || "Battery",
        message: battery?.message || "unknown",
        value: parseDiagValue(battery, "percent"),
      },
      cpu: {
        name: cpu?.name || "CPU",
        message: cpu?.message || "unknown",
        value: parseDiagValue(cpu, "usage"),
      },
      memory: {
        name: memory?.name || "Memory",
        message: memory?.message || "unknown",
        value: parseDiagValue(memory, "usage"),
      },
      raw: entries,
    };
  }

  function toImageDataUrl(msg, maxBytesBase64) {
    const base64 = String(msg?.data || "");
    if (!base64) {
      return "";
    }
    if (!/^[A-Za-z0-9+/=]+$/.test(base64)) {
      return "";
    }
    if (base64.length > maxBytesBase64) {
      return "";
    }
    const format = String(msg?.format || "jpeg").toLowerCase();
    const mime = format.includes("png") ? "image/png" : "image/jpeg";
    return `data:${mime};base64,${base64}`;
  }

  function toBoolean(data) {
    if (typeof data === "boolean") {
      return data;
    }
    if (typeof data === "number") {
      return data !== 0;
    }
    if (typeof data === "string") {
      const normalized = data.trim().toLowerCase();
      if (normalized === "true" || normalized === "1" || normalized === "yes" || normalized === "on") {
        return true;
      }
      if (normalized === "false" || normalized === "0" || normalized === "no" || normalized === "off" || normalized === "") {
        return false;
      }
    }
    return !!data;
  }

  function createDashboardStore(connection, config) {
    const listeners = new Set();
    const unsubscribers = [];
    let staleInterval = null;

    const state = {
      connection: {
        status: "disconnected",
        detail: "",
        reconnectAttempt: 0,
      },
      telemetry: {
        proximity: null,
        thermal: null,
        odometry: null,
        imu: null,
        lidar: null,
        diagnostics: null,
        emergency: false,
        flexbeStatus: "unknown",
        cameraDataUrl: "",
        cameraVideo2DataUrl: "",
        cameraWebcamDataUrl: "",
        cameraHealth: {
          video2Healthy: false,
          webcamHealthy: false,
          primarySource: "none",
        },
      },
      flippers: {
        front_left_flipper_joint: 0,
        front_right_flipper_joint: 0,
        rear_left_flipper_joint: 0,
        rear_right_flipper_joint: 0,
      },
      ui: {
        stale: {
          proximity: true,
          thermal: true,
          odometry: true,
          imu: true,
          lidar: true,
          diagnostics: true,
          emergency: true,
          flexbeStatus: true,
          camera: true,
          cameraVideo2: true,
          cameraWebcam: true,
          cameraHealth: true,
        },
        lastError: "",
      },
      actions: {
        behaviorLabel: "idle",
        estopPending: false,
      },
      timestamps: {
        proximity: 0,
        thermal: 0,
        odometry: 0,
        imu: 0,
        lidar: 0,
        diagnostics: 0,
        emergency: 0,
        flexbeStatus: 0,
        camera: 0,
        cameraVideo2: 0,
        cameraWebcam: 0,
        cameraHealth: 0,
      },
    };

    function emit() {
      listeners.forEach((listener) => listener(state));
    }

    function markUpdate(key) {
      state.timestamps[key] = nowMs();
      state.ui.stale[key] = false;
    }

    function setError(errorMessage) {
      state.ui.lastError = String(errorMessage || "");
      emit();
    }

    function subscribe(listener) {
      listeners.add(listener);
      listener(state);
      return () => listeners.delete(listener);
    }

    function handleConnectionStatus(payload) {
      state.connection.status = payload.status;
      state.connection.detail = payload.detail || "";
      state.connection.reconnectAttempt = payload.reconnectAttempt || 0;
      emit();
    }

    function handleProximity(msg) {
      state.telemetry.proximity = {
        range: Number(msg.range || 0),
        frame_id: msg.header?.frame_id || "",
      };
      markUpdate("proximity");
      emit();
    }

    function handleThermal(msg) {
      state.telemetry.thermal = {
        temperature: Number(msg.temperature || 0),
      };
      markUpdate("thermal");
      emit();
    }

    function handleOdometry(msg) {
      const pos = msg.pose?.pose?.position || { x: 0, y: 0 };
      const orientation = msg.pose?.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 };
      const twist = msg.twist?.twist || { linear: { x: 0 }, angular: { z: 0 } };
      const headingRad = quaternionToHeadingRad(orientation);

      state.telemetry.odometry = {
        x: Number(pos.x || 0),
        y: Number(pos.y || 0),
        headingDeg: radiansToDegrees(headingRad),
        linearX: Number(twist.linear?.x || 0),
        angularZ: Number(twist.angular?.z || 0),
      };
      markUpdate("odometry");
      emit();
    }

    function handleImu(msg) {
      const ang = msg.angular_velocity || { x: 0, y: 0, z: 0 };
      const lin = msg.linear_acceleration || { x: 0, y: 0, z: 0 };
      state.telemetry.imu = {
        angularX: Number(ang.x || 0),
        angularY: Number(ang.y || 0),
        angularZ: Number(ang.z || 0),
        accelX: Number(lin.x || 0),
        accelY: Number(lin.y || 0),
        accelZ: Number(lin.z || 0),
      };
      markUpdate("imu");
      emit();
    }

    function handleLidar(msg) {
      const ranges = Array.isArray(msg.ranges) ? msg.ranges.map((value) => Number(value)) : [];
      state.telemetry.lidar = {
        ranges,
        angleMin: Number(msg.angle_min || 0),
        angleIncrement: Number(msg.angle_increment || 0),
        rangeMin: Number(msg.range_min || 0),
        rangeMax: Number(msg.range_max || 12),
        frameId: msg.header?.frame_id || "",
      };
      markUpdate("lidar");
      emit();
    }

    function handleDiagnostics(msg) {
      state.telemetry.diagnostics = parseDiagnosticArray(msg);
      markUpdate("diagnostics");
      emit();
    }

    function handleEmergency(msg) {
      state.telemetry.emergency = toBoolean(msg.data);
      markUpdate("emergency");
      emit();
    }

    function handleFlexbeStatus(msg) {
      state.telemetry.flexbeStatus = String(msg.data || "unknown");
      state.actions.behaviorLabel = String(msg.data || "idle");
      markUpdate("flexbeStatus");
      emit();
    }

    function handleJointStates(msg) {
      const names = Array.isArray(msg.name) ? msg.name : [];
      const positions = Array.isArray(msg.position) ? msg.position : [];
      names.forEach((name, index) => {
        if (!(name in state.flippers)) {
          return;
        }
        const value = Number(positions[index]);
        if (Number.isFinite(value)) {
          state.flippers[name] = value;
        }
      });
    }

    function handleCameraCompressed(msg) {
      const dataUrl = toImageDataUrl(msg, config.maxImageBytesBase64);
      if (dataUrl) {
        state.telemetry.cameraDataUrl = dataUrl;
        markUpdate("camera");
        emit();
      }
    }

    function handleCameraVideo2Compressed(msg) {
      const dataUrl = toImageDataUrl(msg, config.maxImageBytesBase64);
      if (dataUrl) {
        state.telemetry.cameraVideo2DataUrl = dataUrl;
        markUpdate("cameraVideo2");
        emit();
      }
    }

    function handleCameraWebcamCompressed(msg) {
      const dataUrl = toImageDataUrl(msg, config.maxImageBytesBase64);
      if (dataUrl) {
        state.telemetry.cameraWebcamDataUrl = dataUrl;
        markUpdate("cameraWebcam");
        emit();
      }
    }

    function handleCameraVideo2Healthy(msg) {
      state.telemetry.cameraHealth.video2Healthy = toBoolean(msg.data);
      markUpdate("cameraHealth");
      emit();
    }

    function handleCameraWebcamHealthy(msg) {
      state.telemetry.cameraHealth.webcamHealthy = toBoolean(msg.data);
      markUpdate("cameraHealth");
      emit();
    }

    function handleCameraPrimarySource(msg) {
      const source = String(msg.data || "none").toLowerCase();
      state.telemetry.cameraHealth.primarySource = ["video2", "webcam", "none"].includes(source) ? source : "none";
      markUpdate("cameraHealth");
      emit();
    }

    function updateStaleFlags() {
      const keys = Object.keys(state.timestamps);
      const threshold = config.staleThresholdMs;
      const timestampNow = nowMs();
      let changed = false;

      keys.forEach((key) => {
        const last = state.timestamps[key];
        const stale = !last || timestampNow - last > threshold;
        if (state.ui.stale[key] !== stale) {
          state.ui.stale[key] = stale;
          changed = true;
        }
      });

      if (changed) {
        emit();
      }
    }

    function init() {
      unsubscribers.push(connection.on("status", handleConnectionStatus));
      unsubscribers.push(connection.on("error", (error) => setError(error?.message || String(error))));

      unsubscribers.push(connection.subscribe(config.topics.proximityRange.name, handleProximity, config.topics.proximityRange.type));
      unsubscribers.push(connection.subscribe(config.topics.thermalTemperature.name, handleThermal, config.topics.thermalTemperature.type));
      unsubscribers.push(connection.subscribe(config.topics.hardwareOdometry.name, handleOdometry, config.topics.hardwareOdometry.type));
      unsubscribers.push(connection.subscribe(config.topics.hardwareImu.name, handleImu, config.topics.hardwareImu.type));
      unsubscribers.push(connection.subscribe(config.topics.lidarScan.name, handleLidar, config.topics.lidarScan.type));
      unsubscribers.push(connection.subscribe(config.topics.diagnostics.name, handleDiagnostics, config.topics.diagnostics.type));
      unsubscribers.push(connection.subscribe(config.topics.emergencyStatus.name, handleEmergency, config.topics.emergencyStatus.type));
      unsubscribers.push(connection.subscribe(config.topics.flexbeStatus.name, handleFlexbeStatus, config.topics.flexbeStatus.type));
      unsubscribers.push(connection.subscribe(config.topics.jointStates.name, handleJointStates, config.topics.jointStates.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraCompressed.name, handleCameraCompressed, config.topics.cameraCompressed.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraVideo2Compressed.name, handleCameraVideo2Compressed, config.topics.cameraVideo2Compressed.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraWebcamCompressed.name, handleCameraWebcamCompressed, config.topics.cameraWebcamCompressed.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraVideo2Healthy.name, handleCameraVideo2Healthy, config.topics.cameraVideo2Healthy.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraWebcamHealthy.name, handleCameraWebcamHealthy, config.topics.cameraWebcamHealthy.type));
      unsubscribers.push(connection.subscribe(config.topics.cameraPrimarySource.name, handleCameraPrimarySource, config.topics.cameraPrimarySource.type));

      staleInterval = setInterval(updateStaleFlags, 1000);
      connection.connect();
    }

    function cleanup() {
      unsubscribers.forEach((unsub) => {
        try {
          unsub();
        } catch (error) {
          console.error("Unsubscribe failed", error);
        }
      });
      if (staleInterval) {
        clearInterval(staleInterval);
        staleInterval = null;
      }
      connection.disconnect();
    }

    function publishCmdVel(linearX, angularZ) {
      const cmd = {
        linear: { x: Number(linearX || 0), y: 0, z: 0 },
        angular: { x: 0, y: 0, z: Number(angularZ || 0) },
      };
      connection.publish(config.topics.cmdVel.name, config.topics.cmdVel.type, cmd);
    }

    function publishFlipperCommand(jointNames, position) {
      const names = Array.isArray(jointNames) ? jointNames : [jointNames];
      const positions = Array.isArray(position)
        ? position.map((value) => Number(value || 0))
        : names.map(() => Number(position || 0));
      const msg = {
        joint_names: names,
        points: [
          {
            positions,
            time_from_start: { sec: 0, nanosec: 200000000 },
          },
        ],
      };
      connection.publish("/flipper_cmd", "trajectory_msgs/JointTrajectory", msg);
    }

    function stopCmdVel() {
      publishCmdVel(0, 0);
    }

    async function triggerEstop() {
      state.actions.estopPending = true;
      emit();
      try {
        const response = await connection.callService(
          config.services.emergencyEstop.name,
          config.services.emergencyEstop.type,
          {},
          3000
        );
        return response.values || response;
      } finally {
        state.actions.estopPending = false;
        emit();
      }
    }

    async function executeBehavior(behaviorId) {
      const id = Number(behaviorId);
      if (!config.behaviors[id]) {
        throw new Error(`Unknown behavior id ${behaviorId}`);
      }
      state.actions.behaviorLabel = config.behaviors[id];
      emit();

      const response = await connection.callService(
        config.services.flexbeExecuteBehavior.name,
        config.services.flexbeExecuteBehavior.type,
        { a: id, b: 0 },
        3000
      );
      return response.values || response;
    }

    return {
      state,
      subscribe,
      init,
      cleanup,
      publishCmdVel,
      stopCmdVel,
      publishFlipperCommand,
      triggerEstop,
      executeBehavior,
    };
  }

  global.createDashboardStore = createDashboardStore;
})(window);
