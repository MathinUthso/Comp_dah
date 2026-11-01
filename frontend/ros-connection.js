(function attachRosConnection(global) {
  class RosBridgeConnection {
    constructor(options) {
      this.urls = Array.isArray(options.urls) && options.urls.length
        ? options.urls
        : [options.url || "ws://localhost:9090"];
      this.urlIndex = 0;
      this.url = this.urls[this.urlIndex];
      this.reconnectBaseMs = options.reconnectBaseMs || 1000;
      this.reconnectMaxMs = options.reconnectMaxMs || 10000;
      this.ws = null;
      this.status = "disconnected";
      this.reconnectAttempt = 0;
      this.nextId = 1;
      this.eventHandlers = {
        status: new Set(),
        error: new Set(),
      };
      this.topicHandlersByName = new Map();
      this.subscriptionIdsByName = new Map();
      this.subscriptionTypesByName = new Map();
      this.serviceResolvers = new Map();
      this.advertisedTopics = new Set();
      this.pendingReconnectTimer = null;
      this.closedByUser = false;
    }

    on(eventName, handler) {
      const handlers = this.eventHandlers[eventName];
      if (!handlers) {
        return () => {};
      }
      handlers.add(handler);
      return () => handlers.delete(handler);
    }

    emit(eventName, payload) {
      const handlers = this.eventHandlers[eventName];
      if (!handlers) {
        return;
      }
      handlers.forEach((handler) => {
        try {
          handler(payload);
        } catch (error) {
          console.error("ROS handler error", error);
        }
      });
    }

    updateStatus(status, detail) {
      this.status = status;
      this.emit("status", {
        status,
        detail: detail || "",
        reconnectAttempt: this.reconnectAttempt,
      });
    }

    connect() {
      this.closedByUser = false;
      this.openSocket();
    }

    openSocket() {
      if (this.closedByUser) {
        return;
      }
      if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
        return;
      }

      if (this.reconnectAttempt > 0) {
        this.updateStatus("reconnecting", `attempt ${this.reconnectAttempt} @ ${this.url}`);
      } else {
        this.updateStatus("connecting", this.url);
      }

      try {
        this.ws = new WebSocket(this.url);
      } catch (error) {
        this.emit("error", error);
        this.scheduleReconnect("socket create failed");
        return;
      }

      this.ws.onopen = () => {
        this.reconnectAttempt = 0;
        this.updateStatus("connected", "");
        this.restoreSubscriptions();
        this.restoreAdvertisements();
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(event.data);
      };

      this.ws.onerror = (error) => {
        this.emit("error", error);
      };

      this.ws.onclose = () => {
        if (this.closedByUser) {
          this.updateStatus("disconnected", "closed");
          return;
        }
        this.scheduleReconnect("socket closed");
      };
    }

    scheduleReconnect(detail) {
      this.reconnectAttempt += 1;
      if (this.urls.length > 1) {
        this.urlIndex = (this.urlIndex + 1) % this.urls.length;
        this.url = this.urls[this.urlIndex];
      }
      const backoff = Math.min(
        this.reconnectMaxMs,
        this.reconnectBaseMs * Math.pow(2, Math.max(0, this.reconnectAttempt - 1))
      );
      this.updateStatus("reconnecting", `${detail || `retry in ${backoff}ms`} @ ${this.url}`);
      if (this.pendingReconnectTimer) {
        clearTimeout(this.pendingReconnectTimer);
      }
      this.pendingReconnectTimer = setTimeout(() => {
        this.pendingReconnectTimer = null;
        this.openSocket();
      }, backoff);
    }

    ensureOpen() {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        throw new Error("ROS bridge is not connected");
      }
    }

    send(message) {
      this.ensureOpen();
      this.ws.send(JSON.stringify(message));
    }

    handleMessage(raw) {
      let message;
      try {
        message = JSON.parse(raw);
      } catch (error) {
        this.emit("error", new Error("Unable to parse rosbridge message"));
        return;
      }

      if (message.op === "publish" && message.topic) {
        const handlers = this.topicHandlersByName.get(message.topic);
        if (handlers) {
          handlers.forEach((handler) => {
            try {
              handler(message.msg || {});
            } catch (error) {
              this.emit("error", error);
            }
          });
        }
        return;
      }

      if ((message.op === "service_response" || message.op === "call_service") && message.id) {
        const resolver = this.serviceResolvers.get(message.id);
        if (resolver) {
          this.serviceResolvers.delete(message.id);
          resolver(message);
        }
      }
    }

    subscribe(topicName, callback, topicType) {
      if (!this.topicHandlersByName.has(topicName)) {
        this.topicHandlersByName.set(topicName, new Set());
      }
      this.topicHandlersByName.get(topicName).add(callback);

      if (!this.subscriptionIdsByName.has(topicName)) {
        const id = `subscribe:${topicName}:${this.nextId++}`;
        this.subscriptionIdsByName.set(topicName, id);
        this.subscriptionTypesByName.set(topicName, topicType || null);
        this.trySendSubscribe(topicName, id, topicType || null);
      }

      return () => {
        const handlers = this.topicHandlersByName.get(topicName);
        if (!handlers) {
          return;
        }
        handlers.delete(callback);
        if (handlers.size === 0) {
          this.topicHandlersByName.delete(topicName);
          const subId = this.subscriptionIdsByName.get(topicName);
          if (subId) {
            this.trySendUnsubscribe(topicName, subId);
            this.subscriptionIdsByName.delete(topicName);
            this.subscriptionTypesByName.delete(topicName);
          }
        }
      };
    }

    trySendSubscribe(topicName, id, topicType) {
      try {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
          const message = { op: "subscribe", id, topic: topicName };
          if (topicType) {
            message.type = topicType;
          }
          this.ws.send(JSON.stringify(message));
        }
      } catch (error) {
        this.emit("error", error);
      }
    }

    trySendUnsubscribe(topicName, id) {
      try {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
          this.ws.send(JSON.stringify({ op: "unsubscribe", id, topic: topicName }));
        }
      } catch (error) {
        this.emit("error", error);
      }
    }

    advertise(topicName, topicType) {
      this.advertisedTopics.add(`${topicName}::${topicType}`);
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        return;
      }
      this.ws.send(JSON.stringify({ op: "advertise", topic: topicName, type: topicType }));
    }

    restoreAdvertisements() {
      this.advertisedTopics.forEach((entry) => {
        const separatorIndex = entry.indexOf("::");
        const topicName = entry.slice(0, separatorIndex);
        const topicType = entry.slice(separatorIndex + 2);
        this.ws.send(JSON.stringify({ op: "advertise", topic: topicName, type: topicType }));
      });
    }

    restoreSubscriptions() {
      this.subscriptionIdsByName.forEach((id, topicName) => {
        const topicType = this.subscriptionTypesByName.get(topicName) || null;
        const message = { op: "subscribe", id, topic: topicName };
        if (topicType) {
          message.type = topicType;
        }
        this.ws.send(JSON.stringify(message));
      });
    }

    publish(topicName, topicType, msg) {
      this.advertise(topicName, topicType);
      this.send({ op: "publish", topic: topicName, msg });
    }

    callService(serviceName, serviceType, args, timeoutMs) {
      const id = `service:${serviceName}:${this.nextId++}`;
      const timeout = typeof timeoutMs === "number" ? timeoutMs : 2000;

      return new Promise((resolve, reject) => {
        let done = false;
        const timeoutHandle = setTimeout(() => {
          if (done) {
            return;
          }
          done = true;
          this.serviceResolvers.delete(id);
          reject(new Error(`Service timeout: ${serviceName}`));
        }, timeout);

        this.serviceResolvers.set(id, (message) => {
          if (done) {
            return;
          }
          done = true;
          clearTimeout(timeoutHandle);
          resolve(message);
        });

        try {
          this.send({
            op: "call_service",
            id,
            service: serviceName,
            type: serviceType,
            args: args || {},
          });
        } catch (error) {
          clearTimeout(timeoutHandle);
          this.serviceResolvers.delete(id);
          reject(error);
        }
      });
    }

    disconnect() {
      this.closedByUser = true;
      if (this.pendingReconnectTimer) {
        clearTimeout(this.pendingReconnectTimer);
        this.pendingReconnectTimer = null;
      }
      this.subscriptionIdsByName.forEach((id, topicName) => this.trySendUnsubscribe(topicName, id));
      this.subscriptionIdsByName.clear();
      this.topicHandlersByName.clear();
      if (this.ws) {
        this.ws.close();
      }
      this.ws = null;
      this.updateStatus("disconnected", "manual");
    }
  }

  global.RosBridgeConnection = RosBridgeConnection;
})(window);
