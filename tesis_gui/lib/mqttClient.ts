// lib/mqttClient.ts
"use client";
import mqtt, { MqttClient } from "mqtt";

let client: MqttClient | null = null;

// ===== Config =====
export const DEVICE_ID =
  process.env.NEXT_PUBLIC_DEVICE_ID?.trim() || "esp32-demo";

export const CAM_DEVICE_ID =
  process.env.NEXT_PUBLIC_CAM_DEVICE_ID?.trim() || "esp32cam";


  // ===== Fallas (popup) =====
export type FaultPayload = Partial<{
  code: string;     // p.ej. "SENSOR_TIMEOUT", "MAN_TIMEOUT", etc.
  message: string;  // p.ej. "soff"
  etapa: number;    // etapa en la que ocurrió
  ts: string;       // opcional (si luego envías timestamp desde el ESP32)
}>;

// si ya publicas en 'tesis/<id>/state' déjalo así.
// si migraste a 'tesis/<id>/state/json', cambia esta constante:
const STATE_TOPIC_KIND: "state" | "state/json" = "state"; // <-- cámbialo si usas /state/json

export function getClient() {
  if (client) return client;

  // Debe apuntar a tu listener WebSockets (ws://IP:PUERTO), no al TCP 1883
  client = mqtt.connect(process.env.NEXT_PUBLIC_MQTT_URL!, {
    username: process.env.NEXT_PUBLIC_MQTT_USER,
    password: process.env.NEXT_PUBLIC_MQTT_PASS,
    keepalive: 30,
    reconnectPeriod: 2000,
    protocolVersion: 4, // MQTT v3.1.1
    clean: true,
  });

  client.on("connect", () => console.log("[MQTT] conectado"));
  client.on("reconnect", () => console.log("[MQTT] reconectando…"));
  client.on("error", (e) => console.warn("[MQTT] error", e?.message));

  return client;
}

// ===== GUI -> ESP (texto/console) =====
export function publishText(deviceId: string, line: string) {
  const c = getClient();
  const topic = `tesis/${deviceId}/cmd/text`;
  c.publish(topic, line ?? "", { qos: 0, retain: false });
}

/**
 * Se suscribe a `tesis/<id>/fault/json` (retain=1 en el ESP32).
 * Llama onFault(null) si llega "{}" (fault “limpiada”).
 */
export function subscribeFault(
  deviceId: string,
  onFault: (f: FaultPayload | null) => void
) {
  const c = getClient();
  const topic = `tesis/${deviceId}/fault/json`;

  const handler = (t: string, payload: Buffer) => {
    if (t !== topic) return;

    const text = payload.toString("utf8").trim();
    if (!text || text === "{}") {
      // No hay falla activa (el ESP32 publicó "{}" con retain para limpiar)
      onFault(null);
      return;
    }

    try {
      const obj = JSON.parse(text) as FaultPayload;
      // Normaliza por si viene algo raro
      const fault: FaultPayload = {
        code: typeof obj.code === "string" ? obj.code : "FALLA",
        message: typeof obj.message === "string" ? obj.message : "Revisar el sistema",
        etapa: typeof obj.etapa === "number" ? obj.etapa : undefined,
        ts: typeof obj.ts === "string" ? obj.ts : undefined,
      };
      onFault(fault);
    } catch {
      // Si no es JSON válido, lo pasamos como “mensaje” crudo
      onFault({ code: "PARSE_ERROR", message: text });
    }
  };

  c.subscribe(topic, { qos: 1 });
  c.on("message", handler);

  return () => {
    try {
      c.off("message", handler as any);
      c.unsubscribe(topic);
    } catch {}
  };
}

// ===== Estado del dispositivo =====
export function subscribeState(deviceId: string, onState: (s: any) => void) {
  const c = getClient();
  const topic = `tesis/${deviceId}/${STATE_TOPIC_KIND}`;
  c.subscribe(topic, { qos: 0 });

  const onMsg = (t: string, payload: Buffer) => {
    if (t !== topic) return;
    try {
      const s = JSON.parse(payload.toString("utf8"));
      onState(s);
    } catch (e) {
      console.warn("[MQTT] state JSON inválido:", e);
    }
  };

  c.on("message", onMsg);
  return () => {
    try {
      c.off("message", onMsg as any);
      c.unsubscribe(topic);
    } catch {}
  };
}

export function subscribeSeries(
  deviceId: string,
  sensor: string,                          // "voltage_kv" | "distance_mm"
  onValue: (v: number) => void
) {
  const c = getClient();
  const topic = `tesis/${deviceId}/sensor/${sensor}`;

  const handler = (t: string, payload: Buffer) => {
    if (t !== topic) return;
    const text = payload.toString("utf8").trim();
    let v = Number.NaN;
    if (/^-?\d+(\.\d+)?$/.test(text)) v = parseFloat(text);
    else {
      try {
        const obj = JSON.parse(text);
        const cand = obj?.v ?? obj?.value ?? obj?.val;
        if (typeof cand === "number") v = cand;
      } catch {}
    }
    if (!Number.isNaN(v)) onValue(v);
  };

  c.subscribe(topic, { qos: 0 });
  c.on("message", handler);
  return () => {
    try {
      c.off("message", handler as any);
      c.unsubscribe(topic);
    } catch {}
  };
}

// ===== Logs / ACKs desde el ESP =====
export function subscribeLogs(
  deviceId: string,
  onLine: (line: string) => void
) {
  const c = getClient();
  const ackTopic = `tesis/${deviceId}/ack/text`;
  const logTopic = `tesis/${deviceId}/log/text`;

  const onMsg = (t: string, payload: Buffer) => {
    const text = payload.toString("utf8");
    if (t === ackTopic) onLine(`[ACK] ${text}`);
    else if (t === logTopic) onLine(`[LOG] ${text}`);
  };

  c.subscribe([ackTopic, logTopic], { qos: 0 });
  c.on("message", onMsg);

  return () => {
    try {
      c.off("message", onMsg as any);
      c.unsubscribe(ackTopic);
      c.unsubscribe(logTopic);
    } catch {}
  };
}

// ===== Parámetros (ya los tenías) =====
export type ParamsPatch = Partial<{
  collectorOn: boolean;
  rpm: number;
  kv: number;
  mlmin: number;
  syringeD: number;
  gap: number;
}>;

export function publishParams(deviceId: string, patch: ParamsPatch, apply = false) {
  const c = getClient();
  const topic = `tesis/${deviceId}/cmd/parametros`;
  const payload = JSON.stringify({ ...patch, apply });
  c.publish(topic, payload, { qos: 1, retain: false });
}

export function subscribeCamFrame(
  deviceId: string = CAM_DEVICE_ID,
  cb: (dataUrl: string) => void
) {
  const c = getClient();
  const topic = `cams/${deviceId}/frame_b64`;

  const handler = (t: string, payload: Uint8Array) => {
    if (t !== topic) return;
    // El ESP publica "data:image/jpeg;base64,...."
    // Si algún día publicas sólo base64, aquí le antepones el prefijo.
    const txt = new TextDecoder().decode(payload);
    cb(txt);
  };

  c.subscribe(topic, { qos: 0 });
  c.on("message", handler);

  return () => {
    try {
      c.off("message", handler as any);
      c.unsubscribe(topic);
    } catch {}
  };
}


export function subscribeParamsActive(
  deviceId: string,
  onParams: (p: any) => void
) {
  const c = getClient();
  const topic = `tesis/${deviceId}/params/active`;
  const handler = (t: string, p: Buffer) => {
    if (t !== topic) return;
    try { onParams(JSON.parse(p.toString("utf8"))); } catch {}
  };
  c.subscribe(topic, { qos: 1 });
  c.on("message", handler);
  return () => {
    try {
      c.off("message", handler as any);
      c.unsubscribe(topic);
    } catch {}
  };
  
}




