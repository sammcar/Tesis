// app/(dashboard)/monitoreo/page.tsx
"use client";

import { subscribeLogs, publishText, subscribeSeries, subscribeState, subscribeFault, DEVICE_ID } from "@/lib/mqttClient";
import { useEffect, useMemo, useRef, useState } from "react";
import Link from "next/link";
import { subscribeCamFrame, CAM_DEVICE_ID, publishCamLed } from "@/lib/mqttClient";
import { useRouter } from "next/navigation";
import Spline from "@splinetool/react-spline";
import {
  ArrowLeft, Power, RefreshCcw, Camera, TerminalSquare,
  Activity, Ruler, Circle, X,
} from "lucide-react";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";

/* ========== helpers ========== */
const cn = (...xs: (string | false | undefined)[]) => xs.filter(Boolean).join(" ");
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/* ========== página ========== */
export default function MonitoreoPage() {
  const router = useRouter();

  // ----- estados control -----
  const [isOn, setIsOn] = useState(false);
  const [estopActive, setEstopActive] = useState(false);
  const [faultLatched, setFaultLatched] = useState(false);

  // ----- logs / consola -----
  const [logs, setLogs] = useState<string[]>(["Monitoreo iniciado.", "MQTT: desconectado (mock)"]);
  const [inputLine, setInputLine] = useState("");
  const inputRef = useRef<HTMLInputElement>(null);

  // ----- cámara (mqtt + fallback mock) -----
  const [frameUrl, setFrameUrl] = useState<string | null>(null);
  const [camMqttOk, setCamMqttOk] = useState(false);
  const [camLedOn, setCamLedOn] = useState(false);
  const [cameraTick, setCameraTick] = useState(0);
  const cameraRefreshMs = 3000;
  const cameraUrl = useMemo(
    () => `https://picsum.photos/seed/monitoreo-${cameraTick}/1024/576`,
    [cameraTick]
  );


  // ----- series -----
  const [voltageSeries, setvoltageSeries] = useState<number[]>([]);
  const [distanceSeries, setDistanceSeries] = useState<number[]>([]);

  // ----- luces -----
  const [rs485Ok, setRs485Ok] = useState(true);
  const [rs232Ok, setRs232Ok] = useState(true);

  // ----- timer -----
  const [elapsedMs, setElapsedMs] = useState(0);

  // ----- modales -----
  const [faultOpen, setFaultOpen] = useState(false);
  const [faultTitle, setFaultTitle] = useState("");
  const [faultDesc, setFaultDesc] = useState("");
  const [shakeTick, setShakeTick] = useState(0);

  const [finishOpen, setFinishOpen] = useState(false);
  const [finishDesc, setFinishDesc] = useState("");

  // ----- zoom modal (cámara / charts) -----
  const [zoomOpen, setZoomOpen] = useState(false);
  const [zoomKind, setZoomKind] = useState<"camera" | "voltage" | "distance" | null>(null);

  // ----- refs para bucles estables (evitar “max depth”) -----
  const runningRef = useRef(true);                 // gobierna timer
  const estopRef = useRef(false);
  const faultRef = useRef(false);
  const finishRef = useRef(false);
  // arriba, junto a otros useRef/useState:
  const splineCanvasRef = useRef<HTMLCanvasElement | null>(null);

  const powerBusyRef = useRef(false);
  const lastStartTsRef = useRef(0);
  const bcRef = useRef<BroadcastChannel | null>(null);


  // mantener refs sincronizadas
  useEffect(() => { estopRef.current = estopActive; }, [estopActive]);
  useEffect(() => { faultRef.current = faultLatched; }, [faultLatched]);
  useEffect(() => { finishRef.current = finishOpen; }, [finishOpen]);

  // ======== Spline (sin blur, sin reinicios) ========
  // usamos onLoad para disparar la tecla S una vez que la escena está lista
  // reemplaza handleSplineLoad por esto:
  const handleSplineLoad = (_splineApp: any) => {
    // Sólo guardamos el canvas y lo hacemos focusable
    const canvas: HTMLCanvasElement | null = document.querySelector("canvas");
    if (canvas) {
      canvas.tabIndex = 0;
      splineCanvasRef.current = canvas;
      setLogs(p => ["[Spline] escena lista", ...p].slice(0, 300));
    }
  };

  // Llama a esto cuando llegue el mensaje "fin de prueba" por MQTT
  function handleMqttTestFinished() {
    const seconds = Math.max(0, Math.floor(elapsedMs / 1000));
    const mm = Math.floor(seconds / 60).toString().padStart(2, "0");
    const ss = (seconds % 60).toString().padStart(2, "0");
    setFinishDesc(`La prueba ha terminado en ${mm}:${ss}.`);
    setFinishOpen(true);
    setLogs(p => ["[FIN DE PRUEBA] recibido por MQTT", ...p].slice(0, 300));
  }

  // ======== Timer con un solo rAF (no dependencias) ========
  useEffect(() => {
    let raf = 0;
    let last: number | null = null;

    const loop = (t: number) => {
      if (finishRef.current || faultRef.current || estopRef.current) {
        last = t; // “pausa”: reanudar sin saltos
      } else {
        if (last == null) last = t;
        const dt = t - last;
        last = t;
        setElapsedMs(ms => ms + dt);
      }
      raf = requestAnimationFrame(loop);
    };

    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, []);
  
  useEffect(() => {
  // VOLTAJE (kV)
  const offV = subscribeSeries(DEVICE_ID, "voltage_kv", (v) => {
    setvoltageSeries(s => {
      const next = [...s, v];
      if (next.length > 600) next.shift();
      return next;
    });
  });

  // DISTANCIA (mm)
  const offD = subscribeSeries(DEVICE_ID, "laser_mm", (v) => {
    setDistanceSeries(s => {
      const next = [...s, v];
      if (next.length > 600) next.shift();
      return next;
    });
  });

  return () => { offV?.(); offD?.(); };
}, []);


  // ======== Cámara ========

  // Frames por MQTT (dataURL base64) + “esperando” a 5s si no llega nada
  useEffect(() => {
    const off = subscribeCamFrame(CAM_DEVICE_ID, (dataUrl) => {
      if (typeof dataUrl === "string" && dataUrl.startsWith("data:image/")) {
        setFrameUrl(dataUrl);
        setCamMqttOk(true);
      }
    });
    const t = setTimeout(() => { if (!frameUrl) setCamMqttOk(false); }, 5000);
    return () => { off?.(); clearTimeout(t); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Fallback mock: solo refresca si no hay frame MQTT, y si no hay estop/falla/fin
  useEffect(() => {
    if (frameUrl) return;
    const id = setInterval(() => {
      if (estopRef.current || faultRef.current || finishRef.current) return;
      setCameraTick(t => t + 1);
    }, cameraRefreshMs);
    return () => clearInterval(id);
  }, [frameUrl, cameraRefreshMs]);


  useEffect(() => {
  sendCommand("modo automatico");
  // (Opcional) suscribirse al estado y reflejar en UI:
  const unsub = subscribeState(DEVICE_ID, (st) => {
    // si quieres, sincroniza estop/fallo/isOn/etapa desde el ESP32
    if (typeof st?.fallo === "boolean") setFaultLatched(st.fallo);
  });
  return () => unsub?.();
  // eslint-disable-next-line react-hooks/exhaustive-deps
}, []);

    // Logs reales desde el ESP32 (tesis/<id>/log/text y ack)
  useEffect(() => {
    const off = subscribeLogs(DEVICE_ID, (line) => {
      setLogs((p) => [line, ...p].slice(0, 300));
    });
    return () => off?.();
  }, []);

  // // ======== Fin de prueba mock (cada ~45s) ========
  // useEffect(() => {
  //   const id = setInterval(() => {
  //     if (!finishRef.current && !faultRef.current) finish();
  //   }, 45000);
  //   return () => clearInterval(id);
  // }, []);

  // ======== Heartbeat (mock) ========
  useEffect(() => {
    const id = setInterval(() => {
      setRs485Ok(true);
      setRs232Ok(true);
    }, 5000);
    return () => clearInterval(id);
  }, []);

  useEffect(() => {
  const off = subscribeFault(DEVICE_ID, (f) => {
    if (!f) return; // llegó "{}" → no abrir popup
    setFaultTitle(`Falla ${f.code ?? "FALLA"}`);
    setFaultDesc(
      `Detalle: ${f.message ?? "Revisar el sistema"}${
        typeof f.etapa === "number" ? `\nEtapa: ${f.etapa}` : ""
      }`
    );
    setFaultLatched(true);
    setFaultOpen(true);
    setShakeTick((t) => t + 1);
    setLogs((p) => [`[FAULT] ${f.code}: ${f.message}`, ...p].slice(0, 300));
  });
  return () => off?.();
}, []);

// coordinar con otras pestañas/instancias
useEffect(() => {
  const bc = new BroadcastChannel("tesis-cmd");
  bcRef.current = bc;

  bc.onmessage = (ev) => {
    if (ev?.data?.type === "cmd" && ev.data.line === "start") {
      // otra instancia acaba de mandar 'start' → marca timestamp local para que esta lo ignore
      lastStartTsRef.current = ev.data.ts || Date.now();
      // si quieres, dispara SOLO la animación Spline aquí para mantener la UI simétrica
      // fireSplineDouble(splineCanvasRef.current, "s");
    }
  };

  return () => bc.close();
}, []);

  // acciones
  const panel = "rounded-2xl border border-white/15 bg-black/30 p-4 shadow-[0_4px_20px_rgba(0,0,0,0.18)]"; // sin blur
  const title = "text-sm font-medium text-white/90";
  const globalDisabled = estopActive || faultLatched || finishOpen;

  const sendCommand = (cmd: string) => {
    if (!cmd.trim()) return;
    const now = new Date().toLocaleTimeString();
    setLogs(p => [`[${now}] → ${cmd}`, ...p].slice(0, 300));
    publishText(DEVICE_ID, cmd);
  };


  // helper reutilizable
function fireSplineKeyPress(canvas: HTMLCanvasElement | null, key = "s") {
    if (!canvas) return;
    const opts = { key, code: "KeyS", bubbles: true };

    // asegúrate que el canvas tenga el foco
    canvas.focus();

    // keydown -> pequeño delay -> keyup
    canvas.dispatchEvent(new KeyboardEvent("keydown", opts));

    // 60–100 ms suele ser suficiente para que el handler de Spline registre la pulsación
    setTimeout(() => {
      canvas.dispatchEvent(new KeyboardEvent("keyup", opts));
    }, 80);
}

function publishStartOnce() {
  const now = Date.now();

  // rate-limit global (compartido entre instancias vía BroadcastChannel)
  if (now - lastStartTsRef.current < 350) {
    // opcional: solo animación, sin mandar otro 'start'
    fireSplineDouble(splineCanvasRef.current, "s");
    setLogs(p => [`[Spline] key 's' ×2 (drop dup start)`, ...p].slice(0, 300));
    return;
  }

  lastStartTsRef.current = now;
  bcRef.current?.postMessage({ type: "cmd", line: "start", ts: now });
  sendCommand("start");
}

function fireSplineDouble(canvas: HTMLCanvasElement | null, key = "s") {
  if (!canvas) return;
  // 1ª pulsación
  fireSplineKeyPress(canvas, key);
  // 2ª pulsación con un pequeño delay (ajústalo si tu Spline lo necesita)
  //setTimeout(() => fireSplineKeyPress(canvas, key), 120);
}

  // Power: NO se deshabilita por estop; la GUI decide.
// Si hay estop, primero enviamos "ok" y luego "start".
const togglePower = () => {
  // Bloqueos duros: falla/fin/estop
  if (faultLatched || finishOpen || estopActive) return;

  // anti-reentrada local (doble click)
  if (powerBusyRef.current) return;
  powerBusyRef.current = true;
  setTimeout(() => { powerBusyRef.current = false; }, 300);

  setIsOn(prev => {
    const next = !prev;

    // ← UN SOLO 'start' para todas las instancias
    publishStartOnce();

    // ← pero tu animación va en doble 's'
    fireSplineDouble(splineCanvasRef.current, "s");

    setLogs(p => [
      `[Spline] key 's' ×2 disparada por ${next ? "POWER ON" : "POWER OFF"}`,
      ...p,
    ].slice(0, 300));

    return next;
  });
};

const toggleEstop = () => {
  if (faultLatched) return;
  if (estopActive) {
    sendCommand("ok");
    setEstopActive(false);            // ⬅️ NO toques isOn aquí
  } else {
    sendCommand("paro");
    setEstopActive(true);             // ⬅️ NO hagas setIsOn(false)
    // ⛔ QUITADO: setIsOn(false)
  }
};

const handleReset = () => {
  if (faultLatched) return;
  setLogs((prev) => ["Reset ejecutado.", ...prev].slice(0, 200));
  sendCommand("reset");
  // (opcional) sin tocar isOn; la GUI sigue mandando
};

const handleFaultReset = () => {
  setFaultOpen(false);
  setFaultLatched(false);
  setLogs((prev) => ["Falla reseteada por operador.", ...prev].slice(0, 200));
  sendCommand("rfalla");
};

  const showFault = (title: string, desc: string) => {
    setFaultLatched(true);
    setFaultTitle(title);
    setFaultDesc(desc);
    setFaultOpen(true);
    setShakeTick(t => t + 1);
    setLogs(p => [`[FAULT] ${title}: ${desc}`, ...p].slice(0, 300));
  };

  const finish = () => {
    const seconds = Math.max(0, Math.floor(elapsedMs / 1000));
    const mm = Math.floor(seconds / 60).toString().padStart(2, "0");
    const ss = (seconds % 60).toString().padStart(2, "0");
    setFinishDesc(`La prueba ha terminado en ${mm}:${ss}.`);
    setFinishOpen(true);
    setLogs(p => ["[FIN DE PRUEBA] OK", ...p].slice(0, 300));
  };

  const fmtTime = (ms: number) => {
    const total = Math.max(0, Math.floor(ms / 1000));
    const hh = Math.floor(total / 3600).toString().padStart(2, "0");
    const mm = Math.floor((total % 3600) / 60).toString().padStart(2, "0");
    const ss = (total % 60).toString().padStart(2, "0");
    return `${hh}:${mm}:${ss}`;
  };

  // zoom handlers
  const openZoom = (kind: "camera" | "voltage" | "distance") => {
    setZoomKind(kind);
    setZoomOpen(true);
  };

  return (
    <main className="relative min-h-dvh">
      {/* ===== Escena Spline: SIN blur, centro libre ===== */}
      <div className="fixed inset-0 -z-10">
        <div className="absolute inset-0">
          <Spline
            scene="https://prod.spline.design/uD-NZs1pTeMUa8bY/scene.splinecode"
            onLoad={handleSplineLoad}
          />

        </div>
        {/* overlay muy leve (sin blur) */}
        <div className="absolute inset-0 pointer-events-none bg-gradient-to-b from-indigo-900/10 via-indigo-900/10 to-indigo-900/20" />
      </div>

      {/* ===== Rails laterales angostos ===== */}
      <div
        className={cn(
          "pointer-events-none fixed inset-0 z-10 grid gap-3 p-3",
          "md:grid-cols-[minmax(240px,300px)_1fr_minmax(240px,300px)]"
        )}
      >
        {/* Columna izquierda */}
        <aside className="pointer-events-auto flex flex-col gap-3">
          <div className={panel}>
            <div className="flex items-center justify-between">
              <h1 className="text-lg font-semibold text-white">Monitoreo</h1>
              <Link
                href="/menu"
                className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-black/30 px-3 py-2 text-xs text-white/90 hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/30"
              >
                <ArrowLeft className="h-4 w-4" />
                Volver
              </Link>
            </div>
            <div className="mt-3 flex items-center justify-between">
              <div>
                <p className={title}>Tiempo de prueba</p>
                <div className="mt-1 text-2xl font-semibold text-white">
                  {fmtTime(elapsedMs)}
                </div>
              </div>
              <div className="flex items-center gap-2">
                <CommLight label="RS485" ok={rs485Ok && !faultLatched} />
                <CommLight label="RS232" ok={rs232Ok && !faultLatched} />
              </div>
            </div>
          </div>

          {/* controles */}
          <div className={panel}>
            <p className={title}>Controles globales</p>
            <div className="mt-3 grid grid-cols-1 gap-2">
              <button
                type="button"
                onClick={togglePower}
                disabled={estopActive || faultLatched || finishOpen}
                className={cn(
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  estopActive || faultLatched || finishOpen
                    ? "cursor-not-allowed opacity-50"
                    : isOn
                    ? "bg-emerald-600 text-white hover:bg-emerald-500"
                    : "border border-white/15 bg-black/30 text-white hover:bg-black/40"
                )}
              >
                <span className="inline-flex items-center gap-2">
                  <Power className="h-4 w-4" />
                  {isOn ? "Apagar" : "Encender"}
                </span>
              </button>


              <button
                type="button"
                onClick={toggleEstop}
                className={cn(
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  estopActive
                    ? "bg-emerald-600 text-white hover:bg-emerald-500"
                    : "bg-rose-600 text-white hover:bg-rose-500"
                )}
                title={estopActive ? "Liberar paro" : "Activar paro de emergencia"}
              >
                {estopActive ? "OK" : "STOP"}
              </button>

              <button
                type="button"
                onClick={handleReset}
                disabled={faultLatched || finishOpen}
                className={cn(
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  estopActive || faultLatched || finishOpen
                    ? "cursor-not-allowed opacity-50"
                    : "border border-white/15 bg-black/30 text-white hover:bg-black/40"
                )}
                title="Reset suave (reinicia timer y limpia logs)"
              >
                <span className="inline-flex items-center gap-2">
                  <RefreshCcw className="h-4 w-4" />
                  Reset
                </span>
              </button>

              <button
                type="button"
                onClick={handleFaultReset}
                disabled={!faultLatched}
                className={cn(
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  !faultLatched
                    ? "cursor-not-allowed opacity-50"
                    : "border border-rose-400/40 bg-rose-700/20 text-rose-100 hover:bg-rose-700/30"
                )}
                title="Libera el latch de fallas (habilita botones)"
              >
                Reset de fallas
              </button>
            </div>
          </div>

          {/* consola */}
          <div className={panel}>
            <div className="mb-3 flex items-center gap-2">
              <TerminalSquare className="h-5 w-5 text-white/80" />
              <p className={title}>Consola (MQTT mock)</p>
            </div>

            <div className="mb-3 flex gap-2">
              <input
                ref={inputRef}
                className="min-w-0 flex-1 rounded-xl border border-white/15 bg-black/30 px-3 py-2 text-white placeholder:text-white/50 focus:outline-none focus:ring-2 focus:ring-white/30"
                placeholder="Escribe un comando y presiona Enviar"
                value={inputLine}
                onChange={(e) => setInputLine(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter" && inputLine.trim()) {
                    sendCommand(inputLine);
                    setInputLine("");
                    inputRef.current?.focus();
                  }
                }}
                disabled={estopActive || faultLatched || finishOpen}
              />
              <button
                onClick={() => {
                  if (!inputLine.trim()) return;
                  sendCommand(inputLine);
                  setInputLine("");
                  inputRef.current?.focus();
                }}
                disabled={estopActive || faultLatched || finishOpen || !inputLine.trim()}
                className={cn(
                  "rounded-xl px-4 py-2 text-sm font-medium transition",
                  estopActive || faultLatched || finishOpen || !inputLine.trim()
                    ? "text-white/50 border border-white/15 bg-black/30"
                    : "text-white border border-white/15 bg-black/30 hover:bg-black/40"
                )}
              >
                Enviar
              </button>
            </div>

            <div className="h-56 overflow-auto rounded-xl border border-white/10 bg-zinc-900/70 p-3">
              <ul className="space-y-1 font-mono text-xs text-zinc-100/90">
                {logs.map((line, i) => <li key={i}>{line}</li>)}
              </ul>
            </div>
          </div>
        </aside>

        {/* Centro vacío */}
        <div className="hidden md:block" />

        {/* Columna derecha */}
        <aside className="pointer-events-auto flex flex-col gap-3">
          {/* cámara */}
          <div className={panel}>
            <div className="mb-3 flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Camera className="h-5 w-5 text-white/80" />
                <p className="text-sm font-medium text-white/90">Cámara (ESP32-CAM)</p>
                <span
                  className={[
                    "ml-2 rounded-lg border px-2 py-0.5 text-[11px]",
                    camMqttOk
                      ? "border-emerald-400/40 bg-emerald-500/20 text-emerald-200"
                      : "border-yellow-400/40 bg-yellow-600/20 text-yellow-100",
                  ].join(" ")}
                  title={camMqttOk ? "Recibiendo frames por MQTT" : "Esperando frames por MQTT"}
                >
                  {camMqttOk ? "MQTT cam: activo" : "MQTT cam: esperando…"}
                </span>
              </div>

              <div className="flex items-center gap-2">
                {/* Zoom abre el modal */}
                <button
                  type="button"
                  onClick={() => openZoom("camera")}
                  className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-black/30 px-3 py-2 text-xs text-white/90 hover:bg-black/40"
                  title="Ampliar cámara"
                >
                  <Camera className="h-4 w-4" />
                  Zoom
                </button>

                {/* LED ON/OFF por MQTT (optimista) */}
                <button
                  type="button"
                  onClick={() => {
                    if (estopActive || faultLatched || finishOpen) return;
                    const next = !camLedOn;
                    setCamLedOn(next);
                    publishCamLed(next, CAM_DEVICE_ID);
                  }}
                  className={[
                    "inline-flex items-center gap-2 rounded-xl px-3 py-2 text-xs font-medium transition",
                    camLedOn
                      ? "bg-amber-500/80 text-black hover:bg-amber-400"
                      : "border border-white/15 bg-black/30 text-white/90 hover:bg-black/40",
                  ].join(" ")}
                  disabled={estopActive || faultLatched || finishOpen}
                  title={camLedOn ? "Apagar LED" : "Encender LED"}
                >
                  {camLedOn ? "LED ON" : "LED OFF"}
                </button>
              </div>
            </div>

            {/* Imagen: MQTT si existe, si no mock; quita rotate si no la necesitas */}
            <button
              type="button"
              onClick={() => openZoom("camera")}
              className="block w-full overflow-hidden rounded-xl border border-white/10 bg-black/60 focus:outline-none"
              title="Ampliar"
            >
              <img
                src={frameUrl ?? cameraUrl}
                alt="Snapshot"
                className="h-auto w-full object-cover rotate-180"
              />
            </button>
            <p className="mt-2 text-xs text-white/70">
              {frameUrl ? "Frames vía MQTT (base64)." : `Snapshot mock cada ~${cameraRefreshMs / 1000}s.`}
            </p>
          </div>


          {/* gráficas */}
          <ChartCard
            title="Voltaje (kV)"
            onZoom={() => openZoom("voltage")}
            series={voltageSeries}
            unit="kV"
            yMin={0}
            yMax={40}
            stroke="#0b4f9c"
          />
          <ChartCard
            title="Distancia (mm)"
            onZoom={() => openZoom("distance")}
            series={distanceSeries}
            unit="mm"
            yMin={0}
            yMax={300}
            stroke="#0b7d3b"
          />

          {/* botón mock falla */}
          <div className="text-right">
            <button
              type="button"
              onClick={() => showFault("Falla E42", "Sobrevoltaje en fuente HV")}
              className="rounded-xl border border-white/15 bg-black/30 px-3 py-2 text-xs text-white/80 hover:bg-black/40"
            >
              Simular Falla (mock)
            </button>
          </div>
        </aside>
      </div>

      {/* ===== Modal Falla ===== */}
      <AlertDialog open={faultOpen} onOpenChange={setFaultOpen}>
        <AlertDialogContent
          className={cn(
            "max-w-md rounded-2xl",
            "border border-rose-400/40 bg-rose-600/20",
            "text-white shadow-[0_8px_30px_rgba(255,0,0,0.35)]",
            "animate-in fade-in duration-300"
          )}
        >
          <div key={shakeTick} className="will-change-transform animate-[dialog-shake-x_0.45s]">
            <AlertDialogHeader>
              <AlertDialogTitle className="text-rose-300 font-semibold tracking-wide">
                {faultTitle || "Falla detectada"}
              </AlertDialogTitle>
              <AlertDialogDescription className="whitespace-pre-line text-rose-100/90">
                {faultDesc || "Revisar el sistema y reintentar."}
              </AlertDialogDescription>
            </AlertDialogHeader>
            <AlertDialogFooter>
              <AlertDialogAction
                onClick={() => setFaultOpen(false)}
                className="rounded-xl border border-rose-400/40 bg-rose-700/20 px-4 py-2 text-rose-100 hover:bg-rose-700/30 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-rose-300/60"
              >
                OK
              </AlertDialogAction>
            </AlertDialogFooter>
          </div>
        </AlertDialogContent>
      </AlertDialog>

      {/* ===== Modal Fin (verde) ===== */}
      <AlertDialog open={finishOpen} onOpenChange={setFinishOpen}>
        <AlertDialogContent
          className={cn(
            "max-w-md rounded-2xl",
            "border border-emerald-400/40 bg-emerald-700/20",
            "text-white shadow-[0_8px_30px_rgba(0,128,0,0.35)]",
            "animate-in fade-in duration-300"
          )}
        >
          <AlertDialogHeader>
            <AlertDialogTitle className="text-emerald-300 font-semibold tracking-wide">
              Prueba finalizada
            </AlertDialogTitle>
            <AlertDialogDescription className="whitespace-pre-line text-emerald-100/90">
              {finishDesc || "La prueba ha terminado."}
            </AlertDialogDescription>
          </AlertDialogHeader>
          <AlertDialogFooter>
            <AlertDialogAction
              onClick={() => router.push("/menu")}
              className="rounded-xl border border-emerald-400/40 bg-emerald-700/20 px-4 py-2 text-emerald-100 hover:bg-emerald-700/30 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-emerald-300/60"
            >
              OK
            </AlertDialogAction>
          </AlertDialogFooter>
        </AlertDialogContent>
      </AlertDialog>

      {/* ===== Modal de Zoom (cámara / charts) ===== */}
      <AlertDialog open={zoomOpen} onOpenChange={setZoomOpen}>
        <AlertDialogContent
          className="max-w-5xl rounded-2xl border border-white/15 bg-black/80 text-white"
        >
          <div className="flex items-center justify-between">
            <AlertDialogTitle className="text-white">
              {zoomKind === "camera" ? "Cámara" : zoomKind === "voltage" ? "Ángulo" : "Distancia"}
            </AlertDialogTitle>
            <button
              className="rounded-md p-1 text-white/80 hover:text-white"
              onClick={() => setZoomOpen(false)}
              aria-label="Cerrar"
            >
              <X className="h-5 w-5" />
            </button>
          </div>

          <div className="mt-4">
            {zoomKind === "camera" && (
              <CameraZoomView src={frameUrl ?? cameraUrl} />
            )}
            {zoomKind === "voltage" && (
              <BigChart
                series={voltageSeries}
                unit="kV"
                yMin={0}
                yMax={40}
                stroke="#0b4f9c"
                title="Voltaje (kV)"
              />
            )}
            {zoomKind === "distance" && (
              <BigChart
                series={distanceSeries}
                unit="mm"
                yMin={0}
                yMax={300}
                stroke="#0b7d3b"
                title="Distancia (mm)"
              />
            )}
          </div>
        </AlertDialogContent>
      </AlertDialog>
    </main>
  );
}

/* ========== Luces de comunicación ========== */
function CommLight({ label, ok }: { label: string; ok: boolean }) {
  return (
    <div className="flex items-center gap-2 rounded-xl border border-white/15 bg-black/30 px-3 py-2">
      <Circle className={cn("h-3 w-3", ok ? "text-emerald-400" : "text-rose-400")} fill="currentColor" />
      <span className={cn("text-sm", ok ? "text-white/90" : "text-rose-200/90")}>
        {label} {ok ? "OK" : "ERROR"}
      </span>
    </div>
  );
}

function CameraZoomView({ src }: { src: string }) {
  const [zoom, setZoom] = useState(1.6);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [drag, setDrag] = useState<{active: boolean; sx: number; sy: number; ox: number; oy: number;}>({
    active: false, sx: 0, sy: 0, ox: 0, oy: 0,
  });

  return (
    <div
      className="relative mx-auto max-h-[70vh] w-full overflow-hidden rounded-xl border border-white/15 bg-black/60"
      onWheel={(e) => {
        e.preventDefault();
        const delta = e.deltaY > 0 ? -0.15 : 0.15;
        setZoom(z => Math.min(4, Math.max(1, z + delta)));
      }}
      onMouseDown={(e) => {
        setDrag({ active: true, sx: e.clientX, sy: e.clientY, ox: pan.x, oy: pan.y });
      }}
      onMouseMove={(e) => {
        if (!drag.active) return;
        setPan({ x: drag.ox + (e.clientX - drag.sx), y: drag.oy + (e.clientY - drag.sy) });
      }}
      onMouseUp={() => setDrag(d => ({ ...d, active: false }))}
      onMouseLeave={() => setDrag(d => ({ ...d, active: false }))}
    >
      {/* HUD */}
      <div className="pointer-events-none absolute right-2 top-2 z-10 flex gap-2">
        <div className="pointer-events-auto rounded-md bg-black/50 px-2 py-1 text-xs text-white/80">
          {zoom.toFixed(2)}×
        </div>
        <button
          className="pointer-events-auto rounded-md bg-black/50 px-2 py-1 text-xs text-white/90 hover:bg-black/60"
          onClick={() => { setZoom(1.6); setPan({ x: 0, y: 0 }); }}
        >
          Reset
        </button>
      </div>

      {/* 🔄 Rotación 180° aislada en un wrapper para no romper el pan/zoom */}
      <div className="rotate-180">
        <img
          src={src}
          alt="Cámara ampliada"
          draggable={false}
          className="select-none"
          style={{
            transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
            transformOrigin: "center center",
            width: "100%",
            height: "auto",
            userSelect: "none",
            pointerEvents: "none",
          }}
        />
      </div>
    </div>
  );
}

/* ========== Card de gráfica pequeña (click para zoom) ========== */
function ChartCard({
  title, series, unit, yMin, yMax, stroke, onZoom,
}: {
  title: string;
  series: number[];
  unit: string;
  yMin: number;
  yMax: number;
  stroke: string;
  onZoom?: () => void;
}) {
  return (
    <div className="rounded-2xl border border-white/15 bg-black/30 p-4 shadow-[0_4px_20px_rgba(0,0,0,0.18)]">
      <div className="mb-2 flex items-center justify-between">
        <p className="text-sm font-medium text-white/90">{title}</p>
        <button
          onClick={onZoom}
          className="rounded-md border border-white/15 bg-black/30 px-2 py-1 text-xs text-white/80 hover:bg-black/40"
          title="Ampliar"
        >
          Zoom
        </button>
      </div>
      <MiniChart series={series} unit={unit} yMin={yMin} yMax={yMax} stroke={stroke} />
    </div>
  );
}

/* ========== MiniChart (fondo blanco, ejes y unidades) ========== */
function MiniChart({
  series, unit, yMin, yMax, stroke,
}: {
  series: number[];
  unit: string;
  yMin: number;
  yMax: number;
  stroke: string;
}) {
  const w = 520, h = 220, padL = 48, padR = 12, padT = 16, padB = 32;
  return (
    <ChartSVG
      series={series}
      unit={unit}
      yMin={yMin}
      yMax={yMax}
      stroke={stroke}
      width={w}
      height={h}
      padL={padL}
      padR={padR}
      padT={padT}
      padB={padB}
    />
  );
}

/* ========== BigChart (para zoom modal) ========== */
function BigChart({
  series, unit, yMin, yMax, stroke, title,
}: {
  series: number[];
  unit: string;
  yMin: number;
  yMax: number;
  stroke: string;
  title: string;
}) {
  const w = 1000, h = 420, padL = 56, padR = 16, padT = 24, padB = 40;
  return (
    <div className="rounded-xl border border-white/15 bg-white p-2">
      <p className="px-2 pt-1 text-sm font-medium text-gray-700">{title}</p>
      <ChartSVG
        series={series}
        unit={unit}
        yMin={yMin}
        yMax={yMax}
        stroke={stroke}
        width={w}
        height={h}
        padL={padL}
        padR={padR}
        padT={padT}
        padB={padB}
      />
    </div>
  );
}

/* ========== ChartSVG base (SVG con ejes/labels) ========== */
function ChartSVG({
  series, unit, yMin, yMax, stroke,
  width, height, padL, padR, padT, padB,
}: {
  series: number[];
  unit: string;
  yMin: number;
  yMax: number;
  stroke: string;
  width: number;
  height: number;
  padL: number;
  padR: number;
  padT: number;
  padB: number;
}) {
  const plotW = width - padL - padR;
  const plotH = height - padT - padB;

  const n = series.length;
  const dtSec = 0.15; // mock
  const xToSec = (i: number) => Math.max(0, (n - 1 - i) * dtSec);

  const yScale = (v: number) => {
    const t = clamp((v - yMin) / Math.max(1e-6, yMax - yMin), 0, 1);
    return padT + (1 - t) * plotH;
  };
  const xScale = (i: number) => padL + (plotW * i) / Math.max(1, n - 1);

  const pts = useMemo(() => {
    if (!n) return "";
    return series.map((v, i) => `${xScale(i)},${yScale(v)}`).join(" ");
  }, [series, plotW, plotH]);

  const yTicks = 5;
  const xTicks = 6;
  const yTickVals = Array.from({ length: yTicks + 1 }, (_, k) => yMin + (k * (yMax - yMin)) / yTicks);
  const xTickIdxs = Array.from({ length: xTicks + 1 }, (_, k) => Math.round((k * (n - 1)) / xTicks));

  return (
    <div className="overflow-hidden rounded-xl border border-white/10 bg-white">
      <svg width="100%" height={height} viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
        {/* fondo */}
        <rect x="0" y="0" width={width} height={height} fill="white" />

        {/* área */}
        <rect x={padL} y={padT} width={plotW} height={plotH} fill="white" stroke="#E5E7EB" />

        {/* ejes */}
        <line x1={padL} y1={padT} x2={padL} y2={padT + plotH} stroke="#9CA3AF" />
        <line x1={padL} y1={padT + plotH} x2={padL + plotW} y2={padT + plotH} stroke="#9CA3AF" />

        {/* grid + labels Y */}
        {yTickVals.map((v, i) => {
          const y = yScale(v);
          return (
            <g key={`y-${i}`}>
              <line x1={padL} y1={y} x2={padL + plotW} y2={y} stroke="#E5E7EB" />
              <text x={padL - 6} y={y + 4} textAnchor="end" fontSize="10" fill="#374151">
                {v.toFixed(0)}
              </text>
            </g>
          );
        })}

        {/* grid + labels X (tiempo) */}
        {xTickIdxs.map((idx, i) => {
          const x = xScale(idx);
          const sec = xToSec(idx);
          return (
            <g key={`x-${i}`}>
              <line x1={x} y1={padT} x2={x} y2={padT + plotH} stroke="#E5E7EB" />
              <text x={x} y={padT + plotH + 14} textAnchor="middle" fontSize="10" fill="#374151">
                {sec.toFixed(0)}s
              </text>
            </g>
          );
        })}

        {/* serie */}
        <polyline points={pts} fill="none" stroke={stroke} strokeWidth="2" />

        {/* unidades */}
        <text x={padL - 34} y={padT - 4} textAnchor="start" fontSize="10" fill="#374151">
          {unit}
        </text>
        <text x={padL + plotW} y={padT + plotH + 26} textAnchor="end" fontSize="10" fill="#374151">
          tiempo (s)
        </text>
      </svg>
    </div>
  );
}
