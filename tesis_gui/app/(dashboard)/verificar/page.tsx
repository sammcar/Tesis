// app/(dashboard)/verificar/page.tsx
"use client";

import { subscribeLogs, publishText, subscribeState, subscribeFault, subscribeCamFrame, DEVICE_ID } from "@/lib/mqttClient";
import {CAM_DEVICE_ID} from "@/lib/mqttClient";
import { publishCamLed} from "@/lib/mqttClient";
import { useEffect, useMemo, useRef, useState } from "react";
import Link from "next/link";
import { ArrowLeft, Power, RefreshCcw, Camera, TerminalSquare } from "lucide-react";
import { subscribeSeries } from "@/lib/mqttClient";
import { X } from "lucide-react";


import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";

type Mode = "ciclo" | "etapa" | "manual";

export default function VerificarPage() {
  // ---- Estado global ----
  const [mode, setMode] = useState<Mode>("ciclo");
  const [isOn, setIsOn] = useState<boolean>(false);
  const [estopActive, setEstopActive] = useState<boolean>(false);

  // Consola / variables simuladas
  const [logs, setLogs] = useState<string[]>([
    "Sistema listo.",
    "MQTT: desconectado",
  ]);
  const [inputLine, setInputLine] = useState<string>("");

  // Cámara (mock + MQTT base64)
  const [frameUrl, setFrameUrl] = useState<string | null>(null);
  const [camMqttOk, setCamMqttOk] = useState<boolean>(false);
  const [cameraTick, setCameraTick] = useState<number>(0);
  const cameraRefreshMs = 3000;
  const cameraUrl = useMemo(
    () => `https://picsum.photos/seed/electrospinning-${cameraTick}/960/540`,
    [cameraTick]
  );

  // Falla: modal + latch
  const [faultOpen, setFaultOpen] = useState(false);
  const [faultLatched, setFaultLatched] = useState(false);
  const [faultTitle, setFaultTitle] = useState<string>("");
  const [faultDesc, setFaultDesc] = useState<string>("");
  const [shakeTick, setShakeTick] = useState(0);
  const [camLedOn, setCamLedOn] = useState(false);

  const prevModeRef = useRef<Mode>("ciclo");

  // ----- series (monitoreo rápido) -----
  const [voltageSeries, setVoltageSeries] = useState<number[]>([]);
  const [distanceSeries, setDistanceSeries] = useState<number[]>([]);
  const [zoomOpen, setZoomOpen] = useState(false);
  const [zoomKind, setZoomKind] = useState<"camera" | "voltage" | "distance" | null>(null);


  useEffect(() => {
    const prev = prevModeRef.current;

    // 1) Si salimos de manual → dejar todo en OFF + (opcional) colector off
    if (prev === "manual" && mode !== "manual") {
      ["SERVOHOME","MOTORHOME","VACOFF","SYOFF"].forEach(c => sendCommand(c));
    }

    // 2) Publicar modo actual
    if (mode === "manual") {
      // Entrando a manual: forzar colector ON
      sendCommand("modo manual");
      sendCommand("colector on");
    } else {
      // Ciclo o etapa
      sendCommand(`modo ${mode}`);
    }

    prevModeRef.current = mode;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mode]);

  // === MQTT: frames de cámara (base64) ===
  useEffect(() => {
    const off = subscribeCamFrame(CAM_DEVICE_ID, (dataUrl) => {
      if (typeof dataUrl === "string" && dataUrl.startsWith("data:image/")) {
        setFrameUrl(dataUrl);
        setCamMqttOk(true);
      }
    });
    // Marca “esperando” si no llegó nada en 5s
    const t = setTimeout(() => { if (!frameUrl) setCamMqttOk(false); }, 5000);
    return () => { off?.(); clearTimeout(t); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Refresco de mock si no hay frames reales
  useEffect(() => {
    if (frameUrl) return; // si ya hay frames reales, no refresques el mock
    const id = setInterval(() => {
      if (!faultLatched) setCameraTick((t) => t + 1);
    }, cameraRefreshMs);
    return () => clearInterval(id);
  }, [faultLatched, cameraRefreshMs, frameUrl]);

  // Logs por MQTT
  useEffect(() => {
    const offAck = subscribeLogs(DEVICE_ID, (line) => {
      setLogs(prev => [line, ...prev].slice(0, 200));
    });
    return () => offAck?.();
  }, []);

  // Faults por MQTT
  useEffect(() => {
    const off = subscribeFault(DEVICE_ID, (f) => {
      if (!f) return; // llegó "{}" → no abrir popup ni cambiar nada
      setFaultTitle(`Falla ${f.code ?? "FALLA"}`);
      setFaultDesc(`Detalle: ${f.message ?? "Revisar el sistema"}${typeof f.etapa === "number" ? `\nEtapa: ${f.etapa}` : ""}`);
      setFaultLatched(true);
      setFaultOpen(true);
      setShakeTick((t) => t + 1);
      setLogs((prev) => [`[FAULT] ${f.code}: ${f.message}`, ...prev].slice(0, 200));
    });
    return () => off?.();
  }, []);

  // Series en tiempo real (kV y mm)
useEffect(() => {
  // VOLTAJE (kV)
  const offV = subscribeSeries(DEVICE_ID, "voltage_kv", (v) => {
    setVoltageSeries((s) => {
      const next = [...s, v];
      if (next.length > 600) next.shift();
      return next;
    });
  });

  // DISTANCIA (mm)
  const offD = subscribeSeries(DEVICE_ID, "laser_mm", (v) => {
    setDistanceSeries((s) => {
      const next = [...s, v];
      if (next.length > 600) next.shift();
      return next;
    });
  });

  return () => { offV?.(); offD?.(); };
}, []);


  const sendCommand = (cmd: string) => {
    if (!cmd.trim()) return;
    const now = new Date().toLocaleTimeString();
    setLogs((prev) => [`[${now}] → ${cmd}`, ...prev].slice(0, 200));
    publishText(DEVICE_ID, cmd);
  };

  // Power: NO se deshabilita por estop; la GUI decide.
  const togglePower = () => {
    if (faultLatched) return;           // única condición que bloquea power
    if (estopActive) {
      sendCommand("ok");                // libera paro primero
      setEstopActive(false);            // optimista
    }
    sendCommand("start");
    setIsOn((v) => !v);                 // optimista: la GUI manda
  };

  const toggleEstop = () => {
    if (faultLatched) return;
    if (estopActive) {
      sendCommand("ok");
      setEstopActive(false);            // ⬅️ NO toques isOn aquí
    } else {
      sendCommand("paro");
      setEstopActive(true);             // ⬅️ NO hagas setIsOn(false)
    }
  };

  const handleReset = () => {
    if (faultLatched) return;
    setLogs((prev) => ["Reset ejecutado.", ...prev].slice(0, 200));
    sendCommand("reset");
  };

  const handleFaultReset = () => {
    setFaultOpen(false);
    setFaultLatched(false);
    setLogs((prev) => ["Falla reseteada por operador.", ...prev].slice(0, 200));
    sendCommand("rfalla");
  };

  // Consola
  const inputRef = useRef<HTMLInputElement>(null);
  const onSubmitConsole = () => {
    sendCommand(inputLine);
    setInputLine("");
    inputRef.current?.focus();
  };

  // Deshabilitaciones comunes
  const commonDisabled = faultLatched || estopActive || !isOn;
  const glass = "border border-white/15 bg-white/10 backdrop-blur-lg";
  const card = `rounded-2xl ${glass} p-4 shadow-[0_8px_30px_rgba(0,0,0,0.12)]`;
  const titleClass = "text-sm font-medium text-white/90";

  // justo arriba del `return ( ... )`, junto a tus otros handlers
const openZoom = (kind: "camera" | "voltage" | "distance") => {
  setZoomKind(kind);
  setZoomOpen(true);
};


  return (
    <section className="mx-auto w-full max-w-6xl p-6 md:p-10">
      {/* Header */}
      <div className="mb-6 flex items-center justify-between">
        <h1 className="text-2xl font-semibold tracking-tight text-white">
          Verificar componentes
        </h1>
        <Link
          href="/menu"
          className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 backdrop-blur-md transition hover:bg-white/15 focus:outline-none focus:ring-2 focus:ring-white/30"
        >
          <ArrowLeft className="h-4 w-4" />
          Volver
        </Link>
      </div>

      {/* Selector de modo */}
      <div className={`${card} mb-6`}>
        <p className={titleClass}>Modo de verificación</p>
        <div className="mt-2 inline-flex rounded-xl border border-white/15 bg-white/10 p-1">
          {(["ciclo", "etapa", "manual"] as Mode[]).map((m) => (
            <button
              key={m}
              type="button"
              onClick={() => setMode(m)}
              className={[
                "px-4 py-2 text-sm font-medium rounded-lg transition",
                mode === m
                  ? "bg-white/20 text-white"
                  : "text-white/70 hover:bg-white/10",
              ].join(" ")}
              aria-pressed={mode === m}
              disabled={estopActive || faultLatched}
            >
              {m === "ciclo" ? "Ciclo a ciclo" : m === "etapa" ? "Etapa a etapa" : "Manual"}
            </button>
          ))}
        </div>
      </div>

      {/* Grid principal */}
      <div className="grid grid-cols-1 gap-6 lg:grid-cols-12">
        {/* Izquierda: consola + cámara */}
        <div className="lg:col-span-8 space-y-6">
          {/* Consola */}
          <div className={card}>
            <div className="mb-3 flex items-center gap-2">
              <TerminalSquare className="h-5 w-5 text-white/80" />
              <p className={titleClass}>Consola (MQTT)</p>
            </div>

            <div className="mb-3 flex flex-wrap gap-2 text-xs">
              <span className="rounded-lg border border-white/15 bg-white/10 px-2 py-1 text-white/80">
                Modo: <b className="ml-1 text-white">{mode}</b>
              </span>
              <span className="rounded-lg border border-white/15 bg-white/10 px-2 py-1 text-white/80">
                Energía:{" "}
                <b className={`ml-1 ${isOn ? "text-emerald-300" : "text-white/70"}`}>
                  {isOn ? "ON" : "OFF"}
                </b>
              </span>
              <span className="rounded-lg border border-white/15 bg-white/10 px-2 py-1 text-white/80">
                E-STOP:{" "}
                <b className={`ml-1 ${estopActive ? "text-rose-300" : "text-white/70"}`}>
                  {estopActive ? "ACTIVO" : "LIBRE"}
                </b>
              </span>
              <span className="rounded-lg border border-white/15 bg-white/10 px-2 py-1 text-white/80">
                Falla:{" "}
                <b className={`ml-1 ${faultLatched ? "text-rose-300" : "text-white/70"}`}>
                  {faultLatched ? "LATCHED" : "OK"}
                </b>
              </span>
            </div>

            {/* Input + Enviar */}
            <div className="mb-3 flex gap-2">
              <input
                ref={inputRef}
                className="min-w-0 flex-1 rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-white placeholder:text-white/50 focus:outline-none focus:ring-2 focus:ring-white/30"
                placeholder="Escribe un comando y presiona Enviar"
                value={inputLine}
                onChange={(e) => setInputLine(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter") onSubmitConsole();
                }}
                disabled={faultLatched}
              />
              <button
                onClick={onSubmitConsole}
                disabled={faultLatched || !inputLine.trim()}
                className={[
                  "rounded-xl px-4 py-2 text-sm font-medium transition",
                  "border border-white/15 bg-white/10",
                  faultLatched || !inputLine.trim()
                    ? "text-white/50"
                    : "text-white hover:bg-white/15",
                ].join(" ")}
              >
                Enviar
              </button>
            </div>

            {/* Logs */}
            <div className="h-56 overflow-auto rounded-xl border border-white/10 bg-zinc-900/40 p-3">
              <ul className="space-y-1 font-mono text-xs text-zinc-100/90">
                {logs.map((line, i) => (
                  <li key={i}>{line}</li>
                ))}
              </ul>
            </div>
          </div>

          {/* Cámara */}
          <div className={card}>
            <div className="mb-3 flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Camera className="h-5 w-5 text-white/80" />
                <p className={titleClass}>Cámara (ESP32-CAM)</p>
                {/* Badge MQTT cam */}
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

              {/* Acciones (zoom + LED) */}
              <div className="flex items-center gap-2">
                {/* Nuevo: Zoom cámara */}
                <button
                  type="button"
                  onClick={() => openZoom("camera")}
                  className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-xs text-white/90 hover:bg-white/15"
                  title="Ampliar cámara"
                >
                  <Camera className="h-4 w-4" />
                  Zoom
                </button>

                {/* LED toggle (igual que antes) */}
                <button
                  type="button"
                  onClick={() => {
                    if (estopActive || faultLatched) return;
                    const next = !camLedOn;
                    setCamLedOn(next);                        // optimista
                    publishCamLed(next, CAM_DEVICE_ID);       // cams/<id>/led → "true"/"false"
                  }}
                  className={[
                    "inline-flex items-center gap-2 rounded-xl px-3 py-2 text-xs font-medium transition",
                    camLedOn
                      ? "bg-amber-500/80 text-black hover:bg-amber-400"
                      : "border border-white/15 bg-white/10 text-white/90 hover:bg-white/15",
                  ].join(" ")}
                  disabled={estopActive || faultLatched}
                  title={camLedOn ? "Apagar LED" : "Encender LED"}
                >
                  {camLedOn ? "LED ON" : "LED OFF"}
                </button>
              </div>

            </div>

            <div className="overflow-hidden rounded-xl border border-white/10 bg-black/40">
              <img
                src={frameUrl ?? cameraUrl} // MQTT dataURL o mock
                alt="Stream cámara (ESP32-CAM)"
                className="h-auto w-full object-cover rotate-180"
              />
            </div>
            <p className="mt-2 text-xs text-white/70">
              {frameUrl ? "Frames vía MQTT (base64)." : `Snapshot mock cada ~${cameraRefreshMs / 1000}s.`}
            </p>
          </div>

        </div>

        {/* Derecha: controles y modo */}
        <div className="lg:col-span-4 space-y-6">
          {/* Controles globales */}
          <div className={card}>
            <p className={titleClass}>Controles globales</p>
            <div className="mt-3 grid grid-cols-1 gap-3">
              {/* Power */}
              <button
                type="button"
                onClick={togglePower}
                disabled={faultLatched}
                className={[
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  estopActive || faultLatched
                    ? "cursor-not-allowed opacity-50"
                    : isOn
                    ? "bg-emerald-600 text-white hover:bg-emerald-500"
                    : "border border-white/15 bg-white/10 text-white hover:bg-white/15",
                ].join(" ")}
              >
                <span className="inline-flex items-center gap-2">
                  <Power className="h-4 w-4" />
                  {isOn ? "Apagar" : "Encender"}
                </span>
              </button>

              {/* STOP / OK (deshabilitado durante falla) */}
              <button
                type="button"
                onClick={toggleEstop}
                disabled={faultLatched}
                className={[
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  faultLatched
                    ? "cursor-not-allowed opacity-50"
                    : estopActive
                    ? "bg-emerald-600 text-white hover:bg-emerald-500"
                    : "bg-rose-600 text-white hover:bg-rose-500",
                ].join(" ")}
                title={estopActive ? "Liberar paro" : "Activar paro de emergencia"}
              >
                {estopActive ? "OK" : "STOP"}
              </button>

              {/* Reset normal */}
              <button
                type="button"
                onClick={handleReset}
                disabled={estopActive || faultLatched}
                className={[
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  estopActive || faultLatched
                    ? "cursor-not-allowed opacity-50"
                    : "border border-white/15 bg-white/10 text-white hover:bg-white/15",
                ].join(" ")}
                title="Reset suave (limpia logs)"
              >
                <span className="inline-flex items-center gap-2">
                  <RefreshCcw className="h-4 w-4" />
                  Reset
                </span>
              </button>

              {/* Reset de fallas */}
              <button
                type="button"
                onClick={handleFaultReset}
                disabled={!faultLatched}
                className={[
                  "rounded-xl px-4 py-3 text-sm font-semibold transition",
                  faultLatched
                    ? "border border-rose-400/40 bg-rose-700/20 text-rose-100 hover:bg-rose-700/30"
                    : "cursor-not-allowed opacity-50",
                ].join(" ")}
                title="Liberar latch de falla y re-habilitar controles"
              >
                Reset de fallas
              </button>
            </div>
          </div>

          {/* Acciones por modo */}
          <div className={card}>
            <p className={titleClass}>Acciones del modo</p>
            <div className="mt-3 space-y-3">
              {mode === "ciclo" && (
                <button
                  type="button"
                  onClick={() => sendCommand("clc")}
                  disabled={estopActive || faultLatched || !isOn}
                  className={[
                    "w-full rounded-xl px-4 py-3 text-sm font-semibold transition",
                    estopActive || faultLatched || !isOn
                      ? "cursor-not-allowed opacity-50"
                      : "border border-white/15 bg-white/10 text-white hover:bg-white/15",
                  ].join(" ")}
                >
                  Siguiente ciclo
                </button>
              )}

              {mode === "etapa" && (
                <button
                  type="button"
                  onClick={() => sendCommand("se")}
                  disabled={estopActive || faultLatched || !isOn}
                  className={[
                    "w-full rounded-xl px-4 py-3 text-sm font-semibold transition",
                    estopActive || faultLatched || !isOn
                      ? "cursor-not-allowed opacity-50"
                      : "border border-white/15 bg-white/10 text-white hover:bg-white/15",
                  ].join(" ")}
                >
                  Siguiente etapa
                </button>
              )}

              {mode === "manual" && (
                <div className="grid grid-cols-1 gap-3">
                  <ToggleRow label="Jeringa" disabled={commonDisabled}
                    onToggle={(v) => sendCommand(v ? "SYON" : "SYOFF")} />

                  <ToggleRow label="Alto Voltaje" disabled={commonDisabled}
                    onToggle={(v) => sendCommand(v ? "MOTORMOVE" : "MOTORHOME")} />

                  <ToggleRow label="Giro Colector" disabled={commonDisabled}
                    onToggle={(v) => sendCommand(v ? "VACON" : "VACOFF")} />

                  <ToggleRow label="Actuador Lineal" disabled={commonDisabled}
                    onToggle={(v) => sendCommand(v ? "SERVOMOVE" : "SERVOHOME")} />

                  <div className="mt-2 space-y-2 rounded-xl border border-white/10 bg-white/5 p-3">
                    <CommTestRow
                      label="Comunicación RS232"
                      disabled={estopActive || faultLatched || !isOn}
                      onTest={() => sendCommand("P232")}
                    />
                    <CommTestRow
                      label="Comunicación RS485"
                      disabled={estopActive || faultLatched || !isOn}
                      onTest={() => sendCommand("P485")}
                    />
                  </div>
                </div>
              )}
            </div>
          </div>

          {/* ===== Gráficas de verificación ===== */}
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


          {/* Modal de Falla */}
          <AlertDialog open={faultOpen} onOpenChange={setFaultOpen}>
            <AlertDialogContent
              className={[
                "max-w-md rounded-2xl",
                "border border-rose-400/40 bg-rose-600/20 backdrop-blur-2xl",
                "text-white shadow-[0_8px_30px_rgba(255,0,0,0.35)]",
                "animate-in fade-in duration-300",
              ].join(" ")}
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
        </div>
        {/* ===== Modal de Zoom (cámara / charts) ===== */}
        <AlertDialog open={zoomOpen} onOpenChange={setZoomOpen}>
          <AlertDialogContent className="max-w-5xl rounded-2xl border border-white/15 bg-black/80 text-white">
            <div className="flex items-center justify-between">
              <AlertDialogTitle className="text-white">
                {zoomKind === "camera" ? "Cámara" : zoomKind === "voltage" ? "Voltaje (kV)" : "Distancia (mm)"}
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

      </div>
    </section>
  );
}

/** ToggleRow reutilizable */
function ToggleRow({
  label,
  disabled,
  onToggle,
}: {
  label: string;
  disabled?: boolean;
  onToggle?: (on: boolean) => void;
}) {
  const [on, setOn] = useState(false);
  const glass = "border border-white/15 bg-white/10 backdrop-blur-md";

  const click = (next: boolean) => {
    if (disabled) return;
    setOn(next);
    onToggle?.(next);
  };

  return (
    <div className="flex items-center justify-between gap-3">
      <span className="text-white/90">{label}</span>
      <div className={`inline-flex rounded-xl p-1 ${glass}`}>
        <button
          type="button"
          onClick={() => click(true)}
          disabled={disabled}
          className={[
            "px-4 py-2 text-sm font-medium rounded-lg transition",
            disabled
              ? "text-white/40"
              : on
              ? "bg-white/20 text-white"
              : "text-white/70 hover:bg-white/10",
          ].join(" ")}
          aria-pressed={on}
        >
          On
        </button>
        <button
          type="button"
          onClick={() => click(false)}
          disabled={disabled}
          className={[
            "px-4 py-2 text-sm font-medium rounded-lg transition",
            disabled
              ? "text-white/40"
              : !on
              ? "bg-white/20 text-white"
              : "text-white/70 hover:bg-white/10",
          ].join(" ")}
          aria-pressed={!on}
        >
          Off
        </button>
      </div>
    </div>
  );
}

/** Fila de prueba de comunicación (RS232 / RS485) */
function CommTestRow({
  label,
  disabled,
  onTest,
}: {
  label: string;
  disabled?: boolean;
  onTest?: () => void;
}) {
  return (
    <div className="flex items-center justify-between gap-3">
      <span className="text-white/90">{label}</span>
      <button
        type="button"
        onClick={onTest}
        disabled={!!disabled}
        className={[
          "rounded-xl px-3 py-2 text-xs font-medium transition",
          "border border-white/15 bg-white/10",
          disabled ? "text-white/50 cursor-not-allowed" : "text-white hover:bg-white/15",
        ].join(" ")}
        title="Enviar comando de prueba"
      >
        Probar
      </button>
    </div>
  );
}

// === Cámara con zoom/pan ===
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


// === Card de gráfica pequeña (click → zoom) ===
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
    <div className="rounded-2xl border border-white/15 bg-white/10 p-4 shadow-[0_4px_20px_rgba(0,0,0,0.18)]">
      <div className="mb-2 flex items-center justify-between">
        <p className="text-sm font-medium text-white/90">{title}</p>
        <button
          onClick={onZoom}
          className="rounded-md border border-white/15 bg-white/10 px-2 py-1 text-xs text-white/80 hover:bg-white/15"
          title="Ampliar"
        >
          Zoom
        </button>
      </div>
      <MiniChart series={series} unit={unit} yMin={yMin} yMax={yMax} stroke={stroke} />
    </div>
  );
}

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
  const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));
  const plotW = width - padL - padR;
  const plotH = height - padT - padB;

  const n = series.length;
  const dtSec = 0.15; // mock visual
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
        <rect x="0" y="0" width={width} height={height} fill="white" />
        <rect x={padL} y={padT} width={plotW} height={plotH} fill="white" stroke="#E5E7EB" />
        <line x1={padL} y1={padT} x2={padL} y2={padT + plotH} stroke="#9CA3AF" />
        <line x1={padL} y1={padT + plotH} x2={padL + plotW} y2={padT + plotH} stroke="#9CA3AF" />

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

        <polyline points={pts} fill="none" stroke={stroke} strokeWidth="2" />
        <text x={padL - 34} y={padT - 4} textAnchor="start" fontSize="10" fill="#374151">{unit}</text>
        <text x={padL + plotW} y={padT + plotH + 26} textAnchor="end" fontSize="10" fill="#374151">tiempo (s)</text>
      </svg>
    </div>
  );
}

