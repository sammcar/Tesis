// app/(dashboard)/verificar/page.tsx
"use client";

import { subscribeLogs, publishText, subscribeState, subscribeFault, subscribeCamFrame, DEVICE_ID } from "@/lib/mqttClient";
import {CAM_DEVICE_ID} from "@/lib/mqttClient";
import { useEffect, useMemo, useRef, useState } from "react";
import Link from "next/link";
import { ArrowLeft, Power, RefreshCcw, Camera, TerminalSquare } from "lucide-react";

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

  const prevModeRef = useRef<Mode>("ciclo");

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
              <button
                type="button"
                onClick={() => !faultLatched && setCameraTick((t) => t + 1)}
                className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-xs text-white/90 hover:bg-white/15"
                disabled={estopActive || faultLatched}
                title="Refrescar snapshot"
              >
                <RefreshCcw className="h-4 w-4" />
                Refrescar
              </button>
            </div>
            <div className="overflow-hidden rounded-xl border border-white/10 bg-black/40">
              <img
                src={frameUrl ?? cameraUrl} // MQTT dataURL o mock
                alt="Stream cámara (ESP32-CAM)"
                className="h-auto w-full object-cover"
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
