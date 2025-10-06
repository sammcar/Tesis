"use client";

import { useEffect, useRef, useState } from "react";
import Link from "next/link";
import { useToast } from "@/hooks/use-toast"
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";


/** === Valores recomendados EDITABLES === */
const PRESET = {
  collectorOn: true,
  rpm: 1,
  kv: 2,
  mlmin: 3,
  syringeD: 4,
  gap: 5,
};

type StrVal = string;

export default function ParametrosPage() {
  const { toast } = useToast();

  // Estado de “borrador” (inputs)
  const [collectorOn, setCollectorOn] = useState<boolean>(false);
  const [rpm, setRpm] = useState<StrVal>("0.0");
  const [kv, setKv] = useState<StrVal>("0.0");
  const [mlmin, setMlmin] = useState<StrVal>("0.0");
  const [syringeD, setSyringeD] = useState<StrVal>("0.0");
  const [gap, setGap] = useState<StrVal>("0.0");

  // Flags de “enviado” (confirma al pulsar Enviar)
  const [mlminSent, setMlminSent] = useState(false);
  const [kvSent, setKvSent] = useState(false);
  const [syringeDSent, setSyringeDSent] = useState(false);
  const [gapSent, setGapSent] = useState(false);
  const [variacSent, setVariacSent] = useState(false);

  // Feedback visual para colector si falta confirmarlo al enviar RPM o Todo
  const [collectorError, setCollectorError] = useState(false);

  // Modal de error centrado (glass)
  const [errorOpen, setErrorOpen] = useState(false);
  const [errorTitle, setErrorTitle] = useState<string>("");
  const [errorDesc, setErrorDesc] = useState<string>("");
  const [shakeTick, setShakeTick] = useState(0);

  const dialogRef = useRef<HTMLDivElement>(null);

  function showError(title: string, description: string) {
    setErrorTitle(title);
    setErrorDesc(description);

    // Si el modal ya está abierto, forzamos el shake aquí mismo
    const el = dialogRef.current;
    if (el) {
      el.classList.remove("animate-dialog-shake");
      // @ts-ignore - forzamos reflow
      void el.offsetWidth;
      el.classList.add("animate-dialog-shake");
    }

    // Si no está abierto, lo abrimos; el useEffect lo shakerá al montar
    setErrorOpen(true);
  }


  // START: todo enviado; si colector está en “Sí”, también rpm
  const canStart =
    kvSent &&
    mlminSent &&
    syringeDSent &&
    gapSent &&
    (!collectorOn || variacSent);

  // ---- estilos de inputs ----
  const inputBase =
    "min-w-0 rounded-xl border border-white/15 bg-white/10 px-3 py-2 focus:outline-none focus:ring-2 focus:ring-white/30 placeholder:text-white/50";
  const inputClass = (sent: boolean) => `${inputBase} ${sent ? "!text-rose-300" : "text-white"}`;

  // ---- helpers ----
  function parseAndValidate(raw: string, label: string): number | null {
    const v = Number(String(raw).trim().replace(",", "."));
    if (!Number.isFinite(v) || v < 0) {
      showError(
        "Valor inválido",
        `${label}: ingresa un número ≥ 0 (puedes usar coma o punto).`
      );
      return null;
    }
    return v;
}

  // ---- Envíos individuales (con toasts) ----
  const sendMlmin = () => {
    const v = parseAndValidate(mlmin, "Jeringa/Bomba (ml/min)");
    if (v === null) return;
    setMlminSent(true);
    toast({ title: "Caudal enviado", description: `${v} ml/min`, className: "border-emerald-400/50 bg-emerald-500/15", });
    // TODO: MQTT con v
  };

  const sendKv = () => {
    const v = parseAndValidate(kv, "Fuente de alto Voltaje (kV)");
    if (v === null) return;
    setKvSent(true);
    toast({ title: "Voltaje enviado", description: `${v} kV`,className: "border-emerald-400/50 bg-emerald-500/15", });
  };

  const sendSyringeD = () => {
    const v = parseAndValidate(syringeD, "Diámetro jeringa (mm)");
    if (v === null) return;
    setSyringeDSent(true);
    toast({ title: "Diámetro enviado", description: `${v} mm`, className: "border-emerald-400/50 bg-emerald-500/15", });
  };

  const sendGap = () => {
    const v = parseAndValidate(gap, "Distancia (mm)");
    if (v === null) return;
    setGapSent(true);
    toast({ title: "Distancia enviada", description: `${v} mm`, className: "border-emerald-400/50 bg-emerald-500/15", });
  };

  const sendVariac = () => {
    // Colector se envía/valida en conjunto con RPM
    if (!collectorOn) {
      setVariacSent(false);
      setCollectorError(true); // resalta selector
      toast({
        variant: "destructive",
        title: "Colector desactivado",
        description: "Activa el colector (Sí) para usar RPM.",
      });
      return;
    }
    const v = parseAndValidate(rpm, "Variador trifásico (rpm)");
    if (v === null) {
      setVariacSent(false);
      return;
    }
    setCollectorError(false);
    setVariacSent(true);
    toast({ title: "RPM enviadas", description: `${v} rpm (colector: Sí)`, className: "border-emerald-400/50 bg-emerald-500/15", });
    // TODO: MQTT con { collectorOn: true, rpm: v }
  };

  // ---- Enviar todo ----
  const sendAll = () => {
    const errors: string[] = [];

    const okMlmin = parseAndValidate(mlmin, "Jeringa/Bomba (ml/min)");
    if (okMlmin === null) errors.push("Jeringa/Bomba (ml/min)"); else setMlminSent(true);

    const okKv = parseAndValidate(kv, "Fuente de alto Voltaje (kV)");
    if (okKv === null) errors.push("Fuente de alto Voltaje (kV)"); else setKvSent(true);

    const okSyr = parseAndValidate(syringeD, "Diámetro jeringa (mm)");
    if (okSyr === null) errors.push("Diámetro jeringa (mm)"); else setSyringeDSent(true);

    const okGap = parseAndValidate(gap, "Distancia (mm)");
    if (okGap === null) errors.push("Distancia (mm)"); else setGapSent(true);

    // Colector viaja junto (si está en Sí, exige RPM)
    if (collectorOn) {
      const okRpm = parseAndValidate(rpm, "Variador trifásico (rpm)");
      if (okRpm === null) {
        errors.push("Variador trifásico (rpm)");
        setVariacSent(false);
      } else {
        setVariacSent(true);
      }
      setCollectorError(false);
    } else {
      // Colector en “No”: válido, no se piden RPM
      setCollectorError(false);
      setVariacSent(false);
    }

    if (errors.length) {
      showError(
        "Revisa los parámetros",
        `Campos con error:\n- ${errors.join("\n- ")}`
      );
      return;
    }

    toast({
      title: "Todos enviados",
      description: `Colector: ${collectorOn ? "Sí" : "No"}${collectorOn ? `` : ""}`,
      className: "border-emerald-400/50 bg-emerald-500/15"});
    // TODO: MQTT en lote con { collectorOn, kv, mlmin, syringeD, gap, rpm? }
  };

  // ---- Edit handlers (reset “enviado”) ----
  const onEditMlmin = (v: string) => { setMlmin(v); setMlminSent(false); };
  const onEditKv = (v: string) => { setKv(v); setKvSent(false); };
  const onEditSyringeD = (v: string) => { setSyringeD(v); setSyringeDSent(false); };
  const onEditGap = (v: string) => { setGap(v); setGapSent(false); };
  const onEditRpm = (v: string) => { setRpm(v); setVariacSent(false); };

  // Cambiar colector: limpiar error y estado de variador
  const toggleCollector = (v: boolean) => {
    setCollectorOn(v);
    setCollectorError(false);
    setVariacSent(false);
  };

  // Presets
  const applyPreset = () => {
    toggleCollector(PRESET.collectorOn);
    setRpm(String(PRESET.rpm));
    setKv(String(PRESET.kv));
    setMlmin(String(PRESET.mlmin));
    setSyringeD(String(PRESET.syringeD));
    setGap(String(PRESET.gap));

    setMlminSent(false);
    setKvSent(false);
    setSyringeDSent(false);
    setGapSent(false);
    setVariacSent(false);
  };

  return (
    <section className="mx-auto w-full max-w-3xl p-6 md:p-10">
      {/* Encabezado + Atrás */}
      <div className="mb-6 flex items-center justify-between">
        <h1 className="text-2xl font-semibold tracking-tight text-white">Parámetros</h1>
        <Link
          href="/menu"
          className="inline-flex items-center gap-2 rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 backdrop-blur-md transition hover:bg-white/15 focus:outline-none focus:ring-2 focus:ring-white/30"
        >
          ← Atrás
        </Link>
      </div>

      {/* Tarjeta principal (glass) */}
      <div className="rounded-3xl border border-white/12 bg-white/6 p-5 backdrop-blur-lg shadow-[0_8px_30px_rgba(0,0,0,0.12)]">
        {/* Título: Tipo de muestra */}
        <div className="mb-4">
          <span className="inline-block rounded-xl border border-white/15 bg-white/10 px-4 py-2 text-sm font-medium text-white/90 backdrop-blur-md">
            Tipo de muestra
          </span>
        </div>

        {/* Colector: Sí/No (sin botón enviar) */}
        <div className="mb-6">
          <span className="mb-2 block text-sm font-medium text-white/90">¿Se usará Colector?</span>
          <div
            className={[
              "inline-flex rounded-xl border p-1 backdrop-blur-md",
              collectorError ? "border-rose-400 ring-2 ring-rose-400/60" : "border-white/15",
              "bg-white/10",
            ].join(" ")}
          >
            <button
              type="button"
              onClick={() => toggleCollector(true)}
              className={`px-4 py-2 text-sm font-medium rounded-lg transition ${
                collectorOn ? "bg-white/20 text-white" : "text-white/70 hover:bg-white/10"
              }`}
              aria-pressed={collectorOn}
            >
              Sí
            </button>
            <button
              type="button"
              onClick={() => toggleCollector(false)}
              className={`px-4 py-2 text-sm font-medium rounded-lg transition ${
                !collectorOn ? "bg-white/20 text-white" : "text-white/70 hover:bg-white/10"
              }`}
              aria-pressed={!collectorOn}
            >
              No
            </button>
          </div>
          {collectorError && (
            <p className="mt-2 text-xs text-rose-300">Debes activar el colector para enviar RPM.</p>
          )}
        </div>

        {/* Título: Parámetros */}
        <div className="mb-4">
          <span className="inline-block rounded-xl border border-white/15 bg-white/10 px-4 py-2 text-sm font-medium text-white/90 backdrop-blur-md">
            Parámetros
          </span>
        </div>

        {/* Filas */}
        <div className="space-y-3">
          {/* Jeringa/Bomba */}
          <div className="grid grid-cols-[1fr_auto_auto_auto] items-center gap-3">
            <label className="text-white/90">Jeringa/Bomba</label>
            <input
              className={inputClass(mlminSent)}
              type="number" step="any" min="0" inputMode="decimal"
              value={mlmin}
              onChange={(e) => onEditMlmin(e.target.value)}
              placeholder="0.0"
            />
            <span className="text-sm text-white/80">ml/min</span>
            <button className="rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 hover:bg-white/15" onClick={sendMlmin}>
              Enviar
            </button>
          </div>

          {/* Voltaje */}
          <div className="grid grid-cols-[1fr_auto_auto_auto] items-center gap-3">
            <label className="text-white/90">Fuente de alto Voltaje</label>
            <input
              className={inputClass(kvSent)}
              type="number" step="any" min="0" inputMode="decimal"
              value={kv}
              onChange={(e) => onEditKv(e.target.value)}
              placeholder="0.0"
            />
            <span className="text-sm text-white/80">kV</span>
            <button className="rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 hover:bg-white/15" onClick={sendKv}>
              Enviar
            </button>
          </div>

          {/* Variador (si colector = Sí) */}
          {collectorOn && (
            <div className="grid grid-cols-[1fr_auto_auto_auto] items-center gap-3">
              <label className="text-white/90">Variador trifásico</label>
              <input
                className={inputClass(variacSent)}
                type="number" step="any" min="0" inputMode="decimal"
                value={rpm}
                onChange={(e) => onEditRpm(e.target.value)}
                placeholder="0.0"
              />
              <span className="text-sm text-white/80">rpm</span>
              <button className="rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 hover:bg-white/15" onClick={sendVariac}>
                Enviar
              </button>
            </div>
          )}

          {/* Diámetro jeringa */}
          <div className="grid grid-cols-[1fr_auto_auto_auto] items-center gap-3">
            <label className="text-white/90">Diámetro jeringa</label>
            <input
              className={inputClass(syringeDSent)}
              type="number" step="any" min="0" inputMode="decimal"
              value={syringeD}
              onChange={(e) => onEditSyringeD(e.target.value)}
              placeholder="0.0"
            />
            <span className="text-sm text-white/80">mm</span>
            <button className="rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 hover:bg-white/15" onClick={sendSyringeD}>
              Enviar
            </button>
          </div>

          {/* Distancia */}
          <div className="grid grid-cols-[1fr_auto_auto_auto] items-center gap-3">
            <label className="text-white/90">Distancia</label>
            <input
              className={inputClass(gapSent)}
              type="number" step="any" min="0" inputMode="decimal"
              value={gap}
              onChange={(e) => onEditGap(e.target.value)}
              placeholder="0.0"
            />
            <span className="text-sm text-white/80">mm</span>
            <button className="rounded-xl border border-white/15 bg-white/10 px-3 py-2 text-sm text-white/90 hover:bg-white/15" onClick={sendGap}>
              Enviar
            </button>
          </div>
        </div>

        {/* Enviar todo */}
        <div className="mt-6 grid place-items-center">
          <button
            className="w-full max-w-xs rounded-2xl border border-white/15 bg-white/10 px-5 py-3 text-sm font-medium text-white hover:bg-white/15 focus:outline-none focus:ring-2 focus:ring-white/30"
            onClick={sendAll}
          >
            Enviar todo
          </button>
        </div>
      </div>

      {/* Acciones inferiores */}
      <div className="mt-6 flex flex-wrap items-center gap-3">
        <button
          className="rounded-xl border border-white/15 bg-white/10 px-4 py-2 text-white/90 hover:bg-white/15"
          onClick={applyPreset}
        >
          Usar recomendados
        </button>

        <Link
          href={canStart ? "/monitoreo" : "#"}
          className={`rounded-xl px-4 py-2 font-semibold text-white transition ${canStart ? "bg-green-600 hover:bg-green-700" : "bg-white/15 text-white/60 cursor-not-allowed"}`}
          aria-disabled={!canStart}
        >
          START
        </Link>
      </div>
      {/* Modal de error centrado (glass) */}
      <AlertDialog open={errorOpen} onOpenChange={setErrorOpen}>
      <AlertDialogContent
        className={[
          "max-w-md rounded-2xl",
          // 💡 color rojizo translúcido
          "border border-rose-400/40 bg-rose-600/20 backdrop-blur-2xl",
          // sombra y transición suaves
          "text-white shadow-[0_8px_30px_rgba(255,0,0,0.35)]",
          "animate-in fade-in duration-300",
        ].join(" ")}
      >
        {/* ⬇️ wrapper interno que se vuelve a montar y ANIMA SIEMPRE */}
        <div
          key={shakeTick}
          ref={dialogRef}
          className="will-change-transform animate-[dialog-shake-x_0.45s]"
        >
          <AlertDialogHeader>
            <AlertDialogTitle className="text-rose-300 font-semibold tracking-wide">
              {errorTitle || "Error"}
            </AlertDialogTitle>
            <AlertDialogDescription className="whitespace-pre-line text-rose-100/90">
              {errorDesc}
            </AlertDialogDescription>
          </AlertDialogHeader>

          <AlertDialogFooter>
            <AlertDialogAction
              onClick={() => setErrorOpen(false)}
              className="rounded-xl border border-rose-400/40 bg-rose-700/20 px-4 py-2 text-rose-100 hover:bg-rose-700/30 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-rose-300/60"
            >
              OK
            </AlertDialogAction>
          </AlertDialogFooter>
        </div>
      </AlertDialogContent>
    </AlertDialog>


    </section>
  );
}
