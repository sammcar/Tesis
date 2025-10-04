"use client";
import { useState } from "react";
import Link from "next/link";

export default function ParametrosPage() {
  const [collectorOn, setCollectorOn] = useState(false);
  const [rpm, setRpm] = useState<number | "">("");
  const [kv, setKv] = useState<number | "">("");
  const [mlmin, setMlmin] = useState<number | "">("");
  const [syringeD, setSyringeD] = useState<number | "">("");
  const [gap, setGap] = useState<number | "">("");
  const [allSent, setAllSent] = useState(false); // luego: se pone true solo tras ACK por MQTT

  const canStart =
    (!collectorOn || rpm !== "") && kv !== "" && mlmin !== "" && syringeD !== "" && gap !== "" && allSent;

  return (
    <section className="space-y-6">
      <h1 className="text-xl font-semibold">Parámetros</h1>

      <div className="grid gap-4 md:grid-cols-2">
        <label className="flex items-center gap-3 rounded-xl border p-4">
          <input type="checkbox" checked={collectorOn} onChange={e => setCollectorOn(e.target.checked)} />
          <span>Colector (activado)</span>
        </label>

        {collectorOn && (
          <label className="rounded-xl border p-4">
            <div className="mb-1 text-sm text-gray-600">RPM deseadas</div>
            <input className="w-full rounded-lg border px-3 py-2" type="number" value={rpm as number | ""} onChange={e => setRpm(Number(e.target.value))} />
            <button className="mt-2 rounded-lg border px-3 py-1 text-sm hover:bg-gray-50"
              onClick={() => setAllSent(true) /* luego: publish MQTT y esperar ACK */}>Enviar RPM</button>
          </label>
        )}

        <label className="rounded-xl border p-4">
          <div className="mb-1 text-sm text-gray-600">Voltaje (kV)</div>
          <input className="w-full rounded-lg border px-3 py-2" type="number" value={kv as number | ""} onChange={e => setKv(Number(e.target.value))} />
          <button className="mt-2 rounded-lg border px-3 py-1 text-sm hover:bg-gray-50" onClick={() => setAllSent(true)}>Enviar kV</button>
        </label>

        <label className="rounded-xl border p-4">
          <div className="mb-1 text-sm text-gray-600">Caudal (ml/min)</div>
          <input className="w-full rounded-lg border px-3 py-2" type="number" value={mlmin as number | ""} onChange={e => setMlmin(Number(e.target.value))} />
          <button className="mt-2 rounded-lg border px-3 py-1 text-sm hover:bg-gray-50" onClick={() => setAllSent(true)}>Enviar caudal</button>
        </label>

        <label className="rounded-xl border p-4">
          <div className="mb-1 text-sm text-gray-600">Diámetro jeringa (mm)</div>
          <input className="w-full rounded-lg border px-3 py-2" type="number" value={syringeD as number | ""} onChange={e => setSyringeD(Number(e.target.value))} />
          <button className="mt-2 rounded-lg border px-3 py-1 text-sm hover:bg-gray-50" onClick={() => setAllSent(true)}>Enviar diámetro</button>
        </label>

        <label className="rounded-xl border p-4">
          <div className="mb-1 text-sm text-gray-600">Distancia (mm)</div>
          <input className="w-full rounded-lg border px-3 py-2" type="number" value={gap as number | ""} onChange={e => setGap(Number(e.target.value))} />
          <button className="mt-2 rounded-lg border px-3 py-1 text-sm hover:bg-gray-50" onClick={() => setAllSent(true)}>Enviar distancia</button>
        </label>
      </div>

      <div className="flex flex-wrap gap-3">
        <button className="rounded-xl border px-4 py-2 hover:bg-gray-50" onClick={() => {
          setCollectorOn(true); setRpm(1200); setKv(15); setMlmin(0.5); setSyringeD(4.7); setGap(120);
        }}>
          Usar recomendados
        </button>

        <button className="rounded-xl border px-4 py-2 hover:bg-gray-50" onClick={() => setAllSent(true)}>
          Enviar todos
        </button>

        <Link href="/menu" className="rounded-xl border px-4 py-2">Atrás</Link>

        <Link
          href={canStart ? "/monitoreo" : "#"}
          className={`rounded-xl px-4 py-2 font-semibold text-white ${canStart ? "bg-green-600 hover:bg-green-700" : "bg-gray-400 cursor-not-allowed"}`}
          aria-disabled={!canStart}
        >
          START
        </Link>
      </div>

      <p className="text-xs text-gray-500">Nota: por ahora “ACK” está simulado con <code>setAllSent(true)</code>. Luego lo haremos real con MQTT.</p>
    </section>
  );
}
