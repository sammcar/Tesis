"use client";
import { useState } from "react";

export default function EstopButton() {
  const [clicked, setClicked] = useState(false);

  return (
    <button
      onClick={() => setClicked(true)}
      className="w-full rounded-xl bg-red-600 px-4 py-3 text-white font-bold hover:bg-red-700 active:scale-95 transition"
      title="Paro de emergencia (mock)"
    >
      {clicked ? "¡E-STOP enviado! (mock)" : "PARO DE EMERGENCIA"}
    </button>
  );
}
