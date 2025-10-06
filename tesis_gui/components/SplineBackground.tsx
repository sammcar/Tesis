// components/SplineBackground.tsx
"use client";
import Spline from "@splinetool/react-spline";

export default function SplineBackground() {
  return (
    <div className="pointer-events-none fixed inset-0 -z-10">
      {/* capa spline (sí recibe eventos) */}
      <div className="pointer-events-auto absolute inset-0">
        <Spline scene="https://prod.spline.design/uD-NZs1pTeMUa8bY/scene.splinecode" />
      </div>

      {/* overlay cromático para uniformar paleta/contraste */}
      <div className="pointer-events-none absolute inset-0 bg-gradient-to-b from-indigo-700/50 via-indigo-800/40 to-indigo-950/60" />
      <div className="pointer-events-none absolute inset-0 bg-[radial-gradient(ellipse_at_top,rgba(255,255,255,0.10),transparent_60%)]" />
    </div>
  );
}
