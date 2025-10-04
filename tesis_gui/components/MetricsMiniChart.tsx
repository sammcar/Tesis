"use client";
import { useEffect, useRef } from "react";

// Mini “gráfica” canvas mock (luego la cambiamos por Chart.js/uPlot + datos via MQTT)
export default function MetricsMiniChart() {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d")!;
    let t = 0;
    const draw = () => {
      const W = canvas.width = canvas.clientWidth * devicePixelRatio;
      const H = canvas.height = canvas.clientHeight * devicePixelRatio;
      ctx.clearRect(0, 0, W, H);
      ctx.beginPath();
      for (let x = 0; x < W; x++) {
        const y = H/2 + Math.sin((x + t) * 0.02) * (H/4);
        x === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      }
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 3;
      ctx.stroke();
      t += 2;
      requestAnimationFrame(draw);
    };
    draw();
  }, []);

  return <canvas ref={ref} className="h-40 w-full" />;
}
