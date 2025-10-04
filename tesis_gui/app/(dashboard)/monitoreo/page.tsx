import Card from "@/components/Card";
import EstopButton from "@/components/EstopButton";
import CameraPreview from "@/components/CameraPreview";
import MetricsMiniChart from "@/components/MetricsMiniChart";
// más adelante: import dynamic Hero3D con Spline o R3F

export default function MonitoreoPage() {
  return (
    <div className="space-y-6">
      {/* TODO: Hero3D aquí */}
      <section className="grid grid-cols-1 gap-6 sm:grid-cols-2 lg:grid-cols-4">
        <Card title="Humedad IN"><div className="text-5xl font-semibold">30%</div></Card>
        <Card title="Presión"><div className="text-5xl font-semibold">55 hPa</div></Card>
        <Card title="Caudal"><div className="text-5xl font-semibold">0.5 ml/min</div></Card>
        <Card title="Temperatura OUT"><div className="text-5xl font-semibold">22°C</div></Card>
      </section>

      <section className="grid grid-cols-1 gap-6 lg:grid-cols-3">
        <Card title="Cámara (ESP32-CAM)">
          <CameraPreview />
          <p className="mt-2 text-xs text-gray-500">Snapshot cada ~3s.</p>
        </Card>
        <Card title="Métricas en vivo">
          <MetricsMiniChart />
          <p className="mt-2 text-xs text-gray-500">Luego: MQTT (WSS) → Chart.js/uPlot.</p>
        </Card>
        <Card title="Logs / Notificaciones">
          <ul className="list-disc space-y-1 pl-5 text-sm">
            <li>System ready</li><li>Variac set to 1200 rpm</li><li>Jeringa 0.5 ml/min</li>
          </ul>
          <div className="mt-4"><EstopButton /></div>
        </Card>
      </section>
    </div>
  );
}
