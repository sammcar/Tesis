// /app/api/signal/ice/route.ts
import { db } from "@/lib/firebase/admin";

export async function POST(req: Request) {
  const { searchParams } = new URL(req.url);
  const deviceId = searchParams.get("deviceId") || "esp32cam";
  const from = (searchParams.get("from") || "esp").toLowerCase(); // "esp" o "web"

  const body = await req.text();
  if (!body) return new Response("no body", { status: 400 });

  const lines = body.split("\n").map(s => s.trim()).filter(Boolean);
  const batch = db.batch();
  const col = db.collection("calls").doc(deviceId).collection(from === "web" ? "iceFromWeb" : "iceFromEsp");

  for (const line of lines) {
    // hash simple para dedupe: docId = base64(line).slice(0, 150)
    const id = Buffer.from(line).toString("base64").slice(0, 150);
    batch.set(col.doc(id), {
      candidate: line,
      createdAt: Date.now(),
    }, { merge: true });
  }
  await batch.commit();
  return new Response("OK", { status: 200 });
}

export async function GET(req: Request) {
  const { searchParams } = new URL(req.url);
  const deviceId = searchParams.get("deviceId") || "esp32cam";
  const from = (searchParams.get("from") || "web").toLowerCase(); // ¿quién las SUBIÓ?
  const colName = from === "web" ? "iceFromWeb" : "iceFromEsp";
  const col = db.collection("calls").doc(deviceId).collection(colName);

  const snap = await col.orderBy("createdAt", "asc").limit(100).get();
  if (snap.empty) return new Response("", { status: 200, headers: { "Content-Type": "text/plain; charset=utf-8" } });

  // Concatenamos en texto plano (una por línea)
  const lines: string[] = [];
  snap.forEach(d => lines.push(String(d.data().candidate || "").trim()));

  // Opcional: limpiar los ya servidos para no repetir
  const batch = db.batch();
  snap.forEach(d => batch.delete(d.ref));
  await batch.commit();

  return new Response(lines.join("\n"), {
    status: 200,
    headers: { "Content-Type": "text/plain; charset=utf-8" },
  });
}
