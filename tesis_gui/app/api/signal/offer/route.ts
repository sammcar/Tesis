// /app/api/signal/offer/route.ts
import { NextResponse } from "next/server";
import { db } from "@/lib/firebase/admin";

export async function POST(req: Request) {
  const { searchParams } = new URL(req.url);
  const deviceId = searchParams.get("deviceId") || "esp32cam";
  const offer = await req.text(); // viene como text/plain
  if (!offer) return new Response("no offer", { status: 400 });

  await db.collection("calls").doc(deviceId).set(
    { offer, ready: true, updatedAt: Date.now() },
    { merge: true }
  );
  return new Response("OK", { status: 200 });
}

export async function GET(req: Request) {
  const { searchParams } = new URL(req.url);
  const deviceId = searchParams.get("deviceId") || "esp32cam";

  const snap = await db.collection("calls").doc(deviceId).get();
  const data = snap.exists ? snap.data() : null;
  const answer = data?.answer || "";
  return new Response(answer, {
    status: 200,
    headers: { "Content-Type": "text/plain; charset=utf-8" },
  });
}
