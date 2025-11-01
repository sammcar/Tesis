// /app/api/signal/ping/route.ts
import { NextResponse } from "next/server";
export async function GET() {
  return NextResponse.json({ ok: true, ts: Date.now() });
}
