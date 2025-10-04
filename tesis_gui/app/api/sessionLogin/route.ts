import { NextRequest, NextResponse } from "next/server";
import { adminAuth } from "@/lib/firebase/admin";

export async function POST(req: NextRequest) {
  try {
    const { idToken } = await req.json();
    if (!idToken) {
      return NextResponse.json({ error: "Missing idToken" }, { status: 400 });
    }

    const expiresIn = 60 * 60 * 24 * 5 * 1000; // 5 días
    const sessionCookie = await adminAuth.createSessionCookie(idToken, { expiresIn });

    // ❗ Importante: solo secure en producción o si la request viene por https
    const isHttps = req.nextUrl.protocol === "https:";
    const secure = process.env.NODE_ENV === "production" || isHttps;

    const res = NextResponse.json({ ok: true });
    res.cookies.set("session", sessionCookie, {
      httpOnly: true,
      secure,            // <- aquí el fix
      sameSite: "lax",
      path: "/",
      maxAge: expiresIn / 1000,
    });
    return res;
  } catch (e: any) {
    return NextResponse.json({ error: e.message ?? "Unauthorized" }, { status: 401 });
  }
}
