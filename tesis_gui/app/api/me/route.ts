import { NextResponse } from "next/server";
import { cookies } from "next/headers";
import { adminAuth } from "@/lib/firebase/admin";

export async function GET() {
  const session = (await cookies()).get("session")?.value;
  if (!session) return NextResponse.json({ user: null });
  try {
    const decoded = await adminAuth.verifySessionCookie(session, true);
    return NextResponse.json({ user: decoded }); // { uid, email, name?, picture? }
  } catch {
    return NextResponse.json({ user: null });
  }
}
