import { cookies } from "next/headers";
import { adminAuth } from "@/lib/firebase/admin";

export async function getCurrentUser() {
  const session = (await cookies()).get("session")?.value;
  if (!session) return null;
  try {
    return await adminAuth.verifySessionCookie(session, true); // { uid, email, ... }
  } catch {
    return null;
  }
}
