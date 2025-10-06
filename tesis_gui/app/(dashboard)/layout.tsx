// app/(dashboard)/layout.tsx
import { redirect } from "next/navigation";
import SplineBackground from "@/components/SplineBackground";
import { getCurrentUser } from "@/lib/server/getCurrentUser";
import { Toaster } from "@/components/ui/toaster"; // 👈 este

import "../globals.css";

export default async function DashboardLayout({ children }: { children: React.ReactNode }) {
  const user = await getCurrentUser();
  if (!user) redirect("/login");

  return (
    <>
      <main className="relative min-h-dvh">
        <SplineBackground />
        <div className="relative z-10">{children}</div>
      </main>
      <Toaster /> {/* 👈 montado una sola vez */}
    </>
  );
}
