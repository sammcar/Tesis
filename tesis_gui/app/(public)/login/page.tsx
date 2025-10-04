// app/(public)/login/page.tsx
import React from "react";
import { redirect } from "next/navigation";
import { getCurrentUser } from "@/lib/server/getCurrentUser";
import Spline from "@splinetool/react-spline/next";
import { AuthForm } from "@/components/forms/AuthForm";

export default async function Page({ searchParams }: { searchParams?: { next?: string } }) {
  const user = await getCurrentUser();
  if (user) {
    redirect(searchParams?.next ?? "/menu");
  }
  return (
    <div className="container relative grid h-screen flex-col items-center justify-center lg:max-w-none lg:grid-cols-2 lg:px-0">
      {/* IZQUIERDA: degradado + Spline + branding */}
      <div className="relative hidden h-full flex-col p-10 text-white lg:flex dark:border-r">
        {/* Capa base (degradado) */}
        <div className="absolute inset-0 bg-gradient-to-b from-indigo-700 via-indigo-800 to-indigo-950" />

        {/* Capa 3D (debajo del contenido, encima del degradado) */}
        <div className="absolute inset-0">
          <Spline scene="https://prod.spline.design/uD-NZs1pTeMUa8bY/scene.splinecode" />
        </div>

         {/* Overlay radial SOLO visual (no bloquea mouse) */}
        <div className="absolute inset-0 bg-[radial-gradient(ellipse_at_top,rgba(255,255,255,0.10),transparent_60%)] pointer-events-none" />

        {/* Branding (si no tiene links, también puedes deshabilitar eventos) */}
        <div className="relative z-10 flex items-center gap-3 text-lg font-semibold pointer-events-auto">
          <div className="grid h-10 w-10 place-items-center rounded-xl bg-white/10">TS</div>
          <span>Tesis • Electrospinning</span>
        </div>

        <div className="relative z-10 my-auto pointer-events-none" />
        <div className="relative z-10 pointer-events-none">
          <p className="text-sm/relaxed text-white/70">Panel de control y monitoreo — acceso seguro.</p>
        </div>
      </div>

      {/* DERECHA: formulario (se conserva) */}
      <div className="lg:p-8">
        <div className="mx-auto flex w-full flex-col justify-center space-y-6 sm:w-[380px]">
          <div className="flex flex-col space-y-2 text-center">
            <h1 className="text-2xl font-semibold tracking-tight">Bienvenido</h1>
            <p className="text-sm text-muted-foreground">Inicia sesión para continuar.</p>
          </div>
          <AuthForm />
        </div>
      </div>
    </div>
  );
}



