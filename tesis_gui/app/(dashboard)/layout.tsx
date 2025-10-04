import type { ReactNode } from "react";
import "@/app/globals.css";
import Link from "next/link";

function Navbar() {
  return (
    <header className="sticky top-0 z-30 w-full border-b border-gray-200 bg-background/80 backdrop-blur">
      <div className="mx-auto flex h-14 max-w-7xl items-center justify-between px-4">
        <Link href="/menu" className="font-semibold">Tesis • Panel</Link>
        <nav className="flex items-center gap-4 text-sm">
          <Link href="/parametros">Parámetros</Link>
          <Link href="/monitoreo">Monitoreo</Link>
          <Link href="/verificar">Verificar</Link>
        </nav>
      </div>
    </header>
  );
}

export default function DashboardLayout({ children }: { children: ReactNode }) {
  return (
    <div className="min-h-screen bg-background text-foreground">
      <Navbar />
      <main className="mx-auto max-w-7xl px-4 py-6">{children}</main>
    </div>
  );
}
