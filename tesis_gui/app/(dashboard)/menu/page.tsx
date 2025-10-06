// app/(dashboard)/menu/page.tsx
import Link from "next/link";

export default function MenuPage() {
  return (
    <section className="mx-auto max-w-3xl p-6 md:p-10">
      <h1 className="mb-4 text-2xl font-semibold tracking-tight text-white">
        Menú principal
      </h1>
      <p className="mb-6 text-sm text-zinc-200/80">
        Selecciona una opción para continuar.
      </p>

      <nav aria-label="Secciones del panel">
        <ul className="space-y-4">
          <li>
            <Link
              href="/parametros"
              className="block rounded-2xl border border-white/10 bg-white/8 p-6 backdrop-blur-md transition hover:bg-white/12 focus:outline-none focus:ring-2 focus:ring-white/30"
            >
              <h2 className="text-lg font-semibold text-white">Iniciar Prueba</h2>
              <p className="mt-1 text-sm text-zinc-200/90">
                Configura parámetros y arranca el proceso.
              </p>
            </Link>
          </li>

          <li>
            <Link
              href="/verificar"
              className="block rounded-2xl border border-white/10 bg-white/8 p-6 backdrop-blur-md transition hover:bg-white/12 focus:outline-none focus:ring-2 focus:ring-white/30"
            >
              <h2 className="text-lg font-semibold text-white">Verificar componentes</h2>
              <p className="mt-1 text-sm text-zinc-200/90">
                Ciclo, etapa o verificación manual.
              </p>
            </Link>
          </li>
        </ul>
      </nav>
    </section>
  );
}
