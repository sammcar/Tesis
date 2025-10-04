import Link from "next/link";

export default function MenuPage() {
  return (
    <section className="grid gap-6 md:grid-cols-2">
      <Link href="/parametros" className="rounded-2xl border p-6 hover:bg-gray-50">
        <h2 className="text-lg font-semibold">Iniciar Prueba</h2>
        <p className="text-sm text-gray-600">Configura parámetros y arranca el proceso.</p>
      </Link>

      <Link href="/verificar" className="rounded-2xl border p-6 hover:bg-gray-50">
        <h2 className="text-lg font-semibold">Verificar componentes</h2>
        <p className="text-sm text-gray-600">Ciclo, etapa o verificación manual.</p>
      </Link>
    </section>
  );
}
