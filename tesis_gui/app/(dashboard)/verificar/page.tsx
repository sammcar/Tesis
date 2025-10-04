import Link from "next/link";

export default function VerificarMenu() {
  return (
    <section className="grid gap-6 md:grid-cols-3">
      <Link href="/verificar/ciclo"  className="rounded-2xl border p-6 hover:bg-gray-50">Verificación por ciclo</Link>
      <Link href="/verificar/etapa"  className="rounded-2xl border p-6 hover:bg-gray-50">Verificación por etapa</Link>
      <Link href="/verificar/manual" className="rounded-2xl border p-6 hover:bg-gray-50">Verificación manual</Link>
    </section>
  );
}
