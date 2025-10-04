import "./globals.css";
import type { Metadata } from "next";

export const metadata: Metadata = {
  title: "Tesis Dashboard",
  description: "Interfaz de control y monitoreo",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="es">
      <body>{children}</body>
    </html>
  );
}
