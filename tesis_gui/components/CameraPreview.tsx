"use client";
import Image from "next/image";
import { useEffect, useState } from "react";

export default function CameraPreview() {
  const [src, setSrc] = useState("/placeholder_cam.jpg");

  // Más adelante, esto será una URL de Firebase Storage tipo /latest.jpg?t=Date.now()
  useEffect(() => {
    const id = setInterval(() => {
      setSrc(`/placeholder_cam.jpg?t=${Date.now()}`);
    }, 3000);
    return () => clearInterval(id);
  }, []);

  return (
    <div className="relative aspect-video w-full overflow-hidden rounded-xl bg-black">
      {/* Reemplazaremos este Image por la URL real cuando integremos Firebase */}
      <Image src={src} alt="ESP32-CAM" fill className="object-contain" />
    </div>
  );
}
