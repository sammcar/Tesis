"use client";
import { Button } from "@/components/ui/button";
import { Icons } from "@/components/icons";
import { clientAuth, githubProvider } from "@/lib/firebase/client";
import { signInWithPopup } from "firebase/auth";
import { useRouter, useSearchParams } from "next/navigation";

export default function OAuthForm() {
  const router = useRouter();
  const sp = useSearchParams();
  const next = sp.get("next") ?? "/";

  async function loginWithGithub() {
    try {
      const cred = await signInWithPopup(clientAuth, githubProvider);
      const idToken = await cred.user.getIdToken();
      const r = await fetch("/api/sessionLogin", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ idToken }),
      });
      if (!r.ok) throw new Error("No se pudo crear la sesión");
      router.push(next);
      router.refresh();
    } catch (e: any) {
      console.error(e);
      // opcional: toast destructivo
    }
  }

  return (
    <Button className="w-full" onClick={loginWithGithub}>
      Login With Github
      <Icons.gitHub className="h-6 w-6 mx-3" />
    </Button>
  );
}
