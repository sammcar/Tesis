"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import * as z from "zod";
import { Input } from "@/components/ui/input";
import { Button } from "@/components/ui/button";
import { cn } from "@/lib/utils";
import { Icons } from "@/components/icons";
import { useTransition } from "react";
import { clientAuth } from "@/lib/firebase/client";
import { signInWithEmailAndPassword } from "firebase/auth";
import { useRouter, useSearchParams } from "next/navigation";

const Schema = z.object({
  email: z.string().email({ message: "Correo inválido" }),
  password: z.string().min(1, { message: "Password requerido" }),
});

type FormData = z.infer<typeof Schema>;

export default function SignInForm() {
  const [isPending, startTransition] = useTransition();
  const router = useRouter();
  const sp = useSearchParams();
  const next = sp.get("next") ?? "/menu";

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<FormData>({
    resolver: zodResolver(Schema),
    defaultValues: { email: "", password: "" },
    mode: "onSubmit",
  });

  function onSubmit(data: FormData) {
    startTransition(async () => {
      try {
        const cred = await signInWithEmailAndPassword(
          clientAuth,
          data.email,
          data.password
        );
        const idToken = await cred.user.getIdToken();
        const r = await fetch("/api/sessionLogin", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ idToken }),
        });
        if (!r.ok) throw new Error("No se pudo crear la sesión");
        router.push(next);
        router.refresh();
      } catch (e) {
        console.error(e);
      }
    });
  }

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="w-full space-y-6">
      {/* Email */}
      <div className="space-y-2">
        <label className="flex justify-start text-sm font-medium">Email</label>
        <Input
          placeholder="example@gmail.com"
          type="email"
          autoComplete="email"
          disabled={isPending}
          {...register("email")}
        />
        {errors.email && (
          <p className="text-sm text-red-600">{errors.email.message}</p>
        )}
      </div>

      {/* Password */}
      <div className="space-y-2">
        <label className="flex justify-start text-sm font-medium">Password</label>
        <Input
          placeholder="********"
          type="password"
          autoComplete="current-password"
          disabled={isPending}
          {...register("password")}
        />
        {errors.password && (
          <p className="text-sm text-red-600">{errors.password.message}</p>
        )}
      </div>

      <Button type="submit" disabled={isPending} className="w-full flex gap-2">
        SignIn
        <Icons.spinner className={cn("animate-spin", { hidden: !isPending })} />
      </Button>
    </form>
  );
}
