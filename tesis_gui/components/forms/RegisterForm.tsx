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
import { createUserWithEmailAndPassword } from "firebase/auth";
import { useRouter } from "next/navigation";

const Schema = z
  .object({
    email: z.string().email({ message: "Correo inválido" }),
    password: z.string().min(6, { message: "Mínimo 6 caracteres" }),
    confirm: z.string().min(6, { message: "Mínimo 6 caracteres" }),
  })
  .refine((d) => d.password === d.confirm, {
    path: ["confirm"],
    message: "Las contraseñas no coinciden",
  });

type FormData = z.infer<typeof Schema>;

export default function RegisterForm() {
  const [isPending, startTransition] = useTransition();
  const router = useRouter();

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<FormData>({
    resolver: zodResolver(Schema),
    defaultValues: { email: "", password: "", confirm: "" },
    mode: "onSubmit",
  });

  function onSubmit(data: FormData) {
    startTransition(async () => {
      try {
        const cred = await createUserWithEmailAndPassword(
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
        router.push("/");
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
          autoComplete="new-password"
          disabled={isPending}
          {...register("password")}
        />
        {errors.password && (
          <p className="text-sm text-red-600">{errors.password.message}</p>
        )}
      </div>

      {/* Confirm */}
      <div className="space-y-2">
        <label className="flex justify-start text-sm font-medium">Confirm Password</label>
        <Input
          placeholder="********"
          type="password"
          autoComplete="new-password"
          disabled={isPending}
          {...register("confirm")}
        />
        {errors.confirm && (
          <p className="text-sm text-red-600">{errors.confirm.message}</p>
        )}
      </div>

      <Button type="submit" disabled={isPending} className="w-full flex gap-2">
        Register
        <Icons.spinner className={cn("animate-spin", { hidden: !isPending })} />
      </Button>
    </form>
  );
}


