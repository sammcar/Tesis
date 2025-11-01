"use client";

import { startTransition, useTransition } from "react";
import { Icons } from "../icons";
import { Button } from "../ui/button";
import { Input } from "../ui/input";
import { toast } from "../ui/use-toast";
import * as z from "zod";
import {
  Form,
  FormControl,
  FormDescription,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from "../ui/form";
import { cn } from "@/lib/utils";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";

// ✅ Schema actualizado y compatible
const FormSchema = z.object({
  entityType: z.string().min(1, "entityType is required"),
  entityId: z.string().min(1, "entityId is required"),
  keys: z.string().optional(),
  useStrictDataTypes: z.string().optional(),
});

export const SettingForm = () => {
  const [isPending, startTransition] = useTransition();

  const form = useForm<z.infer<typeof FormSchema>>({
    resolver: zodResolver(FormSchema),
    defaultValues: {
      entityType: "",
      entityId: "",
      keys: "",
      useStrictDataTypes: "false",
    },
  });

  function onSubmit(data: z.infer<typeof FormSchema>) {
    startTransition(async () => {
      try {
        // 🔹 Aquí puedes reemplazar esto por tu lógica real
        // Ejemplo: publicar a MQTT, guardar en Firestore, etc.
        console.log("Setting data:", data);

        // Simula éxito
        toast({
          title: "Setting submitted successfully",
          description: (
            <pre className="mt-2 w-[340px] rounded-md bg-slate-950 p-4">
              <code className="text-white">
                {JSON.stringify(data, null, 2)}
              </code>
            </pre>
          ),
        });
      } catch (e: any) {
        console.error(e);
        toast({
          variant: "destructive",
          title: "Error submitting settings",
          description: e.message ?? "Unknown error",
        });
      }
    });
  }

  return (
    <Form {...form}>
      <form
        onSubmit={form.handleSubmit(onSubmit)}
        className="w-full space-y-6 px-3 md:px-0"
      >
        {/* === Entity Type === */}
        <FormField
          control={form.control}
          name="entityType"
          render={({ field }) => (
            <FormItem>
              <FormLabel className="flex justify-start">EntityType*</FormLabel>
              <FormControl>
                <Input
                  className="border-blue-950"
                  placeholder="DEVICE"
                  {...field}
                />
              </FormControl>
              <FormDescription>
                A string representing the entity type, e.g. 'DEVICE'.
              </FormDescription>
              <FormMessage className="flex justify-start" />
            </FormItem>
          )}
        />

        {/* === Entity ID === */}
        <FormField
          control={form.control}
          name="entityId"
          render={({ field }) => (
            <FormItem>
              <FormLabel className="flex justify-start">EntityId*</FormLabel>
              <FormControl>
                <Input
                  className="border-blue-950"
                  placeholder="784f394c-42b6-435a-983c-b7beff2784f9"
                  {...field}
                />
              </FormControl>
              <FormDescription>
                Unique ID of the entity, e.g.
                '784f394c-42b6-435a-983c-b7beff2784f9'.
              </FormDescription>
              <FormMessage className="flex justify-start" />
            </FormItem>
          )}
        />

        {/* === Optional fields === */}
        <FormField
          control={form.control}
          name="keys"
          render={({ field }) => (
            <FormItem>
              <FormLabel className="flex justify-start">Keys</FormLabel>
              <FormControl>
                <Input
                  className="border-blue-950"
                  placeholder="Optional key string"
                  {...field}
                />
              </FormControl>
              <FormDescription>Optional string of keys</FormDescription>
              <FormMessage className="flex justify-start" />
            </FormItem>
          )}
        />

        <FormField
          control={form.control}
          name="useStrictDataTypes"
          render={({ field }) => (
            <FormItem>
              <FormLabel className="flex justify-start">
                Use Strict Data Types
              </FormLabel>
              <FormControl>
                <Input
                  className="border-blue-950"
                  placeholder="true or false"
                  {...field}
                />
              </FormControl>
              <FormDescription>
                Whether to enforce strict data types (optional).
              </FormDescription>
              <FormMessage className="flex justify-start" />
            </FormItem>
          )}
        />

        <Button type="submit" className="w-full flex gap-2">
          Save Settings
          <Icons.spinner
            className={cn("animate-spin", { hidden: !isPending })}
          />
        </Button>
      </form>
    </Form>
  );
};
