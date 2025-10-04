"use client";
import { getCurrentUser } from "@/lib/server/getCurrentUser";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";

export default async function ProfilePage() {
  const user = await getCurrentUser(); // { uid, email, name?, picture? }
  const email = user?.email ?? "NA";
  const picture = (user as any)?.picture; // si viene de OAuth
  const name = (user as any)?.name ?? "NA";

  return (
    <div className="flex items-center gap-5 justify-center m-10">
      <Avatar className="w-20 h-20">
        <AvatarImage src={picture} />
        <AvatarFallback className="bg-blue-300">AVATAR</AvatarFallback>
      </Avatar>
      <div className="flex flex-col gap-3">
        <h3 className="font-bold">Email:</h3><p className="text-sm">{email}</p>
        <h3 className="font-bold">Full Name:</h3><p className="text-sm">{name}</p>
      </div>
    </div>
  );
}
