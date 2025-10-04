import { NextRequest, NextResponse } from "next/server";

const PUBLIC = ["/", "/login", "/api/sessionLogin", "/api/sessionLogout"];

export function middleware(req: NextRequest) {
  const { pathname } = req.nextUrl;

  if (PUBLIC.some(p => pathname.startsWith(p) || pathname.startsWith("/_next"))) {
    return NextResponse.next();
  }

  const session = req.cookies.get("session")?.value;
  if (!session) {
    return NextResponse.redirect(new URL("/login", req.url));
  }

  return NextResponse.next();
}

export const config = {
  matcher: ["/((?!api/sessionLogin|api/sessionLogout|login|_next|favicon.ico).*)"],
};
