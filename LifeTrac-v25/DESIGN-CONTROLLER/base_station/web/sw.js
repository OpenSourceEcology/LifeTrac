/*
 * sw.js — LifeTrac v25 base-station service worker.
 *
 * Enables PWA offline-shell caching so the operator UI loads instantly
 * on a LAN device (Portenta X8) even while the server is initialising,
 * and shows a graceful offline page when the base station is unreachable.
 *
 * HTTPS note
 * ----------
 * Service workers require a secure origin (HTTPS or localhost).
 * On a LAN the simplest options are:
 *   a) Access the UI via http://localhost:8080 when sitting at the
 *      base-station device itself.
 *   b) Generate a self-signed cert and run uvicorn with --ssl-keyfile /
 *      --ssl-certfile (see BASE_STATION.md § HTTPS / PWA setup).
 *   c) Put an HTTPS reverse proxy (nginx, Caddy) in front of this server.
 *
 * Caching strategy
 * ----------------
 *   • Install:    pre-cache the login page and offline fallback.
 *   • Navigation: network-first — the server always validates the session
 *                 cookie; fall back to the offline page on network failure.
 *   • Static JS/CSS/images: cache-first + background revalidation (stale-
 *                 while-revalidate) so subsequent page loads are instant.
 *   • /api/* and /ws/*: never intercepted — always go to the network.
 */

const CACHE_NAME = 'lifetrac-v25-shell-v1';

// Pre-cached app shell.  Only URLs that are publicly reachable without a
// session cookie are listed here so a failed addAll() doesn't abort install.
const SHELL_URLS = [
  '/login',
  '/static/offline.html',
  '/manifest.json',
  '/static/icons/icon.svg',
];

// ---- install: pre-cache the app shell ----------------------------------
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(SHELL_URLS)),
  );
  // Activate immediately without waiting for existing tabs to close.
  self.skipWaiting();
});

// ---- activate: prune stale caches from previous SW versions ------------
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(
        keys
          .filter((k) => k !== CACHE_NAME)
          .map((k) => {
            console.log('[SW] deleting old cache:', k);
            return caches.delete(k);
          }),
      ),
    ),
  );
  // Take control of all open clients immediately (no need to reload).
  self.clients.claim();
});

// ---- fetch: intercept GET requests from our origin ---------------------
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  // Only handle same-origin GET requests; pass everything else straight
  // through (POST, cross-origin, etc.).
  if (request.method !== 'GET' || url.origin !== self.location.origin) {
    return;
  }

  // Never intercept real-time API or WebSocket upgrade requests.
  if (url.pathname.startsWith('/api/') || url.pathname.startsWith('/ws/')) {
    return;
  }

  // Navigation requests (full-page loads): network-first so the server
  // can enforce session auth and redirect to /login as needed.
  // Fall back to the offline page when the network is unreachable.
  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request).catch(() =>
        caches.match('/static/offline.html').then(
          (r) => r || new Response('Offline', { status: 503 }),
        ),
      ),
    );
    return;
  }

  // Static assets (/static/*): cache-first.  On a cache miss the asset
  // is fetched from the network and stored for the next visit.
  event.respondWith(
    caches.match(request).then((cached) => {
      const networkFetch = fetch(request).then((response) => {
        if (response.ok) {
          const clone = response.clone();
          caches.open(CACHE_NAME).then((cache) => cache.put(request, clone));
        }
        return response;
      });
      return cached || networkFetch;
    }),
  );
});
