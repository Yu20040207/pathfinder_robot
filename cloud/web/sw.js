/* 明添科技机器人控制 PWA：只缓存外壳，接口/WS 永不缓存 */
const CACHE = 'jc-car-v1';
const SHELL = ['./', './car.html', './manifest.webmanifest',
               './icon-192.png', './icon-512.png', './apple-touch-icon.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(SHELL)).catch(() => {}));
  self.skipWaiting();
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys().then((keys) =>
    Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))));
  self.clients.claim();
});

self.addEventListener('fetch', (e) => {
  const url = new URL(e.request.url);
  // 接口与 WebSocket 一律走网络，不缓存
  if (url.pathname.startsWith('/api/') || url.pathname.startsWith('/ws/')) return;
  if (e.request.mode === 'navigate') {
    // 页面：网络优先，失败回退缓存
    e.respondWith(fetch(e.request).catch(() => caches.match('./car.html')));
    return;
  }
  e.respondWith(caches.match(e.request).then((hit) => hit || fetch(e.request)));
});
