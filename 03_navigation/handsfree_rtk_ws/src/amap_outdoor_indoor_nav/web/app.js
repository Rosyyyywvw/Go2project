/* Read-only ROS telemetry and AMap walking previews. No robot commands are sent. */
"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const defaults = { amap_key: "", amap_security_js_code: "", poll_interval_ms: 1000, stale_after_sec: 3 };
  const storageKey = "go2w-amap-session-config";
  let config = { ...defaults };
  let payload = null;
  let receivedAt = 0;
  let apiOnline = false;
  let hasConnected = false;
  let connectionAttempted = false;
  let map = null;
  let robotMarker = null;
  let destinationMarker = null;
  let trackLine = null;
  let routeLine = null;
  let walking = null;
  let destination = null;
  let following = true;
  let hasCentered = false;
  let mapLoadGeneration = 0;
  let routeGeneration = 0;
  let routePending = false;
  let routeReady = false;
  let lastGcjFresh = false;
  let sdkLoaderPromise = null;
  let activeCredentials = null;

  function text(id, value) { $(id).textContent = value; }
  function number(value) { return typeof value === "number" && Number.isFinite(value); }
  function format(value, digits = 2) { return number(value) ? value.toFixed(digits) : "—"; }
  function coordinates(point) {
    return point && number(point.longitude) && number(point.latitude) &&
      point.longitude >= -180 && point.longitude <= 180 && point.latitude >= -90 && point.latitude <= 90;
  }
  function timestampNow() {
    return payload && number(payload.server_time) ? payload.server_time + (performance.now() - receivedAt) / 1000 : Date.now() / 1000;
  }
  function age(point) { return point && number(point.received_at) ? Math.max(0, timestampNow() - point.received_at) : Infinity; }
  function fresh(point) { return age(point) <= config.stale_after_sec; }
  function validFix(point) { return coordinates(point) && number(point.status) && point.status >= 0; }
  function receiverHasNoFix() { return fresh(payload?.quality) && payload.quality.gga_quality === 0; }
  function gcjAvailable() { return apiOnline && navigator.onLine !== false && validFix(payload?.gcj02) && fresh(payload.gcj02) && !receiverHasNoFix(); }
  function ageLabel(point) {
    const elapsed = age(point);
    if (!Number.isFinite(elapsed)) return "未接收";
    if (elapsed < 1) return "刚刚更新";
    if (elapsed < 60) return `${Math.floor(elapsed)} 秒前`;
    if (elapsed < 3600) return `${Math.floor(elapsed / 60)} 分钟前`;
    return `${Math.floor(elapsed / 3600)} 小时前`;
  }
  function renderAge(id, point) {
    text(id, ageLabel(point));
    $(id).classList.toggle("stale", Boolean(point) && !fresh(point));
  }
  function setBadge(id, label, kind, dot = false) {
    const element = $(id);
    element.className = `badge ${kind}`;
    element.replaceChildren();
    if (dot) element.append(document.createElement("i"));
    element.append(document.createTextNode(label));
  }
  function setFollowing(value) {
    following = value;
    $("follow-button").classList.toggle("is-active", value);
    $("follow-button").setAttribute("aria-pressed", String(value));
  }
  function clearRoute(message = "选好终点后，使用当前实时位置预览路线。机器人实际通行能力需现场确认。") {
    routeGeneration += 1;
    if (walking) walking.clear();
    if (routeLine && map) map.remove(routeLine);
    routeLine = null;
    walking = null;
    routePending = false;
    routeReady = false;
    $("route-result").hidden = true;
    $("route-empty").hidden = false;
    text("route-empty", message);
    text("route-result-label", "尚未规划");
    $("route-steps").replaceChildren();
    updatePlanning();
  }
  function updatePlanning() {
    let reason = "选择终点后可预览步行路线。";
    if (navigator.onLine === false) reason = "浏览器离线，恢复网络后才能请求高德路线。";
    else if (!apiOnline) reason = "定位服务未连接，路线规划暂不可用。";
    else if (!coordinates(payload?.gcj02)) reason = "尚未收到有效 GCJ-02 定位，请检查坐标转换节点。";
    else if (!validFix(payload.gcj02)) reason = "GCJ-02 报告无有效定位解，路线规划暂不可用。";
    else if (!fresh(payload.gcj02)) reason = "GCJ-02 定位已过期，已暂停路线规划。";
    else if (receiverHasNoFix()) reason = "最新 GGA 报告无有效定位解，路线规划暂不可用。";
    else if (!map) reason = "请先配置并加载高德地图。";
    else if (routePending) reason = "正在请求高德步行路线…";
    else if (routeReady) reason = "路线基于规划时的位置生成；机器人移动后可重新预览。";
    else if (destination) reason = "起点使用当前 GCJ-02 定位，点击按钮预览路线。";
    $("plan-button").disabled = !map || !gcjAvailable() || routePending;
    text("plan-button", routePending ? "正在规划…" : "预览步行路线");
    text("plan-readiness", reason);
    $("plan-readiness").classList.toggle("warning", !gcjAvailable());
    $("clear-route").disabled = !destination && !routePending && !routeReady;
  }

  function renderTelemetry() {
    const fix = payload?.fix;
    const gcj = payload?.gcj02;
    const quality = payload?.quality;
    const enu = payload?.enu;
    const origin = payload?.origin;
    const connected = apiOnline && navigator.onLine !== false;
    setBadge("connection-badge", connected ? "服务已连接" : hasConnected ? "服务离线" : "等待服务", connected ? "good" : "warning", true);
    $("connection-notice").hidden = connected || !connectionAttempted;
    if (!connected && connectionAttempted) text("connection-notice", hasConnected ? "与定位服务的连接已中断。页面保留最后收到的数据并标注时间，路线规划已暂停；连接会自动重试。" : "暂时无法读取定位服务。请确认面板 ROS 节点正在运行；页面会自动重试，路线规划已暂停。");
    const qualityFresh = connected && fresh(quality);
    if (!quality) {
      setBadge("rtk-badge", fix && connected && fresh(fix) && fix.status >= 0 ? "GNSS" : "等待数据", "neutral");
      text("quality-description", "尚无原始 GGA 质量信息，不能确认 RTK Fixed / Float。");
    } else if (!qualityFresh) {
      setBadge("rtk-badge", "GGA 已过期", "warning");
      text("quality-description", "原始 GGA 质量信息已过期，当前 RTK 状态未知。");
    } else if (quality.gga_quality === 4) {
      setBadge("rtk-badge", "RTK Fixed", "good");
      text("quality-description", "GGA 质量 4 · RTK 固定解。σ 为驱动估计；固定解不代表地图匹配精度。");
    } else if (quality.gga_quality === 5) {
      setBadge("rtk-badge", "RTK Float", "warning");
      text("quality-description", "GGA 质量 5 · RTK 浮点解，定位精度可能波动。");
    } else if (quality.gga_quality === 0) {
      setBadge("rtk-badge", "无有效解", "danger");
      text("quality-description", "GGA 质量 0 · 接收机尚无有效定位解。");
    } else if (number(quality.gga_quality)) {
      setBadge("rtk-badge", quality.gga_quality === 2 ? "差分 GNSS" : "GNSS", "neutral");
      text("quality-description", `GGA 质量 ${quality.gga_quality} · 当前未报告 RTK 固定解。`);
    } else {
      setBadge("rtk-badge", "质量未知", "neutral");
      text("quality-description", "原始数据中暂无可用的 GGA 质量码。");
    }
    text("satellites", qualityFresh && number(quality.satellites) ? quality.satellites : "—");
    text("hdop", qualityFresh ? format(quality.hdop, 1) : "—");
    text("accuracy", connected && fresh(fix) && number(fix?.accuracy_m) && fix.accuracy_m >= 0 ? format(fix.accuracy_m, fix.accuracy_m < 1 ? 3 : 2) : "—");
    text("gcj-longitude", format(gcj?.longitude, 7));
    text("gcj-latitude", format(gcj?.latitude, 7));
    text("fix-longitude", format(fix?.longitude, 7));
    text("fix-latitude", format(fix?.latitude, 7));
    text("fix-altitude", number(fix?.altitude) ? `${format(fix.altitude)} m` : "—");
    const gnssStatuses = { "-1": "无定位", "0": "有效定位", "1": "SBAS 增强", "2": "GBAS 增强" };
    text("fix-status", fix ? gnssStatuses[String(fix.status)] || "未知" : "—");
    renderAge("gcj-age", gcj);
    renderAge("fix-age", fix);
    renderAge("enu-age", enu);
    text("enu-position", enu ? `${format(enu.east)} / ${format(enu.north)} / ${format(enu.up)}` : "—");
    text("enu-frame", enu?.frame_id || "—");
    text("origin-position", coordinates(origin) ? `${format(origin.longitude, 7)}, ${format(origin.latitude, 7)}` : "—");
    text("geo-status", payload?.geo_status ? `坐标转换：${payload.geo_status.text}（${ageLabel(payload.geo_status)}）` : "坐标转换状态：等待数据");
    text("quality-age", `GGA 质量数据：${ageLabel(quality)}`);
    const isFresh = gcjAvailable();
    text("map-position-status", isFresh ? `GCJ-02 实时定位 · ${ageLabel(gcj)}` : coordinates(gcj) ? `GCJ-02 最后位置 · ${ageLabel(gcj)} · 当前不可规划` : "等待有效 GCJ-02 定位");
    $("map-position-status").className = isFresh ? "fresh" : "stale";
    if (lastGcjFresh && !isFresh && (routeReady || routePending)) clearRoute("实时定位不可用，已清除旧路线。定位恢复后请重新预览。");
    lastGcjFresh = isFresh;
    if (robotMarker) robotMarker.setContent(`<span class="robot-marker${isFresh ? "" : " stale"}" aria-label="机器人最后位置"></span>`);
    text("last-update", payload ? `服务数据：${ageLabel({ received_at: payload.server_time })}` : "等待定位服务");
    renderLocalPathStatus();
    updatePlanning();
  }

  function updateMap() {
    const track = Array.isArray(payload?.track) ? payload.track.filter(coordinates) : [];
    text("track-count", `轨迹 ${track.length.toLocaleString("zh-CN")} 个点`);
    $("fit-button").disabled = !map || (!track.length && !coordinates(payload?.gcj02) && !destination);
    if (!map || !window.AMap) return;
    const position = payload?.gcj02;
    if (coordinates(position)) {
      const point = [position.longitude, position.latitude];
      if (!robotMarker) {
        robotMarker = new AMap.Marker({ map, position: point, content: '<span class="robot-marker" aria-label="机器人当前位置"></span>', anchor: "center", zIndex: 120 });
      } else robotMarker.setPosition(point);
      if (gcjAvailable() && (following || !hasCentered)) {
        map.setCenter(point);
        if (!hasCentered) map.setZoom(18);
        hasCentered = true;
      }
    } else if (robotMarker) {
      map.remove(robotMarker);
      robotMarker = null;
    }
    const path = track.map((point) => [point.longitude, point.latitude]);
    if (!trackLine) trackLine = new AMap.Polyline({ map, path, strokeColor: "#277ce0", strokeOpacity: .85, strokeWeight: 4, lineJoin: "round", zIndex: 60 });
    else trackLine.setPath(path); // An empty backend track must also clear the old line.
    if (destination && !destinationMarker) showDestination();
  }
  function showDestination() {
    if (!map || !destination) return;
    const point = [destination.longitude, destination.latitude];
    if (!destinationMarker) destinationMarker = new AMap.Marker({ map, position: point, content: '<span class="destination-marker" aria-label="导航终点"></span>', anchor: "bottom-center", zIndex: 125 });
    else destinationMarker.setPosition(point);
  }
  function selectDestination(point, fromMap = false) {
    if (!coordinates(point)) return;
    clearRoute();
    destination = { longitude: point.longitude, latitude: point.latitude };
    if (fromMap) {
      $("destination-longitude").value = point.longitude.toFixed(7);
      $("destination-latitude").value = point.latitude.toFixed(7);
    }
    showDestination();
    updatePlanning();
  }
  function fitMap() {
    if (!map) return;
    const overlays = [robotMarker, destinationMarker, trackLine].filter(Boolean);
    if (overlays.length) {
      setFollowing(false);
      map.setFitView(overlays, false, [75, 45, 75, 45], 18);
    }
  }

  function localPoints() {
    const path = payload?.navigation?.path;
    return Array.isArray(path?.points) ? path.points.filter((point) => number(point.x) && number(point.y)) : [];
  }
  function renderLocalPathStatus() {
    const path = payload?.navigation?.path;
    if (!localPoints().length) {
      setBadge("local-path-badge", "等待路径", "neutral");
      text("local-path-age", path ? ageLabel(path) : "未接收");
      return;
    }
    const isFresh = apiOnline && navigator.onLine !== false && fresh(path);
    const reached = payload?.navigation?.reached;
    setBadge("local-path-badge", !isFresh ? "最后路径" : fresh(reached) && reached.value === true ? "已到达目标" : "实时路径", isFresh ? "good" : "warning");
    text("local-path-age", `${ageLabel(path)}${isFresh ? "" : " · 数据已过期"}`);
    $("local-path").classList.toggle("stale", !isFresh);
  }
  function renderLocalPath() {
    const navigation = payload?.navigation;
    const path = navigation?.path;
    const points = localPoints();
    const svg = $("local-path");
    svg.replaceChildren();
    // SVGElement does not consistently implement HTMLElement.hidden.
    svg.toggleAttribute("hidden", !points.length);
    $("local-path-empty").hidden = Boolean(points.length);
    if (!points.length) {
      text("local-path-info", path ? `FAR 实际路径 · ${path.frame_id || "未知坐标系"}（米）` : "坐标系：等待数据");
      text("local-path-notice", "仅叠加同坐标系的里程计和目标点；尚未与地理坐标对齐。");
      renderLocalPathStatus();
      return;
    }
    const frame = typeof path.frame_id === "string" ? path.frame_id : "";
    const matches = (record) => record && frame && record.frame_id === frame && number(record.x) && number(record.y);
    const odometry = matches(navigation.odometry) ? navigation.odometry : null;
    const goal = matches(navigation.goal) ? navigation.goal : null;
    const all = [...points, ...[odometry, goal].filter(Boolean)];
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    for (const point of all) { xmin = Math.min(xmin, point.x); xmax = Math.max(xmax, point.x); ymin = Math.min(ymin, point.y); ymax = Math.max(ymax, point.y); }
    const centerX = (xmin + xmax) / 2, centerY = (ymin + ymax) / 2;
    const spanX = Math.max(xmax - xmin, 2), spanY = Math.max(ymax - ymin, 2);
    const scale = Math.min(500 / (spanX * 1.2), 150 / (spanY * 1.2));
    const project = (point) => [300 + (point.x - centerX) * scale, 112 - (point.y - centerY) * scale];
    const create = (tag, attributes, label) => {
      const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
      for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, String(value));
      if (label !== undefined) element.textContent = label;
      svg.append(element);
      return element;
    };
    create("title", {}, `FAR 实际路径，${frame || "未知坐标系"} 坐标系，单位米。`);
    const rawStep = 75 / scale;
    const magnitude = Math.pow(10, Math.floor(Math.log10(rawStep)));
    const step = [1, 2, 5, 10].map((value) => value * magnitude).find((value) => value >= rawStep) || magnitude * 10;
    const gridLeft = centerX - 250 / scale, gridRight = centerX + 250 / scale;
    const gridBottom = centerY - 75 / scale, gridTop = centerY + 75 / scale;
    for (let x = Math.ceil(gridLeft / step) * step, count = 0; x <= gridRight && count < 20; x += step, count++) {
      const px = project({ x, y: centerY })[0];
      create("line", { x1: px, x2: px, y1: 28, y2: 193, stroke: "#e8eef1", "stroke-dasharray": "3 4" });
      create("text", { x: px, y: 209, "text-anchor": "middle", fill: "#9aabb3", "font-size": 10 }, String(Number(x.toFixed(3))));
    }
    for (let y = Math.ceil(gridBottom / step) * step, count = 0; y <= gridTop && count < 20; y += step, count++) {
      const py = project({ x: centerX, y })[1];
      create("line", { x1: 48, x2: 553, y1: py, y2: py, stroke: "#e8eef1", "stroke-dasharray": "3 4" });
      create("text", { x: 41, y: py + 3, "text-anchor": "end", fill: "#9aabb3", "font-size": 10 }, String(Number(y.toFixed(3))));
    }
    create("text", { x: 560, y: 209, fill: "#82979f", "font-size": 10 }, "X / m");
    create("text", { x: 16, y: 16, fill: "#82979f", "font-size": 10 }, "Y / m");
    create("polyline", { points: points.map((point) => project(point).map((n) => n.toFixed(2)).join(",")).join(" "), fill: "none", stroke: "#7a6ed0", "stroke-width": 3.5, "stroke-linejoin": "round", "stroke-linecap": "round" });
    if (odometry) {
      const [cx, cy] = project(odometry);
      create("circle", { cx, cy, r: 6, fill: fresh(odometry) ? "#0a8b90" : "#9baeb4", stroke: "#fff", "stroke-width": 2 });
    }
    if (goal) {
      const [x, y] = project(goal);
      create("rect", { x: x - 5, y: y - 5, width: 10, height: 10, rx: 2, fill: fresh(goal) ? "#e3a041" : "#b9b1a3", stroke: "#fff", "stroke-width": 1.5 });
    }
    const legend = [{ x: 125, label: "FAR 路径", color: "#7a6ed0" }, ...(odometry ? [{ x: 263, label: "里程计", color: "#0a8b90" }] : []), ...(goal ? [{ x: 385, label: "目标点", color: "#e3a041" }] : [])];
    for (const item of legend) {
      create("circle", { cx: item.x, cy: 232, r: 3, fill: item.color });
      create("text", { x: item.x + 10, y: 235, fill: "#7d929c", "font-size": 10 }, item.label);
    }
    text("local-path-info", `FAR 实际路径 · ${frame || "未知坐标系"}（米）`);
    const notices = [];
    if (navigation.odometry && !odometry) notices.push("里程计坐标系不同，未叠加");
    if (navigation.goal && !goal) notices.push("目标点坐标系不同，未叠加");
    if (odometry && !fresh(odometry)) notices.push("里程计已过期");
    if (goal && !fresh(goal)) notices.push("目标点为最后记录");
    text("local-path-notice", [...notices, "独立米制视图，尚未与地理坐标对齐。"].join("；"));
    renderLocalPathStatus();
  }

  async function fetchJSON(path, timeout = 6500) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(path, { cache: "no-store", signal: controller.signal, credentials: "same-origin" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return await response.json();
    } finally { clearTimeout(timer); }
  }
  async function poll() {
    try {
      const data = await fetchJSON("/api/state");
      if (!data || !number(data.server_time)) throw new Error("Invalid state");
      payload = data;
      receivedAt = performance.now();
      apiOnline = true;
      hasConnected = true;
      updateMap();
      renderLocalPath();
    } catch (_) {
      apiOnline = false;
      $("connection-notice").hidden = false;
      text("connection-notice", "暂时无法读取定位服务。请确认面板 ROS 节点正在运行；页面会自动重试，路线规划已暂停。");
    } finally {
      connectionAttempted = true;
      renderTelemetry();
      setTimeout(poll, config.poll_interval_ms);
    }
  }
  function readSession() {
    try { return JSON.parse(sessionStorage.getItem(storageKey) || "{}"); } catch (_) { return {}; }
  }
  function credentials() {
    const saved = readSession();
    return {
      key: config.amap_key || (typeof saved.key === "string" ? saved.key : ""),
      security: config.amap_security_js_code || (typeof saved.security === "string" ? saved.security : "")
    };
  }
  function showMapPlaceholder(title, description, retry = false, configure = false) {
    $("map-placeholder").hidden = false;
    text("map-placeholder-title", title);
    text("map-placeholder-text", description);
    $("map-retry").hidden = !retry;
    $("map-configure").hidden = !configure;
    $("map-tip").hidden = true;
  }
  function amapErrorHelp(result, fallback) {
    // Only recognized codes select fixed guidance. Never display SDK payloads,
    // error messages or URLs, which can contain the user's credentials.
    const code = typeof result === "string" ? result : result?.info;
    const messages = {
      USERKEY_PLAT_NOMATCH: "Key 绑定的平台不匹配。请在高德控制台创建或选择 Web 端（JS API）Key。",
      INVALID_USER_SCODE: "高德安全密钥校验失败。请填写与当前 Key 配套的 securityJsCode，并检查安全配置。",
      INVALID_USER_KEY: "高德 Key 无效。请检查 Key 是否正确、已启用，以及是否属于 Web 端（JS API）。",
      INVALID_USER_DOMAIN: "当前访问域名未获授权。请在高德控制台检查此 Key 的域名白名单。",
      DAILY_QUERY_OVER_LIMIT: "高德服务调用额度已用尽，请在高德控制台检查额度后重试。",
      USER_DAILY_QUERY_OVER_LIMIT: "高德服务调用额度已用尽，请在高德控制台检查额度后重试。",
      USER_BANDWIDTH_OVER_LIMIT: "高德服务流量额度已用尽，请在高德控制台检查额度后重试。",
      IP_QUERY_OVER_LIMIT: "高德服务调用频率或额度受限，请稍后重试并检查控制台额度。",
      ACCESS_TOO_FREQUENT: "高德服务调用过于频繁，请稍后重试。",
      CUQPS_HAS_EXCEEDED_THE_LIMIT: "高德服务并发调用超限，请稍后重试并检查控制台额度。"
    };
    return typeof code === "string" && Object.prototype.hasOwnProperty.call(messages, code) ? messages[code] : fallback;
  }
  function loadSDKLoader() {
    if (window.AMapLoader) return Promise.resolve();
    if (sdkLoaderPromise) return sdkLoaderPromise;
    sdkLoaderPromise = new Promise((resolve, reject) => {
      const script = document.createElement("script");
      let settled = false;
      const timeout = setTimeout(() => finish(new Error("SDK loader timeout")), 15000);
      function finish(error) {
        if (settled) return;
        settled = true;
        clearTimeout(timeout);
        script.onload = script.onerror = null;
        if (error) { script.remove(); sdkLoaderPromise = null; reject(error); }
        else resolve();
      }
      script.src = "https://webapi.amap.com/loader.js";
      script.async = true;
      script.onload = () => finish(window.AMapLoader ? null : new Error("SDK loader unavailable"));
      script.onerror = () => finish(new Error("SDK loader network error"));
      document.head.append(script);
    });
    return sdkLoaderPromise;
  }
  async function initializeMap() {
    const generation = ++mapLoadGeneration;
    const nextCredentials = credentials();
    clearRoute();
    if (map) map.destroy();
    map = null;
    robotMarker = destinationMarker = trackLine = routeLine = null;
    hasCentered = false;
    updatePlanning();
    if (!nextCredentials.key) {
      showMapPlaceholder("添加 Key，开启高德地图", "配置高德 Web JS API Key 后，可显示已接入的位置、轨迹与步行路线。定位状态会独立更新。", false, true);
      return;
    }
    if (navigator.onLine === false) {
      showMapPlaceholder("当前网络离线", "地图需要连接高德服务。恢复网络后点击重试，定位数据会继续独立更新。", true);
      return;
    }
    showMapPlaceholder("正在加载高德地图", "正在连接高德 JS API 2.0，请稍候…");
    try {
      await loadSDKLoader();
      if (generation !== mapLoadGeneration) return;
      if (activeCredentials) window.AMapLoader.reset();
      window._AMapSecurityConfig = { securityJsCode: nextCredentials.security };
      activeCredentials = nextCredentials;
      const sdk = await Promise.race([
        window.AMapLoader.load({ key: nextCredentials.key, version: "2.0", plugins: ["AMap.Walking", "AMap.Scale", "AMap.ToolBar"] }),
        new Promise((_, reject) => setTimeout(() => reject(new Error("SDK timeout")), 20000))
      ]);
      if (generation !== mapLoadGeneration) return;
      const position = coordinates(payload?.gcj02) ? [payload.gcj02.longitude, payload.gcj02.latitude] : undefined;
      map = new sdk.Map("map", { viewMode: "2D", zoom: position ? 18 : 5, ...(position ? { center: position } : {}), resizeEnable: true });
      map.addControl(new sdk.Scale());
      map.addControl(new sdk.ToolBar({ position: { right: "16px", bottom: "65px" } }));
      map.on("click", (event) => selectDestination({ longitude: event.lnglat.getLng(), latitude: event.lnglat.getLat() }, true));
      map.on("dragstart", () => setFollowing(false));
      $("map-placeholder").hidden = true;
      $("map-tip").hidden = false;
      updateMap();
      renderTelemetry();
    } catch (error) {
      if (generation !== mapLoadGeneration) return;
      const hint = amapErrorHelp(error, "请检查网络、Web JS API Key、安全密钥及高德控制台域名设置，然后重试。");
      showMapPlaceholder("高德地图暂时不可用", `${hint}实时定位数据仍会继续更新。`, true, true);
      updatePlanning();
    }
  }

  function formatDistance(meters) { return meters >= 1000 ? `${(meters / 1000).toFixed(2)} km` : `${Math.round(meters)} m`; }
  function formatTime(seconds) {
    const minutes = Math.max(1, Math.ceil(seconds / 60));
    return minutes >= 60 ? `${Math.floor(minutes / 60)} 小时 ${minutes % 60} 分钟` : `${minutes} 分钟`;
  }
  function planRoute(event) {
    event.preventDefault();
    if (!map || !gcjAvailable() || routePending) return;
    const longitude = Number($("destination-longitude").value);
    const latitude = Number($("destination-latitude").value);
    if (!$("destination-longitude").value.trim() || !$("destination-latitude").value.trim() || !coordinates({ longitude, latitude })) return;
    selectDestination({ longitude, latitude });
    const generation = ++routeGeneration;
    const start = [payload.gcj02.longitude, payload.gcj02.latitude];
    const target = [destination.longitude, destination.latitude];
    // Do not attach the asynchronous SDK renderer to the map: only an accepted
    // callback may create a line. Late/failed requests cannot draw stale routes.
    const planner = new AMap.Walking({ hideMarkers: true, autoFitView: false });
    walking = planner;
    routePending = true;
    text("route-empty", "正在向高德请求步行路线…");
    text("route-result-label", "规划中");
    updatePlanning();
    let completed = false;
    const routeTimeout = setTimeout(() => finish("error", null), 18000);
    function finish(status, result) {
      if (completed) { planner.clear(); return; }
      completed = true;
      clearTimeout(routeTimeout);
      if (generation !== routeGeneration || !gcjAvailable() || walking !== planner) { planner.clear(); return; }
      routePending = false;
      const route = result?.routes?.[0];
      if (status !== "complete" || !route || !number(route.distance) || !number(route.time)) {
        planner.clear();
        const hint = amapErrorHelp(result, "请检查网络及终点是否在高德步行服务覆盖范围内，然后重试。");
        text("route-empty", `未能获取步行路线。${hint}`);
        text("route-result-label", "规划未完成");
        updatePlanning();
        return;
      }
      routeReady = true;
      $("route-result").hidden = false;
      $("route-empty").hidden = true;
      text("route-distance", formatDistance(route.distance));
      text("route-time", formatTime(route.time));
      text("route-result-label", "高德步行 · 仅预览");
      const steps = document.createDocumentFragment();
      for (const step of Array.isArray(route.steps) ? route.steps : []) {
        const item = document.createElement("li");
        const content = document.createElement("div");
        content.textContent = step.instruction || "沿路线继续前行";
        if (number(step.distance)) {
          const distance = document.createElement("span");
          distance.className = "step-distance";
          distance.textContent = formatDistance(step.distance);
          content.append(distance);
        }
        item.append(content);
        steps.append(item);
      }
      $("route-steps").replaceChildren(steps);
      const path = [];
      for (const step of Array.isArray(route.steps) ? route.steps : []) {
        for (const point of Array.isArray(step.path) ? step.path : []) {
          const longitude = Array.isArray(point) ? point[0] : typeof point?.getLng === "function" ? point.getLng() : point?.lng;
          const latitude = Array.isArray(point) ? point[1] : typeof point?.getLat === "function" ? point.getLat() : point?.lat;
          if (coordinates({ longitude, latitude })) path.push([longitude, latitude]);
        }
      }
      if (path.length >= 2) routeLine = new AMap.Polyline({ map, path, strokeColor: "#12a887", strokeOpacity: .95, strokeWeight: 6, lineJoin: "round", zIndex: 90 });
      setFollowing(false);
      map.setFitView([robotMarker, destinationMarker, routeLine].filter(Boolean), false, [75, 50, 75, 50], 18);
      updatePlanning();
    }
    try { planner.search(start, target, finish); } catch (error) { finish("error", error); }
  }

  function openSettings() {
    const selected = credentials();
    $("amap-key").value = selected.key;
    $("amap-security-code").value = selected.security;
    $("amap-key").disabled = Boolean(config.amap_key);
    $("amap-security-code").disabled = Boolean(config.amap_security_js_code);
    text("settings-source", config.amap_key || config.amap_security_js_code ? "已配置的服务端参数优先生效；相应字段由启动配置管理。" : "临时配置仅保留在此浏览器会话。也可通过面板服务启动参数配置。" );
    $("settings-error").hidden = true;
    $("settings-dialog").showModal();
  }
  $("open-settings").addEventListener("click", openSettings);
  $("map-configure").addEventListener("click", openSettings);
  $("close-settings").addEventListener("click", () => $("settings-dialog").close());
  $("map-retry").addEventListener("click", initializeMap);
  $("settings-form").addEventListener("submit", (event) => {
    event.preventDefault();
    const key = $("amap-key").value.trim();
    if (!key) {
      text("settings-error", "请输入高德 Web JS API Key。");
      $("settings-error").hidden = false;
      return;
    }
    try { sessionStorage.setItem(storageKey, JSON.stringify({ key: config.amap_key ? "" : key, security: config.amap_security_js_code ? "" : $("amap-security-code").value.trim() })); }
    catch (_) {
      text("settings-error", "当前浏览器不允许会话存储，请使用服务端启动参数配置地图 Key。");
      $("settings-error").hidden = false;
      return;
    }
    $("settings-dialog").close();
    initializeMap();
  });
  $("forget-settings").addEventListener("click", () => {
    try { sessionStorage.removeItem(storageKey); } catch (_) { /* Storage may be blocked. */ }
    $("settings-dialog").close();
    initializeMap();
  });
  $("destination-form").addEventListener("submit", planRoute);
  for (const id of ["destination-longitude", "destination-latitude"]) {
    $(id).addEventListener("input", () => {
      clearRoute();
      destination = null;
      if (destinationMarker && map) map.remove(destinationMarker);
      destinationMarker = null;
      updatePlanning();
    });
  }
  $("clear-route").addEventListener("click", () => {
    clearRoute();
    destination = null;
    $("destination-form").reset();
    if (destinationMarker && map) map.remove(destinationMarker);
    destinationMarker = null;
    updatePlanning();
  });
  $("follow-button").addEventListener("click", () => { setFollowing(!following); updateMap(); });
  $("fit-button").addEventListener("click", fitMap);
  window.addEventListener("offline", () => { apiOnline = false; renderTelemetry(); });
  window.addEventListener("online", () => { renderTelemetry(); if (!map && credentials().key) initializeMap(); });
  setInterval(renderTelemetry, 500);
  async function start() {
    // Poll immediately; a slow map/config request must never block telemetry.
    poll();
    try {
      const value = await fetchJSON("/api/config");
      config = {
        amap_key: typeof value.amap_key === "string" ? value.amap_key : "",
        amap_security_js_code: typeof value.amap_security_js_code === "string" ? value.amap_security_js_code : "",
        poll_interval_ms: number(value.poll_interval_ms) ? Math.max(250, value.poll_interval_ms) : defaults.poll_interval_ms,
        stale_after_sec: number(value.stale_after_sec) ? Math.max(.5, value.stale_after_sec) : defaults.stale_after_sec
      };
    } catch (_) { /* Session configuration and telemetry remain available. */ }
    initializeMap();
  }
  start();
})();
