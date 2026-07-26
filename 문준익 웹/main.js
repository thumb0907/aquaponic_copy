/* =========================
   기본 유틸
========================= */
const $ = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));

/* =========================
   상단 스크롤 progress bar
========================= */
const progress = $("#progress");
function updateProgress() {
  const scrollTop = window.scrollY || document.documentElement.scrollTop;
  const docH = document.documentElement.scrollHeight - window.innerHeight;
  const p = docH <= 0 ? 0 : (scrollTop / docH) * 100;
  progress.style.width = `${p}%`;
}
window.addEventListener("scroll", updateProgress, { passive: true });
updateProgress();

/* =========================
   모바일 메뉴
========================= */
$("#navBtn").addEventListener("click", () => $("#nav").classList.toggle("open"));
$$(".nav a").forEach(a => a.addEventListener("click", () => $("#nav").classList.remove("open")));

/* =========================
   Reveal 애니메이션 (IntersectionObserver)
========================= */
const io = new IntersectionObserver((entries) => {
  entries.forEach(e => {
    if (e.isIntersecting) e.target.classList.add("in");
  });
}, { threshold: 0.12 });

$$(".reveal").forEach(el => io.observe(el));

/* =========================
   Story(스티키) - 단계별 내용 변경
========================= */
const story = [
  {
    pill: "SBC #1",
    line: "TCP/IP → PC · Serial(센서허브)",
    title: "수경재배 + 양식장",
    desc: "STM32 센서허브가 pH/TDS/수온/온습도를 묶어서 SBC로 전송하고, C270 영상도 함께 PC 관제로 보냅니다.",
    icons: ["pH", "TDS", "Temp", "TH"]
  },
  {
    pill: "SBC #2",
    line: "TCP/IP → PC · Serial(직교로봇)",
    title: "파종 컨베이어",
    desc: "직교로봇/엔드스탑 상태를 SBC에서 수집하고, C270 영상과 함께 파종 공정 상태를 관제합니다.",
    icons: ["Robot", "Endstop", "Step", "Cam"]
  },
  {
    pill: "SBC #3",
    line: "TCP/IP → PC · USB(D435)",
    title: "수확 컨베이어",
    desc: "D435 뎁스/영상과 스카라·매니퓰레이터 MCU 상태를 결합해 수확 공정의 인식/조작을 지원합니다.",
    icons: ["D435", "SCARA", "Dyn", "Encoder"]
  },
  {
    pill: "Safety",
    line: "Serial → PC · 로컬 즉시제어",
    title: "안전허브",
    desc: "수위(ADC)를 기준으로 펌프/릴레이를 로컬에서 즉시 제어하고, 이벤트를 PC로 보고합니다.",
    icons: ["WaterLv", "Pump", "Relay", "Alarm"]
  },
];

const storyPill = $("#storyPill");
const storyLine = $("#storyLine");
const storyTitle = $("#storyTitle");
const storyDesc = $("#storyDesc");
const storyIcons = $("#storyIcons");
const steps = $$("#storySteps .step");

function setStory(idx) {
  const s = story[idx];
  storyPill.textContent = s.pill;
  storyLine.textContent = s.line;
  storyTitle.textContent = s.title;
  storyDesc.textContent = s.desc;
  storyIcons.innerHTML = s.icons.map(x => `<span class="ico">${x}</span>`).join("");
  steps.forEach((el, i) => el.classList.toggle("active", i === idx));
}

// 클릭으로도 변경 가능
steps.forEach((el) => {
  el.addEventListener("click", () => {
    const idx = Number(el.getAttribute("data-step"));
    setStory(idx);
    el.scrollIntoView({ behavior: "smooth", block: "center" });
  });
});

// 스크롤 위치에 따라 자동 변경
const stepIO = new IntersectionObserver((entries) => {
  const visible = entries.filter(e => e.isIntersecting)
    .sort((a,b) => b.intersectionRatio - a.intersectionRatio)[0];
  if (!visible) return;
  const idx = Number(visible.target.getAttribute("data-step"));
  setStory(idx);
}, { threshold: [0.2, 0.35, 0.5, 0.65] });

steps.forEach(el => stepIO.observe(el));
setStory(0);

/* =========================
   DRAG Slider (마우스로 잡고 드래그)
========================= */
const track = $("#sliderTrack");
let isDown = false;
let startX = 0;
let startScroll = 0;

track.addEventListener("mousedown", (e) => {
  isDown = true;
  startX = e.pageX;
  startScroll = track.scrollLeft;
});
window.addEventListener("mouseup", () => isDown = false);
window.addEventListener("mousemove", (e) => {
  if (!isDown) return;
  const dx = e.pageX - startX;
  track.scrollLeft = startScroll - dx;
});

/* =========================
   Dashboard (센서 UI)
   - 지금은 Mock 생성
   - 나중에 API_BASE + fetch로 교체 가능
   - 30분 스냅샷 localStorage 저장
========================= */
const CONFIG = {
  API_BASE: "", // 나중에 PC/서버가 생기면: "http://서버IP:PORT"
  SNAPSHOT_MINUTES: 30,
  THRESHOLDS: {
    ph:    { min: 5.5, max: 6.5 },
    tds:   { min: 400, max: 1200 },
    temp:  { min: 18,  max: 26 },
    th:    { min: 40,  max: 80 },   // 습도 기준
    level: { min: 30,  max: 90 },
  }
};

const SENSORS = [
  { key:"ph", name:"pH", unit:"pH" },
  { key:"tds", name:"TDS", unit:"ppm" },
  { key:"temp", name:"수온", unit:"°C" },
  { key:"th", name:"온·습도", unit:"°C / %" },
  { key:"level", name:"수위", unit:"%" },
];

const STORE_KEY = "unidad_snapshots_v1";
const sensorGrid = $("#sensorGrid");
const dashMeta = $("#dashMeta");
const snapInfo = $("#snapInfo");
const snapPreview = $("#snapPreview");
const logEl = $("#log");

function nowIso(){ return new Date().toISOString(); }
function fmtKR(d = new Date()){
  return d.toLocaleString("ko-KR", { hour12:false });
}

function loadSnaps(){
  try { return JSON.parse(localStorage.getItem(STORE_KEY) || "[]"); }
  catch { return []; }
}
function saveSnaps(arr){
  localStorage.setItem(STORE_KEY, JSON.stringify(arr));
}

function wave(base, amp, speed){
  const t = Date.now();
  return base + amp * Math.sin(t / speed);
}

function mockLatest(){
  const waterTemp = +wave(23, 2.1, 90000).toFixed(1);
  const hum = +wave(62, 8.0, 70000).toFixed(1);
  return {
    at: nowIso(),
    ph: +wave(6.1, 0.25, 65000).toFixed(2),
    tds: Math.round(wave(820, 110, 80000)),
    temp: waterTemp,
    th: { temp: +wave(24, 1.8, 100000).toFixed(1), hum },
    level: Math.round(wave(70, 18, 110000)),
  };
}

async function apiLatest(){
  const res = await fetch(`${CONFIG.API_BASE}/api/latest`);
  if (!res.ok) throw new Error(`API ${res.status}`);
  return await res.json();
}

async function getLatest(){
  if (!CONFIG.API_BASE) return mockLatest();
  try { return await apiLatest(); }
  catch { return mockLatest(); }
}

function statusOf(key, latest){
  const th = CONFIG.THRESHOLDS[key];
  if (!th) return { ok:true, text:"OK" };

  let v = latest[key];
  if (key === "th") v = latest.th?.hum;

  if (typeof v !== "number") return { ok:true, text:"OK" };
  const ok = (v >= th.min && v <= th.max);
  return { ok, text: ok ? "OK" : "WARN" };
}

function pushLog(msg){
  const li = document.createElement("li");
  li.textContent = `[${fmtKR()}] ${msg}`;
  logEl.prepend(li);
  while (logEl.children.length > 24) logEl.removeChild(logEl.lastChild);
}

function renderSensors(latest){
  dashMeta.textContent = `업데이트: ${fmtKR(new Date(latest.at || Date.now()))} · 현재 모드: ${CONFIG.API_BASE ? "API" : "Mock"}`;

  sensorGrid.innerHTML = SENSORS.map(s => {
    const st = statusOf(s.key, latest);
    const badCls = st.ok ? "bad" : "bad warn";
    const range = CONFIG.THRESHOLDS[s.key]
      ? `정상범위: ${CONFIG.THRESHOLDS[s.key].min} ~ ${CONFIG.THRESHOLDS[s.key].max}`
      : "";

    let valueText = "-";
    if (s.key === "th") {
      valueText = `${latest.th?.temp ?? "-"} / ${latest.th?.hum ?? "-"}`;
    } else {
      valueText = `${latest[s.key] ?? "-"}`;
    }

    return `
      <div class="sensor">
        <div class="sensor__top">
          <div class="sensor__name">${s.name}</div>
          <div class="${badCls}">${st.text}</div>
        </div>
        <div class="sensor__val">${valueText}</div>
        <div class="sensor__unit">${s.unit}</div>
        <div class="sensor__range">${range}</div>
      </div>
    `;
  }).join("");

  // WARN 발생 시 로그
  for (const s of SENSORS) {
    const st = statusOf(s.key, latest);
    if (!st.ok) pushLog(`${s.name} 임계값 벗어남`);
  }
}

function makeSnapshot(latest){
  return {
    at: latest.at || nowIso(),
    ph: latest.ph,
    tds: latest.tds,
    temp: latest.temp,
    th: latest.th,
    level: latest.level
  };
}

function renderLastSnapshot(){
  const snaps = loadSnaps();
  const last = snaps[snaps.length - 1];
  snapInfo.textContent = last
    ? `총 ${snaps.length}개 · 마지막 저장: ${fmtKR(new Date(last.at))}`
    : "저장된 스냅샷 없음";
  snapPreview.textContent = last ? JSON.stringify(last, null, 2) : "아직 저장된 스냅샷이 없습니다.";
}

async function saveSnapshotNow(){
  const latest = await getLatest();
  const snaps = loadSnaps();
  snaps.push(makeSnapshot(latest));
  if (snaps.length > 2000) snaps.splice(0, snaps.length - 2000);
  saveSnaps(snaps);
  pushLog("스냅샷 저장됨(로컬)");
  renderLastSnapshot();
}

function msUntilNextHalfHour(){
  const d = new Date();
  const m = d.getMinutes();
  const next = (m < 30) ? 30 : 60;
  const target = new Date(d);
  target.setMinutes(next === 60 ? 0 : 30, 0, 0);
  if (next === 60) target.setHours(d.getHours() + 1);
  return Math.max(1000, target - d);
}

function startSnapshotScheduler(){
  setTimeout(() => {
    saveSnapshotNow();
    setInterval(saveSnapshotNow, CONFIG.SNAPSHOT_MINUTES * 60 * 1000);
  }, msUntilNextHalfHour());
}

$("#saveNow").addEventListener("click", saveSnapshotNow);

$("#clearLocal").addEventListener("click", () => {
  if (!confirm("로컬 스냅샷을 모두 삭제할까요?")) return;
  localStorage.removeItem(STORE_KEY);
  pushLog("로컬 초기화 완료");
  renderLastSnapshot();
});

$("#exportJson").addEventListener("click", () => {
  const snaps = loadSnaps();
  const blob = new Blob([JSON.stringify(snaps, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "snapshots.json";
  a.click();
  URL.revokeObjectURL(url);
  pushLog("스냅샷 JSON 내보내기 완료");
});

/* 실시간 갱신(프론트 시연용) */
async function tick(){
  const latest = await getLatest();
  renderSensors(latest);
}
tick();
setInterval(tick, 1200);
renderLastSnapshot();
startSnapshotScheduler();
