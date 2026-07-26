// =====================================================
// 실행 모드
// =====================================================

// 실제 센서값 사용
const TEST_MODE = false;

// 그래프에 표시할 최대 데이터 수
const MAX_POINTS = 20;

// 센서값 요청 주기
const SENSOR_UPDATE_INTERVAL_MS = 1000;

// 정상 센서 로그 출력 주기
const SENSOR_LOG_INTERVAL_MS = 60000;

// 같은 이상 로그 반복 제한 시간
const ALERT_COOLDOWN_MS = 5000;


// =====================================================
// 현재 시각
// =====================================================

function updateClock() {
  const now = new Date();

  const year =
    now.getFullYear();

  const month =
    String(
      now.getMonth() + 1
    ).padStart(2, "0");

  const date =
    String(
      now.getDate()
    ).padStart(2, "0");

  const hours =
    String(
      now.getHours()
    ).padStart(2, "0");

  const minutes =
    String(
      now.getMinutes()
    ).padStart(2, "0");

  const seconds =
    String(
      now.getSeconds()
    ).padStart(2, "0");

  const currentTimeElement =
    document.getElementById(
      "currentTime"
    );

  if (currentTimeElement) {
    currentTimeElement.textContent =
      `${year}-${month}-${date} ` +
      `${hours}:${minutes}:${seconds}`;
  }
}


function formatTime(date) {
  const hours =
    String(
      date.getHours()
    ).padStart(2, "0");

  const minutes =
    String(
      date.getMinutes()
    ).padStart(2, "0");

  const seconds =
    String(
      date.getSeconds()
    ).padStart(2, "0");

  return (
    `${hours}:${minutes}:${seconds}`
  );
}


// =====================================================
// 센서 정상 범위
// =====================================================

const NORMAL_RANGE = {
  level: {
    min: 280,
    max: 380
  },

  temp: {
    min: 22.0,
    max: 28.0
  },

  ph: {
    min: 6.4,
    max: 7.4
  },

  tds: {
    min: 580,
    max: 780
  }
};


// =====================================================
// 현재 센서값
// =====================================================

let currentLevel = null;
let currentTemp = null;
let currentPh = null;
let currentTds = null;

let currentPi3Alive = false;
let currentEmergency = false;
let currentLastUpdate = null;


// =====================================================
// 그래프 데이터
// =====================================================

const labels = [];

const levelData = [];
const tempData = [];
const phData = [];
const tdsData = [];


for (
  let i = MAX_POINTS - 1;
  i >= 0;
  i--
) {
  const time = new Date(
    Date.now() -
    i * SENSOR_UPDATE_INTERVAL_MS
  );

  labels.push(
    formatTime(time)
  );

  levelData.push(null);
  tempData.push(null);
  phData.push(null);
  tdsData.push(null);
}


// =====================================================
// Chart.js 그래프 생성
// =====================================================

function createRealtimeChart(
  canvasId,
  label,
  data,
  borderColor,
  backgroundColor,
  yMin,
  yMax
) {
  const canvas =
    document.getElementById(
      canvasId
    );

  if (!canvas) {
    console.error(
      `Canvas를 찾을 수 없습니다: ${canvasId}`
    );

    return null;
  }

  const context =
    canvas.getContext("2d");

  return new Chart(
    context,
    {
      type: "line",

      data: {
        labels: labels,

        datasets: [
          {
            label: label,
            data: data,

            borderColor:
              borderColor,

            backgroundColor:
              backgroundColor,

            fill: true,

            tension: 0.35,

            pointRadius: 0,

            pointHoverRadius: 4,

            borderWidth: 2,

            spanGaps: true
          }
        ]
      },

      options: {
        responsive: true,

        maintainAspectRatio: false,

        animation: false,

        normalized: true,

        interaction: {
          intersect: false,
          mode: "index"
        },

        plugins: {
          legend: {
            labels: {
              color: "#e5e7eb"
            }
          }
        },

        scales: {
          x: {
            ticks: {
              color: "#94a3b8",
              maxTicksLimit: 6
            },

            grid: {
              color: "#1e293b"
            }
          },

          y: {
            min: yMin,
            max: yMax,

            ticks: {
              color: "#94a3b8"
            },

            grid: {
              color: "#1e293b"
            }
          }
        }
      }
    }
  );
}


// =====================================================
// 그래프 객체
// =====================================================

const levelChart =
  createRealtimeChart(
    "levelChart",
    "수위",
    levelData,
    "#38bdf8",
    "rgba(56, 189, 248, 0.12)",
    0,
    500
  );


const tempChart =
  createRealtimeChart(
    "tempChart",
    "수온",
    tempData,
    "#f59e0b",
    "rgba(245, 158, 11, 0.12)",
    0,
    40
  );


const phChart =
  createRealtimeChart(
    "phChart",
    "pH",
    phData,
    "#a78bfa",
    "rgba(167, 139, 250, 0.12)",
    0,
    14
  );


const tdsChart =
  createRealtimeChart(
    "tdsChart",
    "TDS",
    tdsData,
    "#34d399",
    "rgba(52, 211, 153, 0.12)",
    0,
    1000
  );


// =====================================================
// 센서값 변환
// =====================================================

function parseSensorValue(value) {
  if (
    value === null ||
    value === undefined ||
    value === ""
  ) {
    return null;
  }

  const numberValue =
    Number(value);

  if (
    !Number.isFinite(numberValue)
  ) {
    return null;
  }

  return numberValue;
}


function valueText(
  value,
  digit = 1
) {
  if (
    value === null ||
    value === undefined ||
    !Number.isFinite(
      Number(value)
    )
  ) {
    return "-";
  }

  return Number(value).toFixed(
    digit
  );
}


// =====================================================
// 센서 요약값 표시
// =====================================================

function updateSummary() {
  const levelElement =
    document.getElementById(
      "levelValue"
    );

  const tempElement =
    document.getElementById(
      "tempValue"
    );

  const phElement =
    document.getElementById(
      "phValue"
    );

  const tdsElement =
    document.getElementById(
      "tdsValue"
    );


  if (levelElement) {
    levelElement.textContent =
      valueText(
        currentLevel,
        0
      );
  }


  if (tempElement) {
    tempElement.textContent =
      valueText(
        currentTemp,
        1
      );
  }


  if (phElement) {
    phElement.textContent =
      valueText(
        currentPh,
        2
      );
  }


  if (tdsElement) {
    tdsElement.textContent =
      valueText(
        currentTds,
        0
      );
  }


  updateSummaryCardState(
    "levelValue",
    isOutOfRange(
      currentLevel,
      NORMAL_RANGE.level
    )
  );


  updateSummaryCardState(
    "tempValue",
    isOutOfRange(
      currentTemp,
      NORMAL_RANGE.temp
    )
  );


  updateSummaryCardState(
    "phValue",
    isOutOfRange(
      currentPh,
      NORMAL_RANGE.ph
    )
  );


  updateSummaryCardState(
    "tdsValue",
    isOutOfRange(
      currentTds,
      NORMAL_RANGE.tds
    )
  );
}


function updateSummaryCardState(
  valueElementId,
  abnormal
) {
  const valueElement =
    document.getElementById(
      valueElementId
    );

  if (!valueElement) {
    return;
  }

  const summaryCard =
    valueElement.closest(
      ".summary-card"
    );

  if (!summaryCard) {
    return;
  }

  summaryCard.classList.toggle(
    "abnormal",
    abnormal
  );
}


// =====================================================
// 그래프 데이터 갱신
// =====================================================

function pushChartData() {
  const now = new Date();

  labels.push(
    formatTime(now)
  );

  levelData.push(
    currentLevel
  );

  tempData.push(
    currentTemp
  );

  phData.push(
    currentPh
  );

  tdsData.push(
    currentTds
  );


  if (
    labels.length >
    MAX_POINTS
  ) {
    labels.shift();

    levelData.shift();
    tempData.shift();
    phData.shift();
    tdsData.shift();
  }


  if (levelChart) {
    levelChart.update("none");
  }

  if (tempChart) {
    tempChart.update("none");
  }

  if (phChart) {
    phChart.update("none");
  }

  if (tdsChart) {
    tdsChart.update("none");
  }
}


// =====================================================
// 이벤트 로그
// =====================================================

const eventLog =
  document.getElementById(
    "eventLog"
  );

const clearLogBtn =
  document.getElementById(
    "clearLogBtn"
  );


function addLog(
  message,
  type = "sensor"
) {
  if (!eventLog) {
    return;
  }

  const item =
    document.createElement(
      "div"
    );

  item.className =
    `log-item ${type}`;


  const timeSpan =
    document.createElement(
      "span"
    );

  timeSpan.className =
    "log-time";

  timeSpan.textContent =
    `[${formatTime(new Date())}]`;


  const messageSpan =
    document.createElement(
      "span"
    );

  messageSpan.className =
    "log-message";

  messageSpan.textContent =
    message;


  item.appendChild(
    timeSpan
  );

  item.appendChild(
    messageSpan
  );


  eventLog.prepend(
    item
  );


  const maxLogs = 150;

  while (
    eventLog.children.length >
    maxLogs
  ) {
    eventLog.removeChild(
      eventLog.lastChild
    );
  }
}


// =====================================================
// 정상 센서 로그
// =====================================================

function addPeriodicSensorLog() {
  const allValuesMissing =
    currentLevel === null &&
    currentTemp === null &&
    currentPh === null &&
    currentTds === null;

  if (allValuesMissing) {
    return;
  }


  const abnormalSensors = [];


  if (
    isOutOfRange(
      currentLevel,
      NORMAL_RANGE.level
    )
  ) {
    abnormalSensors.push(
      "수위"
    );
  }


  if (
    isOutOfRange(
      currentTemp,
      NORMAL_RANGE.temp
    )
  ) {
    abnormalSensors.push(
      "수온"
    );
  }


  if (
    isOutOfRange(
      currentPh,
      NORMAL_RANGE.ph
    )
  ) {
    abnormalSensors.push(
      "pH"
    );
  }


  if (
    isOutOfRange(
      currentTds,
      NORMAL_RANGE.tds
    )
  ) {
    abnormalSensors.push(
      "TDS"
    );
  }


  const message =
    `센서값 | ` +
    `수위: ${valueText(currentLevel, 0)}mm / ` +
    `수온: ${valueText(currentTemp, 1)}°C / ` +
    `pH: ${valueText(currentPh, 2)} / ` +
    `TDS: ${valueText(currentTds, 0)}ppm`;


  if (
    abnormalSensors.length === 0
  ) {
    addLog(
      message,
      "sensor"
    );

    return;
  }


  addLog(
    `${message} / 이상 센서: ` +
    `${abnormalSensors.join(", ")}`,
    "alert"
  );
}


// =====================================================
// 이상 로그 반복 제한
// =====================================================

let lastAlertTime = {
  level: 0,
  temp: 0,
  ph: 0,
  tds: 0,
  fetch: 0,
  emergency: 0,
  pi3: 0
};


function canAlert(key) {
  const now = Date.now();

  if (
    lastAlertTime[key] ===
    undefined
  ) {
    lastAlertTime[key] = 0;
  }


  if (
    now -
    lastAlertTime[key] >=
    ALERT_COOLDOWN_MS
  ) {
    lastAlertTime[key] = now;

    return true;
  }

  return false;
}


// =====================================================
// 정상 범위 판정
// =====================================================

function isOutOfRange(
  value,
  range
) {
  if (
    value === null ||
    value === undefined ||
    !Number.isFinite(
      Number(value)
    )
  ) {
    return false;
  }

  return (
    value < range.min ||
    value > range.max
  );
}


// =====================================================
// 센서 이상 확인
// =====================================================

function checkAbnormalEvents() {
  if (
    isOutOfRange(
      currentLevel,
      NORMAL_RANGE.level
    ) &&
    canAlert("level")
  ) {
    addLog(
      `수위 이상: 현재 ` +
      `${currentLevel.toFixed(0)}mm ` +
      `(정상 범위 ` +
      `${NORMAL_RANGE.level.min}` +
      `~${NORMAL_RANGE.level.max}mm)`,
      "alert"
    );
  }


  if (
    isOutOfRange(
      currentTemp,
      NORMAL_RANGE.temp
    ) &&
    canAlert("temp")
  ) {
    addLog(
      `수온 이상: 현재 ` +
      `${currentTemp.toFixed(1)}°C ` +
      `(정상 범위 ` +
      `${NORMAL_RANGE.temp.min}` +
      `~${NORMAL_RANGE.temp.max}°C)`,
      "alert"
    );
  }


  if (
    isOutOfRange(
      currentPh,
      NORMAL_RANGE.ph
    ) &&
    canAlert("ph")
  ) {
    addLog(
      `pH 이상: 현재 ` +
      `${currentPh.toFixed(2)} ` +
      `(정상 범위 ` +
      `${NORMAL_RANGE.ph.min}` +
      `~${NORMAL_RANGE.ph.max})`,
      "alert"
    );
  }


  if (
    isOutOfRange(
      currentTds,
      NORMAL_RANGE.tds
    ) &&
    canAlert("tds")
  ) {
    addLog(
      `TDS 이상: 현재 ` +
      `${currentTds.toFixed(0)}ppm ` +
      `(정상 범위 ` +
      `${NORMAL_RANGE.tds.min}` +
      `~${NORMAL_RANGE.tds.max}ppm)`,
      "alert"
    );
  }


  if (
    currentEmergency &&
    canAlert("emergency")
  ) {
    addLog(
      "비상 정지 신호가 감지되었습니다.",
      "alert"
    );
  }


  if (
    currentLastUpdate !== null &&
    !currentPi3Alive &&
    canAlert("pi3")
  ) {
    addLog(
      "센서 Raspberry Pi 상태가 비정상입니다.",
      "alert"
    );
  }
}


// =====================================================
// Flask API에서 실제 센서값 수신
// =====================================================

async function fetchSensorData() {
  try {
    const response =
      await fetch(
        "/api/sensors",
        {
          cache: "no-store"
        }
      );


    if (!response.ok) {
      throw new Error(
        `HTTP ${response.status}`
      );
    }


    const data =
      await response.json();


    currentLevel =
      parseSensorValue(
        data.level
      );

    currentTemp =
      parseSensorValue(
        data.water_temp
      );

    currentPh =
      parseSensorValue(
        data.ph
      );

    currentTds =
      parseSensorValue(
        data.tds
      );


    currentPi3Alive =
      Boolean(
        data.pi3_alive
      );

    currentEmergency =
      Boolean(
        data.emergency
      );

    currentLastUpdate =
      data.last_update;


    updateSummary();

    pushChartData();

    checkAbnormalEvents();

  } catch (error) {
    console.error(
      "Sensor API error:",
      error
    );


    if (
      canAlert("fetch")
    ) {
      addLog(
        `센서 API 연결 실패: ` +
        `${error.message}`,
        "alert"
      );
    }
  }
}


// =====================================================
// 영상 자동 재생 보조
// =====================================================

function startTimelapseVideos() {
  const videos =
    document.querySelectorAll(
      ".camera-view video"
    );

  videos.forEach(
    (video) => {
      video.muted = true;
      video.loop = true;

      const playPromise =
        video.play();

      if (
        playPromise !== undefined
      ) {
        playPromise.catch(
          (error) => {
            console.warn(
              "영상 자동 재생 실패:",
              error
            );
          }
        );
      }
    }
  );
}


// =====================================================
// 로그 비우기
// =====================================================

if (clearLogBtn) {
  clearLogBtn.addEventListener(
    "click",
    () => {
      if (eventLog) {
        eventLog.innerHTML = "";
      }

      addLog(
        "이벤트 로그가 초기화되었습니다.",
        "system"
      );
    }
  );
}


// =====================================================
// 실행 시작
// =====================================================

updateClock();

setInterval(
  updateClock,
  1000
);


addLog(
  "웹 대시보드가 시작되었습니다.",
  "system"
);

addLog(
  "실제 센서 모니터링 모드가 활성화되었습니다.",
  "system"
);

addLog(
  "타임랩스 영상 반복 재생을 시작합니다.",
  "system"
);


updateSummary();

startTimelapseVideos();

fetchSensorData();


setInterval(
  fetchSensorData,
  SENSOR_UPDATE_INTERVAL_MS
);


setInterval(
  addPeriodicSensorLog,
  SENSOR_LOG_INTERVAL_MS
);