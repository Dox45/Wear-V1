// Global API Configuration
const API_BASE_URL = 'https://medical-triage-v2.onrender.com';
let currentDoctor = null;

/**
 * Constructs a full API URL with ngrok skip warning parameter.
 */
function getApiUrl(path) {
  const baseUrl = API_BASE_URL.replace(/\/$/, '');
  const cleanPath = path.startsWith('/') ? path : '/' + path;
  const url = new URL(baseUrl + cleanPath);
  url.searchParams.set('ngrok-skip-browser-warning', '69420');
  return url.toString();
}

/**
 * Custom fetch wrapper that includes ngrok headers, auth token, and full endpoint URL.
 */
async function apiFetch(path, options = {}) {
  const url = getApiUrl(path);
  const token = localStorage.getItem("doctor_token");
  const headers = {
    'ngrok-skip-browser-warning': '69420',
    ...(token ? { 'Authorization': `Bearer ${token}` } : {}),
    ...(options.headers || {})
  };
  return fetch(url, { ...options, headers });
}

/**
 * Helper to manage button loading states and prevent multiple clicks.
 */
function setButtonLoading(btn, isLoading, loadingText = "Processing...") {
  if (!btn) return;
  if (isLoading) {
    if (!btn.dataset.originalText) {
      btn.dataset.originalText = btn.innerHTML;
    }
    btn.disabled = true;
    btn.innerHTML = `<i class="fa-solid fa-circle-notch fa-spin"></i> ${loadingText}`;
  } else {
    btn.disabled = false;
    if (btn.dataset.originalText) {
      btn.innerHTML = btn.dataset.originalText;
    }
  }
}

// Global Application State
let latestTelemetry = null;
let historyChart = null;
let ppgCanvas, ppgCtx;
let animationFrameId = null;
let wavePhase = 0;

// Initialize Page
document.addEventListener("DOMContentLoaded", () => {
  initPPGCanvas();
  initChart();
  initSSE();
  checkDoctorSession();
  fetchQueue();
  pollTelemetry();

  // Periodic timers for real-time live updates
  setInterval(pollTelemetry, 1000); // Poll vitals telemetry every 1s
  setInterval(fetchQueue, 3000);     // Poll triage queue every 3s
});

// PPG Waveform Renderer (60 FPS Canvas)
function initPPGCanvas() {
  ppgCanvas = document.getElementById("ppgCanvas");
  if (!ppgCanvas) return;
  ppgCtx = ppgCanvas.getContext("2d");
  resizeCanvas();
  window.addEventListener("resize", resizeCanvas);
  requestAnimationFrame(renderPPG);
}

function resizeCanvas() {
  if (!ppgCanvas) return;
  ppgCanvas.width = ppgCanvas.parentElement.clientWidth;
  ppgCanvas.height = ppgCanvas.parentElement.clientHeight;
}

function renderPPG() {
  if (!ppgCtx) return;
  const width = ppgCanvas.width;
  const height = ppgCanvas.height;
  const midY = height / 2;

  ppgCtx.fillStyle = "#090c13";
  ppgCtx.fillRect(0, 0, width, height);

  // Draw Grid lines
  ppgCtx.strokeStyle = "rgba(255, 255, 255, 0.03)";
  ppgCtx.lineWidth = 1;
  for (let x = 0; x < width; x += 40) {
    ppgCtx.beginPath();
    ppgCtx.moveTo(x, 0); ppgCtx.lineTo(x, height);
    ppgCtx.stroke();
  }

  // Draw Waveform curve
  const bpm = latestTelemetry?.bpm || 72;
  const finger = latestTelemetry?.finger_detected ?? false;
  const speed = (bpm / 60) * 0.06;
  wavePhase += speed;

  ppgCtx.beginPath();
  ppgCtx.lineWidth = 2.5;
  ppgCtx.strokeStyle = finger ? "#38bdf8" : "rgba(148, 163, 184, 0.3)";

  for (let x = 0; x < width; x++) {
    let y = midY;
    if (finger) {
      const t = (x / width) * 4 * Math.PI + wavePhase;
      y = midY + Math.sin(t) * 35 + Math.sin(t * 2) * 15;
    } else {
      y = midY + Math.sin(x * 0.05 + wavePhase) * 4;
    }

    if (x === 0) ppgCtx.moveTo(x, y);
    else ppgCtx.lineTo(x, y);
  }
  ppgCtx.stroke();

  requestAnimationFrame(renderPPG);
}

// Chart.js 10-Min Vitals Chart
function initChart() {
  const chartEl = document.getElementById("historyChart");
  if (!chartEl) return;
  const ctx = chartEl.getContext("2d");
  historyChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Heart Rate (BPM)',
          borderColor: '#f87171',
          backgroundColor: 'rgba(248, 113, 113, 0.1)',
          data: [],
          tension: 0.3,
          borderWidth: 2
        },
        {
          label: 'SpO2 (%)',
          borderColor: '#38bdf8',
          backgroundColor: 'rgba(56, 189, 248, 0.1)',
          data: [],
          tension: 0.3,
          borderWidth: 2
        },
        {
          label: 'Temp (°C)',
          borderColor: '#fbbf24',
          backgroundColor: 'rgba(251, 191, 36, 0.1)',
          data: [],
          tension: 0.3,
          borderWidth: 2
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: { legend: { labels: { color: '#94a3b8' } } },
      scales: {
        x: { ticks: { color: '#64748b' }, grid: { color: 'rgba(255,255,255,0.04)' } },
        y: { ticks: { color: '#64748b' }, grid: { color: 'rgba(255,255,255,0.04)' } }
      }
    }
  });
}

// Connect to Server-Sent Events (SSE) stream
function initSSE() {
  try {
    const sseUrl = getApiUrl('/readings/stream');
    const evtSource = new EventSource(sseUrl);
    evtSource.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'telemetry') {
          updateTelemetryUI(msg.data);
        } else if (msg.type === 'triage_update') {
          fetchQueue();
        }
      } catch (e) { console.error("SSE parse error:", e); }
    };

    evtSource.onerror = (err) => {
      pollTelemetry();
    };
  } catch (err) {
    console.warn("SSE init failed, using polling fallback:", err);
  }
}

async function pollTelemetry() {
  try {
    const res = await apiFetch('/readings/latest');
    if (res.ok) {
      const data = await res.json();
      updateTelemetryUI(data);
    }
  } catch (e) {
    console.error("Telemetry poll failed:", e);
  }
}

// Update Telemetry UI Cards & Badges
function updateTelemetryUI(data) {
  latestTelemetry = data;

  // Update Device Connection Status
  const devBadge = document.getElementById("deviceStatusBadge");
  const devText = document.getElementById("deviceStatusText");
  if (devBadge && devText) {
    if (data.device_connected || data.bpm > 0) {
      devBadge.classList.add("connected");
      devText.innerText = "CONNECTED / STREAMING";
    } else {
      devBadge.classList.remove("connected");
      devText.innerText = "DISCONNECTED";
    }
  }

  // Update Finger Detection Status
  const fingerBadge = document.getElementById("fingerStatusBadge");
  const fingerText = document.getElementById("fingerStatusText");
  const fingerBanner = document.getElementById("fingerBanner");

  if (fingerBadge && fingerText && fingerBanner) {
    if (data.finger_detected) {
      fingerBadge.classList.add("active");
      fingerText.innerText = "FINGER DETECTED";
      fingerBanner.classList.add("hidden");
    } else {
      fingerBadge.classList.remove("active");
      fingerText.innerText = "NO FINGER";
      fingerBanner.classList.remove("hidden");
    }
  }

  // Update KPI Values
  const elBpm = document.getElementById("kpiBpm");
  const elSpo2 = document.getElementById("kpiSpo2");
  const elTemp = document.getElementById("kpiTemp");
  const elTempF = document.getElementById("kpiTempF");
  const elBp = document.getElementById("kpiBp");
  const elPtt = document.getElementById("kpiPtt");

  if (elBpm) elBpm.innerText = data.bpm || "--";
  if (elSpo2) elSpo2.innerText = data.spo2 || "--";
  if (elTemp) elTemp.innerText = data.temp_body_c ? data.temp_body_c.toFixed(1) : "--";
  if (elTempF) elTempF.innerText = data.temp_body_f ? `${data.temp_body_f.toFixed(1)} °F` : "-- °F";

  if (elBp) {
    if (data.sbp && data.dbp) {
      elBp.innerText = `${data.sbp} / ${data.dbp}`;
    } else {
      elBp.innerText = "-- / --";
    }
  }

  if (elPtt) {
    elPtt.innerText = data.ptt_ms ? `PTT: ${data.ptt_ms.toFixed(1)} ms` : "PTT: -- ms";
  }

  // Append to Chart
  if (data.bpm > 0 && historyChart) {
    const timeLabel = new Date().toLocaleTimeString();
    if (historyChart.data.labels.length > 20) {
      historyChart.data.labels.shift();
      historyChart.data.datasets.forEach(ds => ds.data.shift());
    }
    historyChart.data.labels.push(timeLabel);
    historyChart.data.datasets[0].data.push(data.bpm);
    historyChart.data.datasets[1].data.push(data.spo2);
    historyChart.data.datasets[2].data.push(data.temp_body_c);
    historyChart.update('none');
  }
}

// Fetch and Render Triage Queue
async function fetchQueue() {
  try {
    const res = await apiFetch('/api/triage/queue');
    if (!res.ok) return;
    const data = await res.json();
    renderQueue(data.queue || []);
  } catch (e) { }
}

function renderQueue(queue) {
  const tbody = document.getElementById("queueTableBody");
  const countBadge = document.getElementById("queueCountBadge");
  if (countBadge) countBadge.innerText = `${queue.length} Patients`;

  if (!tbody) return;

  if (queue.length === 0) {
    tbody.innerHTML = `<tr><td colspan="4" style="text-align: center; color: var(--text-dim); padding: 2rem;">No triaged patients in queue. Click "New Patient Intake" to add one.</td></tr>`;
    return;
  }

  tbody.innerHTML = queue.map(p => {
    const name = `${p.patient_details.first_name} ${p.patient_details.last_name}`;
    const esi = p.acuity?.esi_level || 5;
    const severity = p.acuity?.severity || 'MINIMAL';
    const rawScore = p.acuity?.raw_score || 0;
    const status = p.status || 'TRIAGED';
    const statusClass = `status-${status.toLowerCase()}`;
    const deviceBadge = p.device_id
      ? `<span style="font-size:0.7rem; color:var(--sky); background:rgba(56,189,248,0.1); padding:0.15rem 0.4rem; border-radius:4px; margin-left:0.4rem;"><i class="fa-solid fa-microchip"></i> ${p.device_id}</span>`
      : '';

    return `
      <tr>
        <td>
          <div class="patient-name">${name}</div>
          <div class="patient-id-tag">${p.patient_id}${deviceBadge}</div>
        </td>
        <td>
          <span class="esi-tag esi-${esi}">
            ESI ${esi} - ${severity}
          </span>
          <span class="status-tag ${statusClass}" style="margin-left: 0.4rem;">
            ${status.replace('_', ' ')}
          </span>
        </td>
        <td>
          <span style="font-family: 'JetBrains Mono'; font-weight: 700;">${rawScore} pts</span>
        </td>
        <td>
          <button class="btn-secondary" style="padding: 0.35rem 0.75rem; font-size: 0.75rem;" onclick="viewPatientDetail('${p.patient_id}')">
            View Profile
          </button>
        </td>
      </tr>
    `;
  }).join('');
}

// Modals Handling
function openIntakeModal() {
  if (!currentDoctor) {
    alert("Doctor authentication required. Please log in to access Patient Intake.");
    openAuthModal();
    return;
  }
  const modal = document.getElementById("intakeModal");
  if (modal) modal.classList.add("active");
}

function closeIntakeModal() {
  const modal = document.getElementById("intakeModal");
  if (modal) modal.classList.remove("active");
}

function closeSummaryModal() {
  const modal = document.getElementById("summaryModal");
  if (modal) modal.classList.remove("active");
}

function updatePainBadge(val) {
  const badge = document.getElementById("painBadge");
  if (badge) badge.innerText = val;
}

// Form Submission with Loading & Click Protection
async function handleIntakeSubmit(e) {
  e.preventDefault();
  const btn = document.getElementById("btnSubmitIntake");
  setButtonLoading(btn, true, "Submitting Intake...");

  const symptoms = Array.from(document.querySelectorAll('input[name="symptom"]:checked')).map(el => el.value);
  const history = Array.from(document.querySelectorAll('input[name="history"]:checked')).map(el => el.value);
  const assignedDevice = document.getElementById("assignedDeviceId")?.value || null;

  const payload = {
    device_id: assignedDevice,
    patient_details: {
      first_name: document.getElementById("firstName").value,
      last_name: document.getElementById("lastName").value,
      date_of_birth: document.getElementById("dob").value || "Unknown",
      gender: document.getElementById("gender").value
    },
    chief_complaint: document.getElementById("chiefComplaint").value,
    pain_level: parseInt(document.getElementById("painLevel").value),
    symptoms: symptoms,
    medical_history: history
  };

  try {
    const res = await apiFetch('/api/patient-intake', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });

    if (res.ok) {
      const result = await res.json();
      closeIntakeModal();
      fetchQueue();
      viewPatientDetail(result.patient_id);
    } else {
      alert("Failed to submit patient intake.");
    }
  } catch (err) {
    alert("Failed to submit patient intake.");
  } finally {
    setButtonLoading(btn, false);
  }
}

// View Patient Detail
async function viewPatientDetail(pId) {
  try {
    const res = await apiFetch(`/api/triage/patient/${pId}`);
    if (!res.ok) return;
    const p = await res.json();

    const name = `${p.patient_details.first_name} ${p.patient_details.last_name}`;
    document.getElementById("summaryModalTitle").innerText = `Triage Profile: ${name} (${p.patient_id})`;

    const esi = p.acuity?.esi_level || 5;
    const factors = p.acuity?.contributing_factors || [];
    const status = p.status || 'TRIAGED';
    const assignedDev = p.device_id || 'Unassigned';

    let markdownHTML = (p.medical_summary || '')
      .replace(/### (.*?)\n/g, '<h3>$1</h3>')
      .replace(/• (.*?)\n/g, '<li>$1</li>')
      .replace(/\n\n/g, '<br>');

    document.getElementById("summaryModalContent").innerHTML = `
      <div style="display: flex; gap: 1rem; align-items: center; justify-content: space-between; margin-bottom: 1.25rem;">
        <div style="display: flex; gap: 0.75rem; align-items: center;">
          <span class="esi-tag esi-${esi}" style="font-size: 0.85rem; padding: 0.4rem 0.85rem;">
            ESI Level ${esi} — ${p.acuity?.severity}
          </span>
          <span class="status-tag status-${status.toLowerCase()}" style="font-size: 0.8rem; padding: 0.35rem 0.75rem;">
            Status: ${status.replace('_', ' ')}
          </span>
          <span style="font-size: 0.8rem; padding: 0.35rem 0.75rem; background: rgba(56,189,248,0.1); color: var(--sky); border-radius: 4px; border: 1px solid rgba(56,189,248,0.2);">
            IoT Sensor Device: <strong>${assignedDev}</strong>
          </span>
        </div>
        <span style="color: var(--text-muted); font-size: 0.82rem;">
          Action: <strong>${p.acuity?.action}</strong>
        </span>
      </div>

      <!-- Doctor Triage Administration Control Panel -->
      <div style="background: rgba(255, 255, 255, 0.02); border: 1px solid var(--border); border-radius: var(--radius-md); padding: 1.1rem; margin-bottom: 1.25rem;">
        <div style="font-weight: 700; color: var(--primary); font-size: 0.88rem; margin-bottom: 0.75rem;">
          Doctor Clinical Record Administration
        </div>
        <div class="form-row">
          <div class="form-group" style="margin-bottom: 0;">
            <label class="form-label">Update Patient Status</label>
            <select id="adminPatientStatus" class="form-select">
              <option value="TRIAGED" ${status === 'TRIAGED' ? 'selected' : ''}>TRIAGED (Pending)</option>
              <option value="UNDER_CARE" ${status === 'UNDER_CARE' ? 'selected' : ''}>UNDER CARE (Attending Physician)</option>
              <option value="DISCHARGED" ${status === 'DISCHARGED' ? 'selected' : ''}>DISCHARGED (Complete)</option>
            </select>
          </div>
          <div class="form-group" style="margin-bottom: 0;">
            <label class="form-label">Assign / Reassign IoT Sensor</label>
            <div style="display: flex; gap: 0.5rem;">
              <select id="adminAssignDevice" class="form-select">
                <option value="esp32-01" ${p.device_id === 'esp32-01' ? 'selected' : ''}>esp32-01</option>
                <option value="esp32-02" ${p.device_id === 'esp32-02' ? 'selected' : ''}>esp32-02</option>
                <option value="esp32-03" ${p.device_id === 'esp32-03' ? 'selected' : ''}>esp32-03</option>
              </select>
              <button class="btn-secondary" style="font-size:0.75rem;" onclick="assignDeviceToPatient('${p.patient_id}')">Bind</button>
            </div>
          </div>
        </div>
        <div class="form-group" style="margin-top: 0.75rem; margin-bottom: 0;">
          <label class="form-label">Clinical Action Notes</label>
          <input type="text" id="adminDoctorNotes" class="form-input" placeholder="e.g. Administered IV fluids..." value="${p.doctor_notes || ''}">
        </div>
        <div style="display: flex; gap: 0.75rem; justify-content: flex-end; margin-top: 1rem;">
          <button class="btn-secondary" id="btnDischargeRecord" style="color: var(--crimson); border-color: rgba(248,113,113,0.3);" onclick="dischargePatientRecord('${p.patient_id}')">
            Discharge / Delete Record
          </button>
          <button class="btn-primary" id="btnSaveAdmin" onclick="savePatientAdminStatus('${p.patient_id}')">
            Save Record Update
          </button>
        </div>
      </div>


      <div style="margin-bottom: 1rem;">
        <strong>Contributing Clinical Factors:</strong>
        <ul style="padding-left: 1.1rem; color: var(--gold); margin-top: 0.35rem;">
          ${factors.map(f => `<li>${f}</li>`).join('')}
        </ul>
      </div>

      <div class="summary-box">
        ${markdownHTML}
      </div>
    `;

    document.getElementById("summaryModal").classList.add("active");
  } catch (e) {
    console.error("View patient detail failed:", e);
  }
}

// Doctor Auth & Record Administration Logic
function openAuthModal() {
  const modal = document.getElementById("authModal");
  if (modal) modal.classList.add("active");
}

function closeAuthModal() {
  const modal = document.getElementById("authModal");
  if (modal) modal.classList.remove("active");
}

function switchAuthTab(tab) {
  if (tab === 'login') {
    document.getElementById("loginForm").style.display = "block";
    document.getElementById("signupForm").style.display = "none";
    document.getElementById("tabLogin").style.borderBottom = "2px solid var(--primary)";
    document.getElementById("tabSignup").style.borderBottom = "none";
  } else {
    document.getElementById("loginForm").style.display = "none";
    document.getElementById("signupForm").style.display = "block";
    document.getElementById("tabSignup").style.borderBottom = "2px solid var(--primary)";
    document.getElementById("tabLogin").style.borderBottom = "none";
  }
}

async function checkDoctorSession() {
  const token = localStorage.getItem("doctor_token");
  if (!token) {
    updateDoctorUI(null);
    return;
  }
  try {
    const res = await apiFetch("/api/auth/me");
    if (res.ok) {
      const data = await res.json();
      currentDoctor = data.doctor;
      updateDoctorUI(currentDoctor);
    } else {
      localStorage.removeItem("doctor_token");
      updateDoctorUI(null);
    }
  } catch (e) {
    updateDoctorUI(null);
  }
}

function updateDoctorUI(doctor) {
  const navBtn = document.getElementById("doctorLoginNavBtn");
  const badge = document.getElementById("doctorProfileBadge");
  const intakeBtn = document.getElementById("navIntakeBtn");
  const simBtn = document.getElementById("navSimulateBtn");

  if (doctor) {
    if (navBtn) navBtn.style.display = "none";
    if (badge) badge.style.display = "inline-flex";
    if (intakeBtn) intakeBtn.style.display = "inline-flex";
    if (simBtn) simBtn.style.display = "inline-flex";
    const elName = document.getElementById("docNameText");
    const elDept = document.getElementById("docDeptText");
    if (elName) elName.innerText = doctor.full_name;
    if (elDept) elDept.innerText = `${doctor.department} (${doctor.license_number})`;
  } else {
    if (navBtn) navBtn.style.display = "inline-flex";
    if (badge) badge.style.display = "none";
    if (intakeBtn) intakeBtn.style.display = "none";
    if (simBtn) simBtn.style.display = "none";
  }
}

async function handleLoginSubmit(e) {
  e.preventDefault();
  const btn = document.getElementById("btnLoginSubmit");
  setButtonLoading(btn, true, "Logging in...");

  const u = document.getElementById("loginUsername").value;
  const p = document.getElementById("loginPassword").value;

  try {
    const res = await apiFetch("/api/auth/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ username: u, password: p })
    });
    if (res.ok) {
      const data = await res.json();
      localStorage.setItem("doctor_token", data.token);
      currentDoctor = data.doctor;
      updateDoctorUI(currentDoctor);
      closeAuthModal();
    } else {
      const err = await res.json();
      alert(err.detail || "Invalid doctor login credentials.");
    }
  } catch (err) {
    alert("Doctor login failed.");
  } finally {
    setButtonLoading(btn, false);
  }
}

async function handleSignupSubmit(e) {
  e.preventDefault();
  const btn = document.getElementById("btnSignupSubmit");
  setButtonLoading(btn, true, "Registering...");

  const payload = {
    full_name: document.getElementById("signUpFullName").value,
    license_number: document.getElementById("signUpLicense").value,
    department: document.getElementById("signUpDept").value,
    username: document.getElementById("signUpUsername").value,
    password: document.getElementById("signUpPassword").value
  };

  try {
    const res = await apiFetch("/api/auth/signup", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    if (res.ok) {
      alert("Doctor registration successful! Please log in.");
      switchAuthTab('login');
      document.getElementById("loginUsername").value = payload.username;
      document.getElementById("loginPassword").value = payload.password;
    } else {
      const err = await res.json();
      alert(err.detail || "Doctor registration failed.");
    }
  } catch (err) {
    alert("Registration failed.");
  } finally {
    setButtonLoading(btn, false);
  }
}

async function handleLogout() {
  await apiFetch("/api/auth/logout", { method: "POST" });
  localStorage.removeItem("doctor_token");
  currentDoctor = null;
  updateDoctorUI(null);
}

async function savePatientAdminStatus(pId) {
  const btn = document.getElementById("btnSaveAdmin");
  setButtonLoading(btn, true, "Saving...");

  const status = document.getElementById("adminPatientStatus").value;
  const notes = document.getElementById("adminDoctorNotes").value;

  try {
    const res = await apiFetch(`/api/triage/patient/${pId}/status`, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ status: status, doctor_notes: notes })
    });

    if (res.ok) {
      closeSummaryModal();
      fetchQueue();
    } else {
      alert("Failed to update patient status.");
    }
  } catch (e) {
    alert("Error saving doctor update.");
  } finally {
    setButtonLoading(btn, false);
  }
}

async function dischargePatientRecord(pId) {
  if (!confirm(`Are you sure you want to discharge and delete patient ${pId} from SQLite database?`)) return;

  const btn = document.getElementById("btnDischargeRecord");
  setButtonLoading(btn, true, "Discharging...");

  try {
    const res = await apiFetch(`/api/triage/patient/${pId}`, { method: "DELETE" });
    if (res.ok) {
      closeSummaryModal();
      fetchQueue();
    } else {
      alert("Failed to discharge patient.");
    }
  } catch (e) {
    alert("Error discharging patient.");
  } finally {
    setButtonLoading(btn, false);
  }
}

// Simulation Handlers
async function triggerSimulation() {
  if (!currentDoctor) {
    alert("Doctor authentication required. Please log in to simulate patient data.");
    openAuthModal();
    return;
  }

  const btn = document.getElementById("navSimulateBtn");
  setButtonLoading(btn, true, "Simulating...");

  const sampleNames = [["Amina", "Yusuf"], ["Chidi", "Okonkwo"], ["Fatima", "Bello"], ["Emeka", "Nnamdi"]];
  const sampleComplaints = [
    "Severe crushing chest pain radiating to left arm",
    "High fever with intense migraine and chills",
    "Shortness of breath and wheezing",
    "Persistent lower back pain"
  ];
  const choice = Math.floor(Math.random() * sampleNames.length);

  const payload = {
    patient_details: {
      first_name: sampleNames[choice][0],
      last_name: sampleNames[choice][1],
      date_of_birth: "1985-06-15",
      gender: "Male"
    },
    chief_complaint: sampleComplaints[choice],
    pain_level: Math.floor(Math.random() * 5) + 5,
    symptoms: ["Chest Pain", "Shortness of Breath"],
    medical_history: ["Hypertension"]
  };

  try {
    await apiFetch('/api/patient-intake', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    fetchQueue();
  } catch (e) {
  } finally {
    setButtonLoading(btn, false);
  }
}

async function triggerFingerSimulation() {
  const btn = document.getElementById("btnSimFinger");
  setButtonLoading(btn, true, "Simulating...");

  const simReading = {
    device_id: "esp32-01",
    timestamp_ms: Date.now(),
    bpm: 78,
    bpm_valid: true,
    spo2: 98,
    spo2_valid: true,
    temp_body_c: 36.8,
    temp_body_f: 98.2,
    temp_die_c: 34.5,
    finger_detected: true,
    device_connected: true,
    ptt_ms: 450.0
  };

  try {
    await apiFetch('/readings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(simReading)
    });
  } catch (e) {
  } finally {
    setButtonLoading(btn, false);
  }
}

async function assignDeviceToPatient(pId) {
  const deviceId = document.getElementById("adminAssignDevice")?.value;
  if (!deviceId) return;

  try {
    const res = await apiFetch('/api/sessions/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ patient_id: pId, device_id: deviceId })
    });

    if (res.ok) {
      alert(`IoT Device ${deviceId} bound to patient ${pId}.`);
      closeSummaryModal();
      fetchQueue();
      viewPatientDetail(pId);
    } else {
      alert("Failed to bind device.");
    }
  } catch (e) {
    alert("Error binding device.");
  }
}

