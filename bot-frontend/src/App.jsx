import React, { useState, useEffect, useRef, memo } from 'react';
import mqtt from 'mqtt';
import { Play, Square, Activity, Wifi, MapPin, Terminal, SlidersHorizontal, Gauge, Gamepad2, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, GitBranch, Navigation } from 'lucide-react';
import './App.css';

// ============================================
// DESTINATIONS: 2 turns + stop for each
// Edit routes to match your track layout
// ============================================
const DESTINATIONS = [
  { id: 'or',      name: 'Operating Room', icon: '🏥', route: 'L,R,X' },
  { id: 'med',     name: 'Med Room',       icon: '💊', route: 'R,L,X' },
  { id: 'station', name: 'Bot Station',    icon: '🏠', route: 'S,L,X' },
];

const Slider = memo(({ label, value, onChange, min, max, step, orange }) => {
  const [v, setV] = useState(value);
  useEffect(() => { setV(value); }, [value]);
  return (
    <div className="s-group">
      <div className="s-head">
        <label>{label}</label>
        <span className={`s-val ${orange ? 'orange' : ''}`}>{v}</span>
      </div>
      <input type="range" className={`slider ${orange ? 'orange' : ''}`}
        min={min} max={max} step={step} value={v}
        onChange={e => setV(e.target.value)}
        onMouseUp={() => onChange(Number(v))}
        onTouchEnd={() => onChange(Number(v))}
      />
    </div>
  );
});

const App = () => {
  const [client, setClient] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isBotLive, setIsBotLive] = useState(false);
  const [botStatus, setBotStatus] = useState('Idle');
  const [driveMode, setDriveMode] = useState('line');

  const [kp, setKp] = useState(50);
  const [ki, setKi] = useState(0);
  const [kd, setKd] = useState(30);
  const [baseSpeed, setBaseSpeed] = useState(95);

  const [leftMotor, setLeftMotor] = useState(0);
  const [rightMotor, setRightMotor] = useState(0);
  const [sensors, setSensors] = useState([0, 0, 0, 0, 0]);
  const [logs, setLogs] = useState([]);
  const [navStatus, setNavStatus] = useState('IDLE');
  const logsEndRef = useRef(null);
  const lastHeartbeat = useRef(0);

  // Heartbeat checker
  useEffect(() => {
    const id = setInterval(() => {
      if (lastHeartbeat.current > 0 && Date.now() - lastHeartbeat.current > 1500) setIsBotLive(false);
    }, 1000);
    return () => clearInterval(id);
  }, []);

  // MQTT
  useEffect(() => {
    const mc = mqtt.connect('ws://192.168.137.1:8883');
    mc.on('connect', () => {
      setIsConnected(true);
      ['status', 'logs', 'telemetry', 'sensors', 'nav', 'alive'].forEach(t => mc.subscribe('ankit/bot/' + t));
    });
    mc.on('close', () => { setIsConnected(false); setIsBotLive(false); });
    mc.on('message', (topic, msg) => {
      const m = msg.toString();
      lastHeartbeat.current = Date.now();
      setIsBotLive(true);

      if (topic.endsWith('/status')) setBotStatus(m);
      else if (topic.endsWith('/logs')) setLogs(p => [...p, `[${new Date().toLocaleTimeString()}] ${m}`].slice(-20));
      else if (topic.endsWith('/telemetry')) { const [l, r] = m.split(','); setLeftMotor(+l); setRightMotor(+r); }
      else if (topic.endsWith('/sensors')) setSensors(m.split(',').map(Number));
      else if (topic.endsWith('/nav')) setNavStatus(m);
    });
    setClient(mc);
    return () => mc.end();
  }, []);

  // Actions
  const pub = (t, m) => { if (client && isConnected) client.publish('ankit/bot/' + t, m); };
  const updatePID = (p, i, d) => { setKp(p); setKi(i); setKd(d); pub('pid', `${p},${i},${d}`); };
  const sendCommand = cmd => { pub('command', cmd); setBotStatus(cmd === 'START' ? 'Moving' : 'Idle'); };
  const changeMode = m => { setDriveMode(m); pub('mode', m); };
  const manualDrive = dir => { if (botStatus === 'Moving') pub('manual', dir); };

  // Navigate: sends route → firmware auto-starts
  const navigate = route => {
    pub('route', route);
    setBotStatus('Moving');
  };

  useEffect(() => { logsEndRef.current?.scrollIntoView({ behavior: 'smooth' }); }, [logs]);

  // Nav badge
  const navBadge = (() => {
    if (navStatus.includes('TURNING_LEFT'))       return { text: '↰ Turning Left', cls: 'nav-turning' };
    if (navStatus.includes('TURNING_RIGHT'))      return { text: '↱ Turning Right', cls: 'nav-turning' };
    if (navStatus.includes('CROSSING'))           return { text: '⬆ Crossing', cls: 'nav-crossing' };
    if (navStatus.includes('NODE_DETECTED'))      return { text: '📍 Node', cls: 'nav-node' };
    if (navStatus.includes('ROUTE_LOADED'))       return { text: '📋 Route Loaded', cls: 'nav-loaded' };
    if (navStatus.includes('DESTINATION_REACHED'))return { text: '✅ Arrived!', cls: 'nav-done' };
    if (navStatus.includes('FOLLOWING'))          return { text: '━━ Following', cls: 'nav-follow' };
    if (navStatus.includes('STOPPED'))            return { text: '⏹ Stopped', cls: 'nav-stopped' };
    return { text: '⏸ Idle', cls: 'nav-idle' };
  })();

  return (
    <div className="app">
      <div className="panel">
        <div className="top-bar">
          <div className="logo"><Activity className="icon-pulse" size={18} /><h1>Bot Dashboard</h1></div>
          <div className="top-badges">
            <div className={`nav-badge ${navBadge.cls}`}>{navBadge.text}</div>
            <div className={`badge ${isBotLive ? 'on' : 'off'}`}><Wifi size={12} /><span>{isBotLive ? 'Live' : 'Off'}</span></div>
          </div>
        </div>

        <div className="grid-layout">
          {/* Column 1 */}
          <div className="col">
            <div className="card controls-card">
              <div className="row-controls">
                <button className={`btn pri ${botStatus === 'Moving' ? 'active' : ''}`} onClick={() => sendCommand('START')}><Play size={14} /> Start</button>
                <button className="btn dan" onClick={() => sendCommand('STOP')}><Square size={14} /> Stop</button>
              </div>
              <div className="mode-toggle">
                <button className={`mode-btn ${driveMode === 'line' ? 'active' : ''}`} onClick={() => changeMode('line')}><GitBranch size={12} /> Line</button>
                <button className={`mode-btn ${driveMode === 'manual' ? 'active' : ''}`} onClick={() => changeMode('manual')}><Gamepad2 size={12} /> Manual</button>
              </div>
            </div>

            <div className={`card dpad-card ${driveMode !== 'manual' ? 'dimmed' : ''}`}>
              <div className="dpad-container">
                <div className="dpad">
                  <button className="d-btn up" onPointerDown={() => manualDrive('FWD')} onPointerUp={() => manualDrive('STOP')} onPointerLeave={() => manualDrive('STOP')}><ArrowUp size={20} /></button>
                  <button className="d-btn left" onPointerDown={() => manualDrive('LEFT')} onPointerUp={() => manualDrive('STOP')} onPointerLeave={() => manualDrive('STOP')}><ArrowLeft size={20} /></button>
                  <button className="d-btn right" onPointerDown={() => manualDrive('RIGHT')} onPointerUp={() => manualDrive('STOP')} onPointerLeave={() => manualDrive('STOP')}><ArrowRight size={20} /></button>
                  <button className="d-btn down" onPointerDown={() => manualDrive('BWD')} onPointerUp={() => manualDrive('STOP')} onPointerLeave={() => manualDrive('STOP')}><ArrowDown size={20} /></button>
                </div>
              </div>
            </div>

            <div className="card">
              <div className="card-head sm"><MapPin size={12} className="text-blue" /><h3>Sensors</h3></div>
              <div className="sensor-dots">
                {['FL', 'ML', 'C', 'MR', 'FR'].map((l, i) => (
                  <div key={i} className="dot-wrap"><div className={`dot ${sensors[i] ? 'on' : ''}`}></div><span>{l}</span></div>
                ))}
              </div>
            </div>
          </div>

          {/* Column 2 */}
          <div className="col">
            <div className="card">
              <div className="card-head sm"><SlidersHorizontal size={12} className="text-blue" /><h3>PID Tuning</h3></div>
              <Slider label="Kp" value={kp} onChange={v => updatePID(v, ki, kd)} min={0} max={80} step={0.5} />
              <Slider label="Ki" value={ki} onChange={v => updatePID(kp, v, kd)} min={0} max={20} step={0.1} />
              <Slider label="Kd" value={kd} onChange={v => updatePID(kp, ki, v)} min={0} max={80} step={0.5} />
            </div>

            <div className="card">
              <div className="card-head sm"><Gauge size={12} className="text-orange" /><h3>Speed</h3></div>
              <Slider label="Base Speed" value={baseSpeed} onChange={v => { setBaseSpeed(v); pub('speeds', `${v}`); }} min={0} max={255} step={5} orange />
            </div>

            <div className="card nav-card">
              <div className="card-head sm"><Navigation size={12} className="text-purple" /><h3>Navigate To</h3></div>
              <div className="dest-grid">
                {DESTINATIONS.map(d => (
                  <button key={d.id} className="dest-btn" onClick={() => navigate(d.route)}>
                    <span className="dest-icon">{d.icon}</span>
                    <span className="dest-name">{d.name}</span>
                    <span className="dest-route">{d.route}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* Column 3 */}
          <div className="col">
            <div className="card">
              <div className="card-head sm"><Activity size={12} className="text-blue" /><h3>Motors</h3></div>
              <div className="motor-row"><span>L</span><div className="bar-bg"><div className="bar-fill blue" style={{ width: `${Math.min(Math.abs(leftMotor) / 255 * 100, 100)}%` }}></div></div><span className="mv">{leftMotor}</span></div>
              <div className="motor-row"><span>R</span><div className="bar-bg"><div className="bar-fill green" style={{ width: `${Math.min(Math.abs(rightMotor) / 255 * 100, 100)}%` }}></div></div><span className="mv">{rightMotor}</span></div>
            </div>

            <div className="card terminal-card">
              <div className="card-head sm"><Terminal size={12} className="text-green" /><h3>Logs</h3></div>
              <div className="terminal">
                {logs.length === 0 ? <span className="muted">Waiting for bot...</span> : logs.map((log, i) => <div key={i} className="log">{log}</div>)}
                <div ref={logsEndRef} />
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default App;
