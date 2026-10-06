/* ============================================================
   OPA-FSO 仿真平台 · 动态三维场景引擎（Three.js r128）
   用法：OPA3D.mount(hostEl, config)
   config: { mode:'m1'..'m6', nOPA, beamColor:0x.., wavelength:nm, title }
   内置 5 种场景：direct 直接协同(障碍通信) · relay 多跳中继Mesh
                  free 自由空间(开阔远距) · confined 有限空间(室内/舱内)
                  obstructed 密集障碍通信
   ============================================================ */
(function (global) {
  'use strict';
  const T = global.THREE;
  if (!T) { console.warn('Three.js 未就绪，三维场景不可用'); return; }

  /* BER：优先复用 common.js 口径；common.js 晚加载时用同口径回退 */
  function erfc(x) {
    const sign = x < 0 ? -1 : 1; x = Math.abs(x);
    const t = 1 / (1 + 0.3275911 * x);
    const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-x * x);
    return sign < 0 ? 2 - y : y;
  }
  function qfunc(x) { return 0.5 * erfc(x / Math.SQRT2); }
  function berOf(pr) {
    if (global.OPA && typeof global.OPA.berFromPower === 'function') return global.OPA.berFromPower(pr);
    const snr = Math.pow(10, (95.7 + 2.2 * pr) / 10);
    return Math.min(0.5, Math.max(1e-12, qfunc(Math.sqrt(snr / 2))));
  }

  /* 模块差异化配置（algo 名字最多 5 个，供中继场景） */
  const MOD = {
    m1: { name: 'PAT 闭环跟踪', algo: ['PAT#1', 'PAT#2', 'PAT#3', 'PAT#4', 'PAT#5'] },
    m2: { name: '多波长相干合成', algo: ['λ-A', 'λ-B', 'λ-C', 'λ-D', 'λ-E'] },
    m3: { name: '深度学习波束控制', algo: ['DNN-A', 'DNN-B', 'DNN-C', 'DNN-D', 'DNN-E'] },
    m4: { name: '单片集成 OPA', algo: ['Chip-A', 'Chip-B', 'Chip-C', 'Chip-D', 'Chip-E'] },
    m5: { name: '相位算法优化', algo: ['爬山-A', '遗传-B', '爬山-C', '遗传-D', '爬山-E'] },
    m6: { name: '高速扫描控制', algo: ['FPGA-A', 'FPGA-B', 'FPGA-C', 'FPGA-D', 'FPGA-E'] }
  };

  /* 场景库：env 环境 / layout OPA布局 / obs 障碍物 / R,H 轨迹参数 / cam 相机 */
  const SCENES = {
    direct: {
      label: '直接协同 · 障碍通信',
      env: { g: 60, room: 30, rh: 12, fn: 34, ff: 78 },
      nOPA: 4,
      layout: [[-11, 2.6, -11], [11, 3.4, -10], [-10, 3, 10], [10, 2.4, 11]],
      obs: [
        { b: [6, 4, .4], p: [-4, 2, -2], c: 0x414c6b },
        { b: [.5, 5, 5], p: [5, 2.5, 3], c: 0x48536f },
        { b: [4, .3, 4], p: [2, 1.6, -5], c: 0x3c4765 },
        { b: [.35, 5.5, 6], p: [0, 2.75, 2], c: 0x56618a, dyn: { axis: 'x', amp: 4.5, sp: .5 } }
      ],
      R: [3, 11, 7], H: [1.5, 8, 3.2], origin: [0, 0],
      cam: [[17, 13, 20], [0, 2.5, 0]],
      flags: { obsPos: 1 }, atmos: 0
    },
    free: {
      label: '自由空间 · 开阔远距',
      env: { g: 220, room: 170, rh: 50, fn: 150, ff: 320 },
      nOPA: 3,
      layout: [[-60, 7, -45], [60, 9, -50], [-50, 8, 55]],
      obs: [],
      R: [20, 75, 48], H: [6, 28, 12], origin: [0, 0],
      cam: [[125, 82, 155], [0, 10, 0]],
      flags: { atmos: 1 }, atmos: 6
    },
    confined: {
      label: '有限空间 · 室内/舱内',
      env: { g: 18, room: 12, rh: 5, fn: 11, ff: 28 },
      nOPA: 4,
      layout: [[-5, 3.7, -5], [5, 3.7, -5], [-5, 3.7, 5], [5, 3.7, 5]],
      obs: [
        { b: [5, 2.6, .25], p: [-1, 1.3, .5], c: 0x46517a },
        { b: [.25, 2.6, 5], p: [2.2, 1.3, -1], c: 0x4a557e },
        { b: [2, 1.1, 1.6], p: [-3, .55, 2.6], c: 0x39435f },
        { b: [1.6, .9, 1.2], p: [3.2, .45, 3], c: 0x39435f }
      ],
      R: [1.5, 4.5, 3], H: [1, 4, 2.4], origin: [0, 0],
      cam: [[12, 8.5, 14], [0, 2, 0]],
      flags: {}, atmos: 0
    },
    obstructed: {
      label: '密集障碍通信',
      env: { g: 46, room: 26, rh: 11, fn: 26, ff: 60 },
      nOPA: 4,
      layout: [[-9, 3, -9], [9, 3.6, -8], [-8, 3, 9], [9, 2.6, 9]],
      obs: [
        { b: [5, 3.6, .4], p: [-3, 1.8, -1], c: 0x414c6b },
        { b: [.6, 4.2, 4], p: [4, 2.1, 2], c: 0x48536f },
        { b: [3, .3, 3], p: [-2, 1.5, -4], c: 0x3c4765 },
        { b: [.3, 4, 5], p: [0, 2, 1], c: 0x5a6590, dyn: { axis: 'x', amp: 6, sp: .7 } },
        { b: [5, .25, .3], p: [1, 2.4, 0], c: 0x5a6590, dyn: { axis: 'z', amp: 5, sp: .5, ph: 1.6 } }
      ],
      R: [3, 9, 6], H: [1.5, 7, 3], origin: [0, 0],
      cam: [[15, 11, 18], [0, 2.4, 0]],
      flags: {}, atmos: 0
    },
    relay: {
      label: '多跳中继 Mesh · 信号接力',
      env: { g: 150, room: 120, rh: 34, fn: 95, ff: 240 },
      nOPA: 5,
      layout: [[-50, 5, -20], [-25, 6, -2], [0, 5, 12], [25, 6, -2], [48, 5, 12]],
      obs: [
        { b: [6, 16, 2], p: [-42, 8, -18], c: 0x3a4663 },
        { b: [8, 16, 8], p: [5, 8, -4], c: 0x3a4663 }
      ],
      R: [6, 12, 8], H: [5, 13, 9], origin: [48, 8],
      cam: [[25, 62, 125], [0, 5, 0]],
      flags: { atmos: 1 }, atmos: 3
    },
    hop2: {
      label: '两跳中继 · 2 台 OPA',
      env: { g: 55, room: 36, rh: 14, fn: 30, ff: 72 },
      nOPA: 2,
      layout: [[-28, 5, -5], [0, 6, 12]],
      obs: [
        { b: [10, 14, 10], p: [0, 7, -5], c: 0x3a4663 }
      ],
      R: [2, 8, 4], H: [4, 8, 6], origin: [28, -5],
      cam: [[20, 26, 42], [0, 5, 0]],
      chain: [0, 1, 2], speed: .3, flags: {}, atmos: 0
    },
    hop3: {
      label: '三跳中继 · 3 台 OPA',
      env: { g: 72, room: 48, rh: 18, fn: 40, ff: 95 },
      nOPA: 3,
      layout: [[-40, 5, 0], [-15, 6, 18], [15, 6, 18]],
      obs: [
        { b: [10, 14, 10], p: [0, 7, 0], c: 0x3a4663 }
      ],
      R: [2, 8, 4], H: [4, 8, 6], origin: [40, 0],
      cam: [[26, 30, 48], [0, 6, 0]],
      chain: [0, 1, 2, 3], speed: .3, flags: {}, atmos: 0
    },
    hop4: {
      label: '四跳中继 · 4 台 OPA',
      env: { g: 66, room: 44, rh: 16, fn: 38, ff: 88 },
      nOPA: 4,
      layout: [[-45, 5, 0], [-27, 6, 13], [-9, 5, -13], [12, 6, 13]],
      obs: [
        { b: [6, 14, 10], p: [20, 7, 0], c: 0x3a4663 }
      ],
      R: [1, 4, 2], H: [4, 8, 6], origin: [34, 0],
      cam: [[22, 28, 42], [0, 5, 0]],
      chain: [0, 1, 2, 3, 4], speed: .3, flags: {}, atmos: 0
    }
  };

  function mount(host, cfg) {
    cfg = cfg || {};
    const beamHex = cfg.beamColor || 0x2fd7d7;
    const beamColor = new T.Color(beamHex);
    const redColor = new T.Color(0xf87171);
    const md = MOD[cfg.mode] || MOD.m1;

    /* ---------------- DOM 结构 ---------------- */
    host.innerHTML =
      '<div class="s3d">' +
        '<div class="s3d-view"></div>' +
        '<div class="s3d-hud panel">' +
          '<h4><span class="dot"></span>' + (cfg.title || '三维场景 · 多 OPA 通信') + '</h4>' +
          '<div class="s3d-status"><span class="pill-status" id="s3d-netstate"><span class="b"></span>初始化</span>' +
          '<span class="s3d-switch" id="s3d-switch"></span></div>' +
          '<div class="s3d-links" id="s3d-links"></div>' +
          '<div class="metrics" id="s3d-global"></div>' +
          '<div class="metrics" id="s3d-extra"></div>' +
        '</div>' +
        '<div class="s3d-ctrl panel">' +
          '<h4><span class="dot"></span>场景控制</h4>' +
          '<div class="ctrl"><label>通信场景模式</label>' +
            '<select id="s3d-scene">' +
              '<option value="direct">直接协同 · 障碍通信</option>' +
              '<option value="relay">多跳中继 Mesh · 信号接力</option>' +
              '<option value="free">自由空间 · 开阔远距</option>' +
              '<option value="confined">有限空间 · 室内/舱内</option>' +
              '<option value="obstructed">密集障碍通信</option>' +
              '<option value="hop2">两跳中继 · 2 台 OPA</option>' +
              '<option value="hop3">三跳中继 · 3 台 OPA</option>' +
              '<option value="hop4">四跳中继 · 4 台 OPA</option>' +
            '</select></div>' +
          '<div class="ctrl"><label>目标运动模式</label>' +
            '<select id="s3d-amode"><option value="auto">自动巡航</option><option value="manual">手动控制</option></select></div>' +
          '<div class="ctrl" id="s3d-autogroup"><label>轨迹类型</label>' +
            '<select id="s3d-path"><option value="circle">圆形巡航</option><option value="lissajous">8 字机动</option>' +
            '<option value="rect">矩形巡逻</option><option value="wave">多航点穿梭</option></select></div>' +
          '<div class="ctrl" id="s3d-speedgroup"><label>目标速度 <span class="val"></span></label>' +
            '<input type="range" id="s3d-speed" min="0.2" max="4" step="0.1" value="1"></div>' +
          '<div class="ctrl"><label>轨迹范围 R <span class="val"></span></label>' +
            '<input type="range" id="s3d-range" min="3" max="11" step="0.5" value="7"></div>' +
          '<div class="ctrl"><label>飞行高度 <span class="val"></span></label>' +
            '<input type="range" id="s3d-height" min="1.5" max="8" step="0.1" value="3.2"></div>' +
          '<div class="ctrl s3d-manual" style="display:none"><label>手动 X <span class="val"></span></label>' +
            '<input type="range" id="s3d-mx" min="-11" max="60" step="0.2" value="0"></div>' +
          '<div class="ctrl s3d-manual" style="display:none"><label>手动 Y(高) <span class="val"></span></label>' +
            '<input type="range" id="s3d-my" min="0.5" max="28" step="0.1" value="3"></div>' +
          '<div class="ctrl s3d-manual" style="display:none"><label>手动 Z <span class="val"></span></label>' +
            '<input type="range" id="s3d-mz" min="-11" max="60" step="0.2" value="4"></div>' +
          '<div class="ctrl"><label>发射功率 Pt <span class="val"></span></label>' +
            '<input type="range" id="s3d-pt" min="0" max="26" step="1" value="15"></div>' +
          '<div class="ctrl" id="s3d-atmosgroup" style="display:none"><label>大气衰减(天气) <span class="val"></span></label>' +
            '<input type="range" id="s3d-atmos" min="0" max="60" step="1" value="6"></div>' +
          '<div class="switch-row" id="s3d-dynrow"><span>动态遮挡物</span><label class="switch">' +
            '<input type="checkbox" id="s3d-dynobs" checked><span class="slider-sw"></span></label></div>' +
          '<div class="ctrl" id="s3d-obsposgroup"><label>遮挡物位置 <span class="val"></span></label>' +
            '<input type="range" id="s3d-obspos" min="-6" max="6" step="0.2" value="0"></div>' +
          '<div class="btn-row"><button class="btn sm" id="s3d-pause">暂停</button>' +
            '<button class="btn sm ghost" id="s3d-reset">重置视角</button></div>' +
          '<div class="note" id="s3d-hint" style="margin:12px 0 0;font-size:11.5px"></div>' +
        '</div>' +
      '</div>';

    const view = host.querySelector('.s3d-view');

    /* ---------------- 渲染器 / 场景 / 相机 ---------------- */
    const renderer = new T.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(2, global.devicePixelRatio || 1));
    renderer.shadowMap.enabled = true; renderer.shadowMap.type = T.PCFSoftShadowMap;
    view.appendChild(renderer.domElement);

    const scene = new T.Scene();
    scene.background = new T.Color(0x070b14);
    scene.fog = new T.Fog(0x070b14, 34, 78);

    const camera = new T.PerspectiveCamera(52, 1, 0.1, 1000);
    camera.position.set(17, 13, 20);
    const controls = new T.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = .08;
    controls.maxPolarAngle = Math.PI * .495; controls.target.set(0, 2.5, 0);

    /* ---------------- 灯光 ---------------- */
    scene.add(new T.AmbientLight(0x8fa8d8, .55));
    scene.add(new T.HemisphereLight(0x9fc4ff, 0x140d1c, .5));
    const sun = new T.DirectionalLight(0xffffff, .95);
    sun.position.set(12, 30, 10); sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -90; sun.shadow.camera.right = 90;
    sun.shadow.camera.top = 90; sun.shadow.camera.bottom = -90; sun.shadow.camera.far = 200;
    scene.add(sun);

    /* ---------------- 环境组（地面/网格/房间，可重建） ---------------- */
    const envGroup = new T.Group(); scene.add(envGroup);
    function buildEnv(E) {
      while (envGroup.children.length) envGroup.remove(envGroup.children[0]);
      const ground = new T.Mesh(new T.PlaneGeometry(E.g * 1.6, E.g * 1.6),
        new T.MeshStandardMaterial({ color: 0x0b1120, roughness: .95, metalness: .1 }));
      ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true; envGroup.add(ground);
      const grid = new T.GridHelper(E.g * 1.6, E.g * 1.6 / 2, 0x2a4a6a, 0x182740);
      grid.material.transparent = true; grid.material.opacity = .5; envGroup.add(grid);
      const room = new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(E.room, E.rh, E.room)),
        new T.LineBasicMaterial({ color: 0x22355a, transparent: true, opacity: .35 }));
      room.position.y = E.rh / 2; envGroup.add(room);
      scene.fog = new T.Fog(scene.background, E.fn, E.ff);
    }

    /* ---------------- 纹理工具 ---------------- */
    function glowTexture(hex) {
      const c = document.createElement('canvas'); c.width = c.height = 128;
      const x = c.getContext('2d');
      const g = x.createRadialGradient(64, 64, 0, 64, 64, 64);
      const col = '#' + new T.Color(hex).getHexString();
      g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(.25, col);
      g.addColorStop(1, 'rgba(0,0,0,0)');
      x.fillStyle = g; x.fillRect(0, 0, 128, 128);
      return new T.CanvasTexture(c);
    }
    function label(text, color) {
      const c = document.createElement('canvas'); c.width = 256; c.height = 72;
      const x = c.getContext('2d');
      x.font = "bold 32px 'JetBrains Mono',sans-serif";
      x.fillStyle = color || '#8fe3e6'; x.textAlign = 'center';
      x.shadowColor = color || '#2fd7d7'; x.shadowBlur = 12;
      x.fillText(text, 128, 48);
      const sp = new T.Sprite(new T.SpriteMaterial({ map: new T.CanvasTexture(c), transparent: true, depthWrite: false }));
      sp.scale.set(2.2, .62, 1); return sp;
    }

    /* ---------------- OPA 设备建模 ---------------- */
    function buildOPA(i) {
      const g = new T.Group();
      const metal = new T.MeshStandardMaterial({ color: 0x2a3550, roughness: .4, metalness: .75 });
      const dark = new T.MeshStandardMaterial({ color: 0x141c2e, roughness: .6, metalness: .5 });
      const base = new T.Mesh(new T.BoxGeometry(1.7, .28, 1.3), metal);
      base.position.y = .14; base.castShadow = true; g.add(base);
      const stand = new T.Mesh(new T.BoxGeometry(.3, .9, .3), dark);
      stand.position.y = .7; stand.castShadow = true; g.add(stand);
      const head = new T.Group(); head.position.y = 1.25; g.add(head);
      const body = new T.Mesh(new T.BoxGeometry(1.5, .8, 1.0),
        new T.MeshStandardMaterial({ color: 0x1b2742, roughness: .35, metalness: .6 }));
      body.castShadow = true; head.add(body);
      const face = new T.Mesh(new T.BoxGeometry(1.3, .62, .08),
        new T.MeshStandardMaterial({ color: 0x0d2b33, roughness: .2, metalness: .3,
          emissive: beamHex, emissiveIntensity: .55 }));
      face.position.z = .52; head.add(face);
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) {
        const lens = new T.Mesh(new T.SphereGeometry(.05, 10, 10),
          new T.MeshBasicMaterial({ color: beamHex }));
        lens.position.set(a * .4, b * .2, .58); head.add(lens);
      }
      const led = new T.Mesh(new T.SphereGeometry(.07, 10, 10),
        new T.MeshBasicMaterial({ color: 0x4ade9a }));
      led.position.set(.6, .45, .2); head.add(led);
      const origin = new T.Object3D(); origin.position.set(0, 0, .62); head.add(origin);
      const lb = label(md.algo[i], '#' + beamColor.getHexString());
      lb.position.y = .95; g.add(lb);
      g.userData = { head, origin, led, lockT: 0 };
      return g;
    }

    /* ---------------- 接收终端（无人机） ---------------- */
    function buildReceiver() {
      const g = new T.Group();
      const body = new T.Mesh(new T.BoxGeometry(.9, .35, .9),
        new T.MeshStandardMaterial({ color: 0x22304d, roughness: .4, metalness: .6 }));
      body.castShadow = true; g.add(body);
      const rotors = [];
      [[-.5, -.5], [.5, -.5], [-.5, .5], [.5, .5]].forEach(p => {
        const arm = new T.Mesh(new T.BoxGeometry(.5, .05, .05),
          new T.MeshStandardMaterial({ color: 0x33405f }));
        arm.position.set(p[0] * .7, .12, p[1] * .7); g.add(arm);
        const r = new T.Mesh(new T.BoxGeometry(.55, .02, .08),
          new T.MeshBasicMaterial({ color: 0x6c86b8, transparent: true, opacity: .7 }));
        r.position.set(p[0] * .95, .16, p[1] * .95); g.add(r); rotors.push(r);
      });
      const rl = new T.Mesh(new T.SphereGeometry(.1, 12, 12),
        new T.MeshBasicMaterial({ color: 0xf6b942 }));
      rl.position.y = .3; g.add(rl);
      const lb = label('终端 RX', '#f6b942'); lb.position.y = 1; g.add(lb);
      g.userData = { rotors, rl };
      return g;
    }
    const receiver = buildReceiver(); scene.add(receiver);

    /* ---------------- 动态世界组（OPA/障碍物，随场景重建） ---------------- */
    const world = new T.Group(); scene.add(world);
    let opas = [], obstacles = [], dynObs = [];

    function clearWorld() {
      while (world.children.length) {
        const c = world.children[0]; world.remove(c);
        c.traverse(o => {
          if (o.geometry) o.geometry.dispose();
          if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => m.dispose());
        });
      }
      opas = []; obstacles = []; dynObs = [];
    }
    function buildWorld(S) {
      clearWorld();
      for (let i = 0; i < S.nOPA; i++) {
        const o = buildOPA(i);
        o.position.set(S.layout[i][0], S.layout[i][1], S.layout[i][2]);
        world.add(o); opas.push(o);
      }
      S.obs.forEach(d => {
        const m = new T.Mesh(new T.BoxGeometry(d.b[0], d.b[1], d.b[2]),
          new T.MeshStandardMaterial({ color: d.c || 0x3a4663, roughness: .8, metalness: .15,
            transparent: true, opacity: .93 }));
        m.position.set(d.p[0], d.p[1], d.p[2]);
        m.castShadow = true; m.receiveShadow = true;
        world.add(m); obstacles.push(m);
        if (d.dyn) dynObs.push({ mesh: m, base: d.p.slice(), cf: d.dyn });
      });
    }

    /* ---------------- 光束（固定 6 条，按 visible 复用） ---------------- */
    const glowTex = glowTexture(beamHex), redGlowTex = glowTexture(0xf87171);
    const beams = [];
    for (let i = 0; i < 6; i++) {
      const grp = new T.Group(); grp.visible = false; scene.add(grp);
      const cyl = new T.Mesh(new T.CylinderGeometry(.06, .09, 1, 10, 1, true),
        new T.MeshBasicMaterial({ color: beamHex, transparent: true, opacity: .22,
          blending: T.AdditiveBlending, depthWrite: false }));
      grp.add(cyl);
      const core = new T.Mesh(new T.CylinderGeometry(.015, .015, 1, 6, 1, true),
        new T.MeshBasicMaterial({ color: beamHex, transparent: true, opacity: .8,
          blending: T.AdditiveBlending, depthWrite: false }));
      grp.add(core);
      const end = new T.Sprite(new T.SpriteMaterial({ map: glowTex, blending: T.AdditiveBlending,
        depthWrite: false, transparent: true }));
      end.scale.set(1.4, 1.4, 1); grp.add(end);
      const start = new T.Sprite(new T.SpriteMaterial({ map: glowTex, blending: T.AdditiveBlending,
        depthWrite: false, transparent: true }));
      start.scale.set(.8, .8, 1); grp.add(start);
      beams.push({ grp, cyl, core, end, start });
    }
    const Y_AXIS = new T.Vector3(0, 1, 0);
    function placeBeam(b, from, to, los) {
      const dir = new T.Vector3().subVectors(to, from);
      const len = dir.length();
      const mid = new T.Vector3().addVectors(from, to).multiplyScalar(.5);
      b.grp.position.copy(mid);
      b.grp.quaternion.setFromUnitVectors(Y_AXIS, dir.clone().normalize());
      b.cyl.scale.set(1, len, 1); b.core.scale.set(1, len, 1);
      const col = los ? beamColor : redColor;
      b.cyl.material.color.copy(col); b.core.material.color.copy(col);
      b.cyl.material.opacity = los ? .22 : .3;
      const qi = b.grp.quaternion.clone().invert();
      b.end.position.copy(new T.Vector3().subVectors(to, mid).applyQuaternion(qi));
      b.start.position.copy(new T.Vector3().subVectors(from, mid).applyQuaternion(qi));
      b.end.material.map = los ? glowTex : redGlowTex;
      b.end.scale.set(los ? 1.4 : 1.1, los ? 1.4 : 1.1, 1);
      b.grp.visible = true;
    }
    function hideBeams(from) { for (let i = from || 0; i < beams.length; i++) beams[i].grp.visible = false; }

    /* ---------------- Mesh 淡网（仅 relay） ---------------- */
    const meshLine = new T.LineSegments(new T.BufferGeometry(),
      new T.LineBasicMaterial({ color: beamHex, transparent: true, opacity: .28 }));
    meshLine.visible = false; scene.add(meshLine);

    /* ---------------- 控件引用 / 状态 ---------------- */
    const ui = {};
    ['s3d-scene', 's3d-amode', 's3d-path', 's3d-speed', 's3d-range', 's3d-height',
     's3d-mx', 's3d-my', 's3d-mz', 's3d-pt', 's3d-atmos', 's3d-dynobs', 's3d-obspos',
     's3d-pause', 's3d-reset', 's3d-links', 's3d-global', 's3d-extra', 's3d-netstate',
     's3d-switch', 's3d-atmosgroup', 's3d-dynrow', 's3d-obsposgroup', 's3d-hint',
     's3d-autogroup', 's3d-speedgroup'].forEach(id => ui[id] = host.querySelector('#' + id));

    let tt = 0, paused = false, activePrev = -1, S = SCENES.direct;
    const raycaster = new T.Raycaster();

    const HINTS = {
      direct: '每台 OPA 直连终端，光束遇障变红断链，系统自动选视距内最佳 OPA。',
      relay: '源 OPA 距离过远且直连被建筑遮挡，经再生中继逐跳把信号接力传到终端；淡青色为可用 mesh 邻接，亮光束为活动路由。',
      free: '开阔无遮挡的远距离 FSO，链路主要受路径损耗与大气衰减(天气)影响，拖动大气衰减可模拟雾天。',
      confined: '封闭室内/舱内，OPA 置于四角高处，隔断与设备形成遮挡，距离短、遮挡频繁。',
      obstructed: '多个静态与两块正交运动的遮挡板，链路频繁断开重连，考验多 OPA 快速切换。',
      hop2: '源 OPA 直连终端被建筑遮挡，系统下达指令：源把信号发给唯一可同时连通双方的中继 OPA，再由它转发终端（红色为被挡的直连尝试，亮色为实际两跳路由）。',
      hop3: '源 OPA 经两台中继、三跳把信号绕过建筑送到终端；再生中继逐跳恢复信号，端到端 BER 由各跳累积。',
      hop4: '源 OPA 经三台中继、四跳沿折线绕过遮挡到达缓慢移动的终端；跳数越多越能体现中继覆盖与误码累积的权衡。'
    };

    /* ---------------- 场景切换 ---------------- */
    function setRange(el, lo, hi, val) {
      el.min = lo; el.max = hi; el.value = val;
      const out = el.parentElement.querySelector('.val');
      if (out) out.textContent = el.value;
    }
    function buildScene(key) {
      S = SCENES[key];
      buildEnv(S.env);
      buildWorld(S);
      hideBeams();
      tt = 0; activePrev = -1;
      // 终端速度（中继场景默认缓慢）
      ui['s3d-speed'].value = S.speed !== undefined ? S.speed : 1;
      const spOut = ui['s3d-speed'].parentElement.querySelector('.val');
      if (spOut) spOut.textContent = ui['s3d-speed'].value;
      // 轨迹滑块范围
      setRange(ui['s3d-range'], S.R[0], S.R[1], S.R[2]);
      setRange(ui['s3d-height'], S.H[0], S.H[1], S.H[2]);
      setRange(ui['s3d-atmos'], 0, 60, S.atmos);
      // 控件显隐
      ui['s3d-dynrow'].style.display = dynObs.length ? '' : 'none';
      ui['s3d-obsposgroup'].style.display = S.flags.obsPos ? '' : 'none';
      ui['s3d-atmosgroup'].style.display = S.flags.atmos ? '' : 'none';
      meshLine.visible = key === 'relay';
      ui['s3d-hint'].textContent = HINTS[key];
      // 相机
      camera.position.set(S.cam[0][0], S.cam[0][1], S.cam[0][2]);
      controls.target.set(S.cam[1][0], S.cam[1][1], S.cam[1][2]);
    }

    /* ---------------- 目标位置 ---------------- */
    function targetPos() {
      const R = +ui['s3d-range'].value, H = +ui['s3d-height'].value;
      const ox = S.origin[0], oz = S.origin[1];
      if (ui['s3d-amode'].value === 'manual') {
        return new T.Vector3(+ui['s3d-mx'].value, +ui['s3d-my'].value, +ui['s3d-mz'].value);
      }
      const x = tt * +ui['s3d-speed'].value;
      const path = ui['s3d-path'].value;
      if (path === 'circle') return new T.Vector3(ox + R * Math.sin(x), H, oz + R * Math.cos(x));
      if (path === 'lissajous') return new T.Vector3(ox + R * Math.sin(x), H + .8 * Math.cos(x * 2), oz + R * .7 * Math.sin(x * 2));
      if (path === 'rect') {
        const c = 2.4, u = (x % (c * 4)) / c, e = Math.floor(u), f = u - e;
        const cs = [[-R, -R], [R, -R], [R, R], [-R, R]];
        const a = cs[e], b = cs[(e + 1) % 4];
        return new T.Vector3(ox + a[0] + (b[0] - a[0]) * f, H, oz + a[1] + (b[1] - a[1]) * f);
      }
      return new T.Vector3(ox + R * Math.sin(x * .8) * Math.cos(x * .35),
        H + 1.2 * Math.sin(x * 1.3), oz + R * Math.cos(x * .6));
    }

    /* ---------------- 链路质量 ---------------- */
    function linkQuality(dist, los, pt, atmos) {
      let pr = pt - 20 - 20 * Math.log10(Math.max(.5, dist));
      if (atmos) pr -= dist / 1000 * atmos;
      if (!los) pr -= 45;
      return { pr, ber: berOf(pr) };
    }
    function fmtBer(b) {
      if (b >= .1) return b.toFixed(2);
      if (b === 0) return '~0';
      return b.toExponential(1);
    }
    function losBetween(A, B) {
      const dir = new T.Vector3().subVectors(B, A);
      const dist = dir.length();
      raycaster.set(A, dir.normalize()); raycaster.far = dist;
      const hits = raycaster.intersectObjects(obstacles, false);
      return { los: hits.length === 0, point: hits.length ? hits[0].point : B, dist };
    }

    /* ---------------- 直连系更新（direct/free/confined/obstructed） ---------------- */
    function updateStar(tp, dt, pt, atmos) {
      const links = [];
      opas.forEach((o, i) => {
        o.updateMatrixWorld(true);
        const origin = new T.Vector3();
        o.userData.origin.getWorldPosition(origin);
        const r = losBetween(origin, tp);
        placeBeam(beams[i], origin, r.point, r.los);
        o.userData.head.lookAt(tp);
        if (r.los && !paused) o.userData.lockT += dt;
        else o.userData.lockT = Math.max(0, o.userData.lockT - dt * 2);
        o.userData.led.material.color.set(r.los ? 0x4ade9a : 0xf87171);
        const q = linkQuality(r.dist, r.los, pt, atmos);
        links.push({ dist: r.dist, los: r.los, pr: q.pr, ber: q.ber });
      });
      hideBeams(opas.length);
      let active = -1, best = -Infinity;
      links.forEach((l, i) => { if (l.los && l.pr > best) { best = l.pr; active = i; } });
      return { links, active };
    }

    /* ---------------- Mesh 中继更新（relay） ---------------- */
    const BER_GATE = 1e-2;
    function dijkstra(edges, N, src, dst) {
      const adj = {};
      edges.forEach(e => {
        if (!e.usable) return;
        (adj[e.a] = adj[e.a] || []).push([e.b, e]);
        (adj[e.b] = adj[e.b] || []).push([e.a, e]);
      });
      const d = {}, pv = {}, seen = {};
      d[src] = 0;
      for (;;) {
        let u = -1, best = Infinity;
        for (let i = 0; i < N; i++)
          if (!seen[i] && d[i] !== undefined && d[i] < best) { best = d[i]; u = i; }
        if (u < 0) break;
        if (u === dst) break;
        seen[u] = 1;
        (adj[u] || []).forEach(([v, e]) => {
          const nd = d[u] + e.dist;
          if (d[v] === undefined || nd < d[v]) { d[v] = nd; pv[v] = [u, e]; }
        });
      }
      if (pv[dst] === undefined) return null;
      const seq = []; let cur = dst;
      while (cur !== src) { const p = pv[cur]; seq.push(p[1]); cur = p[0]; }
      seq.reverse(); return seq;
    }
    function updateRelay(tp, dt, pt, atmos) {
      const n = opas.length, N = n + 1;
      const nodePos = [];
      opas.forEach(o => {
        o.updateMatrixWorld(true);
        const v = new T.Vector3(); o.userData.origin.getWorldPosition(v); nodePos.push(v);
      });
      nodePos.push(tp.clone());
      // 所有节点对
      const edges = [];
      for (let a = 0; a < N; a++) for (let b = a + 1; b < N; b++) {
        const r = losBetween(nodePos[a], nodePos[b]);
        const q = linkQuality(r.dist, r.los, pt, atmos);
        edges.push({ a, b, dist: r.dist, los: r.los, pr: q.pr, ber: q.ber,
          usable: r.los && q.ber <= BER_GATE });
      }
      const path = dijkstra(edges, N, 0, N - 1);
      // 淡网：所有可用邻接
      const verts = [];
      edges.forEach(e => {
        if (!e.usable) return;
        verts.push(nodePos[e.a].x, nodePos[e.a].y, nodePos[e.a].z);
        verts.push(nodePos[e.b].x, nodePos[e.b].y, nodePos[e.b].z);
      });
      meshLine.geometry.dispose();
      meshLine.geometry = new T.BufferGeometry();
      meshLine.geometry.setAttribute('position', new T.Float32BufferAttribute(verts, 3));

      // 头部朝向：默认朝终端，路径上的朝下一跳
      opas.forEach(o => { o.userData.head.lookAt(tp); o.userData.led.material.color.set(0xf87171); });
      if (!path) { hideBeams(); return { links: [], active: -1, edges, hops: [] }; }
      // 节点序列
      const nodes = [0];
      path.forEach(e => { const prev = nodes[nodes.length - 1]; nodes.push(e.a === prev ? e.b : e.a); });
      path.forEach((e, k) => {
        placeBeam(beams[k], nodePos[e.a], nodePos[e.b], true);
        const ai = nodes[k];
        opas[ai].userData.head.lookAt(nodePos[nodes[k + 1]]);
        opas[ai].userData.led.material.color.set(0x4ade9a);
        opas[ai].userData.lockT = Math.min(3, opas[ai].userData.lockT + dt);
      });
      hideBeams(path.length);
      return { links: path, active: path.length ? 0 : -1, edges, hops: path, nodes };
    }

    /* ---------------- 指令驱动 链式中继（hop2/hop3/hop4） ---------------- */
    function updateChain(tp, dt, pt, atmos) {
      const n = opas.length;
      const nodePos = [];
      opas.forEach(o => {
        o.updateMatrixWorld(true);
        const v = new T.Vector3(); o.userData.origin.getWorldPosition(v); nodePos.push(v);
      });
      nodePos.push(tp.clone());
      const ch = S.chain, hops = [];
      let brokenAt = -1;
      for (let k = 0; k < ch.length - 1; k++) {
        const A = nodePos[ch[k]], B = nodePos[ch[k + 1]];
        const r = losBetween(A, B);
        const q = linkQuality(r.dist, r.los, pt, atmos);
        hops.push({ a: ch[k], b: ch[k + 1], dist: r.dist, los: r.los, point: r.point, pr: q.pr, ber: q.ber });
        if (!r.los && brokenAt < 0) brokenAt = k;
      }
      // 源 -> 终端 直连（被挡，红色提示）
      const dr = losBetween(nodePos[0], nodePos[n]);
      const dq = linkQuality(dr.dist, dr.los, pt, atmos);
      // 头部默认朝终端
      opas.forEach(o => { o.userData.head.lookAt(tp); o.userData.led.material.color.set(0xf87171); });
      // 中继各跳
      hops.forEach((h, k) => {
        placeBeam(beams[k], nodePos[h.a], h.los ? nodePos[h.b] : h.point, h.los);
        opas[h.a].userData.head.lookAt(nodePos[h.b]);
        if (h.los) {
          opas[h.a].userData.led.material.color.set(0x4ade9a);
          opas[h.a].userData.lockT = Math.min(3, opas[h.a].userData.lockT + dt);
        } else { opas[h.a].userData.lockT = 0; }
      });
      // 源直连红色光束（占用最后一条 beam）
      placeBeam(beams[hops.length], nodePos[0], dr.point, dr.los);
      hideBeams(hops.length + 1);
      // 端到端 BER（再生中继）
      let e2e = 0, totalD = 0;
      hops.forEach(h => { e2e = 1 - (1 - e2e) * (1 - h.ber); totalD += h.dist; });
      return { hops, e2e, totalD, brokenAt, complete: brokenAt < 0,
        direct: { los: dr.los, dist: dr.dist, pr: dq.pr, ber: dq.ber } };
    }
    function hudChain(r, n) {
      let html = '';
      r.hops.forEach((h, k) => {
        html += '<div class="lk ' + (h.los ? 'active hop' : 'nlos hop') + '"><span class="lk-n">H' + (k + 1) + ' · ' +
          nodeName(h.a, n) + '→' + nodeName(h.b, n) + '</span>' +
          '<span class="lk-badge">' + (h.los ? 'HOP' : '断') + '</span>' +
          '<span class="lk-d">' + h.dist.toFixed(1) + 'm</span>' +
          '<span class="lk-pr">' + h.pr.toFixed(0) + ' dBm</span>' +
          '<span class="lk-ber">BER ' + fmtBer(h.ber) + '</span></div>';
      });
      const d = r.direct;
      html += '<div class="lk nlos"><span class="lk-n">直连 · ' + nodeName(0, n) + '→终端</span>' +
        '<span class="lk-badge">遮挡</span>' +
        '<span class="lk-d">' + d.dist.toFixed(1) + 'm</span>' +
        '<span class="lk-pr">' + d.pr.toFixed(0) + ' dBm</span>' +
        '<span class="lk-ber">BER ' + fmtBer(d.ber) + '</span></div>';
      ui['s3d-links'].innerHTML = html;
      const net = ui['s3d-netstate'];
      if (r.complete) {
        net.className = 'pill-status run'; net.style.color = '';
        net.innerHTML = '<span class="b"></span>指令中继连通 · ' + r.hops.length + ' 跳 → 终端';
      } else {
        net.className = 'pill-status'; net.style.color = '#f87171';
        net.innerHTML = '<span class="b" style="background:#f87171"></span>链路中断 @ H' + (r.brokenAt + 1);
      }
      ui['s3d-global'].innerHTML =
        gmet('端到端 BER', r.complete ? fmtBer(r.e2e) : '0.5', r.complete && r.e2e < 1e-3 ? 'green' : 'amber') +
        gmet('中继跳数', '' + r.hops.length, 'cyan');
      ui['s3d-extra'].innerHTML =
        gmet('指令状态', r.complete ? '已执行 · 转发中' : '等待链路', '', r.complete ? '#4ade9a' : '#f6b942') +
        gmet('源直连终端', d.los ? '视距通' : '被建筑遮挡', '', d.los ? '#4ade9a' : '#f87171') +
        gmet('端到端距离', r.totalD.toFixed(0) + ' m', '', '#f6b942') +
        gmet('中继节点', '' + (r.hops.length - 1), '', '#2fd7d7');
    }

    /* ---------------- 模块特有指标（直连系） ---------------- */
    function extraStar(links, active) {
      const a = active >= 0 ? links[active] : null;
      const lock = a ? Math.min(1, opas[active].userData.lockT / 2.2) : 0;
      const nLos = links.filter(l => l.los).length;
      switch (cfg.mode) {
        case 'm1': return [
          ['跟踪误差', a ? (.9 * (1 - lock) + .06).toFixed(2) + '°' : '—', lock > .6 ? '#4ade9a' : '#f6b942'],
          ['锁定状态', lock > .8 ? '已锁定' : (a ? '捕获中' : '失锁'), lock > .8 ? '#4ade9a' : '#f87171']];
        case 'm2': return [
          ['相干 OPA 数', '' + nLos, '#2fd7d7'],
          ['相干增益', 10 * Math.log10(Math.max(1, nLos)).toFixed(1) + ' dB', '#e26bd6']];
        case 'm3': return [
          ['波束 SMSR', a ? (6 + lock * 6).toFixed(2) + ' dB' : '—', '#2fd7d7'],
          ['DNN 置信', a ? (.55 + .44 * lock).toFixed(2) : '—', '#e26bd6']];
        case 'm4': return [
          ['主瓣 HPBW', a ? (.55 + .15 * (1 - lock)).toFixed(2) + '°' : '—', '#2fd7d7'],
          ['辐射效率', a ? (40 + 12 * lock).toFixed(0) + '%' : '—', '#f6b942']];
        case 'm5': {
          const alg = active >= 0 ? md.algo[active] : '—';
          return [['当前算法', alg, alg.indexOf('遗传') >= 0 ? '#e26bd6' : '#2fd7d7'],
            ['锁定迭代', a ? Math.round(lock * 360) : '—', '#f6b942']];
        }
        case 'm6': return [
          ['扫描帧率', a ? (2000 + lock * 8000).toFixed(0) + ' fps' : '—', '#2fd7d7'],
          ['相位配置', '<5 μs', '#f6b942']];
        default: return [['—', '—', '#fff']];
      }
    }

    /* ---------------- HUD ---------------- */
    function gmet(label, val, cls, color) {
      const col = color ? 'style="color:' + color + '"' : (cls ? 'class="' + cls + '"' : '');
      return '<div class="metric"><div class="ml">' + label + '</div><div class="mv ' + (cls || '') + '" ' +
        (color ? col : '') + '>' + val + '</div></div>';
    }
    function hudStar(r) {
      let html = '';
      r.links.forEach((l, i) => {
        const cls = l.los ? (i === r.active ? 'active' : 'los') : 'nlos';
        html += '<div class="lk ' + cls + '"><span class="lk-n">' + md.algo[i] + '</span>' +
          '<span class="lk-badge">' + (l.los ? (i === r.active ? 'ACTIVE' : 'LOS') : '遮挡') + '</span>' +
          '<span class="lk-d">' + l.dist.toFixed(1) + 'm</span>' +
          '<span class="lk-pr">' + l.pr.toFixed(0) + ' dBm</span>' +
          '<span class="lk-ber">BER ' + fmtBer(l.ber) + '</span></div>';
      });
      ui['s3d-links'].innerHTML = html;
      const net = ui['s3d-netstate'];
      if (r.active >= 0) {
        net.className = 'pill-status run'; net.style.color = '';
        net.innerHTML = '<span class="b"></span>链路连通 · ' + md.algo[r.active];
      } else {
        net.className = 'pill-status'; net.style.color = '#f87171';
        net.innerHTML = '<span class="b" style="background:#f87171"></span>全部遮挡 · 断链';
      }
      const a = r.active >= 0 ? r.links[r.active] : null;
      ui['s3d-global'].innerHTML =
        gmet('活动链路接收', a ? a.pr.toFixed(1) + ' dBm' : '—', 'cyan') +
        gmet('系统 BER', a ? fmtBer(a.ber) : '0.5', a && a.ber < 1e-3 ? 'green' : 'amber');
      ui['s3d-extra'].innerHTML = extraStar(r.links, r.active).map(e => gmet(e[0], e[1], '', e[2])).join('');
    }
    function nodeName(i, n) { return i < n ? md.algo[i] : '终端'; }
    function hudRelay(r, n) {
      if (!r.hops.length) {
        ui['s3d-links'].innerHTML = '';
        const net = ui['s3d-netstate'];
        net.className = 'pill-status'; net.style.color = '#f87171';
        net.innerHTML = '<span class="b" style="background:#f87171"></span>网络不可达 · 无中继路径';
        ui['s3d-global'].innerHTML = gmet('端到端 BER', '0.5', 'amber') + gmet('中继跳数', '—', '');
        ui['s3d-extra'].innerHTML =
          gmet('可用邻接', '' + r.edges.filter(e => e.usable).length, '', '#2fd7d7') +
          gmet('网络状态', '未连通', '', '#f87171');
        return;
      }
      let html = '', e2e = 0, totalD = 0;
      r.hops.forEach((e, k) => {
        e2e = 1 - (1 - e2e) * (1 - e.ber); totalD += e.dist;
        html += '<div class="lk active hop"><span class="lk-n">H' + (k + 1) + ' · ' +
          nodeName(e.a, n) + '→' + nodeName(e.b, n) + '</span>' +
          '<span class="lk-badge">HOP</span>' +
          '<span class="lk-d">' + e.dist.toFixed(1) + 'm</span>' +
          '<span class="lk-pr">' + e.pr.toFixed(0) + ' dBm</span>' +
          '<span class="lk-ber">BER ' + fmtBer(e.ber) + '</span></div>';
      });
      ui['s3d-links'].innerHTML = html;
      const net = ui['s3d-netstate'];
      net.className = 'pill-status run'; net.style.color = '';
      net.innerHTML = '<span class="b"></span>多跳连通 · ' + r.hops.length + ' 跳 → 终端';
      ui['s3d-global'].innerHTML =
        gmet('端到端 BER', fmtBer(e2e), e2e < 1e-3 ? 'green' : 'amber') +
        gmet('中继跳数', '' + r.hops.length, 'cyan');
      const nUsable = r.edges.filter(e => e.usable).length;
      ui['s3d-extra'].innerHTML =
        gmet('再生中继节点', '' + (r.nodes.length - 1), '', '#2fd7d7') +
        gmet('冗余可用链路', '' + (nUsable - r.hops.length), '', '#e26bd6') +
        gmet('端到端距离', totalD.toFixed(0) + ' m', '', '#f6b942') +
        gmet('中继拓扑', md.name, '', '#4ade9a');
    }

    /* ---------------- 事件 ---------------- */
    function bindRange(id, suf) {
      const el = ui[id], out = el.parentElement.querySelector('.val');
      function u() { if (out) out.textContent = el.value + (suf || ''); }
      el.addEventListener('input', u); u();
    }
    bindRange('s3d-speed'); bindRange('s3d-range', ' m'); bindRange('s3d-height', ' m');
    bindRange('s3d-mx'); bindRange('s3d-my'); bindRange('s3d-mz');
    bindRange('s3d-pt', ' dBm'); bindRange('s3d-obspos', ' m'); bindRange('s3d-atmos', ' dB/km');

    ui['s3d-scene'].addEventListener('change', () => buildScene(ui['s3d-scene'].value));
    ui['s3d-amode'].addEventListener('change', () => {
      const man = ui['s3d-amode'].value === 'manual';
      host.querySelectorAll('.s3d-manual').forEach(e => e.style.display = man ? '' : 'none');
      ui['s3d-autogroup'].style.display = man ? 'none' : '';
      ui['s3d-speedgroup'].style.display = man ? 'none' : '';
    });
    ui['s3d-pause'].addEventListener('click', () => {
      paused = !paused; ui['s3d-pause'].textContent = paused ? '继续' : '暂停';
    });
    ui['s3d-reset'].addEventListener('click', () => {
      camera.position.set(S.cam[0][0], S.cam[0][1], S.cam[0][2]);
      controls.target.set(S.cam[1][0], S.cam[1][1], S.cam[1][2]);
    });

    /* ---------------- 尺寸 ---------------- */
    function resize() {
      const w = view.clientWidth, h = view.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    global.addEventListener('resize', resize);
    if (typeof ResizeObserver !== 'undefined')
      new ResizeObserver(function () { resize(); }).observe(view);

    /* ---------------- 主循环 ---------------- */
    let frame = 0, last = performance.now();
    function loop() {
      requestAnimationFrame(loop);
      const now = performance.now(), dt = Math.min(.05, (now - last) / 1000); last = now;
      if (!paused) tt += dt;

      // 动态障碍物
      dynObs.forEach(d => {
        const m = d.mesh, cf = d.cf, b = d.base;
        if (ui['s3d-dynobs'].checked) {
          const off = Math.sin(tt * cf.sp + (cf.ph || 0)) * cf.amp;
          m.position[cf.axis] = b[cf.axis === 'x' ? 0 : (cf.axis === 'y' ? 1 : 2)] + off;
        } else {
          // direct 单动态板跟随位置滑块，其余回 base
          if (S.flags.obsPos && dynObs.length === 1) m.position.x = +ui['s3d-obspos'].value;
          else { m.position.x = b[0]; m.position.y = b[1]; m.position.z = b[2]; }
        }
      });

      const tp = targetPos();
      receiver.position.copy(tp);
      receiver.userData.rotors.forEach((r, i) => r.rotation.y += .3 + i * .02);

      const pt = +ui['s3d-pt'].value, atmos = +ui['s3d-atmos'].value;
      let r;
      if (S === SCENES.relay) {
        r = updateRelay(tp, dt, pt, atmos);
      } else if (S.chain) {
        r = updateChain(tp, dt, pt, atmos);
      } else {
        r = updateStar(tp, dt, pt, atmos);
      }

      // 直连切换提示
      if (S !== SCENES.relay && !S.chain) {
        if (r.active !== activePrev) {
          ui['s3d-switch'].textContent = r.active >= 0 && activePrev >= 0 ? '切换 → ' + md.algo[r.active] : '';
          activePrev = r.active;
        }
      }

      frame++;
      if (frame % 6 === 0) {
        if (S === SCENES.relay) hudRelay(r, opas.length);
        else if (S.chain) hudChain(r, opas.length);
        else hudStar(r);
      }
      controls.update();
      renderer.render(scene, camera);
    }

    buildScene(cfg.scene || 'direct');
    resize();
    loop();

    return {
      resize,
      dispose: function () { global.removeEventListener('resize', resize); renderer.dispose(); host.innerHTML = ''; }
    };
  }

  global.OPA3D = { mount };
})(window);
