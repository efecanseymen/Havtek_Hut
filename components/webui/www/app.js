/*
 * app.js -- HAVTEK v0.1 arayuzu.
 *
 * Tek WebSocket: komutlar gider, telemetri (20 Hz) ve log paketleri (10 Hz)
 * gelir. Log kartta degil burada birikir; CSV'yi tarayici uretir.
 */

const $ = (s) => document.querySelector(s);
let ws = null;
let sonDurum = null;
let logSatirlar = [];

const LOG_BASLIK = [
  't_us', 'gx', 'gy', 'gz', 'ax', 'ay', 'az',
  'roll', 'pitch', 'yaw', 'adim_az', 'adim_el',
  'sps_az', 'sps_el', 'hedef', 'hata', 'kp'
];

/* ----------------------------------------------------------- baglanti */

function baglan() {
  ws = new WebSocket('ws://' + location.host + '/ws');

  ws.onopen = () => {
    $('#baglanti').textContent = 'bagli';
    $('#baglanti').className = 'rozet iyi';
    ayarlariYukle();
  };

  ws.onclose = () => {
    $('#baglanti').textContent = 'bagli degil';
    $('#baglanti').className = 'rozet kotu';
    setTimeout(baglan, 1000);
  };

  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch (e) { return; }

    if (m.L) { logEkle(m.L); return; }
    if (m.err) { console.warn('kart:', m.err); return; }
    if (m.ack !== undefined) return;

    sonDurum = m;
    ciz(m);
  };
}

function gonder(obj) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

/* --------------------------------------------------------------- cizim */

function ciz(d) {
  $('#mod').textContent = 'mod: ' + d.mod;
  const h = $('#hata');
  h.textContent = 'hata: ' + d.hata;
  h.className = 'rozet ' + (d.hata === 'yok' ? 'iyi' : 'kotu');

  /* cihaz tablosu */
  const govde = $('#cihaz-tablo tbody');
  govde.innerHTML = '';
  (d.eksen || []).forEach((e) => {
    const tr = document.createElement('tr');
    tr.innerHTML =
      '<td>hat ' + e.hat + ' / 0x' + e.adres.toString(16).toUpperCase() + '</td>' +
      '<td>M20 surucu</td>' +
      '<td><select data-i="' + e.i + '" class="rol-sec">' +
        '<option value="0"' + (e.rol === 0 ? ' selected' : '') + '>yatay (azimut)</option>' +
        '<option value="1"' + (e.rol === 1 ? ' selected' : '') + '>dikey (elevasyon)</option>' +
      '</select></td>' +
      '<td>' + (e.yon > 0 ? '+' : '-') + '</td>' +
      '<td><button class="tani" data-i="' + e.i + '">Eksen Tani</button> ' +
          '<button class="ters" data-rol="' + e.rol + '">Yonu Ters</button></td>';
    govde.appendChild(tr);
  });
  if (d.imu && d.imu.var) {
    $('#i2c-liste').textContent =
      'IMU: 0x' + d.imu.adres.toString(16) + '  manyetometre: ' + (d.imu.mag || 'yok');
  } else {
    $('#i2c-liste').textContent = 'IMU bulunamadi.';
  }

  /* motor */
  $('#m-durum').textContent = (d.eksen || []).map((e) =>
    (e.rol === 0 ? 'yatay ' : 'dikey ') +
    'hat' + e.hat + ' 0x' + e.adres.toString(16) +
    '  adim ' + e.adim +
    '  aci ' + e.der.toFixed(2) + ' derece' +
    '  hiz ' + e.sps.toFixed(0) + ' adim/s' +
    (e.hata ? '  I2C HATA ' + e.hata : '')
  ).join('\n') || 'surucu yok';

  /* sensor */
  const i = d.imu || {};
  $('#sensor-olcum').textContent =
    'roll   ' + f(i.r) + '   pitch ' + f(i.p) + '   yaw ' + f(i.y) + '\n' +
    'gyro   ' + f(i.gx) + ' ' + f(i.gy) + ' ' + f(i.gz) + ' derece/s\n' +
    'ivme   ' + f(i.ax) + ' ' + f(i.ay) + ' ' + f(i.az) + ' g\n' +
    'pusula ' + (i.pusula >= 0 ? f(i.pusula) : 'yok') +
    '   gyro kalibrasyonu: ' + (i.kalibre ? 'yapildi' : 'YAPILMADI');

  /* stabilizasyon */
  const s = d.stab || {};
  $('#st-durum').textContent =
    'durum   ' + (s.on ? 'ACIK' : 'kapali') + '   eksen: ' + (s.rol === 0 ? 'yatay' : 'dikey') + '\n' +
    'hedef   ' + f(s.hedef) + '\n' +
    'olculen ' + f(s.olculen) + '\n' +
    'hata    ' + f(s.e) + '\n' +
    'Kp      ' + f(s.kp) + '   komut ' + f(s.sps) + ' adim/s';

  /* sistem */
  const y = d.sis || {};
  $('#log-durum').textContent =
    'kayit     ' + (y.log ? 'ACIK' : 'kapali') + '\n' +
    'tarayici  ' + logSatirlar.length + ' satir\n' +
    'kart      ' + y.satir + ' orneklem, dusen ' + y.dusen + '\n' +
    'dongu     ' + y.dongu + ' us   dilim asimi ' + y.asim + '\n' +
    'istemci   ' + y.istemci;
}

function f(v) {
  return (v === undefined || v === null) ? '-' : Number(v).toFixed(2).padStart(8);
}

/* ----------------------------------------------------------------- log */

function logEkle(satirlar) {
  for (const s of satirlar) logSatirlar.push(s);
  /* Tarayici belleginin sinirsiz sismesini engelle: ~10 dakika @100 Hz */
  if (logSatirlar.length > 60000) logSatirlar.splice(0, logSatirlar.length - 60000);
}

function csvIndir() {
  if (!logSatirlar.length) { alert('kayit bos'); return; }
  const metin = LOG_BASLIK.join(',') + '\n' +
                logSatirlar.map((s) => s.join(',')).join('\n');
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([metin], { type: 'text/csv' }));
  a.download = 'havtek_' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
  a.click();
  URL.revokeObjectURL(a.href);
}

/* ------------------------------------------------------------- ayarlar */

async function ayarlariYukle() {
  try {
    const c = await (await fetch('/api/info')).json();
    document.querySelectorAll('[data-param]').forEach((el) => {
      const d = c[el.dataset.param];
      if (d === undefined) return;
      if (el.type === 'checkbox') el.checked = !!d; else el.value = d;
    });
    $('#surum').textContent = 'v' + c.surum;
    $('#st-fuzzy').value = c.fuzzy ? '1' : '0';
  } catch (e) { /* sunucu henuz hazir degil */ }
}

/* ------------------------------------------------------------ olaylar */

document.querySelectorAll('#sekmeler button').forEach((b) => {
  b.onclick = () => {
    document.querySelectorAll('#sekmeler button').forEach((x) => x.classList.remove('aktif'));
    document.querySelectorAll('.sekme').forEach((x) => x.classList.remove('aktif'));
    b.classList.add('aktif');
    $('#s-' + b.dataset.sekme).classList.add('aktif');
  };
});

$('#estop').onclick    = () => gonder({ c: 'estop' });
$('#hata-sil').onclick = () => gonder({ c: 'hata_sil' });

$('#lim-alt').onclick = () => gonder({ c: 'limit_ogret', rol: +$('#m-rol').value, ust: 0 });
$('#lim-ust').onclick = () => gonder({ c: 'limit_ogret', rol: +$('#m-rol').value, ust: 1 });

/* Olcum acisi: hangi eksen icin degistirildigini stab panelindeki rol belirler. */
$('#st-olcum').onchange = (e) =>
  gonder({ c: 'olcum', rol: +$('#st-rol').value, eksen: +e.target.value });
$('#tara').onclick  = () => gonder({ c: 'tara' });

/* jog: basili tutuldugu surece doner. Fare/parmak kalkinca durur -- bu
   kasitli, kontrol koparsa hareket devam etmesin. */
function jogBagla(dugme, yon) {
  const bas = (e) => {
    e.preventDefault();
    gonder({ c: 'jog', rol: +$('#m-rol').value, yon: yon, sps: +$('#m-sps').value });
  };
  const birak = () => gonder({ c: 'jog_dur', rol: +$('#m-rol').value });
  dugme.addEventListener('mousedown', bas);
  dugme.addEventListener('touchstart', bas, { passive: false });
  ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach((o) =>
    dugme.addEventListener(o, birak));
}
jogBagla($('#jog-ileri'), 1);
jogBagla($('#jog-geri'), -1);

$('#m-dur').onclick     = () => gonder({ c: 'dur' });
$('#m-git').onclick     = () => gonder({ c: 'git', rol: +$('#m-rol').value,
                                         derece: +$('#m-derece').value });
$('#m-ref').onclick     = () => gonder({ c: 'ref', rol: +$('#m-rol').value });
$('#m-serbest').onclick = () => gonder({ c: 'serbest', rol: +$('#m-rol').value });

$('#gyro-kal').onclick    = () => gonder({ c: 'gyro_kalibre' });
$('#mag-kal-bas').onclick = () => gonder({ c: 'mag_kalibre', on: 1 });
$('#mag-kal-bit').onclick = () => gonder({ c: 'mag_kalibre', on: 0 });

$('#st-kilit').onclick = () => gonder({ c: 'kilit' });
$('#st-ac').onclick    = () => gonder({ c: 'stab', on: 1, rol: +$('#st-rol').value });
$('#st-kapat').onclick = () => gonder({ c: 'stab', on: 0 });
$('#st-fuzzy').onchange = (e) => gonder({ c: 'param', k: 'fuzzy', v: +e.target.value });

$('#log-bas').onclick   = () => gonder({ c: 'log', on: 1 });
$('#log-dur').onclick   = () => gonder({ c: 'log', on: 0 });
$('#log-indir').onclick = csvIndir;
$('#log-sil').onclick   = () => { logSatirlar = []; };

document.querySelectorAll('[data-param]').forEach((el) => {
  el.onchange = () => {
    const v = (el.type === 'checkbox') ? (el.checked ? 1 : 0) : +el.value;
    gonder({ c: 'param', k: el.dataset.param, v: v });
  };
});

/* Tablo icindeki dugmeler her cizimde yeniden olusuyor: olayi govdeye bagla. */
$('#cihaz-tablo').addEventListener('click', (e) => {
  if (e.target.classList.contains('tani')) {
    gonder({ c: 'tani', i: +e.target.dataset.i });
  } else if (e.target.classList.contains('ters')) {
    gonder({ c: 'yon_ters', rol: +e.target.dataset.rol });
  }
});
$('#cihaz-tablo').addEventListener('change', (e) => {
  if (e.target.classList.contains('rol-sec')) {
    gonder({ c: 'rol', i: +e.target.dataset.i, rol: +e.target.value });
  }
});

baglan();
