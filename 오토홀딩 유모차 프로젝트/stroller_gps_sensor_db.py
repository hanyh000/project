import serial
import sqlite3
import time
import threading
import os
from datetime import datetime
import pynmea2

# ── 시리얼 포트 설정 ──────────────────────────────────────────
SERIAL_TEMP  = '/dev/ttyACM0'  # 온습도 아두이노
SERIAL_CDS   = '/dev/ttyACM1'  # CDS 아두이노
SERIAL_BT    = '/dev/ttyACM2'  # 블루투스 아두이노
SERIAL_FLEX  = '/dev/ttyACM3'  # 압력센서 아두이노
SERIAL_GPS   = '/dev/ttyAMA0'  # GPS UART 직접 연결

BAUD = 9600

# ── DB 경로 (실행 유저 홈 자동 적용) ────────────────────────
DB_PATH = os.path.join(os.path.expanduser('~'), 'sensor_data.db')

# ── 임계값 ───────────────────────────────────────────────────
TEMP_THRESHOLD = 24
FLEX_THRESHOLD = 10

# ── 공유 데이터 + 락 ─────────────────────────────────────────
data = {
    'temp': 0.0,
    'hum':  0.0,
    'cds':  0,
    'flex': 999,
    'lat':  0.0,
    'lon':  0.0,
}
lock = threading.Lock()

# 데이터 준비 여부 플래그
data_ready = {
    'temp': False,
    'hum':  False,
    'cds':  False,
    'flex': False,
}

# 온습도 시리얼 공유 객체
ser_temp_shared = None
ser_temp_lock   = threading.Lock()

# ── DB ───────────────────────────────────────────────────────
def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS sensor_log (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            temp      REAL,
            hum       REAL,
            cds       INTEGER,
            flex      INTEGER,
            lat       REAL,
            lon       REAL
        )
    ''')
    conn.commit()
    conn.close()
    print(f"[DB] {DB_PATH} 초기화 완료")

def save_to_db(temp, hum, cds, flex, lat, lon):
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute('''
            INSERT INTO sensor_log (timestamp, temp, hum, cds, flex, lat, lon)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
              temp, hum, cds, flex, lat, lon))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[DB 저장 오류] {e}")

# ── 시리얼 연결 헬퍼 (실패시 재시도) ────────────────────────
def open_serial(port, baud=9600, retry_delay=3):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"[시리얼] {port} 연결 성공")
            return ser
        except Exception as e:
            print(f"[시리얼] {port} 연결 실패: {e} → {retry_delay}초 후 재시도")
            time.sleep(retry_delay)

# ── 온습도 아두이노 스레드 ───────────────────────────────────
def handle_temp_serial():
    global ser_temp_shared
    while True:
        ser = open_serial(SERIAL_TEMP)
        with ser_temp_lock:
            ser_temp_shared = ser
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('tem,'):
                    parts = line.split(',', 1)
                    if len(parts) == 2:
                        try:
                            with lock:
                                data['temp'] = float(parts[1])
                                data_ready['temp'] = True
                        except ValueError:
                            pass
                elif line.startswith('hum,'):
                    parts = line.split(',', 1)
                    if len(parts) == 2:
                        try:
                            with lock:
                                data['hum'] = float(parts[1])
                                data_ready['hum'] = True
                        except ValueError:
                            pass
        except Exception as e:
            print(f"[온습도 끊김] {e} → 재연결 시도")
            with ser_temp_lock:
                ser_temp_shared = None
            try:
                ser.close()
            except:
                pass

# ── CDS 아두이노 스레드 ──────────────────────────────────────
def handle_cds_serial():
    while True:
        ser = open_serial(SERIAL_CDS)
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('lig,'):
                    parts = line.split(',', 1)
                    if len(parts) == 2:
                        try:
                            with lock:
                                data['cds'] = int(float(parts[1]))
                                data_ready['cds'] = True
                        except ValueError:
                            pass
        except Exception as e:
            print(f"[CDS 끊김] {e} → 재연결 시도")
            try:
                ser.close()
            except:
                pass

# ── 압력센서 아두이노 스레드 ─────────────────────────────────
def handle_flex_serial():
    while True:
        ser = open_serial(SERIAL_FLEX)
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('sensor :'):
                    parts = line.split(':', 1)  # maxsplit=1 로 안전하게
                    if len(parts) == 2:
                        try:
                            with lock:
                                data['flex'] = int(parts[1].strip())
                                data_ready['flex'] = True
                        except ValueError:
                            pass
        except Exception as e:
            print(f"[압력 끊김] {e} → 재연결 시도")
            try:
                ser.close()
            except:
                pass

# ── GPS 스레드 ───────────────────────────────────────────────
def read_gps():
    while True:
        ser = open_serial(SERIAL_GPS)
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    try:
                        msg = pynmea2.parse(line)
                        if msg.status == 'A':
                            with lock:
                                data['lat'] = round(msg.latitude,  6)
                                data['lon'] = round(msg.longitude, 6)
                            print(f"[GPS] 위도:{data['lat']} 경도:{data['lon']}")
                    except pynmea2.ParseError:
                        pass
        except Exception as e:
            print(f"[GPS 끊김] {e} → 재연결 시도")
            try:
                ser.close()
            except:
                pass

# ── 블루투스 아두이노 스레드 ─────────────────────────────────
def handle_bt_serial():
    while True:
        ser_bt = open_serial(SERIAL_BT)
        last_send = 0
        try:
            while True:
                # 앱 명령 수신 → 온습도 아두이노로 전달
                if ser_bt.in_waiting > 0:
                    line = ser_bt.readline().decode('utf-8', errors='ignore').strip()
                    if line in ('3', '4', '7', '8'):
                        with ser_temp_lock:
                            if ser_temp_shared and ser_temp_shared.is_open:
                                ser_temp_shared.write((line + '\n').encode())
                                print(f"[명령 전달] 앱({line}) → 온습도 아두이노")
                            else:
                                print(f"[명령 전달 실패] 온습도 아두이노 미연결")

                # 1초마다 앱으로 전송 + DB 저장
                now = time.time()
                if now - last_send >= 1.0:
                    with lock:
                        t    = data['temp']
                        h    = data['hum']
                        cds  = data['cds']
                        flex = data['flex']
                        lat  = data['lat']
                        lon  = data['lon']
                        # 필수 센서값이 준비됐는지 확인
                        ready = (data_ready['temp'] and
                                 data_ready['hum']  and
                                 data_ready['cds'])

                    if ready:
                        msg = f"11,{t},{h},{lat},{lon},{cds},00\n"
                        ser_bt.write(msg.encode())
                        print(f"[전송] {msg.strip()}")

                        if lat != 0.0 and lon != 0.0:
                            print(f"[구글맵] https://www.google.co.kr/maps/@{lat},{lon},18z?hl=ko&entry=ttu")

                        save_to_db(t, h, cds, flex, lat, lon)
                    else:
                        print("[대기] 센서 초기화 중...")

                    last_send = now

                time.sleep(0.05)

        except Exception as e:
            print(f"[블루투스 끊김] {e} → 재연결 시도")
            try:
                ser_bt.close()
            except:
                pass

# ── 메인 ─────────────────────────────────────────────────────
if __name__ == '__main__':
    init_db()
    print("=== 라즈베리파이 센서 허브 시작 ===")

    threads = [
        threading.Thread(target=handle_temp_serial, name="온습도",   daemon=True),
        threading.Thread(target=handle_cds_serial,  name="CDS",      daemon=True),
        threading.Thread(target=handle_flex_serial, name="압력",     daemon=True),
        threading.Thread(target=read_gps,           name="GPS",      daemon=True),
        threading.Thread(target=handle_bt_serial,   name="블루투스", daemon=True),
    ]
    for t in threads:
        t.start()
        time.sleep(0.5)  # 포트 동시 오픈 충돌 방지

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n종료")