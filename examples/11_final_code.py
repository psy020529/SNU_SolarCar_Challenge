import network
import socket
import time
from drv8833 import DRV8833
from mux04 import LineSensor
from ina226 import INA226
from myservo import myServo

# --- 1. Wi-Fi 및 네트워크 설정 ---
SSID = "sjkim"
PASSWORD = "12345678"
PC_IP = "192.168.137.1"
PC_PORT = 5005

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    time.sleep(0.5)
    wlan.active(True)
    
    try: 
        wlan.config(txpower=10)
        print("TX Power 10으로 설정 완료 (전력 안정화)")
    except Exception as e: 
        print("TX Power 설정 미지원:", e)
    
    if not wlan.isconnected():
        print(f'{SSID} 연결 중...')
        wlan.connect(SSID, PASSWORD)
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(".", end="")
            
    if wlan.isconnected():
        print('\nWi-Fi 연결 성공! IP:', wlan.ifconfig()[0])
        return True
    return False

# --- 2. 하드웨어 초기화 
sensor = LineSensor()
motor = DRV8833()
servo = myServo(pin=10)

ina_0x40 = INA226(address=0x40)
ina_0x41 = INA226(address=0x41)

# 주행 설정
kp = 1.5
base_speed = 100

try:
    ina_0x40.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x41.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x40.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    ina_0x41.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    print("INA226 센서 초기화 완료")
except Exception as e:
    print("INA226 설정 실패:", e)

# Wi-Fi 연결 및 소켓(UDP) 생성
wifi_status = connect_wifi()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_send_time = time.ticks_ms()

def solar_tracking():
    data = {}
    for i in range(90):
        angle = 45 + i
        servo.myServoWriteAngle(angle,10)
        
        v1 = ina_0x40.read_bus_voltage()
        ma1 = ina_0x40.read_shunt_current()
        v2 = ina_0x41.read_bus_voltage()
        ma2 = ina_0x41.read_shunt_current()
        
        data[angle] = {'v1': v1, 'ma1': ma1, 'v2': v2, 'ma2': ma2}
        
        print(f"{angle:7d} | {v1:9.3f} | {ma1:10.2f} | {v2:9.3f} | {ma2:10.2f}")
        
        time.sleep(0.1)
    
    print("-" * 55)
    print("Done")
    
    # v1 값이 가장 높은 angle 찾기
    max_angle = max(data, key=lambda k: data[k]['v1'])
    print(f"Moving servo to angle {max_angle} with highest v1: {data[max_angle]['v1']}")
    servo.myServoWriteAngle(max_angle,15)

# --- 3. 메인 주행 루프 ---
try:
    print("주행 및 센서 데이터 송신 시작...")
    while True:
        # 1. 센서 데이터 읽기 및 반전
        raw_channels = sensor.read_channels()
        channels = [1 - c for c in raw_channels]
        
        active_count = sum(channels) # 1의 개수 파악
        
        # --- [수정] 정지선 감지 및 10초 대기 중 데이터 송신 ---
        if active_count == 8:
            print("정지선 감지! 정지 및 Solar Tracking 시작...")
            motor.set_speed(0, 0)  # 우선 정지
            
            solar_tracking()       # 솔라 트래킹 실행 (이 함수 실행 중에는 잠시 송신이 멈춤)
            
            print("10초 대기 및 데이터 송신 시작...")
            wait_start_time = time.ticks_ms()  # 대기 시작 시간 기록
            
            # 10초(10000ms)가 지날 때까지 반복하는 루프
            while time.ticks_diff(time.ticks_ms(), wait_start_time) < 10000:
                current_time = time.ticks_ms()
                
                # 대기 중에도 0.5초마다 데이터 송신
                if time.ticks_diff(current_time, last_send_time) > 500:
                    try:
                        v1, ma1 = ina_0x40.read_bus_voltage(), ina_0x40.read_shunt_current()
                        v2, ma2 = ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
                        
                        if wifi_status and None not in (v1, ma1, v2, ma2):
                            message = f"{v1:.3f},{ma1*1000:.1f},{v2:.3f},{ma2*1000:.1f}"
                            sock.sendto(message.encode(), (PC_IP, PC_PORT))
                            print(f"[대기중 송신] 0x40: {v1:.2f}V | 0x41: {v2:.2f}V")
                        
                        last_send_time = current_time # 송신 시간 업데이트
                    except Exception as e:
                        print("송신 에러:", e)
                
                time.sleep(0.01) # CPU 과부하 방지용 미세 대기
            
            print("10초 대기 완료! 정지선 탈출 중...")
            motor.set_speed(base_speed, base_speed)
            time.sleep(1) # 정지선을 확실히 벗어날 만큼 전진
            continue

        # --- A. 일반 라인 트레이싱 로직 ---
        weights = [-55, -30, -20, -10, 10, 20, 30, 55]
        error_sum = 0
        steering = 0
        
        if active_count > 0:
            for i in range(8):
                if channels[i] == 1:
                    error_sum += weights[i]
            
            # 평균 에러를 사용하거나 active_count로 나누어 보정 가능
            steering = (error_sum / active_count) * kp
            motor.set_speed((base_speed + steering), base_speed - steering)
        else:
            # 라인을 놓쳤을 때의 처리 (정지 혹은 직진)
            motor.set_speed(base_speed, base_speed) 

        # --- B. INA226 데이터 읽기 및 Wi-Fi 송신 (0.5초마다) ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_send_time) > 500:
            try:
                v1, ma1 = ina_0x40.read_bus_voltage(), ina_0x40.read_shunt_current()
                v2, ma2 = ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
                
                if None not in (v1, ma1, v2, ma2):
                    if wifi_status:
                        message = f"{v1:.3f},{ma1*1000:.1f},{v2:.3f},{ma2*1000:.1f}"
                        sock.sendto(message.encode(), (PC_IP, PC_PORT))
                        print(f"전송 -> 0x40: {v1:.2f}V/{ma1*1000:.0f}mA | 0x41: {v2:.2f}V/{ma2*1000:.0f}mA")
                else:
                    print("데이터 읽기 오류: 센서 배선을 확인하세요.")
            except Exception as e:
                pass
                
            last_send_time = current_time

        time.sleep(0.02) # 주행 응답성을 위해 대기 시간 단축

except KeyboardInterrupt:
    print("\n정지합니다.")
    motor.set_speed(0, 0)
    sock.close()