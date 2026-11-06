"""
Mission Planner SITL <-> Python MAVLink baglanti testi.

Gereksinim:  pip install pymavlink
Onden:       Mission Planner > Ctrl+F > Simulation > Multirotor
             (Mission Planner TCP 5760 kullanir, bu script 5762'ye baglanir)

Kullanim:
    python mavlink_test.py            # sadece baglan + telemetri oku
    python mavlink_test.py --fly      # ARM + 10 m takeoff + telemetri
"""
import sys
import time
from pymavlink import mavutil

CONNECTION = "tcp:127.0.0.1:5762"   # UDP kullanacaksan: "udpin:0.0.0.0:14550"
TAKEOFF_ALT = 10                     # metre


def connect():
    print(f"Baglaniliyor: {CONNECTION}")
    master = mavutil.mavlink_connection(CONNECTION)
    print("Heartbeat bekleniyor...")
    master.wait_heartbeat(timeout=15)
    print(f"Baglandi! system={master.target_system} component={master.target_component}")
    return master


def request_streams(master):
    # Telemetri verisini istemek icin
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)


def set_mode(master, mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Mod degistirildi: {mode}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("ARM komutu gonderildi, onay bekleniyor...")
    master.motors_armed_wait()
    print("Arac ARM edildi.")


def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)
    print(f"Takeoff komutu gonderildi: {alt} m")


def print_telemetry(master, duration=30):
    end = time.time() + duration
    while time.time() < end:
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS", "HEARTBEAT"],
            blocking=True, timeout=2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            print(f"Konum: lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f} "
                  f"irtifa={msg.relative_alt/1000:.1f} m")
        elif t == "VFR_HUD":
            print(f"Hiz={msg.groundspeed:.1f} m/s  Yon={msg.heading}  Throttle={msg.throttle}%")
        elif t == "SYS_STATUS":
            print(f"Batarya: {msg.voltage_battery/1000:.2f} V")
        elif t == "HEARTBEAT":
            print(f"Heartbeat: mod={mavutil.mode_string_v10(msg)}")


if __name__ == "__main__":
    master = connect()
    request_streams(master)

    if "--fly" in sys.argv:
        set_mode(master, "GUIDED")
        time.sleep(1)
        arm(master)
        takeoff(master, TAKEOFF_ALT)

    print_telemetry(master, duration=30)