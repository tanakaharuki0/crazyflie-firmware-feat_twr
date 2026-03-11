import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import csv

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()
logging.basicConfig(level=logging.INFO)

# --------------------------
# ログ用変数とCSVファイル
log_variables = [
    "vl53l8cx.tick",
    "vl53l8cx.s0",
    "vl53l8cx.s1",
    "vl53l8cx.s2",
    "vl53l8cx.s3",
    "vl53l8cx.s4",
    "vl53l8cx.s5",
    "vl53l8cx.s6",
    "vl53l8cx.s7",
    "vl53l8cx.s8",
    "vl53l8cx.s9",
    "vl53l8cx.s10",
]
LOG_FILE = "crazyflie_log.csv"

# --------------------------
# データをCSVに保存する関数
def write_csv_header():
    with open(LOG_FILE, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp"] + log_variables)

def log_data_callback(timestamp, data, logconf):
    row = [timestamp] + [data.get(var, '') for var in log_variables]
    with open(LOG_FILE, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(row)

def log_error_callback(logconf, msg):
    print("Log error:", msg)

# --------------------------
def param_deck_flow(name, value_str):
    # cflib passes (name, value_str) to param callbacks
    try:
        value = int(value_str)
    except Exception:
        value = 0
    if value:
        print('Deck is attached! (param {} = {})'.format(name, value_str))
    else:
        print('Deck is NOT attached! (param {} = {})'.format(name, value_str))
    deck_attached_event.set()

# --------------------------
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        try:
            # Optional: set per-sensor CS mapping from host. Adjust cs_mapping to match
            # your deck wiring. Values are deckPin_t.id (see deck_constants.c), e.g.
            # DECK_GPIO_IO1=4, DECK_GPIO_IO2=5, ..., DECK_GPIO_MOSI=12
            # Example mapping (length must be vl53l8cx_NUM_SENSORS == 11):
            cs_mapping = [4, 5, 6, 7, 8, 9, 10, 11, 12, 2, 3]
            try:
                for i, pin in enumerate(cs_mapping):
                    scf.cf.param.set_value(f'vl53l8cx.cs{i}', str(pin))
                    time.sleep(0.02)
            except Exception:
                # If param write fails, we continue — user can still set params manually
                print('Warning: failed to set some vl53l8cx.csN params')

            scf.cf.param.set_value('vl53l8cx.testGen', '0')   # まず擬似データを OFF に
            scf.cf.param.set_value('vl53l8cx.enable', '1')    # ドライバ/タスク ON
        except Exception as e:
            print("Param set error:", e)

        # Wait for blobs_ok to become 1 (firmware blobs in FLASH)
        blobs_ok = False
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                val = scf.cf.param.get_value('vl53l8cx.blobs_ok')
                print("vl53l8cx.blobs_ok =", val)
                if val == '1':
                    blobs_ok = True
                    break
            except Exception:
                pass
            time.sleep(0.2)

        if not blobs_ok:
            print("vl53l8cx.blobs_ok is not 1. Falling back to test generator (no real sensor or blobs missing).")
            try:
                scf.cf.param.set_value('vl53l8cx.testGen', '1')
            except Exception:
                pass

        # Optional: enumerate deck params for debugging
        try:
            for name in scf.cf.param.get_params():
                if name.startswith('deck.'):
                    print("deck param:", name)
        except Exception:
            pass

        # --------------------------
        # ログ設定
        log_conf = LogConfig(name='Logging', period_in_ms=100)
        for var in log_variables:
            log_conf.add_variable(var)

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(log_data_callback)
        log_conf.error_cb.add_callback(log_error_callback)
        write_csv_header()  # ヘッダー書き込み
        log_conf.start()

        # --------------------------
        # 離陸処理（例：アーミング）
        scf.cf.platform.send_arming_request(True)
        time.sleep(20.0)
        print("Success!")
        # --------------------------
        # ログ停止
        log_conf.stop()