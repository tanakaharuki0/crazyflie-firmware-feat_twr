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
        # Register deck param callback early so we catch the current value when params arrive
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)

        # Set firmware params to use REAL sensor data:
        # turn OFF test generator and enable vl53l8cx driver (task will be created)
        try:
            scf.cf.param.set_value('vl53l8cx.testGen', '0')   # 擬似データ OFF
            scf.cf.param.set_value('vl53l8cx.enable', '1')    # ドライバ/タスク ON
        except Exception as e:
            print("Param set error:", e)

        # Wait for the param system to update; either via callback or short sleep
        if not deck_attached_event.wait(timeout=1.0):
            # If callback didn't fire, attempt to read param directly (correct API usage)
            try:
                val_enable = scf.cf.param.get_value('vl53l8cx.enable')
                val_testgen = scf.cf.param.get_value('vl53l8cx.testGen')
                val_blobs = scf.cf.param.get_value('vl53l8cx.blobs_ok')
                print("vl53l8cx.enable =", val_enable)
                print("vl53l8cx.testGen =", val_testgen)
                print("vl53l8cx.blobs_ok =", val_blobs)
            except Exception as e:
                print("Could not read vl53l8cx params directly:", e)

            # Also enumerate deck params to help debugging
            try:
                for name in scf.cf.param.get_params():
                    if name.startswith('deck.'):
                        print("deck param:", name)
            except Exception:
                pass

            # give one more short chance for the deck callback to arrive
            time.sleep(0.2)

        if not deck_attached_event.is_set():
            print('No flow deck detected!')
            sys.exit(1)

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