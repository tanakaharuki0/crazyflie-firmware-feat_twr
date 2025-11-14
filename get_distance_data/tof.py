import argparse
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
    "vl11.tick",
    "vl11.s0",
    "vl11.s1",
    "vl11.s2",
    "vl11.s3",
    "vl11.s4",
    "vl11.s5",
    "vl11.s6",
    "vl11.s7",
    "vl11.s8",
    "vl11.s9",
    "vl11.s10",
]
LOG_FILE = "crazyflie_log.csv"

# グローバル変数: --auto-real モードで使用
rows_received = []
real_data_detected = Event()

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

def log_data_callback_auto_real(timestamp, data, logconf):
    """--auto-real モード用: データをメモリに蓄積し、非ゼロ値を検出したらイベントをセット"""
    row = [timestamp] + [data.get(var, '') for var in log_variables]
    rows_received.append(row)
    
    # 距離データ (vl11.s0 ~ vl11.s10) に非ゼロ値があるかチェック
    distance_values = [data.get(var, 0) for var in log_variables if var.startswith('vl11.s')]
    if any(v > 0 for v in distance_values):
        print(f"Real distance data detected! Sample: {distance_values[:3]}...")
        real_data_detected.set()

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
    parser = argparse.ArgumentParser(description='TOF data logger')
    parser.add_argument('--real', action='store_true', help='Use real sensor (disable testGen) and force init')
    parser.add_argument('--auto-real', action='store_true', help='Use real sensor and auto-detect when real data arrives')
    args = parser.parse_args()

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Register param callback early so we catch the current value when params arrive.
        # The driver exposes the vl11.* params, so watch vl11.enable instead of
        # deck.bcVL53L8CX11 which this driver does not add.
        scf.cf.param.add_update_callback(group='vl11', name='enable', cb=param_deck_flow)

        # Show firmware DEBUG_PRINT / console output in this script
        # cfclientd already does this; here we attach to the Crazyflie.console
        # receivedChar callback so DEBUG_PRINT and consolePrintf() lines are
        # forwarded to stdout.
        try:
            def _console_incoming(text):
                # text already contains newline(s) as printed from firmware
                print("DEBUG_PRINT:", text, end='')

            scf.cf.console.receivedChar.add_callback(_console_incoming)
        except Exception:
            # Older/newer cflib versions may use different API names; ignore
            # if console hook is not available.
            pass

        # If requested, switch to real sensor behaviour and request forced init
        if args.real or args.auto_real:
            try:
                print('Requesting real sensor mode: setting vl11.testGen=0')
                scf.cf.param.set_value('vl11.testGen', '0')
            except Exception as e:
                print('Failed to set vl11.testGen:', e)

        # Enable hardware driver inside firmware (vl11)
        scf.cf.param.set_value('vl11.enable', '1')

        # If requested, ask the firmware to force ULD init (vl11.forceInit = 1)
        if args.real or args.auto_real:
            try:
                print('Requesting forced init: setting vl11.forceInit=1')
                scf.cf.param.set_value('vl11.forceInit', '1')
            except Exception as e:
                print('Failed to set vl11.forceInit:', e)

        # Wait for the param system to update; either via callback or short sleep
        # We'll wait up to 1.0s for deck callback to set the event
        if not deck_attached_event.wait(timeout=1.0):
            # If callback didn't fire, attempt to read param directly (correct API usage)
            try:
                # get_value expects a single string "group.name"
                val_enable = scf.cf.param.get_value('vl11.enable')
                val_testgen = scf.cf.param.get_value('vl11.testGen')
                print("vl11.enable =", val_enable)
                print("vl11.testGen =", val_testgen)
            except Exception as e:
                print("Could not read vl11 params directly:", e)

            # Also enumerate deck params to help debugging
            try:
                for name in scf.cf.param.get_params():
                    if name.startswith('deck.'):
                        print("deck param:", name)
            except Exception:
                # Some cflib versions might not expose get_params; ignore if not available
                pass

            # give one more short chance for the deck callback to arrive
            time.sleep(0.2)

        if not deck_attached_event.is_set():
            print('No flow deck detected!')
            # Decide: exit or continue. We'll exit since deck expected.
            sys.exit(1)

        # --------------------------
        # ログ設定
        log_conf = LogConfig(name='Logging', period_in_ms=100)
        for var in log_variables:
            log_conf.add_variable(var)

        scf.cf.log.add_config(log_conf)
        
        if args.auto_real:
            # --auto-real モード: メモリにデータを蓄積し、非ゼロ値検出で終了
            log_conf.data_received_cb.add_callback(log_data_callback_auto_real)
            log_conf.error_cb.add_callback(log_error_callback)
            log_conf.start()
            
            print("Waiting for real distance data (non-zero values)...")
            max_wait = 30.0  # 最大30秒待機
            if real_data_detected.wait(timeout=max_wait):
                print("Real distance data detected! Saving to CSV and exiting.")
                write_csv_header()
                with open(LOG_FILE, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    for row in rows_received:
                        writer.writerow(row)
                print(f"Saved {len(rows_received)} rows to {LOG_FILE}")
            else:
                print(f"Timeout: No real distance data detected after {max_wait}s")
                print("Firmware may need re-initialization or sensor is not responding.")
            
            log_conf.stop()
        else:
            # 通常モードまたは --real モード: 直接CSVに書き込み
            log_conf.data_received_cb.add_callback(log_data_callback)
            log_conf.error_cb.add_callback(log_error_callback)
            write_csv_header()  # ヘッダー書き込み
            log_conf.start()

            # --------------------------
            # 離陸処理
            scf.cf.platform.send_arming_request(True)
            time.sleep(10.0)
            print("Success!")
            # --------------------------
            # ログ停止
            log_conf.stop()