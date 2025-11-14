# check_decks.py
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

logging.basicConfig(level=logging.INFO)

# Initialize available CRTP drivers (required to use radio/usb interfaces)
cflib.crtp.init_drivers()

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

try:
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        # param TOC is stored in scf.cf.param.toc.toc as {group: {name: element}}
        for group, names in scf.cf.param.toc.toc.items():
            if not group.startswith('deck'):
                continue
            for name in names.keys():
                complete = f"{group}.{name}"
                try:
                    v = scf.cf.param.get_value(complete)
                except Exception:
                    v = '<err>'
                print(complete, '=', v)
except Exception as e:
    print('Connection failed:', e)
    print('Hints:')
    print('- Ensure your Crazyradio (or USB radio) is plugged in and visible to the OS.')
    print("- If on Linux, check 'lsusb' and udev permissions (you may need to run as root or add udev rules).")
    print("- Confirm the URI is correct. For USB radio use something like 'radio://0/80/2M/XXXXXX' or 'usb://0' depending on your setup.")
    raise