/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * deck_drivers.c - Deck drivers loading and handling
 */

#define DEBUG_MODULE "DECK_DRIVERS"

#include <stdlib.h>
#include <string.h>

#include "deck.h"
#include "debug.h"

#ifdef CONFIG_DEBUG
  #define DECK_DRV_DBG_PRINT(fmt, ...)  DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
  #define DECK_DRV_DBG_PRINT(...)
#endif

/* Symbols set by the linker script */
extern const struct deck_driver * _deckDriver_start;
extern const struct deck_driver * _deckDriver_stop;

static const struct deck_driver ** drivers;
static int driversLen;

// Init the toc access variables. Lazy initialisation: it is going to be done
// the first time any api function is called.
// この関数は最初のAPI呼び出し時に一度だけ実行される
// 処理内容: リンカスクリプトで定義されたシンボルからデッキドライバの配列とその長さを取得する
// これにより、デッキドライバの情報が初期化され、以降のAPI呼び出しで使用可能になる
// static変数initで初期化済みかどうかを管理し、二重初期化を防止する
// drivers変数にデッキドライバの配列の先頭アドレスを設定し、driversLen変数にドライバの数を設定する
// デバッグ出力で見つかったドライバの数と各ドライバのVID、PID、名前を表示する
// この関数はdeckDriverCount()、deckGetDriver()、deckFindDriverByVidPid()、deckFindDriverByName()で呼び出される
// これにより、デッキドライバの情報が必要なときにのみ初期化され、効率的なリソース管理が可能になる
static void deckdriversInit() {
  static bool init = false;
  if (!init) {
    int i;

    drivers = &_deckDriver_start;
    driversLen = &_deckDriver_stop - &_deckDriver_start;
    init = true;

    DECK_DRV_DBG_PRINT("Found %d drivers\n", driversLen);
    for (i=0; i<driversLen; i++) {
      if (drivers[i]->name) {
        DECK_DRV_DBG_PRINT("VID:PID %02x:%02x (%s)\n", drivers[i]->vid, drivers[i]->pid, drivers[i]->name);
      } else {
        DECK_DRV_DBG_PRINT("VID:PID %02x:%02x\n", drivers[i]->vid, drivers[i]->pid);
      }

    }
  }
}

// この関数は登録されているデッキドライバの数を返す
// deckdriversInit()を呼び出して初期化を行い、driversLen変数に格納されたドライバの数を返す
// この関数はdeckDriverCount()で呼び出される
// これにより、デッキドライバの数を効率的に取得できる
// 戻り値: 登録されているデッキドライバの数
int deckDriverCount() {
  deckdriversInit();

  return driversLen;
}

const struct deck_driver* deckGetDriver(int i) {
  deckdriversInit();

  if (i<driversLen) {
    return drivers[i];
  }
  return NULL;
}

const DeckDriver* deckFindDriverByVidPid(uint8_t vid, uint8_t pid) {
  int i;

  deckdriversInit();

  for (i=0; i<driversLen; i++) {
    if ((vid == drivers[i]->vid) && (pid == drivers[i]->pid)) {
      return drivers[i];
    }
  }
  return NULL;
}

const DeckDriver* deckFindDriverByName(char* name) {
  int i;

  deckdriversInit();

  for (i=0; i<driversLen; i++) {
    if (!strcmp(name, drivers[i]->name)) {
      return drivers[i];
    }
  }
  return NULL;
}
