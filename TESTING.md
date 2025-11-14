# VL53L8CX11 実機テスト手順書

このドキュメントは、VL53L8CX11 deck の改修版ファームウェアを実機で動作確認するための手順をまとめたものです。

## 前提条件

- **ビルド済みファームウェア**: `build/cf2.bin` および `build/cf2.hex` が存在すること（`make -j$(nproc)` で生成済み）
- **Crazyflie 2.x**: 実機本体（VL53L8CX11 deck 搭載）
- **Crazyradio PA**: USBドングル（Crazyflie と通信するため）
- **cfclient / cfloader**: Python の pip でインストール済み（`pip install cfclient cfloader` 等）
- **Python 環境**: `cflib` がインストール済み（`get_distance_data/tof.py` の実行に必要）

---

## ステップ 1: ファームウェアのフラッシュ

### 方法 A: 手動でブートローダーモードに入れてフラッシュ

1. **Crazyflie の電源を切る**
2. **ブートローダーモードで起動**:
   - 電源ボタンを **3秒間押し続ける**
   - 青色 LED が両方点滅し始めたらブートローダーモードに入っています
3. **ターミナルでフラッシュコマンドを実行**:
   ```bash
   cd /home/h-tanaka/workspace/crazyflie/crazyflie-firmware-feat_twr
   make cload
   ```
4. **結果**:
   - `make cload` が自動的にブートローダーモードの Crazyflie を検出してファームウェアを書き込みます
   - 書き込み完了後、Crazyflie が自動的に再起動します

**注意**: 周囲に複数の Crazyflie がブートローダーモードで起動していると予期しない動作をする可能性があります。

---

### 方法 B: URI を指定して自動フラッシュ

（Crazyflie が正常に起動している場合に使用）

1. **Crazyflie の電源を入れる**（通常起動）
2. **Crazyflie の URI を確認**:
   - 例: `radio://0/80/2M/E7E7E7E7E7` (デフォルト)
   - cfclient で接続して URI を確認することもできます
3. **ターミナルでフラッシュコマンドを実行**:
   ```bash
   cd /home/h-tanaka/workspace/crazyflie/crazyflie-firmware-feat_twr
   CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
   ```
   または直接 cfloader を使用:
   ```bash
   cfloader flash build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7
   ```
4. **結果**:
   - Crazyflie が自動的にブートローダーモードに入り、ファームウェアが書き込まれます
   - 書き込み完了後、自動的に再起動します

**注意**: この方法は、現在のファームウェアが正常に動作している場合にのみ使用できます。ファームウェアが壊れている場合は方法 A を使用してください。

---

## ステップ 2: 動作確認（tof.py を使用）

フラッシュ完了後、`get_distance_data/tof.py` スクリプトを使って VL53L8CX11 deck が正しく動作するか確認します。

### 2-1: デッキ検出とパラメータ確認

```bash
cd /home/h-tanaka/workspace/crazyflie/crazyflie-firmware-feat_twr
python3 ./get_distance_data/tof.py
```

**期待される出力**:
- デッキが検出される（`vl11.enable` パラメータが見つかる）
- ファームウェアからの DEBUG_PRINT メッセージがコンソールに表示される
- CSV ファイルにログデータが保存される

---

### 2-2: リアル距離データの取得テスト（`--real` モード）

```bash
python3 ./get_distance_data/tof.py --real
```

**動作**:
- `vl11.testGen=0` を設定（テストデータ生成を無効化）
- `vl11.forceInit=1` を設定（強制初期化を実行）
- 通常のログ収集を開始

**期待される出力**:
- ファームウェアから `"forceInit triggered, setting g_blobsOk=1"` のようなデバッグメッセージが出る
- ログデータに実際の距離測定値が含まれる（ゼロ以外の値）
- CSV ファイルに保存される

---

### 2-3: 自動リアルモード（`--auto-real` モード）

```bash
python3 ./get_distance_data/tof.py --auto-real
```

**動作**:
- `vl11.testGen=0` を設定
- `vl11.forceInit=1` を設定
- ログを収集し、**ゼロ以外の距離データが見つかるまで**繰り返し `forceInit` をトリガー
- 実データが見つかったら CSV に保存して終了

**期待される出力**:
- "Waiting for real distance data..." というメッセージが表示される
- 実際に測定されたデータが見つかったら "Real distance data detected! Saving and exiting." と表示される
- CSV ファイルに実距離データが保存される

---

## ステップ 3: 成功基準

以下の条件が満たされていれば、今回の改修は成功です:

1. **ファームウェアが正常にフラッシュされる**（エラーなし）
2. **tof.py でデッキが検出される**（`vl11.enable` パラメータが見つかる）
3. **ファームウェアのデバッグメッセージが見える**（`DEBUG_PRINT` 出力が転送される）
4. **`--real` モードで実距離データが取得できる**（ゼロ以外の値）
5. **`--auto-real` モードで自動的に実データが検出・保存される**

---

## トラブルシューティング

### 問題 1: `make cload` が Crazyflie を見つけられない

- **原因**: Crazyflie がブートローダーモードに入っていない、または Crazyradio PA が認識されていない
- **対処**:
  - Crazyflie を再度ブートローダーモードで起動（電源ボタン 3秒長押し）
  - Crazyradio PA が USB に正しく接続されているか確認
  - `lsusb` で Crazyradio PA が見えるか確認

### 問題 2: tof.py が `vl11.enable` を見つけられない

- **原因**: ファームウェアが正しくフラッシュされていない、またはデッキが接続されていない
- **対処**:
  - VL53L8CX11 deck が物理的に Crazyflie に装着されているか確認
  - ファームウェアを再フラッシュ
  - cfclient で接続してパラメータリストを確認（`vl11.*` が見えるか）

### 問題 3: 距離データがすべてゼロ

- **原因**: センサー初期化が失敗している、または `testGen=1` のままになっている
- **対処**:
  - `--real` または `--auto-real` モードを使用して `vl11.testGen=0` を確実に設定
  - `vl11.forceInit=1` をトリガーして初期化を再実行
  - ファームウェアのコンソール出力を確認（`g_blobsOk` や `vl11Task` の状態）

### 問題 4: ファームウェアのデバッグメッセージが見えない

- **原因**: コンソール転送が有効になっていない、または cflib のバージョンが古い
- **対処**:
  - tof.py のコンソールコールバックが正しく登録されているか確認
  - cfclient を起動してコンソールタブで確認
  - cflib を最新版にアップデート (`pip install --upgrade cflib`)

---

## 追加情報

### パラメータ一覧（vl11 グループ）

| パラメータ名       | 型    | 説明                                         |
|-------------------|-------|---------------------------------------------|
| `vl11.enable`     | UINT8 | VL11 ドライバの有効/無効（0=無効, 1=有効）      |
| `vl11.testGen`    | UINT8 | テストデータ生成モード（0=実データ, 1=疑似データ）|
| `vl11.forceInit`  | UINT8 | 強制初期化トリガー（書き込むと初期化実行）       |

### ログ変数一覧（vl11 グループ）

- `vl11.d0` ～ `vl11.d63`: 8x8 ゾーンの距離データ (mm)
- `vl11.running`: ドライバタスクの実行状態

---

## 参考資料

- **Crazyflie Firmware ビルド・フラッシュ公式ドキュメント**: 
  `docs/building-and-flashing/build.md`
- **cfloader コマンドヘルプ**:
  ```bash
  cfloader --help
  ```
- **VL53L8CX ULD API リファレンス**:
  ST Microelectronics 公式ドキュメント

---

## まとめ

このドキュメントの手順に従うことで、改修版ファームウェアを実機にフラッシュし、VL53L8CX11 deck からリアル距離データを取得できることを確認できます。問題が発生した場合は、トラブルシューティングセクションを参照してください。

**次のステップ**: 実機テストで問題が見つかった場合は、ファームウェアのデバッグ出力とログデータを確認し、必要に応じてコードを修正してください。
