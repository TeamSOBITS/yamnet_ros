<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# YAMNet ROS

<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
    <li><a href="#パラメーター">パラメーター</a></li>
    <li><a href="#トピックとアクション">トピックとアクション</a></li>
    <li><a href="#ライセンス">ライセンス</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>


## 概要
`yamnet_ros` は，Googleの音声分類モデル **YAMNet** を ROS 2 で利用するためのラッパーパッケージです．

YAMNet は **521クラス**の音響イベントをリアルタイムに分類できます．デフォルト設定ではドアベル・ベル音の検出に最適化されていますが，`target_labels` パラメータを変更するだけで，**任意の音響イベント検出**に転用できます．

音声キャプチャには ALSA（`arecord`）を直接使用するため，追加の音声ライブラリは不要です．

**主な機能:**
- 521クラス対応のリアルタイム音響イベント検出（デフォルト：ドアベル・ベル系）
- 検出のたびに `SoundDetection` メッセージを配信
- アクションサーバ（`ListenForSound`）によるSMACHステートマシン・ビヘイビアツリーとの連携 — 検出またはタイムアウトまでブロック
- `target_labels` をYAMLで変更するだけで検出対象を自由にカスタマイズ可能
- LifecycleNode による起動・停止の明示的な制御（未使用時はCPUゼロ）

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## セットアップ

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

| System | Version |
| ------ | ------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |

### インストール方法
1. ROS 2 の `src` フォルダに移動します．
   ```sh
   cd ~/colcon_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   git clone -b jazzy-devel https://github.com/TeamSOBITS/yamnet_ros.git
   ```
3. レポジトリの中へ移動します．
   ```sh
   cd yamnet_ros
   ```
4. 依存パッケージをインストールします（ウェイトファイル・YAMNetソース・Python依存関係を自動ダウンロード）．
   ```sh
   bash install.sh
   ```
5. パッケージをビルドします．
   ```sh
   cd ~/colcon_ws/
   colcon build --packages-select sobits_interfaces yamnet_ros
   source install/setup.bash
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 実行・操作方法

1. ノードを起動します．
   ```sh
   ros2 launch yamnet_ros yamnet_ros.launch.py
   ```

2. 別のターミナルで LifecycleNode を設定・有効化します．
   ```sh
   ros2 lifecycle set /yamnet_ros configure
   ros2 lifecycle set /yamnet_ros activate
   ```

3. 検出イベントを確認します．
   ```sh
   ros2 topic echo /yamnet_ros/sound_detection
   ```

4. アクションサーバでベル音を待機します（スクリプトやステートマシンから利用）．
   ```sh
   # 最大30秒待機（検出次第即時リターン）
   ros2 action send_goal /yamnet_ros/listen_for_sound \
     sobits_interfaces/action/ListenForSound \
     "{target_labels: ['Doorbell', 'Bell'], timeout: {sec: 30, nanosec: 0}}"
   ```

5. ファイルを編集せずにパラメータを上書きして起動できます．
   ```sh
   ros2 launch yamnet_ros yamnet_ros.launch.py \
     detection_threshold:=0.25 \
     hop_secs:=0.25 \
     audio_device:=hw:1,0
   ```

6. 利用可能なマイクデバイスを確認します．
   ```sh
   arecord -l
   ```

> **注意:** このノードはALSAに直接アクセスするため，デスクトップのマイクアイコンは表示されません．これは正常な動作です．PulseAudioをバイパスすることで低遅延を実現しています．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## パラメーター

全パラメータは [`config/yamnet_ros.yaml`](config/yamnet_ros.yaml) で設定し，launchファイルから上書き可能です．

| パラメータ | 説明 | デフォルト値 |
| --------- | ---- | ----------- |
| `weights_path` | `yamnet.h5` のパス．空文字 = パッケージshareから自動解決． | `''` |
| `sample_rate` | 音声サンプリングレート (Hz)．YAMNetは16000が必須． | `16000` |
| `detection_threshold` | 検出とみなす最小YAMNetスコア [0–1]．小さいほど感度が高い． | `0.15` |
| `hop_secs` | 推論実行間隔（秒）．小さいほど応答が速くCPU負荷が高い． | `0.5` |
| `window_secs` | YAMNetに渡すスライディング音声窓の長さ（秒）．最小約0.96秒． | `1.0` |
| `audio_device` | ALSAキャプチャデバイス名．`arecord -l` で一覧確認可能． | `hw:1,7` |
| `audio_channels` | キャプチャチャンネル数．DMIC（hw:1,7）は2，アナログマイクは通常1． | `2` |
| `cooldown_secs` | 連続検出イベント間の最小間隔（秒）． | `2.0` |
| `target_labels` | YAMNetクラス名と照合するキーワードリスト． | yamlを参照 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## トピックとアクション

### パブリッシュ

| トピック | 型 | 説明 |
| ------- | -- | ---- |
| `/yamnet_ros/sound_detection` | `sobits_interfaces/SoundDetection` | ベル音検出のたびに配信．ラベル・スコア・上位クラスを含む． |

### アクションサーバ

| アクション | 型 | 説明 |
| --------- | -- | ---- |
| `/yamnet_ros/listen_for_sound` | `sobits_interfaces/action/ListenForSound` | 指定した音の検出または `timeout` 経過までブロック．空の `target_labels` はノード既定のラベルを使います．`detected`・`label`・`score`・`elapsed_time` を返し，ホップごとにフィードバックを送信します． |

### SoundDetection.msg フィールド

| フィールド | 型 | 説明 |
| --------- | -- | ---- |
| `header` | `std_msgs/Header` | 検出タイムスタンプ |
| `label` | `string` | 最も一致したベル系クラス名（例：`"Doorbell"`，`"Bell"`） |
| `score` | `float32` | そのクラスのYAMNetスコア |
| `top_label` | `string` | 全クラス中の最上位クラス（診断用） |
| `top_score` | `float32` | 最上位スコア（診断用） |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ライセンス
本パッケージは **BSD 3-Clause License** で配布されています．全文は [LICENSE](LICENSE) を参照してください．

### サードパーティコード
[`yamnet_ros/yamnet_src/`](yamnet_ros/yamnet_src/) 配下のファイル（`yamnet.py`，`features.py`，`params.py`，`yamnet_class_map.csv`）は [tensorflow/models](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet) から取得したものであり，**Apache License 2.0**（Copyright 2019 The TensorFlow Authors）が適用されます．元のライセンスヘッダはそのまま保持しています．

学習済みウェイトファイル `weights/yamnet.h5` は Google Research が Apache License 2.0 のもとで公開しているものです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 参考文献
- [YAMNet — TensorFlow Models (AudioSet)](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet)
- [AudioSet Ontology](https://research.google.com/audioset/ontology/index.html)
- [Google Research — YAMNet](https://tfhub.dev/google/yamnet/1)

[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/yamnet_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/yamnet_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/yamnet_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/yamnet_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[license-url]: LICENSE
