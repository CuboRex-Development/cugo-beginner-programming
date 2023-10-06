# クローラロボット開発プラットフォーム　スタートガイド
旧製品：ArduinoKitをご利用の方は、こちらをご覧ください。</br>
https://github.com/CuboRex-Development/cugo-beginner-programming/tree/uno

より発展的な開発をされたい方はこちらのSDKコマンドリファレンスをご覧ください。</br>
https://github.com/CuboRex-Development/cugo-sdk-samples/tree/pico

## 1. はじめに
本リポジトリはクローラロボット開発プラットフォームを利用するためのサンプルコードです。
クローラロボット開発プラットフォームを使ってラジコン走行や自動走行を実現できます。</br>
![V4メイン](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/e1b76ade-498c-49db-9c62-2013c0201fa4)

## 2. 準備
クローラロボット開発プラットフォームの利用開始までの手順を説明します。

### 2-1. Arduino IDEのインストール
1. 公式ページ( https://www.arduino.cc/en/software )へ移動
2. DOWNLOAD OPTIONSからご自身のOSのバージョンを選択
3. JUST DOWNLOADかCONTRIBUTE & DOWNLOADを選択
4. ダウンロードされたらファイルを実行して指示に従いインストール
### 2-2. 学習用ソースコードダウンロード
1. Codeボタンを選択し、Download ZIPをクリックしてダウンロード
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/ea21aa32-1b2e-4d48-852d-678ad300485b)

2. ダウンロードしたファイルを解凍
3. CugoBeginnerProgramming.inoをダブルクリックし、ArduinoIDEを起動

### 2-3. Rasberry Pi Picoの初期設定
Arduino IDE でRaspberryPiPicoに書き込む場合、IDEにRaspberryPiPicoのボード情報をあらかじめ取得する必要があります。</br>

Arduino IDE バージョン2系（最新版）の場合

1. ファイル ＞ 環境設定を選択</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f08cbb59-36f2-4ba0-80a3-889a7c337e0f)

2. 追加のボードマネージャのURLに以下のURLを入力し、OKを押します。右のウィンドウボタンをクリックすると入力できるようになります。</br>
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/fc20fa37-3747-4060-8da3-9ae169b9df93)

4. ツール ＞ ボード ＞ ボードマネージャ…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/320f1434-1135-4788-8b6e-df6194c24781)

5. "pico"で検索し、”Raspberry Pi Pico/RP2040”を見つけます。”INSTALL”ボタンを押します（すでに入っている場合はUPDATEボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/98a99647-b980-4e2b-aec4-b91b3419649b)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f5135740-f1cb-436a-8d63-f68a6cd3122f)

6. スケッチ ＞ ライブラリをインクルード… ＞ ライブラリを管理…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/1e1775bd-3399-4f40-82d8-8d5590d21918)

7. RPi_Pico_TimerInterrupt.hで検索し、”RPI_PICO_TimerInterrupt”を見つけます。”INSTALL”ボタンを押します（すでに入っている場合はUPDATEボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/1216aeee-82ef-4b6a-8cab-25935b4cab82)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/d236d67f-2156-4f86-a9bd-0e3b68070bc9)




Arduino IDE バージョン1.8.19（レガシー）の場合

<details>
1. ファイル ＞ 環境設定を選択</br>

![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/f86d603a-0910-4281-a71f-8a6cde1fee40)


2. 追加のボードマネージャのURLに以下のURLを入力し、OKを押します。右のウィンドウボタンをクリックすると入力できるようになります。</br>
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/0b957362-a55b-4591-a8e7-7bb681eed5a0)

4. ツール ＞ ボード ＞ ボードマネージャ…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/ac1f3c62-d372-4bab-ae1a-3f9090a69e10)


5. "pico"で検索し、”Raspberry Pi Pico/RP2040”を見つけます。”インストール”ボタンを押します（すでに入っている場合は"更新"ボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/6341088e-1532-4c35-85d6-fcca6491ac46)
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/be7e4e8c-fc26-4fa0-9182-6f0392029942)


6. スケッチ ＞ ライブラリをインクルード… ＞ ライブラリを管理…を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/cf2bb2d8-17b1-4421-8b2a-f86456e95f83)


7. RPi_Pico_TimerInterrupt.hで検索し、”RPI_PICO_TimerInterrupt”を見つけます。”インストール”ボタンを押します（すでに入っている場合は"更新"ボタンを押して最新にします）。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/af765969-7bbd-4d24-8ce6-1c5496885bd2)


</details>


Pythonの'encoding'モジュールが見つからないエラーでビルドできない場合
<details>
以下の図のようにpythonがうまく実行できないことでコンパイルが通らないことがあります。これはボードマネージャでRP2040をインストールしたのち、
削除してもう一度入れなおした際に発生します。すでに一度RP2040をインストールした後に2回目以降にインストールする際に必要なデータが展開しきれず、必要なものが再配置されないバグが発生していると考えられます。
  
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/22425319/bdbb5a6a-901c-4152-b1f5-6bb783291450)


コマンドプロンプトなどのなんらかのShellを起動して以下のファイルを削除してください。</br>
C:\Users\YutaNakamura\AppData\Local\Arduino15\packages\rp2040\tools\pqt-python3\1.0.1-base-3a57aed\

![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/22425319/6f26ac47-6050-43bc-bdfa-2f72e3d013c7)


上記のファイルを削除したのち、ボードマネージャからRP2040を削除します。
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/22425319/02531edd-d647-4972-8e04-e68af82a8892)


再度RP2040をインストールし、コンパイルできるようになったかを確認します。
![image](https://github.com/CuboRex-Development/cugo-sdk-samples/assets/22425319/af1ba464-8246-4cb7-8ad5-cc361c9c6bf0)

</details>

### 2-4. 制御パラメータの確認
クローラロボット開発プラットフォームには、V3iモデルとV4モデルがあります。各モデルごとに制御パラメータが異なる場所がありますので、確認をしてください。</br>

Arduino IDE上部タブから”CugoCommandMode.h”を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/317a7a9c-4c36-41dc-a885-6b6a8229162e)


下記パラメータを使用しているモデルに合わせます。
https://github.com/CuboRex-Development/cugo-beginner-programming/blob/339cfe4f8766082ebc18254b1fb26a08e4c47f0f/CugoBeginnerProgramming/CugoCommandMode.h#L13-L25

プログラムが下の図と同じになるように必要に応じてプログラムを書き換え、ご自身のモデルのパラメータが反映されるように調整してください。</br>

V4の場合</br>
”CugoCommandMode.h”は、サンプルプログラムから変更する必要はありません。</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/27883660/db5f101e-2856-4625-b521-bacb0cdf0d25)

V3iの場合</br>
”CugoCommandMode.h”の15行目から19行目の行頭に`//`を追記し、21行目から25行目の行頭の`//`を削除します。</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/27883660/7a9fe102-1705-432c-af83-82d7e4e5e73e)



### 2-5. Rasberry Pi Picoへの書き込み

1. CugoBeginnerProgramming.inoがArduino IDEで開かれていることを確認
2. USBケーブルでパソコンとRaspberry Pi Picoを接続。PCに認識されないときは基板にある”BOOTSEL”ボタンを押しながらPCに挿してください。</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/483c2a4e-8850-4924-90de-a17b09dd775f)

4. ツール ＞ ボード ＞ Raspberry Pi Pico/RP2040 ＞ Rasberry Pi Pico を選択
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/db962aa7-1deb-4125-a7f9-eec2b2707433)

6. ツール ＞ ポート からRasberry Pi Picoのポートを選択します。ポートはUSBを接続する前と後を比較して増えたものがRaspberry Pi Picoなので、それを選択します。</br>
USBを接続する前↓</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/02be584c-bd9d-4c96-a9dc-0bca2dbb5e5a)

USBを接続した後↓</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/50e47ad7-b85d-4dd5-82ba-f441ba3da83d)

8. 矢印ボタン " → "を選択し、マイコンボードへ書き込むを実行</br>
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/a7bd9db9-d3d0-4221-963a-2bff952eb833)

9. ボードへの書き込みが完了しましたの記載があれば書き込み完了です。

### 2-6. LD-2のコマンドモードを有効化
クローラロボット開発プラットフォームでは、安全のため、出荷時にプログラム動作する、コマンドモードを無効化しています。</br>
以下の図に従って、電源が切れていることを確認し、DIPスイッチの2番をON側に倒してください。大変小さなスイッチですので、つまようじなどを用意して操作してください。</br>
詳細は、取扱説明書をご覧ください。</br>
https://drive.google.com/drive/folders/18MQUVMLYS_4JgkeGd2v7dVHmdmFaMaZc?usp=drive_link

![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/fca21c7a-01e0-45cb-9bff-3d91ef784300)



## 3. 使用方法
クローラロボット開発プラットフォームでは2つのモードが利用できます
### 3-1. ラジコンモードの利用
ラジコンモードはCugoBeginnerProgramming.inoを書き込み後、付属のコントローラの左スティックを左側に倒すことでラジコンモードが開始します。 <br>
![controller](https://user-images.githubusercontent.com/22425319/208835611-c366764d-4b30-477a-aac8-c848712c4710.png)

ラジコンモードでの操作方法は<br>
- 左スティックの上下操作が左クローラーの前進後進移動
- 右スティックの上下操作が右クローラーの前進後進移動　<br>

になります<br>
![radiocontrol](https://user-images.githubusercontent.com/22425319/208835778-1dd170dc-3de1-4dce-b7ee-83f2c1e0838d.png)
### 3-2. プログラムモードの利用
CugoBeginnerProgramming内の一番下にあるCMD_EXECUTEの関数内をプログラミングすることで自動走行が可能です。
### コマンド 一覧
CMD_EXECUTE内では以下のコマンドの実行が可能です。<br>

- 基本コマンド 一覧<br>

|  コマンド名  |  動作  |　備考 |
| ---- | ---- | ---- |
|  matsu(1000)  |  1000ミリ秒待機  | 入力した()内の数字ミリ秒だけ待機する。1000ミリ秒＝1秒。 |
|  susumu(1.0)  |  前に1m進む  | ()内の数字を変更することで進む距離が変更できる |
|  modoru(1.0)  |  後ろに1m進む  | ()内の数字を変更することで進む距離が変更できる |
|  migimawari45()  |  右回りに45度回転  |      |
|  hidarimawari45()  |  左回りに45度回転  |      |
|  migimawari90()  |  右回りに90度回転  |      |
|  hidarimawari90()  |  左回りに90度回転  |      |
|  migimawari180()  |  右回りに180度回転  |      |
|  hidarimawari180()  |  左回りに180度回転  |      |
   
- 応用コマンド 一覧<br> 

|  コマンド名  |  動作  |　備考 |
| ---- | ---- | ---- |
| susumu(1.0,90) | 上限速度90rpmで1.0m前に進む  | ()内に移動距離と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合も　|
| sagaru(1.0,90) | 上限速度90rpmで1.0m後ろに進む | ()内の設定はsusumu(1.0,90)と同様 |
| migimawari45(90) | 上限速度90rpmで45度右に回転する | ()内に上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合も　|
| migimawari90(90) | 上限速度90rpmで90度右に回転する | ()内の設定はmigimawari45(90)と同様 |
| migimawari180(90) | 上限速度90rpmで180度右に回転する  | ()内の設定はmigimawari45(90)と同様 |
| hidarimawari45(90) | 上限速度90rpmで45度左に回転する | ()内の設定はmigimawari45(90)と同様 |
| hidarimawari90(90) | 上限速度90rpmで90度左に回転する | ()内の設定はmigimawari45(90)と同様 |
| hidarimawari180(90) | 上限速度90rpmで180度左に回転する | ()内の設定はmigimawari45(90)と同様 |
| turn_clockwise(60,90) | 上限速度90rpmで60度右に回転する | ()内に回転角度と上限速度を設定。上限速度は最大180rpmで距離が短いと上限速度に到達しない場合も |
| turn_counter_clockwise(60,90) | 上限速度90rpmで60度左に回転する | ()内の設定はturn_clockwise(60,90)と同様 |
## 4. サンプルコード解説
### 4-0. サンプルコードの使い方
サンプルコードはCMD_EXECUTE()の中を変更するとコマンドを読み取り、CuGoが動作します。その他の部分を変更すると正常に動作しなくなります。何を変更してしまって、わけが分からなくなった場合は再DLしてもう一度編集しなおしてください。
![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/30005762-9c94-4e0a-abd8-a90d3870ea01)

![image](https://github.com/CuboRex-Development/cugo-beginner-programming/assets/22425319/447efb9e-b5b8-4b9e-93f0-3d6374e6ac50)


### 4-1. サンプルコードの実行
1. ファイル内のプログラミングの一番下にある下のコードを確認してください。
- <details>
  <summary>コードを確認する場合はこちらをクリック</summary>
  
  ```c
  void CMD_EXECUTE()
  {
  cmd_manager();  // おまじない
  // ここから↓を改造していこう！

  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000);
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000);  
  // ここから↑を改造していこう！
  cmd_end(motor_controllers);      // おまじない
  }
  ```
</details>

2. コマンドの説明 <br>
こちらのコマンドを実行するとクローラロボットが正方形に移動します。


### 4-2. サンプルコードの変更
#### 4-2-1. 長方形を描くには
- 問題：クローラロボットが長方形を描く方法を考えましょう。
  - ヒント：susumu() のコマンドのカッコ()の中は進む距離を表しています。
  - いろいろ試して確認してみましょう正解は詳細例に記載しています。

- <details>
  <summary>回答はこちらをクリック</summary>
  回答例はこちら<br>

   ```c
  void CMD_EXECUTE()
  {
  cmd_manager();  // おまじない
  // ここから↓を改造していこう！

  susumu(0.5); // ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(0.5); // ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000);
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000);  

  // ここから↑を改造していこう！
  cmd_end(motor_controllers);      // おまじない
  }
  ```
</details>

#### 4-2-2. 速度を変えるには
- 問題：クローラロボットが速度を変えて進む方法を考えましょう。
  - ヒント：susumu() の応用コマンドのカッコ()の中は進む距離と上限速度を表しています。
  - いろいろ試して確認してみましょう正解は詳細例に記載しています。

- <details>
  <summary>回答はこちらをクリック</summary>
  susumuコマンドの中を変更して、向かい合う辺は同じ距離を進むように変更しよう<br>

   ```c
  void CMD_EXECUTE()
  {
  cmd_manager();  // おまじない
  // ここから↓を改造していこう！

  susumu(1.5,60); // ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.0,120);// ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.5,40); // ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000);
  
  susumu(1.0,120);// ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000);  
  
  // ここから↑を改造していこう！
  cmd_end(motor_controllers);      // おまじない
  }
  ```
</details>

## 5. お問い合わせ先
- 疑問点、不明点がある場合は issue を立ててください。
  - https://github.com/CuboRex-Development/cugo-arduino-beginner-programming/issues
