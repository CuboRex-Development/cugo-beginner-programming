# CugoArduinoKitクイックスタートガイド
## 1. はじめに
本リポジトリはCugoArduinoKitを利用するためのサンプルコードです。
CugoArduinoKitを使ってラジコン走行や自動走行を実現できます。</br>
**★ここに画像もらう**
## 2. 準備
CugoArduinoKitの利用開始までの手順を説明します。
### 2-1. 事前準備
クイックスタートの前に以下の手順が完了しているかを確認してください<br>
- [ ] CugoArduinoKitの組み立て
  - 組み立てマニュアルはこちら　**★URL**
- [ ] USBケーブル：Arduinoとパソコンを接続する用
- [ ] パソコン (WindowsまたはMac)
### 2-2. Arduino IDEのインストール
1. 公式ページ( https://www.arduino.cc/en/software )へ移動
2. DOWNLOAD OPTIONSから適切なバージョンを選択
3. JUST DOWNLOADかCONTRIBUTE & DOWNLOADを選択
4. ダウンロードされたらファイルを実行して指示に従いインストール
### 2-2. 学習用ソースコードダウンロード
1. ここをクリックしてダウンロード
   - **★スクショ赤字の四角枠**
2. ダウンロードしたファイルを解凍
3. CuGoArduinoBeginnerProgramming.inoをダブルクリックし、ArduinoIDEを起動
### 2-3. Aruduino UNOへの書き込み

1. CuGoArduinoBeginnerProgramming.inoがArduinoIDEで開かれていることを確認
2. USBケーブルでパソコンとAruduinoを接続
3. ツール ＞ ボード から"Arduino Uno"を選択　
4. ツール ＞ ポート からArduinoのポートを選択
   - ★リンクをはるか説明入れる？他サイトも確認
5. 矢印ボタン " → "を選択し、マイコンボードへ書き込むを実行
6. ボードへの書き込みが完了しましたの記載があれば書き込み完了
## 3. 使用方法
CugoArduinoKitでは2つのモードが利用できます
### 3-1. ラジコンモードの利用
ラジコンモードはCuGoArduinoBeginnerProgramming.inoを書き込み後、付属のコントローラの左スティックを左側に倒すことでラジコンモードが開始します。 <br>
**★ここに画像　編集する** <br>
ラジコンモードでの操作方法は<br>
- 左スティックの上下操作が左クローラーの前進後進移動
- 右スティックの上下操作が右クローラーの前進後進移動　<br>

になります<br>
**★ここに画像　編集する** <br>
### 3-2. 自動走行モードの利用
CuGoArduinoBeginnerProgramming内の一番下にあるCMD_EXECUTEの関数内をプログラミングすることで自動走行が可能です。
### コマンド 一覧
CMD_EXECUTE内では以下のコマンドの実行が可能です。<br>

- 基本コマンド 一覧<br>

|  コマンド名  |  動作  |　備考 |
| ---- | ---- | ---- |
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
|  susumu(1.0,180)  |  前に1m進む  | ()内の数字を変更することで進む距離と速度が変更できる |
|  modoru(1.0,180)  |  後ろに1m進む  | ()内の数字を変更することで進む距離と速度が変更できる |
|  migimawari45()  |  右回りに45度回転  |      |
|  hidarimawari45()  |  左回りに45度回転  |      |
|  migimawari90()  |  右回りに90度回転  |      |
|  hidarimawari90()  |  左回りに90度回転  |      |
|  migimawari180()  |  右回りに180度回転  |      |
|  hidarimawari180()  |  左回りに180度回転  |      |

## 4. サンプルコード解説
### 4-1. サンプルコードの実行
1. ファイル内のプログラミングの一番下にある下のコードを確認してください。

<details>

<summary>コードを確認する場合はこちらをクリック</summary>

 ```c
  void CMD_EXECUTE()
  {
  cmd_manager();  // おまじない
  // ここから↓を改造していこう！
  button();//ボタン押し待ち
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
こちらのコマンドを実行するとCugoArduinoKitが正方形に移動します。


### 4-2. サンプルコードの変更
#### 4-2-1. 長方形を描くには
- 問題：長方形を書くにはどうすれば良いでしょうか？
  - ヒント：susumu() のコマンドのカッコ()の中は進む距離を表しています。
  - いろいろ試して確認してみましょう正解は詳細例に記載しています。

<details>
<summary>回答はこちらをクリック</summary>

デモのsusumuコマンドの中を変更して、向かい合う辺は同じ距離を進むように変更しよう<br>
以下、正解例
 ```c
  void CMD_EXECUTE()
  {
  cmd_manager();  // おまじない
  // ここから↓を改造していこう！
  button();//ボタン押し待ち
  susumu(2.0); // ★デモプログラミングからの変更箇所
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(1.0);
  matsu(1000); 
  
  migimawari90();
  matsu(1000); 
  
  susumu(2.0); // ★デモプログラミングからの変更箇所
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

## 5. お問い合わせ先
- 疑問点、不明点がある場合は issue を立ててください。
  - **イシューのリンク**