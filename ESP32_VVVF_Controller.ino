// Wi-Fiライブラリをインポート
#include <WiFi.h>
#include <Wire.h>

// Wi-Fi接続情報を入力
const char* ssid     = "********";
const char* password = "******";

// ウェブサーバーをポート80で開始
WiFiServer server(80);

// HTTPリクエストを保存しておく変数
String header;

// ノッチの状態を格納
int = notch = 2; //初期状態 = N
const int notch_max = 4;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // ssidとpasswordを用いてWi-Fiに接続
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // IPアドレスを出力し、webserverをスタート
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   //クライアント（スマホやPCなど）がつながっているかどうかをclientに出力

  if (client) {                             // クライアントが来たとき
    Serial.println("New Client.");
    String currentLine = "";                // クライアントからくるデータを格納する変数
    while (client.connected()) {            // クライアントがつながっている間、以下をループ
      if (client.available()) {             // クライアントからデータが来ているとき
        char c = client.read();             // データを読み込み
        Serial.write(c);                    // 届いたデータをシリアルモニタに出力
        header += c;
        if (c == '\n') {                    // 届いたデータが改行コードだった時
          // もし現在の行が空白ならば、この改行コードのみ受け取る
          // つまりHTTPリクエストの終わりなので、レスポンスを返す
          if (currentLine.length() == 0) {
            // HTTPヘッダは（HTTP/1.1 200 OK)のようなステータスコードから始まる
            // 次にコンテントタイプを送信。今回はhtml形式なので以下のようにする
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // リクエストに従ってGPIOをスイッチする
            if (header.indexOf("GET /notch/up") >= 0) {
              Serial.println("Notch Up");
			  if(notch =< notch_max){
				  notch += 1;
			  }
		  } else if (header.indexOf("GET /notch/down") >= 0) {
              Serial.println("Notch Down");
              if(notch > 0){
				  notch -= 1;
			  }
		  }
		String buf = string(notch);
		Wire.beginTransmission(0x04);
		Wire.write(buf);
		Wire.endTransmission();

            // htmlを表示
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // ON/OFFボタンのためのCSS
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // ページ本体（bodyタグ内）
            client.println("<body><h1>ESP32 Web Server</h1>");
            // 現在のピンの状態と、オンオフ用のボタンを出力
            client.println("<p>Notch " + notch + "</p>");
            // ノッチアップ/ダウンボタン
			client.println("<p><a href=\"/notch/down\"><button class=\"button\">Notch DOWN</button></a></p>");
            client.println("<p><a href=\"/notch/up\"><button class=\"button button2\">Notch UP</button></a></p>");
            client.println("</body></html>");

            //　HTTPレスポンスの最後は改行で終了
            client.println();
            // whileループの終了
            break;
          } else { // 改行コードを取得したら、currentLineをリセット
            currentLine = "";
          }
        } else if (c != '\r') {  // 改行以外の何かしらのコードが来ているとき
          currentLine += c;      // currentLineに追加
        }
      }
    }
    // ヘッダーをリセット
    header = "";
    // 接続をリセット
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
