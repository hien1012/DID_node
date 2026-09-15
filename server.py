from flask import Flask, jsonify, request
import time
import datetime

app = Flask(__name__)

@app.route('/time', methods=['GET'])
def get_time():
    current_epoch = int(time.time())
    print(f"[{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] ESP32 請求時間 -> 發送 Epoch: {current_epoch}")
    return jsonify({"epoch": current_epoch}), 200

# 新增：接收 ESP32 傳來的感測器資料
@app.route('/upload', methods=['POST'])
def upload_data():
    if request.is_json:
        data = request.get_json()
        print(f"[{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 收到資料: {data}")
        # 這裡未來可以寫入 SQLite 或存成檔案
        return jsonify({"status": "success", "message": "Data saved"}), 200
    else:
        return jsonify({"status": "error", "message": "Not JSON format"}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)